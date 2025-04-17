// ===============================
// NodeA - Sensing and Receiving Node
// ===============================

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include "net/linkaddr.h"
#include "lib/random.h"
#include "board-peripherals.h"  // For sensor access
#include <string.h>
#include <stdio.h> 
#include "node-id.h"

// Link quality threshold (RSSI in dBm)
#define RSSI_THRESHOLD -70

// Data collection parameters
#define DATA_COLLECTION_RATE CLOCK_SECOND  // 1Hz sampling rate (as per requirements)
#define MAX_READINGS 60                    // Store 60 readings (as per requirements)

// Packet types
#define PACKET_TYPE_BEACON 1
#define PACKET_TYPE_DATA 2
#define PACKET_TYPE_ACK 3  // New packet type for acknowledgments

// Maximum readings to send in one packet
#define READINGS_PER_PACKET 10

// Maximum transmission attempts
#define MAX_RETRIES 3

// Timeout for acknowledgment
#define ACK_TIMEOUT (CLOCK_SECOND * 2)

// Beacon packet structure
typedef struct {
  uint8_t packet_type;
  unsigned long src_id;
  unsigned long timestamp;
  unsigned long seq;
} beacon_packet_struct;

// Data packet structure for sensor readings
typedef struct {
  uint8_t packet_type;
  unsigned long src_id;
  unsigned long start_idx;
  unsigned long num_readings;
  int16_t light_readings[READINGS_PER_PACKET];
  int16_t motion_readings[READINGS_PER_PACKET];
} data_packet_struct;

// Acknowledgment packet structure
typedef struct {
  uint8_t packet_type;
  unsigned long src_id;
  unsigned long received_count;  // Number of readings received so far
} ack_packet_struct;

// Variables for data collection
static int16_t light_readings[MAX_READINGS];
static int16_t motion_readings[MAX_READINGS];
static unsigned int reading_count = 0;
static unsigned int last_sent_idx = 0;
static struct etimer sensing_timer;

// Variables for data transfer
static unsigned int last_ack_count = 0;
static struct etimer ack_timer;
static unsigned int retry_count = 0;

// Variables for motion detection
static int prev_ax = 0;
static int prev_ay = 0;
static int prev_az = 0;
static uint8_t first_reading = 1;

// Variables for node discovery
static linkaddr_t last_neighbor;
static int8_t last_rssi = -128;
static uint8_t neighbor_found = 0;
static uint8_t data_transfer_active = 0;

// Processes
PROCESS(sensing_process, "Node A - Sensing Process");
PROCESS(data_transfer_process, "Node A - Data Transfer Process");
AUTOSTART_PROCESSES(&sensing_process);

// Initialize sensors
static void init_sensors(void) {
  printf("Initializing sensors...\n");
  SENSORS_ACTIVATE(opt_3001_sensor);
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

// Read light sensor (in lux)
static int16_t read_light_sensor(void) {
  int value = opt_3001_sensor.value(0);
  if (value == CC26XX_SENSOR_READING_ERROR) {
    printf("Error reading light sensor\n");
    return -1;
  }
  // Reactivate light sensor for next reading
  SENSORS_ACTIVATE(opt_3001_sensor);
  return (int16_t)value;
}

// Read motion sensor (accelerometer magnitude)
static int16_t read_motion_sensor(void) {
  // Get accelerometer readings
  int ax = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  int ay = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  int az = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  
  // Check for sensor errors
  if (ax == CC26XX_SENSOR_READING_ERROR || 
      ay == CC26XX_SENSOR_READING_ERROR || 
      az == CC26XX_SENSOR_READING_ERROR) {
    printf("Error reading motion sensor\n");
    return -1;
  }
  
  // Calculate motion (change in acceleration)
  int16_t motion_value = 0;
  
  if (!first_reading) {
    // Calculate difference from previous readings
    int dx = ax - prev_ax;
    int dy = ay - prev_ay;
    int dz = az - prev_az;
    
    // Calculate magnitude of change vector
    int32_t squared_delta = (int32_t)dx * dx + (int32_t)dy * dy + (int32_t)dz * dz;
    motion_value = (int16_t)(squared_delta / 100);
  } else {
    // First reading, no previous value to compare
    first_reading = 0;
  }
  
  // Store current values for next comparison
  prev_ax = ax;
  prev_ay = ay;
  prev_az = az;
  
  // Reactivate motion sensor for next reading
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
  
  return motion_value;
}

// Function called after reception of a packet
void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
  // Check if the received packet is a beacon
  if (len >= sizeof(uint8_t)) {
    uint8_t packet_type = *(uint8_t *)data;
    
    if (packet_type == PACKET_TYPE_BEACON && len == sizeof(beacon_packet_struct)) {
      beacon_packet_struct *received_packet = (beacon_packet_struct *)data;
      
      // Get current timestamp
      unsigned long curr_timestamp = clock_time();
      
      // Get RSSI (signal strength of the received packet)
      signed short rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
      
      // Log detection as per required format
      printf("%lu DETECT %lu\n", curr_timestamp / CLOCK_SECOND, received_packet->src_id);
      
      // Store neighbor information
      linkaddr_copy(&last_neighbor, src);
      last_rssi = rssi;
      neighbor_found = 1;
      
      // Check link quality - proceed with transfer if:
      // 1. RSSI is above threshold
      // 2. We have readings to send
      // 3. No transfer is currently active
      if (rssi >= RSSI_THRESHOLD && reading_count > 0 && !data_transfer_active) {
        // Log transfer as per required format
        printf("%lu TRANSFER %lu RSSI: %d\n", curr_timestamp / CLOCK_SECOND, 
               received_packet->src_id, rssi);
        
        // Start data transfer process
        data_transfer_active = 1;
        process_start(&data_transfer_process, NULL);
      } else {
        if (rssi < RSSI_THRESHOLD) {
          printf("Link quality not good enough (RSSI = %d)\n", rssi);
        } else if (reading_count == 0) {
          printf("No readings to transfer yet\n");
        } else if (data_transfer_active) {
          printf("Data transfer already in progress\n");
        }
      }
    }
    // Handle acknowledgment packets
    else if (packet_type == PACKET_TYPE_ACK && len == sizeof(ack_packet_struct)) {
      ack_packet_struct *ack_packet = (ack_packet_struct *)data;
      
      // Update last acknowledged count
      last_ack_count = ack_packet->received_count;
      
      printf("Received ACK: %u readings confirmed\n", last_ack_count);
      
      // Signal receipt of acknowledgment
      etimer_stop(&ack_timer);
      process_post(&data_transfer_process, PROCESS_EVENT_CONTINUE, NULL);
    }
  }
}

// Sensing process
PROCESS_THREAD(sensing_process, ev, data) {
  PROCESS_BEGIN();
  
  // Initialize sensors
  init_sensors();
  
  // Set up the reception callback for beacons
  nullnet_set_input_callback(receive_packet_callback);
  
  printf("Node A starting sensing process\n");
  printf("Collecting data at 1Hz for 60 seconds\n");
  printf("Listening for beacons from Node B...\n");
  
  // Turn radio on to listen for beacons
  NETSTACK_RADIO.on();
  
  // Continuously collect sensor data
  while (1) {
    // Set timer for periodic sensing (1Hz)
    etimer_set(&sensing_timer, DATA_COLLECTION_RATE);
    
    // Wait for timer
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
    
    // Skip collecting new data if transfer is active
    if (data_transfer_active) {
      printf("Data transfer active, pausing data collection\n");
      continue;
    }
    
    // Collect new sensor reading
    int16_t light = read_light_sensor();
    int16_t motion = read_motion_sensor();
    
    if (reading_count < MAX_READINGS) {
      // Buffer not full, add new reading
      light_readings[reading_count] = light;
      motion_readings[reading_count] = motion;
      
      printf("Reading %u: Light: %d lux, Motion: %d\n", 
             reading_count, light, motion);
      
      reading_count++;
      
      // Print message when buffer is full (60 readings collected)
      if (reading_count == MAX_READINGS) {
        printf("Buffer full with %u readings - ready for transfer\n", reading_count);
      }
    } else {
      // Buffer full - implement circular buffer
      
      // Check if we've sent all data
      if (last_sent_idx >= MAX_READINGS) {
        // Reset buffer and counters
        reading_count = 0;
        last_sent_idx = 0;
        
        // Store new reading
        light_readings[0] = light;
        motion_readings[0] = motion;
        printf("Reset buffer - Reading 0: Light: %d lux, Motion: %d\n", light, motion);
        reading_count = 1;
      } else {
        // Shift readings (circular buffer)
        unsigned int i;
        for (i = 0; i < MAX_READINGS - 1; i++) {
          light_readings[i] = light_readings[i + 1];
          motion_readings[i] = motion_readings[i + 1];
        }
        
        // Add new reading at the end
        light_readings[MAX_READINGS - 1] = light;
        motion_readings[MAX_READINGS - 1] = motion;
        
        printf("Buffer full - Added new reading (circular buffer)\n");
        
        // Adjust sent index if needed
        if (last_sent_idx > 0) {
          last_sent_idx--;
        }
      }
    }
  }
  
  PROCESS_END();
}

// Data transfer process
PROCESS_THREAD(data_transfer_process, ev, data) {
  PROCESS_BEGIN();
  
  static data_packet_struct data_packet;
  static unsigned int packet_count;
  static unsigned int total_packets;
  static unsigned int packet_size = 0;  // Initialize to avoid compiler warning
  static struct etimer packet_timer;
  
  // Initialize data packet
  data_packet.packet_type = PACKET_TYPE_DATA;
  data_packet.src_id = node_id;
  
  // Reset retry counter
  retry_count = 0;
  
  // Use last_ack_count to determine where to start sending
  if (last_ack_count > 0 && last_ack_count <= reading_count) {
    // Resume from where the receiver left off
    last_sent_idx = last_ack_count;
    printf("Resuming transfer from index %u\n", last_sent_idx);
  }
  
  // Calculate number of packets needed
  total_packets = (reading_count - last_sent_idx + READINGS_PER_PACKET - 1) / READINGS_PER_PACKET;
  
  printf("Starting data transfer: %u readings to send (from index %u to %u)\n", 
         reading_count - last_sent_idx, last_sent_idx, reading_count - 1);
  
  for (packet_count = 0; packet_count < total_packets; packet_count++) {
    // If we've received an ACK with all readings, we're done
    if (last_ack_count >= reading_count) {
      printf("All readings confirmed received, transfer complete\n");
      break;
    }
    
    // Calculate readings for this packet
    unsigned int remaining = reading_count - last_sent_idx;
    packet_size = (remaining < READINGS_PER_PACKET) ? remaining : READINGS_PER_PACKET;
    
    // Fill packet with readings
    data_packet.start_idx = last_sent_idx;
    data_packet.num_readings = packet_size;
    
    // Copy readings to packet
    unsigned int i;
    for (i = 0; i < packet_size; i++) {
      data_packet.light_readings[i] = light_readings[last_sent_idx + i];
      data_packet.motion_readings[i] = motion_readings[last_sent_idx + i];
    }
    
    // Reset retry count for this packet
    retry_count = 0;
    
    // Send packet (with retries if needed)
    while (retry_count < MAX_RETRIES) {
      // Send packet
      nullnet_buf = (uint8_t *)&data_packet;
      nullnet_len = sizeof(data_packet);
      
      printf("Sending packet %u/%u with %u readings starting at index %u (try %u)\n", 
             packet_count + 1, total_packets, data_packet.num_readings, 
             data_packet.start_idx, retry_count + 1);
      
      NETSTACK_NETWORK.output(&last_neighbor);
      
      // Wait for acknowledgment
      etimer_set(&ack_timer, ACK_TIMEOUT);
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE || 
                              (ev == PROCESS_EVENT_TIMER && etimer_expired(&ack_timer)));
      
      // If we got an ACK with a count that covers our sent data, proceed to next packet
      if (last_ack_count > last_sent_idx) {
        printf("Packet confirmed - last_ack_count: %u\n", last_ack_count);
        break;
      }
      
      // No ACK received or not all readings confirmed
      retry_count++;
      if (retry_count >= MAX_RETRIES) {
        printf("Maximum retries reached for packet %u, continuing anyway\n", packet_count + 1);
      } else {
        printf("Retrying packet %u (attempt %u/%u)\n", 
               packet_count + 1, retry_count + 1, MAX_RETRIES);
      }
    }
    
    // Update last sent index based on ACK
    if (last_ack_count > last_sent_idx) {
      // Update based on ACK (might have jumped ahead if NodeB received multiple packets)
      last_sent_idx = last_ack_count;
    } else {
      // No ACK or no progress, still increment our sent index
      last_sent_idx += packet_size;
    }
    
    // Small delay between packets
    etimer_set(&packet_timer, CLOCK_SECOND / 5);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
  }
  
  // Final verification - check if all data was received
  if (last_ack_count >= reading_count) {
    printf("Data transfer complete, all %u readings confirmed received\n", reading_count);
  } else {
    printf("Data transfer ended, %u/%u readings confirmed received\n", 
           last_ack_count, reading_count);
  }
  
  // Transfer complete
  data_transfer_active = 0;
  
  PROCESS_END();
}
