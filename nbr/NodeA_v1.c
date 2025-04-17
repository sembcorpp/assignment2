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
  printf("Received packet: len=%u\n", len);  // Debug: Show packet length
  
  // Check if the received packet is a beacon
  if (len >= sizeof(uint8_t)) {
    uint8_t packet_type = *(uint8_t *)data;
    printf("Packet type: %u\n", packet_type);  // Debug: Show packet type
    
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
    } else {
      printf("Unknown or unsupported packet type: %u, len: %u\n", packet_type, len);
    }
  } else {
    printf("Packet too small to process\n");
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
      
      // If buffer is now full, show message
      if (reading_count == MAX_READINGS) {
        printf("All 60 readings collected, ready for transfer\n");
      }
    } else {
      printf("Buffer full, waiting for transfer to Node B\n");
    }
  }
  
  PROCESS_END();
}

// Data transfer process
PROCESS_THREAD(data_transfer_process, ev, data) {
  PROCESS_BEGIN();
  
  static struct etimer packet_timer;
  static data_packet_struct data_packet;
  
  // Initialize the data packet
  data_packet.packet_type = PACKET_TYPE_DATA;
  data_packet.src_id = node_id;
  
  printf("Starting data transfer process\n");
  
  // Reset the last sent index
  last_sent_idx = 0;
  
  // While we have more readings to send
  while (last_sent_idx < reading_count) {
    // Calculate how many readings to send in this packet
    unsigned int readings_to_send = reading_count - last_sent_idx;
    if (readings_to_send > READINGS_PER_PACKET) {
      readings_to_send = READINGS_PER_PACKET;
    }
    
    // Fill the data packet
    data_packet.start_idx = last_sent_idx;
    data_packet.num_readings = readings_to_send;
    
    unsigned int i;
    for (i = 0; i < readings_to_send; i++) {
      data_packet.light_readings[i] = light_readings[last_sent_idx + i];
      data_packet.motion_readings[i] = motion_readings[last_sent_idx + i];
    }
    
    // Set the nullnet buffer
    nullnet_buf = (uint8_t *)&data_packet;
    nullnet_len = sizeof(data_packet);
    
    printf("Sending data packet %u with %u readings (packet size: %u bytes)\n", 
           last_sent_idx / READINGS_PER_PACKET, 
           readings_to_send,
           sizeof(data_packet));
    
    // Send the packet
    int status = NETSTACK_NETWORK.output(&last_neighbor);
    if (status != 0) {
      printf("ERROR: Failed to send data packet, error code: %d\n", status);
    }
    
    // Set a timeout for acknowledgment (1 second)
    etimer_set(&ack_timer, CLOCK_SECOND);
    
    // Wait for acknowledgment or timeout
    PROCESS_WAIT_EVENT();
    
    if (ev == PROCESS_EVENT_TIMER && data == &ack_timer) {
      // Timeout, retry send
      printf("Acknowledgment timeout, retrying...\n");
      continue; // Retry sending the same packet
    }
    else if (ev == PROCESS_EVENT_CONTINUE) {
      // Received acknowledgment, check how many were acknowledged
      if (last_ack_count > last_sent_idx) {
        // Update last sent index with acknowledged count
        last_sent_idx = last_ack_count;
        printf("Successfully sent readings up to index %u\n", last_sent_idx);
        
        // Wait a bit before sending the next packet (to avoid flooding)
        etimer_set(&packet_timer, CLOCK_SECOND / 4);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &packet_timer);
      } else {
        printf("No progress in acknowledgment, still at index %u\n", last_sent_idx);
        
        // Wait a bit longer before retrying
        etimer_set(&packet_timer, CLOCK_SECOND / 2);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &packet_timer);
      }
    }
  }
  
  printf("Data transfer complete - all %u readings sent\n", reading_count);
  data_transfer_active = 0;
  
  PROCESS_END();
}
