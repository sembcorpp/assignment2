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

// Link quality threshold
#define RSSI_THRESHOLD -70

// Data collection parameters
#define DATA_COLLECTION_RATE CLOCK_SECOND  // 1Hz sampling rate
#define MAX_READINGS 60              // Store 60 readings

// Packet types
#define PACKET_TYPE_BEACON 1
#define PACKET_TYPE_DATA 2

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
  int16_t light_readings[10];  // Send 10 readings at a time
  int16_t motion_readings[10]; // Send 10 readings at a time
} data_packet_struct;

// Variables for data collection
static int16_t light_readings[MAX_READINGS];
static int16_t motion_readings[MAX_READINGS];
static unsigned int reading_count = 0;
static unsigned int last_sent_idx = 0;
static uint8_t is_collecting = 0;
static struct etimer sensing_timer;

// Variables for node discovery
static linkaddr_t last_neighbor;
static int8_t last_rssi = -128;
static uint8_t neighbor_found = 0;
static uint8_t data_transfer_active = 0;
static unsigned long curr_timestamp;

// Processes
PROCESS(sensing_process, "Node A - Sensing Process");
PROCESS(data_transfer_process, "Node A - Data Transfer Process");
AUTOSTART_PROCESSES(&sensing_process);

// Initialize sensors
static void init_sensors(void) {
  SENSORS_ACTIVATE(opt_3001_sensor);
  SENSORS_ACTIVATE(mpu_9250_sensor);
}

// Read light sensor
static int16_t read_light_sensor(void) {
  int value = opt_3001_sensor.value(0);
  if (value == CC26XX_SENSOR_READING_ERROR) {
    printf("Error reading light sensor\n");
    return -1;
  }
  return (int16_t)value;
}

// Read motion sensor (using accelerometer magnitude as proxy)
static int16_t read_motion_sensor(void) {
  int ax = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  int ay = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  int az = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  
  // Calculate magnitude (approximation)
  return (int16_t)((ax*ax + ay*ay + az*az) / 100);
}

// Function called after reception of a packet
void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
  // Check if the received packet is a beacon
  if (len >= sizeof(uint8_t)) {
    uint8_t packet_type = *(uint8_t *)data;
    
    if (packet_type == PACKET_TYPE_BEACON && len == sizeof(beacon_packet_struct)) {
      beacon_packet_struct *received_packet = (beacon_packet_struct *)data;
      
      // Get current timestamp
      curr_timestamp = clock_time();
      
      // Get RSSI (signal strength of the received packet)
      signed short rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
      
      // Log detection
      printf("%lu DETECT %lu\n", curr_timestamp / CLOCK_SECOND, received_packet->src_id);
      
      // Store neighbor information
      linkaddr_copy(&last_neighbor, src);
      last_rssi = rssi;
      neighbor_found = 1;
      
      // Check link quality
      if (rssi >= RSSI_THRESHOLD && reading_count > 0 && !data_transfer_active) {
        printf("%lu TRANSFER %lu RSSI: %d\n", curr_timestamp / CLOCK_SECOND, received_packet->src_id, rssi);
        
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
  printf("Listening for beacons from Node B...\n");
  
  // Turn radio on to listen for beacons
  NETSTACK_RADIO.on();
  
  // Continuously collect sensor data
  while (1) {
    // Set timer for periodic sensing
    etimer_set(&sensing_timer, DATA_COLLECTION_RATE);
    
    // Wait for timer
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
    
    // If we have space in our buffer or all data has been sent
    if (reading_count < MAX_READINGS) {
      // Read sensors
      light_readings[reading_count] = read_light_sensor();
      motion_readings[reading_count] = read_motion_sensor();
      
      printf("Reading %u: Light: %d, Motion: %d\n", reading_count, light_readings[reading_count], motion_readings[reading_count]);
      
      reading_count++;
    } else {
      // Buffer is full
      printf("Buffer full, implementing circular buffer approach\n");
      
      // Check if we've sent all data
      if (last_sent_idx >= MAX_READINGS) {
        // All data sent, reset counters
        reading_count = 0;
        last_sent_idx = 0;
        
        // Take new reading at position 0
        light_readings[reading_count] = read_light_sensor();
        motion_readings[reading_count] = read_motion_sensor();
        
        printf("Reading %u: Light: %d, Motion: %d\n", reading_count, light_readings[reading_count], motion_readings[reading_count]);
        
        reading_count++;
      } else {
        // We haven't sent all data, but buffer is full
        // Implement circular buffer - shift readings to make room for new ones
        unsigned int i;
        for (i = 0; i < MAX_READINGS - 1; i++) {
          light_readings[i] = light_readings[i + 1];
          motion_readings[i] = motion_readings[i + 1];
        }
        
        // Add new reading at the end
        light_readings[MAX_READINGS - 1] = read_light_sensor();
        motion_readings[MAX_READINGS - 1] = read_motion_sensor();
        
        printf("New Reading (shifted buffer): Light: %d, Motion: %d\n", 
               light_readings[MAX_READINGS - 1], motion_readings[MAX_READINGS - 1]);
        
        // If we've already sent some readings, adjust last_sent_idx
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
  static unsigned int readings_per_packet = 10;
  static unsigned int total_packets;
  static struct etimer packet_timer;
  
  // Initialize data packet
  data_packet.packet_type = PACKET_TYPE_DATA;
  data_packet.src_id = node_id;
  
  // Calculate how many packets needed
  total_packets = (reading_count - last_sent_idx + readings_per_packet - 1) / readings_per_packet;
  
  printf("Starting data transfer: %u readings to send (from index %u to %u)\n", 
         reading_count - last_sent_idx, last_sent_idx, reading_count - 1);
  
  for (packet_count = 0; packet_count < total_packets; packet_count++) {
    // Calculate actual number of readings in this packet
    unsigned int remaining = reading_count - last_sent_idx;
    unsigned int packet_size = (remaining < readings_per_packet) ? remaining : readings_per_packet;
    
    // Fill data packet with readings
    data_packet.start_idx = last_sent_idx;
    data_packet.num_readings = packet_size;
    
    // Copy readings to packet
    unsigned int i;
    for (i = 0; i < packet_size; i++) {
      if (last_sent_idx + i < reading_count) {
        data_packet.light_readings[i] = light_readings[last_sent_idx + i];
        data_packet.motion_readings[i] = motion_readings[last_sent_idx + i];
      }
    }
    
    // Send packet
    nullnet_buf = (uint8_t *)&data_packet;
    nullnet_len = sizeof(data_packet);
    
    printf("Sending data packet %u/%u with %lu readings starting at index %lu\n", 
           packet_count + 1, total_packets, data_packet.num_readings, data_packet.start_idx);
    
    NETSTACK_NETWORK.output(&last_neighbor);
    
    // Update last sent index
    last_sent_idx += packet_size;
    
    // Small delay between packets
    etimer_set(&packet_timer, CLOCK_SECOND / 5);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
  }
  
  printf("Data transfer complete, sent %u readings (last_sent_idx = %u)\n", 
         reading_count - (reading_count - last_sent_idx), last_sent_idx);
  data_transfer_active = 0;
  
  PROCESS_END();
}
