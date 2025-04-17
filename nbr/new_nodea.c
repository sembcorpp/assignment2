#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include "net/linkaddr.h"
#include "board-peripherals.h"
#include "node-id.h"

#define RSSI_THRESHOLD -70
#define MAX_READINGS 60

// Define packet types
#define PACKET_TYPE_BEACON 1
#define PACKET_TYPE_LIGHT_DATA 2
#define PACKET_TYPE_MOTION_DATA 3

typedef struct {
  uint8_t type;         // Packet type identifier
  uint16_t src_id;      // Source node ID
  uint32_t timestamp;   // Timestamp
  uint32_t seq;         // Sequence number
  // For beacons, remaining fields not used
  // For data packets, payload contains the readings
  int readings[60];     // For data packets, contains readings
} data_packet_t;

// Buffers for 60 samples
static int light_readings[MAX_READINGS];
static int motion_readings[MAX_READINGS];
static int write_index = 0;
static int total_samples = 0;
static int is_ready_to_transfer = 0;

static uint32_t curr_timestamp;
static linkaddr_t last_beacon_sender;

/*------------------------------------------------------------*/
void transfer_sensor_data(uint16_t node_id, int rssi) {
  if (!is_ready_to_transfer) return;

  printf("%lu DETECT %d\n", clock_seconds(), node_id);
  printf("%lu TRANSFER %d RSSI: %d\n", clock_seconds(), node_id, rssi);

  data_packet_t data_packet;
  
  // Send Light readings
  data_packet.type = PACKET_TYPE_LIGHT_DATA;
  data_packet.src_id = node_id;
  data_packet.timestamp = clock_seconds();
  data_packet.seq = 0;
  
  // Copy light readings to packet
  for (int i = 0; i < MAX_READINGS; i++) {
    int idx = (write_index + i) % MAX_READINGS;
    data_packet.readings[i] = light_readings[idx];
  }
  
  // Send packet
  nullnet_buf = (uint8_t *)&data_packet;
  nullnet_len = sizeof(data_packet_t);
  NETSTACK_NETWORK.output(&last_beacon_sender);
  
  // Send Motion readings
  data_packet.type = PACKET_TYPE_MOTION_DATA;
  data_packet.seq = 1;
  
  // Copy motion readings to packet
  for (int i = 0; i < MAX_READINGS; i++) {
    int idx = (write_index + i) % MAX_READINGS;
    data_packet.readings[i] = motion_readings[idx];
  }
  
  // Send packet
  nullnet_buf = (uint8_t *)&data_packet;
  nullnet_len = sizeof(data_packet_t);
  NETSTACK_NETWORK.output(&last_beacon_sender);

  // Reset for next round
  write_index = 0;
  total_samples = 0;
  is_ready_to_transfer = 0;

  printf("Transfer complete. Starting new collection cycle.\n");
}

/*------------------------------------------------------------*/
// NULLNET receive callback
void receive_packet_callback(const void *data, uint16_t len, 
                             const linkaddr_t *src, const linkaddr_t *dest) 
{
  if(len == sizeof(data_packet_t)) {
    data_packet_t *received_packet = (data_packet_t *)data;
    
    // Check if it's a beacon
    if(received_packet->type == PACKET_TYPE_BEACON) {
      curr_timestamp = clock_time();
      int16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
      
      // Store the sender's address for response
      linkaddr_copy(&last_beacon_sender, src);

      // Print DETECT
      printf("%lu DETECT %d\n", curr_timestamp / CLOCK_SECOND, received_packet->src_id);

      // Check RSSI threshold and send data if ready
      if (rssi >= RSSI_THRESHOLD) {
        transfer_sensor_data(received_packet->src_id, rssi);
      } else {
        printf("Link quality not good enough (RSSI = %d)\n", rssi);
      }
    }
  }
}

/*------------------------------------------------------------*/
PROCESS(node_a_process, "Node A Process");
AUTOSTART_PROCESSES(&node_a_process);

PROCESS_THREAD(node_a_process, ev, data)
{
  static struct etimer sampling_timer;
  int light_val, acc_x_val;

  PROCESS_BEGIN();

  printf("Node A started.\n");

  // Activate sensors
  SENSORS_ACTIVATE(opt_3001_sensor);
  SENSORS_ACTIVATE(mpu_9250_sensor);

  // Set nullnet callback
  nullnet_set_input_callback(receive_packet_callback);

  // Start 1Hz sampling
  while (1) {
    etimer_set(&sampling_timer, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sampling_timer));

    light_val = opt_3001_sensor.value(0);
    acc_x_val = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);

    light_readings[write_index] = light_val;
    motion_readings[write_index] = acc_x_val;

    printf("Sample #%d @ %lu - Light: %d, Motion: %d\n",
           total_samples + 1, clock_seconds(), light_val, acc_x_val);

    write_index = (write_index + 1) % MAX_READINGS;
    total_samples++;

    if (total_samples >= MAX_READINGS) {
      is_ready_to_transfer = 1;
    }
  }

  PROCESS_END();
}
