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

#define PACKET_TYPE_BEACON 1
#define PACKET_TYPE_LIGHT_DATA 2
#define PACKET_TYPE_MOTION_DATA 3

typedef struct {
  uint8_t type;
  uint16_t src_id;
  uint32_t timestamp;
  uint32_t seq;
  int readings[60];
} data_packet_t;

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

  printf("[DEBUG] Sending Light Readings:\n");
  for (int i = 0; i < MAX_READINGS; i++) {
    int idx = (write_index + i) % MAX_READINGS;
    data_packet.readings[i] = light_readings[idx];
    printf("%d", data_packet.readings[i]);
    if (i < MAX_READINGS - 1) printf(", ");
  }
  printf("\n");

  nullnet_buf = (uint8_t *)&data_packet;
  nullnet_len = sizeof(data_packet_t);
  NETSTACK_NETWORK.output(&last_beacon_sender);

  // Send Motion readings
  data_packet.type = PACKET_TYPE_MOTION_DATA;
  data_packet.seq = 1;

  printf("[DEBUG] Sending Motion Readings:\n");
  for (int i = 0; i < MAX_READINGS; i++) {
    int idx = (write_index + i) % MAX_READINGS;
    data_packet.readings[i] = motion_readings[idx];
    printf("%d", data_packet.readings[i]);
    if (i < MAX_READINGS - 1) printf(", ");
  }
  printf("\n");

  nullnet_buf = (uint8_t *)&data_packet;
  nullnet_len = sizeof(data_packet_t);
  NETSTACK_NETWORK.output(&last_beacon_sender);

  write_index = 0;
  total_samples = 0;
  is_ready_to_transfer = 0;

  printf("Transfer complete. Starting new collection cycle.\n");
}

/*------------------------------------------------------------*/
void receive_packet_callback(const void *data, uint16_t len, 
                             const linkaddr_t *src, const linkaddr_t *dest) 
{
  if(len == sizeof(data_packet_t)) {
    data_packet_t *received_packet = (data_packet_t *)data;

    if(received_packet->type == PACKET_TYPE_BEACON) {
      curr_timestamp = clock_time();
      int16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
      linkaddr_copy(&last_beacon_sender, src);

      printf("%lu DETECT %d\n", curr_timestamp / CLOCK_SECOND, received_packet->src_id);

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

  printf("Node A started. Initializing sensors...\n");

  // Activate sensors
  SENSORS_ACTIVATE(opt_3001_sensor);
  SENSORS_ACTIVATE(mpu_9250_sensor);

  // Sensor warm-up delay
  etimer_set(&sampling_timer, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sampling_timer));
  printf("Sensors warmed up. Starting sampling...\n");

  nullnet_set_input_callback(receive_packet_callback);

  while (1) {
    etimer_set(&sampling_timer, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sampling_timer));

    light_val = opt_3001_sensor.value(0);
    acc_x_val = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);

    printf("[DEBUG] Sample #%d @ %lu - Light: %d, Motion: %d\n",
           total_samples + 1, clock_seconds(), light_val, acc_x_val);

    light_readings[write_index] = light_val;
    motion_readings[write_index] = acc_x_val;

    write_index = (write_index + 1) % MAX_READINGS;
    total_samples++;

    if (total_samples >= MAX_READINGS) {
      is_ready_to_transfer = 1;
    }
  }

  PROCESS_END();
}
