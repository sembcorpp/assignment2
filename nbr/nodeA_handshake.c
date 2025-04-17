// NodeA with Beacon-Based Discovery and Handshake
// ===============================
// Node A - Circular Buffer + Handshake
// ===============================

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include "net/linkaddr.h"
#include "lib/random.h"
#include "board-peripherals.h"
#include <string.h>
#include <stdio.h> 
#include "node-id.h"

#define RSSI_THRESHOLD -70
#define DATA_COLLECTION_RATE CLOCK_SECOND
#define MAX_READINGS 60
#define READINGS_PER_PACKET 10
#define MAX_RETRIES 10
#define HANDSHAKE_WAIT CLOCK_SECOND / 5

#define PACKET_TYPE_BEACON         1
#define PACKET_TYPE_DATA           2
#define PACKET_TYPE_READY_TO_SEND  3
#define PACKET_TYPE_OK_TO_RECEIVE  4
#define PACKET_TYPE_FINISH_SENDING 5

typedef struct {
  uint8_t packet_type;
  unsigned long src_id;
  unsigned long timestamp;
  unsigned long seq;
} handshake_packet_t;

typedef struct {
  uint8_t packet_type;
  unsigned long src_id;
  unsigned long start_idx;
  unsigned long num_readings;
  int16_t light_readings[READINGS_PER_PACKET];
  int16_t motion_readings[READINGS_PER_PACKET];
} data_packet_struct;

static int16_t light_readings[MAX_READINGS];
static int16_t motion_readings[MAX_READINGS];
static unsigned int write_index = 0;
static uint8_t buffer_full = 0;
static struct etimer sensing_timer;

static linkaddr_t last_neighbor;
static uint8_t ok_to_send_received = 0;
static uint8_t data_transfer_active = 0;

PROCESS(sensing_process, "Sensing Process");
PROCESS(data_transfer_process, "Data Transfer Process");
AUTOSTART_PROCESSES(&sensing_process);

static void init_sensors(void) {
  SENSORS_ACTIVATE(opt_3001_sensor);
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

static int16_t read_light_sensor(void) {
  int value = opt_3001_sensor.value(0);
  if (value == CC26XX_SENSOR_READING_ERROR) return -1;
  return (int16_t)(value / 100);
}

static int16_t read_motion_sensor(void) {
  static int prev_ax = 0, prev_ay = 0, prev_az = 0;
  static uint8_t first = 1;
  int ax = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  int ay = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  int az = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  if (ax == CC26XX_SENSOR_READING_ERROR || ay == CC26XX_SENSOR_READING_ERROR || az == CC26XX_SENSOR_READING_ERROR) return -1;
  int motion = 0;
  if (!first) {
    int dx = ax - prev_ax, dy = ay - prev_ay, dz = az - prev_az;
    motion = (dx*dx + dy*dy + dz*dz) / 100;
  } else first = 0;
  prev_ax = ax; prev_ay = ay; prev_az = az;
  return (int16_t)motion;
}

void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
  if (len >= sizeof(uint8_t)) {
    uint8_t packet_type = *(uint8_t *)data;
    if (packet_type == PACKET_TYPE_BEACON && len == sizeof(handshake_packet_t)) {
      signed short rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
      if (rssi >= RSSI_THRESHOLD && !data_transfer_active && (buffer_full || write_index > 0)) {
        printf("Node B detected. RSSI %d. Beginning handshake...
", rssi);
        linkaddr_copy(&last_neighbor, src);
        ok_to_send_received = 0;
        data_transfer_active = 1;
        process_start(&data_transfer_process, NULL);
      }
    } else if (packet_type == PACKET_TYPE_OK_TO_RECEIVE) {
      ok_to_send_received = 1;
    }
  }
}

PROCESS_THREAD(sensing_process, ev, data) {
  PROCESS_BEGIN();
  init_sensors();
  nullnet_set_input_callback(receive_packet_callback);
  NETSTACK_RADIO.on();

  while (1) {
    etimer_set(&sensing_timer, DATA_COLLECTION_RATE);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sensing_timer));
    int16_t light = read_light_sensor();
    int16_t motion = read_motion_sensor();
    light_readings[write_index] = light;
    motion_readings[write_index] = motion;
    write_index = (write_index + 1) % MAX_READINGS;
    if (write_index == 0) buffer_full = 1;
  }
  PROCESS_END();
}

PROCESS_THREAD(data_transfer_process, ev, data) {
  PROCESS_BEGIN();

  static handshake_packet_t handshake_packet;
  static data_packet_struct data_packet;
  static struct etimer timer;
  unsigned int retries = 0;
  ok_to_send_received = 0;

  handshake_packet.packet_type = PACKET_TYPE_READY_TO_SEND;
  handshake_packet.src_id = node_id;
  handshake_packet.timestamp = clock_time();
  handshake_packet.seq = 0;

  while (!ok_to_send_received && retries < MAX_RETRIES) {
    nullnet_buf = (uint8_t *)&handshake_packet;
    nullnet_len = sizeof(handshake_packet);
    NETSTACK_NETWORK.output(&last_neighbor);
    etimer_set(&timer, HANDSHAKE_WAIT);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    retries++;
  }

  if (!ok_to_send_received) {
    printf("Handshake failed.
");
    data_transfer_active = 0;
    PROCESS_EXIT();
  }

  unsigned int count = buffer_full ? MAX_READINGS : write_index;
  unsigned int start = buffer_full ? write_index : 0;
  unsigned int packets = (count + READINGS_PER_PACKET - 1) / READINGS_PER_PACKET;

  for (unsigned int p = 0; p < packets; p++) {
    data_packet.packet_type = PACKET_TYPE_DATA;
    data_packet.src_id = node_id;
    data_packet.start_idx = p * READINGS_PER_PACKET;
    data_packet.num_readings = (count - p * READINGS_PER_PACKET < READINGS_PER_PACKET)
                               ? count - p * READINGS_PER_PACKET
                               : READINGS_PER_PACKET;

    for (unsigned int i = 0; i < data_packet.num_readings; i++) {
      int idx = (start + p * READINGS_PER_PACKET + i) % MAX_READINGS;
      data_packet.light_readings[i] = light_readings[idx];
      data_packet.motion_readings[i] = motion_readings[idx];
    }

    nullnet_buf = (uint8_t *)&data_packet;
    nullnet_len = sizeof(data_packet);
    NETSTACK_NETWORK.output(&last_neighbor);
    etimer_set(&timer, CLOCK_SECOND / 5);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
  }

  handshake_packet.packet_type = PACKET_TYPE_FINISH_SENDING;
  nullnet_buf = (uint8_t *)&handshake_packet;
  nullnet_len = sizeof(handshake_packet);
  NETSTACK_NETWORK.output(&last_neighbor);
  printf("Finished sending data.
");

  write_index = 0;
  buffer_full = 0;
  data_transfer_active = 0;

  PROCESS_END();
}
