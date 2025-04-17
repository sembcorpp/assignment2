// NodeA with Beacon-Based Discovery and Handshake

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
static unsigned int reading_count = 0;
static unsigned int last_sent_idx = 0;
static struct etimer sensing_timer;

static linkaddr_t last_neighbor;
static int8_t last_rssi = -128;
static uint8_t ok_to_send_received = 0;
static uint8_t data_transfer_active = 0;

PROCESS(sensing_process, "Sensing Process");
PROCESS(data_transfer_process, "Data Transfer with Handshake");
AUTOSTART_PROCESSES(&sensing_process);

static void init_sensors(void) {
  SENSORS_ACTIVATE(opt_3001_sensor);
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

static int16_t read_light_sensor(void) {
  int value = opt_3001_sensor.value(0) / 100;
  if (value == CC26XX_SENSOR_READING_ERROR) return -1;
  return (int16_t)value;
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
      if (rssi >= RSSI_THRESHOLD && reading_count > 0 && !data_transfer_active) {
        printf("Discovered Node B with good RSSI %d. Starting handshake.\n", rssi);
        linkaddr_copy(&last_neighbor, src);
        last_rssi = rssi;
        process_start(&data_transfer_process, NULL);
        data_transfer_active = 1;
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
    if (reading_count < MAX_READINGS) {
      light_readings[reading_count] = light;
      motion_readings[reading_count] = motion;
      reading_count++;
    }
  }
  PROCESS_END();
}

PROCESS_THREAD(data_transfer_process, ev, data) {
  PROCESS_BEGIN();

  static handshake_packet_t handshake_packet;
  static data_packet_struct data_packet;
  static unsigned int packet_size, packet_count, total_packets;
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
    printf("Sent READY_TO_SEND\n");
    etimer_set(&timer, HANDSHAKE_WAIT);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    retries++;
  }

  if (!ok_to_send_received) {
    printf("Handshake failed. Aborting transfer.\n");
    data_transfer_active = 0;
    PROCESS_EXIT();
  }

  if (reading_count > last_sent_idx) {
    total_packets = (reading_count - last_sent_idx + READINGS_PER_PACKET - 1) / READINGS_PER_PACKET;
  } else {
    printf("No data to send.\n");
    data_transfer_active = 0;
    PROCESS_EXIT();
  }

  for (packet_count = 0; packet_count < total_packets; packet_count++) {
    unsigned int remaining = reading_count - last_sent_idx;
    packet_size = (remaining < READINGS_PER_PACKET) ? remaining : READINGS_PER_PACKET;
    data_packet.packet_type = PACKET_TYPE_DATA;
    data_packet.src_id = node_id;
    data_packet.start_idx = last_sent_idx;
    data_packet.num_readings = packet_size;

    for (unsigned int i = 0; i < packet_size; i++) {
      data_packet.light_readings[i] = light_readings[last_sent_idx + i];
      data_packet.motion_readings[i] = motion_readings[last_sent_idx + i];
    }

    nullnet_buf = (uint8_t *)&data_packet;
    nullnet_len = sizeof(data_packet);
    NETSTACK_NETWORK.output(&last_neighbor);
    etimer_set(&timer, CLOCK_SECOND / 5);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    last_sent_idx += packet_size;
  }

  handshake_packet.packet_type = PACKET_TYPE_FINISH_SENDING;
  nullnet_buf = (uint8_t *)&handshake_packet;
  nullnet_len = sizeof(handshake_packet);
  NETSTACK_NETWORK.output(&last_neighbor);
  printf("Sent FINISH_SENDING\n");
  data_transfer_active = 0;

  PROCESS_END();
}
