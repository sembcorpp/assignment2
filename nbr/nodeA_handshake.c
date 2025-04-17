// NodeA with Beacon-Based Handshake and Data Transfer

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

PROCESS(sensing_process, "Sensing Process");
PROCESS(data_transfer_process, "Data Transfer with Handshake");
AUTOSTART_PROCESSES(&sensing_process);

typedef struct {
  uint8_t packet_type;
  unsigned long src_id;
  unsigned long timestamp;
  unsigned long seq;
} beacon_packet_struct;

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

// Variables for motion detection
static int prev_ax = 0;
static int prev_ay = 0;
static int prev_az = 0;
static uint8_t first_reading = 1;

static linkaddr_t last_neighbor;
static uint8_t ok_to_send_received = 0;
static uint8_t ready_to_start_transfer = 0;

static void init_sensors(void) {
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


void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
  if (len >= sizeof(uint8_t)) {
    uint8_t packet_type = *(uint8_t *)data;
    signed short rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);

    if (packet_type == PACKET_TYPE_BEACON && len == sizeof(beacon_packet_struct)) {
      beacon_packet_struct *beacon = (beacon_packet_struct *)data;
      printf("%lu DETECT %lu\n", clock_time() / CLOCK_SECOND, beacon->src_id);

      if (rssi >= RSSI_THRESHOLD && reading_count == 60) {
        linkaddr_copy(&last_neighbor, src);
        ready_to_start_transfer = 1;
        process_start(&data_transfer_process, NULL);
      }
    }
    else if (packet_type == PACKET_TYPE_OK_TO_RECEIVE) {
      printf("OK_TO_RECEIVE received from %u\n", src->u8[1]);
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
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
    int16_t light = read_light_sensor();
    int16_t motion = read_motion_sensor();
    if (reading_count < MAX_READINGS) {
      light_readings[reading_count] = light;
      motion_readings[reading_count] = motion;
			printf("Reading %u: Light: %d lux, Motion: %d\n", reading_count, light, motion);
      reading_count++;
    }
  }
  PROCESS_END();
}

PROCESS_THREAD(data_transfer_process, ev, data) {
  PROCESS_BEGIN();

  static handshake_packet_t handshake_packet;
  static data_packet_struct data_packet;
  static unsigned int packet_size;
  static unsigned int packet_count;
  static unsigned int total_packets;
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
    printf("Sent READY_TO_SEND to Node B\n");
    etimer_set(&timer, HANDSHAKE_WAIT);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    retries++;
  }

  if (!ok_to_send_received) {
    printf("Handshake failed. Aborting transfer.\n");
    PROCESS_EXIT();
  }

	printf("reading_count = %u, last_sent_idx = %u\n", reading_count, last_sent_idx);

  if (reading_count > last_sent_idx) {
    total_packets = (reading_count - last_sent_idx + READINGS_PER_PACKET - 1) / READINGS_PER_PACKET;
  }

	printf("total_packets = %u\n", total_packets);

  if (total_packets == 0) {
    printf("No data to send.\n");
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

  PROCESS_END();
}
