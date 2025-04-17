// NodeB with Handshake Protocol (Updated to support discovery-based handshake from Node A)

// ===============================
// Node B - Handshake Receiver with Logging
// ===============================

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/linkaddr.h"
#include "net/packetbuf.h"
#include "lib/random.h"
#include <string.h>
#include <stdio.h>
#include "node-id.h"

#define MAX_READINGS 60
#define WAKE_TIME RTIMER_SECOND/10
#define SLEEP_SLOT RTIMER_SECOND/10
#define SLEEP_CYCLE 9
#define NUM_SEND 2

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
  int16_t light_readings[10];
  int16_t motion_readings[10];
} data_packet_struct;

static int16_t light_buffer[MAX_READINGS];
static int16_t motion_buffer[MAX_READINGS];
static unsigned int total_received = 0;

static linkaddr_t dest_addr;
static struct rtimer rt;
static struct pt pt;
static beacon_packet_struct beacon_packet;
static uint8_t listening = 0;

PROCESS(node_b_process, "Node B - Receiver");
AUTOSTART_PROCESSES(&node_b_process);

void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
  if (len < sizeof(uint8_t)) return;
  uint8_t type = *(uint8_t *)data;

  if (type == PACKET_TYPE_READY_TO_SEND && len == sizeof(handshake_packet_t)) {
    printf("Received READY_TO_SEND from %u
", src->u8[1]);

    handshake_packet_t reply;
    reply.packet_type = PACKET_TYPE_OK_TO_RECEIVE;
    reply.src_id = node_id;
    reply.timestamp = clock_time();
    reply.seq = 0;

    nullnet_buf = (uint8_t *)&reply;
    nullnet_len = sizeof(reply);
    NETSTACK_NETWORK.output(src);

    listening = 1;
    total_received = 0;
    printf("Sent OK_TO_RECEIVE
");

  } else if (type == PACKET_TYPE_DATA && len == sizeof(data_packet_struct)) {
    data_packet_struct *packet = (data_packet_struct *)data;

    for (unsigned int i = 0; i < packet->num_readings && total_received < MAX_READINGS; i++) {
      light_buffer[total_received] = packet->light_readings[i];
      motion_buffer[total_received] = packet->motion_readings[i];
      total_received++;
    }

    printf("Received DATA from %lu (batch of %lu readings)
", packet->src_id, packet->num_readings);

  } else if (type == PACKET_TYPE_FINISH_SENDING) {
    printf("Received FINISH_SENDING
");
    if (total_received > 0) {
      printf("Light: ");
      for (unsigned int i = 0; i < total_received; i++) {
        printf("%d%s", light_buffer[i], (i < total_received - 1) ? ", " : "
");
      }

      printf("Motion: ");
      for (unsigned int i = 0; i < total_received; i++) {
        printf("%d%s", motion_buffer[i], (i < total_received - 1) ? ", " : "
");
      }
    }
    total_received = 0;
    listening = 0;
  }
}

char beacon_scheduler(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static int NumSleep = 0;
  PT_BEGIN(&pt);

  while (1) {
    if (!listening) {
      NETSTACK_RADIO.on();
      printf("Sending beacons...
");

      for (i = 0; i < NUM_SEND; i++) {
        beacon_packet.seq++;
        beacon_packet.timestamp = clock_time();
        nullnet_buf = (uint8_t *)&beacon_packet;
        nullnet_len = sizeof(beacon_packet);
        NETSTACK_NETWORK.output(&dest_addr);

        printf("Beacon sent (seq %lu)
", beacon_packet.seq);
        if (i < NUM_SEND - 1) {
          rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)beacon_scheduler, ptr);
          PT_YIELD(&pt);
        }
      }

      NETSTACK_RADIO.off();
      NumSleep = random_rand() % (2 * SLEEP_CYCLE + 1);
      printf("Sleeping %d slots
", NumSleep);

      for (i = 0; i < NumSleep; i++) {
        rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT, 1, (rtimer_callback_t)beacon_scheduler, ptr);
        PT_YIELD(&pt);
      }

    } else {
      rtimer_set(t, RTIMER_TIME(t) + CLOCK_SECOND, 1, (rtimer_callback_t)beacon_scheduler, ptr);
      PT_YIELD(&pt);
    }
  }

  PT_END(&pt);
}

PROCESS_THREAD(node_b_process, ev, data) {
  PROCESS_BEGIN();

  beacon_packet.packet_type = PACKET_TYPE_BEACON;
  beacon_packet.src_id = node_id;
  beacon_packet.seq = 0;

  linkaddr_copy(&dest_addr, &linkaddr_null);
  nullnet_set_input_callback(receive_packet_callback);

  printf("Node B started (ID: %u)
", node_id);
  rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND / 1000, 1, (rtimer_callback_t)beacon_scheduler, NULL);

  while (1) {
    PROCESS_YIELD();
  }

  PROCESS_END();
}
