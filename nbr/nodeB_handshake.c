// NodeB with Handshake Protocol (Updated to support discovery-based handshake from Node A)

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "lib/random.h"
#include "net/linkaddr.h"
#include "net/packetbuf.h"
#include <string.h>
#include <stdio.h>
#include "node-id.h"

#define WAKE_TIME RTIMER_SECOND/10
#define SLEEP_CYCLE 9
#define SLEEP_SLOT RTIMER_SECOND/10
#define NUM_SEND 2

#define PACKET_TYPE_BEACON         1
#define PACKET_TYPE_DATA           2
#define PACKET_TYPE_READY_TO_SEND  3
#define PACKET_TYPE_OK_TO_RECEIVE  4
#define PACKET_TYPE_FINISH_SENDING 5

#define MAX_READINGS 60

// Beacon packet structure
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

static linkaddr_t dest_addr;
static struct rtimer rt;
static struct pt pt;
static beacon_packet_struct beacon_packet;
static unsigned long curr_timestamp;

static int16_t received_light_readings[MAX_READINGS];
static int16_t received_motion_readings[MAX_READINGS];
static unsigned int received_readings_count = 0;
static uint8_t listening_for_data = 0;

PROCESS(nbr_discovery_process, "Node B - Handshake Receiver");
AUTOSTART_PROCESSES(&nbr_discovery_process);

void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
  if (len >= sizeof(uint8_t)) {
    uint8_t packet_type = *(uint8_t *)data;
    curr_timestamp = clock_time();
    signed short rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);

    if (packet_type == PACKET_TYPE_READY_TO_SEND) {
      printf("%lu RECEIVED READY_TO_SEND from Node %u (RSSI: %d)\n", curr_timestamp / CLOCK_SECOND, src->u8[1], rssi);
      handshake_packet_t reply;
      reply.packet_type = PACKET_TYPE_OK_TO_RECEIVE;
      reply.src_id = node_id;
      reply.timestamp = clock_time();
      reply.seq = 0;
      nullnet_buf = (uint8_t *)&reply;
      nullnet_len = sizeof(reply);
      NETSTACK_NETWORK.output(src);
      NETSTACK_RADIO.on();
      listening_for_data = 1;
      printf("Sent OK_TO_RECEIVE to Node %u\n", src->u8[1]);
    }

    else if (packet_type == PACKET_TYPE_DATA && len == sizeof(data_packet_struct)) {
			printf("Receiving data...");
      data_packet_struct *data_packet = (data_packet_struct *)data;
      unsigned int i;
      for (i = 0; i < data_packet->num_readings && data_packet->start_idx + i < MAX_READINGS; i++) {
        unsigned int idx = data_packet->start_idx + i;
        received_light_readings[idx] = data_packet->light_readings[i];
        received_motion_readings[idx] = data_packet->motion_readings[i];
        if (idx + 1 > received_readings_count) {
          received_readings_count = idx + 1;
        }
      }

      if (received_readings_count == MAX_READINGS) {
        printf("Light: ");
        for (i = 0; i < received_readings_count; i++) {
          printf("%d%s", received_light_readings[i], (i < received_readings_count - 1) ? ", " : "\n");
        }
        printf("Motion: ");
        for (i = 0; i < received_readings_count; i++) {
          printf("%d%s", received_motion_readings[i], (i < received_readings_count - 1) ? ", " : "\n");
        }
        received_readings_count = 0;
      }
    }

    else if (packet_type == PACKET_TYPE_FINISH_SENDING) {
      printf("%lu RECEIVED FINISH_SENDING\n", curr_timestamp / CLOCK_SECOND);
      listening_for_data = 0;
      NETSTACK_RADIO.off();
    }
  }
}

char sender_scheduler(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static int NumSleep = 0;
  PT_BEGIN(&pt);

  while(1) {
    if (!listening_for_data) {
      printf("Radio ON: sending beacons...\n");
      NETSTACK_RADIO.on();

      for(i = 0; i < NUM_SEND; i++) {
        nullnet_buf = (uint8_t *)&beacon_packet;
        nullnet_len = sizeof(beacon_packet);
        beacon_packet.seq++;
        beacon_packet.timestamp = clock_time();
        NETSTACK_NETWORK.output(&dest_addr);
        printf("Node B sent beacon seq %lu\n", beacon_packet.seq);

        if(i != NUM_SEND - 1) {
          rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
          PT_YIELD(&pt);
        }
      }

			if (!listening_for_data) {
				rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
				PT_YIELD(&pt);

				printf("Radio OFF\n");
				NETSTACK_RADIO.off();

				NumSleep = random_rand() % (2 * SLEEP_CYCLE + 1);
				printf("Sleep for %d slots\n", NumSleep);
				for(i = 0; i < NumSleep; i++) {
						rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT, 1, (rtimer_callback_t)sender_scheduler, ptr);
						PT_YIELD(&pt);
				}
			}
    } else {
      rtimer_set(t, RTIMER_TIME(t) + CLOCK_SECOND, 1, (rtimer_callback_t)sender_scheduler, ptr);
      PT_YIELD(&pt);
    }
  }
  PT_END(&pt);
}

PROCESS_THREAD(nbr_discovery_process, ev, data) {
  PROCESS_BEGIN();

  beacon_packet.packet_type = PACKET_TYPE_BEACON;
  beacon_packet.src_id = node_id;
  beacon_packet.seq = 0;
  linkaddr_copy(&dest_addr, &linkaddr_null);

  nullnet_set_input_callback(receive_packet_callback);

  printf("Node B started. NodeID: %u\n", node_id);

  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);

  while(1) {
    PROCESS_YIELD();
  }

  PROCESS_END();
}
