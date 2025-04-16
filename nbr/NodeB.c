// ===============================
// NodeB.c - Broadcaster
// ===============================

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "lib/random.h"
#include "net/linkaddr.h"
#include <string.h>
#include <stdio.h>
#include "node-id.h"

#define WAKE_TIME RTIMER_SECOND/10  // Active 0.1s
#define SLEEP_CYCLE 9               // Random 0–18 sleep slots
#define SLEEP_SLOT RTIMER_SECOND/10
#define NUM_SEND 2                 // Send 2 packets per wake

typedef struct {
  unsigned long src_id;
  unsigned long timestamp;
  unsigned long seq;
} data_packet_struct;

linkaddr_t dest_addr;               // Broadcast address
static struct rtimer rt;
static struct pt pt;
static data_packet_struct data_packet;
unsigned long curr_timestamp;

PROCESS(nbr_discovery_process, "Node B - Broadcaster");
AUTOSTART_PROCESSES(&nbr_discovery_process);

// Main sending loop with randomized sleep schedule
char sender_scheduler(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static int NumSleep = 0;
  PT_BEGIN(&pt);

  while(1) {
		printf("Radio ON: sending beacons...\n");
    NETSTACK_RADIO.on();

		// Send NUM_SEND number of neighbour discovery beacon packets
    for(i = 0; i < NUM_SEND; i++) {
			// Initialize the nullnet module with information of packet to be transmitted
      nullnet_buf = (uint8_t *)&data_packet; // Data transmitted
      nullnet_len = sizeof(data_packet); // Length of data transmitted
      data_packet.seq++;
      data_packet.timestamp = clock_time();

			// Packet transmission
			printf("Node B sent beacon seq %lu at time %lu\n", data_packet.seq, clock_time() / CLOCK_SECOND);
      NETSTACK_NETWORK.output(&dest_addr);

			// wait for WAKE_TIME before sending the next packet
      if(i != NUM_SEND - 1) {
        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }

		// sleep for a random number of slots
		if(SLEEP_CYCLE != 0){
			printf("Radio OFF\n");
			NETSTACK_RADIO.off();

			// get a value that is uniformly distributed between 0 and 2*SLEEP_CYCLE
			// the average is SLEEP_CYCLE 
			NumSleep = random_rand() % (2 * SLEEP_CYCLE + 1);
			printf("Sleep for %d slots\n", NumSleep);

			for(i = 0; i < NumSleep; i++) {
				rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT, 1, (rtimer_callback_t)sender_scheduler, ptr);
				PT_YIELD(&pt);
			}
		}
	}

  PT_END(&pt);
}

PROCESS_THREAD(nbr_discovery_process, ev, data) {
  PROCESS_BEGIN();

  data_packet.src_id = node_id;
  data_packet.seq = 0;
  linkaddr_copy(&dest_addr, &linkaddr_null);
  nullnet_set_input_callback(NULL); // This node does not receive

  printf("Node B starting beaconing.\n");
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);

  PROCESS_END();
}
