#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "lib/random.h"
#include "net/linkaddr.h"
#include "net/packetbuf.h"
#include "node-id.h"

// === Settings ===
#define RSSI_THRESHOLD -70
#define WAKE_TIME RTIMER_SECOND/10
#define SLEEP_CYCLE 9
#define SLEEP_SLOT RTIMER_SECOND/10
#define NUM_SEND 2

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

static data_packet_t beacon_packet;
static int light_readings[MAX_READINGS];
static int motion_readings[MAX_READINGS];
static int light_received = 0;
static int motion_received = 0;

static struct rtimer rt;
static struct pt pt;
static linkaddr_t dest_addr;

// === NULLNET Receiver Callback ===
void nullnet_receive_callback(const void *data, uint16_t len, 
                             const linkaddr_t *src, const linkaddr_t *dest) 
{
  if (len == sizeof(data_packet_t)) {
    data_packet_t *packet = (data_packet_t *)data;
    
    // Handle different packet types
    if (packet->type == PACKET_TYPE_LIGHT_DATA) {
      // Store light sensor readings
      memcpy(light_readings, packet->readings, sizeof(int) * MAX_READINGS);
      light_received = 1;
      printf("Received light data from node %d\n", packet->src_id);
    } 
    else if (packet->type == PACKET_TYPE_MOTION_DATA) {
      // Store motion sensor readings
      memcpy(motion_readings, packet->readings, sizeof(int) * MAX_READINGS);
      motion_received = 1;
      printf("Received motion data from node %d\n", packet->src_id);
    }
    
    // If both types of data received, print them
    if (light_received && motion_received) {
      printf("Light: ");
      for (int i = 0; i < MAX_READINGS; i++) {
        printf("%d", light_readings[i]);
        if (i < MAX_READINGS - 1) printf(", ");
      }
      printf("\n");
      
      printf("Motion: ");
      for (int i = 0; i < MAX_READINGS; i++) {
        printf("%d", motion_readings[i]);
        if (i < MAX_READINGS - 1) printf(", ");
      }
      printf("\n");
      
      // Reset flags
      light_received = 0;
      motion_received = 0;
    }
  }
}

// === Beacon Scheduler using rtimer ===
char sender_scheduler(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static int NumSleep = 0;
  PT_BEGIN(&pt);

  while(1) {
    printf("Radio ON: sending beacons...\n");
    NETSTACK_RADIO.on();

    for(i = 0; i < NUM_SEND; i++) {
      // Set up beacon packet
      beacon_packet.type = PACKET_TYPE_BEACON;
      beacon_packet.src_id = node_id;
      beacon_packet.timestamp = clock_time();
      beacon_packet.seq++;
      
      nullnet_buf = (uint8_t *)&beacon_packet;
      nullnet_len = sizeof(data_packet_t);

      printf("Node B sent beacon seq %lu at time %lu\n", 
             beacon_packet.seq, clock_time() / CLOCK_SECOND);
      NETSTACK_NETWORK.output(&dest_addr);

      if(i != NUM_SEND - 1) {
        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, 
                  (rtimer_callback_t)sender_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }

    if(SLEEP_CYCLE != 0) {
      printf("Radio OFF\n");
      NETSTACK_RADIO.off();
      NumSleep = random_rand() % (2 * SLEEP_CYCLE + 1);
      printf("Sleep for %d slots\n", NumSleep);

      for(i = 0; i < NumSleep; i++) {
        rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT, 1, 
                  (rtimer_callback_t)sender_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }
  }

  PT_END(&pt);
}

// === Beacon Sender Process ===
PROCESS(beacon_process, "Node B - Beacon Sender");
AUTOSTART_PROCESSES(&beacon_process);

PROCESS_THREAD(beacon_process, ev, data)
{
  PROCESS_BEGIN();

  printf("Node B starting beaconing + nullnet receiver...\n");

  // Init packet info
  beacon_packet.seq = 0;

  // Set broadcast destination for beacon
  linkaddr_copy(&dest_addr, &linkaddr_null);

  // Set up nullnet callback for receiving data
  nullnet_set_input_callback(nullnet_receive_callback);
  
  // Set up beacon send via rtimer
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, 
            (rtimer_callback_t)sender_scheduler, NULL);

  PROCESS_END();
}
