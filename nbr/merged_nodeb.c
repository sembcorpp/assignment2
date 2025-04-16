#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/rime/rime.h"
#include "lib/random.h"
#include "net/linkaddr.h"
#include "node-id.h"

// === Settings ===
#define RSSI_THRESHOLD -70
#define WAKE_TIME RTIMER_SECOND/10
#define SLEEP_CYCLE 9
#define SLEEP_SLOT RTIMER_SECOND/10
#define NUM_SEND 2

#define MAX_BUFFER 512

// === Beacon Packet Struct ===
typedef struct {
  unsigned long src_id;
  unsigned long timestamp;
  unsigned long seq;
} data_packet_struct;

static struct unicast_conn uc;
static char light_buffer[MAX_BUFFER];
static char motion_buffer[MAX_BUFFER];
static int light_received = 0;
static int motion_received = 0;

static struct rtimer rt;
static struct pt pt;
static data_packet_struct data_packet;
static linkaddr_t dest_addr;

// === Unicast Receiver Callback ===
static void recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
  const char *msg = (const char *)packetbuf_dataptr();

  if(strncmp(msg, "Light:", 6) == 0) {
    snprintf(light_buffer, sizeof(light_buffer), "%s", msg);
    light_received = 1;
  } else if(strncmp(msg, "Motion:", 7) == 0) {
    snprintf(motion_buffer, sizeof(motion_buffer), "%s", msg);
    motion_received = 1;
  }

  if(light_received && motion_received) {
    printf("%s\n", light_buffer);
    printf("%s\n", motion_buffer);
    light_received = 0;
    motion_received = 0;
  }
}

static const struct unicast_callbacks unicast_call = {recv_uc};

// === Beacon Scheduler using rtimer ===
char sender_scheduler(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static int NumSleep = 0;
  PT_BEGIN(&pt);

  while(1) {
    printf("Radio ON: sending beacons...\n");
    NETSTACK_RADIO.on();

    for(i = 0; i < NUM_SEND; i++) {
      nullnet_buf = (uint8_t *)&data_packet;
      nullnet_len = sizeof(data_packet);
      data_packet.seq++;
      data_packet.timestamp = clock_time();

      printf("Node B sent beacon seq %lu at time %lu\n", data_packet.seq, clock_time() / CLOCK_SECOND);
      NETSTACK_NETWORK.output(&dest_addr);

      if(i != NUM_SEND - 1) {
        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }

    if(SLEEP_CYCLE != 0) {
      printf("Radio OFF\n");
      NETSTACK_RADIO.off();
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

// === Beacon Sender Process ===
PROCESS(beacon_process, "Node B - Beacon Sender");
AUTOSTART_PROCESSES(&beacon_process);

PROCESS_THREAD(beacon_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)

  PROCESS_BEGIN();

  printf("Node B starting beaconing + unicast receiver...\n");

  // Init packet info
  data_packet.src_id = node_id;
  data_packet.seq = 0;

  // Set broadcast destination for beacon
  linkaddr_copy(&dest_addr, &linkaddr_null);

  // Set up beacon send via rtimer
  nullnet_set_input_callback(NULL);
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);

  // Set up unicast reception
  unicast_open(&uc, 146, &unicast_call);

  PROCESS_END();
}
