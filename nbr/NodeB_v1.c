// ===============================
// NodeB.c - Broadcaster and Receiver
// ===============================

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "lib/random.h"
#include "net/linkaddr.h"
#include "net/packetbuf.h"
#include <string.h>
#include <stdio.h>
#include "node-id.h"

// Parameters for neighbor discovery
#define WAKE_TIME RTIMER_SECOND/10    // 100ms wake time
#define SLEEP_CYCLE 9
#define SLEEP_SLOT RTIMER_SECOND/10   // 100ms sleep slot
#define NUM_SEND 2                    // Send 2 packets per wake

// Packet types
#define PACKET_TYPE_BEACON 1
#define PACKET_TYPE_DATA 2

// Maximum readings to store (60 readings for 60 seconds)
#define MAX_READINGS 60

// Beacon packet structure
typedef struct {
  uint8_t packet_type;
  unsigned long src_id;
  unsigned long timestamp;
  unsigned long seq;
} beacon_packet_struct;

// Data packet structure for receiving sensor readings
typedef struct {
  uint8_t packet_type;
  unsigned long src_id;
  unsigned long start_idx;
  unsigned long num_readings;
  int16_t light_readings[10];  // Receive 10 readings at a time
  int16_t motion_readings[10]; // Receive 10 readings at a time
} data_packet_struct;

// For neighbor discovery
static linkaddr_t dest_addr;               // Broadcast address
static struct rtimer rt;
static struct pt pt;
static beacon_packet_struct beacon_packet;
static unsigned long curr_timestamp;

// For data reception
static int16_t received_light_readings[MAX_READINGS];
static int16_t received_motion_readings[MAX_READINGS];
static unsigned int received_readings_count = 0;
static linkaddr_t last_sender;
static unsigned long last_sender_id = 0;

// Neighbor tracking
static struct {
  linkaddr_t addr;
  unsigned long node_id;
  int8_t rssi;
  unsigned long last_heard;
  uint8_t discovered;
} neighbors[10];
static uint8_t num_neighbors = 0;

PROCESS(nbr_discovery_process, "Node B - Broadcaster and Receiver");
AUTOSTART_PROCESSES(&nbr_discovery_process);

// Find neighbor index by node ID
static int find_neighbor(unsigned long node_id) {
  int i;
  for (i = 0; i < num_neighbors; i++) {
    if (neighbors[i].node_id == node_id) {
      return i;
    }
  }
  return -1;
}

// Function called after reception of a packet
void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
  if (len >= sizeof(uint8_t)) {
    uint8_t packet_type = *(uint8_t *)data;
    
    // Get current timestamp and RSSI
    curr_timestamp = clock_time();
    signed short rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
    
    // Handle beacon packet
    if (packet_type == PACKET_TYPE_BEACON && len == sizeof(beacon_packet_struct)) {
      beacon_packet_struct *beacon = (beacon_packet_struct *)data;
      
      // Log detection in required format
      printf("%lu DETECT %lu\n", curr_timestamp / CLOCK_SECOND, beacon->src_id);
      
      // Store/update neighbor information
      int idx = find_neighbor(beacon->src_id);
      if (idx == -1 && num_neighbors < 10) {
        // New neighbor
        idx = num_neighbors++;
        linkaddr_copy(&neighbors[idx].addr, src);
        neighbors[idx].node_id = beacon->src_id;
        neighbors[idx].discovered = 1;
      }
      
      if (idx != -1) {
        // Update neighbor information
        neighbors[idx].rssi = rssi;
        neighbors[idx].last_heard = curr_timestamp;
      }
    }
    
    // Handle data packet
    else if (packet_type == PACKET_TYPE_DATA && len == sizeof(data_packet_struct)) {
      data_packet_struct *data_packet = (data_packet_struct *)data;
      
      // Keep radio on while receiving data
      NETSTACK_RADIO.on();
      
      // Store the sender information
      last_sender_id = data_packet->src_id;
      linkaddr_copy(&last_sender, src);
      
      // Process and store the readings
      unsigned int i;
      for (i = 0; i < data_packet->num_readings && 
           data_packet->start_idx + i < MAX_READINGS; i++) {
        
        unsigned int idx = data_packet->start_idx + i;
        if (idx < MAX_READINGS) {
          received_light_readings[idx] = data_packet->light_readings[i];
          received_motion_readings[idx] = data_packet->motion_readings[i];
          
          // Update count of received readings if needed
          if (idx + 1 > received_readings_count) {
            received_readings_count = idx + 1;
          }
        }
      }
      
      printf("Received data packet from %lu with %lu readings starting at index %lu (RSSI: %d)\n",
             data_packet->src_id, data_packet->num_readings, data_packet->start_idx, rssi);
      
      // If we've received a complete set, display them
      if (received_readings_count == MAX_READINGS) {
        printf("Light: ");
        for (i = 0; i < received_readings_count; i++) {
          printf("%d", received_light_readings[i]);
          if (i < received_readings_count - 1) {
            printf(", ");
          }
        }
        printf("\n");
        
        printf("Motion: ");
        for (i = 0; i < received_readings_count; i++) {
          printf("%d", received_motion_readings[i]);
          if (i < received_readings_count - 1) {
            printf(", ");
          }
        }
        printf("\n");
        
        // Reset for next round
        received_readings_count = 0;
      }
    }
  }
}

// Main sending loop with randomized sleep schedule
char sender_scheduler(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static int NumSleep = 0;
  PT_BEGIN(&pt);

  while(1) {
    printf("Radio ON: sending beacons and listening...\n");
    NETSTACK_RADIO.on();

    // Send NUM_SEND number of neighbour discovery beacon packets
    for(i = 0; i < NUM_SEND; i++) {
      // Initialize the nullnet module with information of packet to be transmitted
      nullnet_buf = (uint8_t *)&beacon_packet; // Data transmitted
      nullnet_len = sizeof(beacon_packet); // Length of data transmitted
      beacon_packet.seq++;
      beacon_packet.timestamp = clock_time();

      // Packet transmission
      printf("Node B sent beacon seq %lu at time %lu\n", beacon_packet.seq, clock_time() / CLOCK_SECOND);
      NETSTACK_NETWORK.output(&dest_addr);

      // wait for WAKE_TIME before sending the next packet
      if(i != NUM_SEND - 1) {
        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }
    
    // Stay awake a bit longer after sending beacons to listen for responses
    rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
    PT_YIELD(&pt);

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

  // Initialize the beacon packet
  beacon_packet.packet_type = PACKET_TYPE_BEACON;
  beacon_packet.src_id = node_id;
  beacon_packet.seq = 0;
  
  // Set broadcast address
  linkaddr_copy(&dest_addr, &linkaddr_null);
  
  // Set up the reception callback
  nullnet_set_input_callback(receive_packet_callback);

  printf("Node B starting beaconing and data collection, NodeID: %u\n", node_id);
  
  // Start sender in one millisecond
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);

  // Keep the process alive
  while(1) {
    PROCESS_YIELD();
  }

  PROCESS_END();
}
