// ===============================
// NodeA - Detector and Sender
// ===============================

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include "net/linkaddr.h"
#include <string.h>
#include <stdio.h> 
#include "node-id.h"

#define RSSI_THRESHOLD -70 // dBm // JUSTIFY OR CHANGE ACCORDINGLY
#define MAX_READINGS 60     // Number of light and motion readings to store

typedef struct {
  unsigned long src_id;
  unsigned long timestamp;
  unsigned long seq;
} data_packet_struct;

static unsigned long curr_timestamp;

PROCESS(node_a_process, "Node A - Detector");
AUTOSTART_PROCESSES(&node_a_process);

// Function called after reception of a packet
void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) 
{
  // Check if the received packet size matches with what we expect it to be
  if(len == sizeof(data_packet_struct)) {

    static data_packet_struct received_packet_data;
    // Copy the content of packet into the data structure
    memcpy(&received_packet_data, data, len);

    // Get current timestamp
    curr_timestamp = clock_time();

    // Get RSSI (signal strength of the received packet)
    signed short rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);

    // Print DETECT log e.g: 123 DETECT 34567
    printf("%lu DETECT %d\n", curr_timestamp / CLOCK_SECOND, received_packet.src_id);
    
    // Link Quality Check: If signal > threshold (good enough), start transferring
    if (rssi >= RSSI_THRESHOLD) {
        // TODO: transferring logic / function call
        printf("%lu TRANSFER %d RSSI: %d\n", curr_timestamp / CLOCK_SECOND, received_packet.src_id, rssi);
    } else {
        printf("Link quality not good enough (RSSI = %d)\n", rssi);
    } 
  }
}

PROCESS_THREAD(node_a_process, ev, data)
{
  PROCESS_BEGIN();

  // Set this function to be called whenever a packet is received
  nullnet_set_input_callback(receive_packet_callback);

  printf("Node A ready to receive beacons...\n");

  // Idle loop – this process waits forever, the callback does the work
  PROCESS_WAIT_EVENT();

  PROCESS_END();
}