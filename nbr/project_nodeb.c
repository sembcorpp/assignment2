#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "net/rime/rime.h"

PROCESS(node_b_process, "Node B Process");
AUTOSTART_PROCESSES(&node_b_process);

static char light_buffer[512];
static char motion_buffer[512];
static int light_received = 0;
static int motion_received = 0;

static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
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
static struct unicast_conn uc;

PROCESS_THREAD(node_b_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)

  PROCESS_BEGIN();

  printf("Node B started.\n");
  unicast_open(&uc, 146, &unicast_call); // Must match Node A

  PROCESS_END();
}
