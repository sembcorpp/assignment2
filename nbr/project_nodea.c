#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "net/rime/rime.h"
#include "board-peripherals.h"

#define MAX_READINGS 60

static int light_readings[MAX_READINGS];
static int motion_readings[MAX_READINGS];
static int write_index = 0;
static int total_samples = 0;
static int is_ready_to_transfer = 0;

static struct unicast_conn uc;

/*------------------------------------------------------------*/
void transfer_unsent_data(int node_id, int rssi, const linkaddr_t *to) {
  if (!is_ready_to_transfer) return;

  printf("%lu DETECT %d\n", clock_seconds(), node_id);
  printf("%lu TRANSFER %d RSSI: %d\n", clock_seconds(), node_id, rssi);

  static char msg[512];

  // Send Light readings
  snprintf(msg, sizeof(msg), "Light: ");
  for (int i = 0; i < MAX_READINGS; i++) {
    int idx = (write_index + i) % MAX_READINGS;
    char temp[16];
    sprintf(temp, "%d", light_readings[idx]);
    strcat(msg, temp);
    if (i < MAX_READINGS - 1) strcat(msg, ", ");
  }
  packetbuf_copyfrom(msg, strlen(msg) + 1);
  unicast_send(&uc, to);

  // Send Motion readings
  snprintf(msg, sizeof(msg), "Motion: ");
  for (int i = 0; i < MAX_READINGS; i++) {
    int idx = (write_index + i) % MAX_READINGS;
    char temp[16];
    sprintf(temp, "%d", motion_readings[idx]);
    strcat(msg, temp);
    if (i < MAX_READINGS - 1) strcat(msg, ", ");
  }
  packetbuf_copyfrom(msg, strlen(msg) + 1);
  unicast_send(&uc, to);

  // Reset buffer
  write_index = 0;
  total_samples = 0;
  is_ready_to_transfer = 0;

  printf("Transfer complete. Starting new collection cycle.\n");
}

/*------------------------------------------------------------*/
PROCESS(sensor_node_process, "Sensor Node A");
AUTOSTART_PROCESSES(&sensor_node_process);

/*------------------------------------------------------------*/
PROCESS_THREAD(sensor_node_process, ev, data)
{
  static struct etimer sampling_timer;
  int light_val, acc_x_val;

  PROCESS_EXITHANDLER(unicast_close(&uc);)

  PROCESS_BEGIN();

  printf("Node A started.\n");

  SENSORS_ACTIVATE(opt_3001_sensor);
  SENSORS_ACTIVATE(mpu_9250_sensor);
  unicast_open(&uc, 146, NULL); // Matching Node B’s channel

  while (1) {
    etimer_set(&sampling_timer, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sampling_timer));

    light_val = opt_3001_sensor.value(0);
    acc_x_val = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);

    light_readings[write_index] = light_val;
    motion_readings[write_index] = acc_x_val;

    printf("Sample #%lu @ %lu - Light: %d, Motion: %d\n",
           total_samples + 1, clock_seconds(), light_val, acc_x_val);

    write_index = (write_index + 1) % MAX_READINGS;
    total_samples++;

    if (total_samples >= MAX_READINGS) {
      is_ready_to_transfer = 1;
    }
  }

  PROCESS_END();
}
