#include <stdio.h>
#include "contiki.h"
#include "sys/etimer.h"
#include "buzzer.h"
#include "board-peripherals.h"

#define SIGNIFICANT_LIGHT_CHANGE 300
#define SIGNIFICANT_MOTION_CHANGE 500 //random value.... TODO: what should we put here?


#define BUZZER_DURATION 2
#define WAITING_TIME 2
#define IDLE_TIME 16

// Sampling Constraints 
#define LIGHT_INTERVAL (CLOCK_SECOND / 4) // 1000ms / 4 = 250 ms
#define MOTION_INTERVAL (CLOCK_SECOND / 50) // 1000ms / 50  = 20ms

enum state { IDLE, BUZZ, WAIT };
static enum state current_state = IDLE;

PROCESS(process_task_two, "process_task_two");
AUTOSTART_PROCESSES(&process_task_two);

// TODO: 
// reference etimer-buzzer for buzzer code
// reference lightSensor to detect significant light change
// reference IMUSensor to detect significant motion change

PROCESS_THREAD(process_task_two, ev, data)
{ 
    PROCESS_BEGIN();

    PROCESS_END();
}