#include <stdio.h>
#include <stdlib.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "buzzer.h"
#include "sys/rtimer.h"
#include "board-peripherals.h"

#include <stdint.h>

PROCESS(process_rtimer, "RTimer");
AUTOSTART_PROCESSES(&process_rtimer);

enum State {
    _IDLE,
    _INTERIM,
    _BUZZ,
    _WAIT
};

static enum State curr_state = _IDLE;
static int counter_rtimer;
static struct rtimer timer_rtimer;
static rtimer_clock_t timeout_rtimer = RTIMER_SECOND / 4;

// Additional Timers
static rtimer_clock_t buzz_duration = RTIMER_SECOND * 2;
static rtimer_clock_t pause_duration = RTIMER_SECOND * 4;
static int buzzstate_countdown = 4;

static int prev_lux_reading = 0;
static void init_opt_reading(void);
static void check_change_in_light_reading(void);
static void init_mpu_reading(void);
static void check_change_in_mpu_reading(void);

#define IMU_BOUND 100
#define LUX_BOUND 300

int buzzerFrequency[8] = {2093, 2349, 2637, 2794, 3156, 3520, 3951, 4186};

// Rtimer Timeout
void do_rtimer_timeout(struct rtimer *timer, void *ptr) {
    printf("Curr State: %d\n", curr_state);

    if (curr_state == _IDLE) {
        check_change_in_mpu_reading();
        check_change_in_light_reading();
        rtimer_set(&timer_rtimer, RTIMER_NOW() + timeout_rtimer, 0, do_rtimer_timeout, NULL);
    } 
    else if (curr_state == _INTERIM) {
        printf("Checking Light in INTERIM...\n");
        check_change_in_light_reading();
        if (curr_state == _INTERIM) {
            rtimer_set(&timer_rtimer, RTIMER_NOW() + timeout_rtimer, 0, do_rtimer_timeout, NULL);
        }
        if (curr_state == _BUZZ) {  
            // ✅ Ensure proper transition to `_BUZZ`
            printf("Light detected! Transitioning to BUZZ.\n");
            rtimer_set(&timer_rtimer, RTIMER_NOW() + timeout_rtimer, 0, do_rtimer_timeout, NULL);
    
            return;  // Stop further execution here
        }
    } 
    else if (curr_state == _BUZZ) {
        printf("BUZZING for 2s!\n");
        buzzer_start(buzzerFrequency[0]);
        rtimer_set(&timer_rtimer, RTIMER_NOW() + buzz_duration, 0, do_rtimer_timeout, NULL);
        check_change_in_light_reading();
        if (curr_state == _IDLE) {  // FIX: Ensure IDLE takes effect before scheduling next state
            printf("Light detected during BUZZ! Stopping everything.\n");
            buzzer_stop();
            return;  // **Exit before scheduling _WAIT**
        }

        curr_state = _WAIT;  // Only transition to _WAIT after scheduling is confirmed
    } 
    else if (curr_state == _WAIT) {
        printf("WAITING for 4s...\n");
        buzzer_stop();

        rtimer_set(&timer_rtimer, RTIMER_NOW() + pause_duration, 0, do_rtimer_timeout, NULL);
        curr_state = _BUZZ;  // Only transition to _BUZZ after scheduling is confirmed
    }
}




// Light Sensor Checking
static void check_change_in_light_reading() {
    int value;
    value = opt_3001_sensor.value(0);
    
    if (value != CC26XX_SENSOR_READING_ERROR) {
        printf("OPT: Light=%d.%02d lux\n", value / 100, value % 100);

        int lux_difference = abs((value - prev_lux_reading) / 100);

        if (lux_difference > LUX_BOUND) { 
            printf("Significant Light Change Detected! Transitioning...\n");

            if (curr_state == _INTERIM) { 
                printf("Transitioning to BUZZ\n");
                curr_state = _BUZZ;
                buzzstate_countdown = 4;
            } 
            else if (curr_state == _BUZZ) { 
                printf("Transitioning to IDLE\n");
                curr_state = _IDLE;
            }
        }
        prev_lux_reading = value;  // Update for next cycle
    }
     else {
        printf("Light Sensor Not Ready\n");
    }
    init_opt_reading();
}


static void init_opt_reading(void) {
    SENSORS_ACTIVATE(opt_3001_sensor);
}

// Motion Sensor Checking
static void check_change_in_mpu_reading() {
    int x, y, z;
    x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
    y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
    z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
    
    if ((abs(x) > IMU_BOUND || abs(y) > IMU_BOUND || abs(z) > IMU_BOUND) && curr_state == _IDLE) {
        printf("Significant Motion Detected - Entering INTERIM State\n");
        curr_state = _INTERIM;
    }
}

static void init_mpu_reading(void) {
    mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

PROCESS_THREAD(process_rtimer, ev, data) {
    PROCESS_BEGIN();
    init_opt_reading();
    init_mpu_reading();
    buzzer_init();

    while (1) {
        rtimer_set(&timer_rtimer, RTIMER_NOW() + timeout_rtimer, 0, do_rtimer_timeout, NULL);
        PROCESS_YIELD();
    }
    PROCESS_END();
}


