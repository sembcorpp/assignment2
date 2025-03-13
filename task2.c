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
	_BUZZ,
	_WAIT
};

static enum State curr_state = _IDLE; 

static int counter_rtimer;
static struct rtimer timer_rtimer;
static rtimer_clock_t timeout_rtimer = RTIMER_SECOND / 4;

//Additional Timers
static rtimer_clock_t cycle_rtimer = RTIMER_SECOND * 2;
static int buzzstate_countdown = 4;
static int switch_change = 1;

static int prev_lux_reading = 0;
static void init_opt_reading(void);
static void check_change_in_light_reading(void);
static void update_lux(void);
static void init_mpu_reading(void);
static void check_change_in_mpu_reading(void);

#define IMU_BOUND 100
#define LUX_BOUND 300

int buzzerFrequency[8]={2093,2349,2637,2794,3156,3520,3951,4186}; // hgh notes on a piano


//Rtimer Timeout
void
do_rtimer_timeout(struct rtimer *timer, void *ptr)
{

  rtimer_clock_t now=RTIMER_NOW();

  int s, ms1,ms2,ms3;
  s = now /RTIMER_SECOND;
  ms1 = (now% RTIMER_SECOND)*10/RTIMER_SECOND;
  ms2 = ((now% RTIMER_SECOND)*100/RTIMER_SECOND)%10;
  ms3 = ((now% RTIMER_SECOND)*1000/RTIMER_SECOND)%10;
  
  counter_rtimer++;
  printf(": %d (cnt) %d (ticks) %d.%d%d%d (sec) \n",counter_rtimer,now, s, ms1,ms2,ms3);
  printf("Curr State: %d", curr_state);
  if (curr_state == _IDLE)
    {
      check_change_in_light_reading();
      check_change_in_mpu_reading();
      if (curr_state == _IDLE) {
        rtimer_set(&timer_rtimer, RTIMER_NOW() + timeout_rtimer, 0, do_rtimer_timeout, NULL);
      }
    }
    else if (curr_state == _WAIT)
    {
      buzzer_stop();
      update_lux();
      rtimer_set(&timer_rtimer, RTIMER_NOW() + cycle_rtimer, 0, do_rtimer_timeout, NULL);
      curr_state = _BUZZ;
      buzzstate_countdown--;
      printf("\nCurrent Buzz State Cycle: %d", buzzstate_countdown);
      if (buzzstate_countdown == 0) {
        curr_state = _IDLE;
      }
    }
    else if (curr_state == _BUZZ)
    {
      buzzer_start(buzzerFrequency[0]);
      rtimer_set(&timer_rtimer, RTIMER_NOW() + cycle_rtimer, 0, do_rtimer_timeout, NULL);
      printf("\nFinish Buzzing");
      curr_state = _WAIT;
    }
}


//Rtimer Light Sensing and Activation
static void
check_change_in_light_reading()
{
  int value;

  value = opt_3001_sensor.value(0);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    printf("OPT: Light=%d.%02d lux\n", value / 100, value % 100);
    if (abs((value - prev_lux_reading) / 100) > LUX_BOUND && curr_state == _IDLE) {
      printf("Changing to Buzz State");
      curr_state = _BUZZ;
      buzzstate_countdown = 4;
    }

    prev_lux_reading = value;
  } else {
    printf("OPT: Light Sensor's Warming Up\n\n");
  }
  init_opt_reading();
}

static void
init_opt_reading(void)
{
  SENSORS_ACTIVATE(opt_3001_sensor);
}

static void
update_lux(void) 
{
  int value;
  value = opt_3001_sensor.value(0);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    printf("OPT: Light=%d.%02d lux\n", value / 100, value % 100);
    prev_lux_reading = value;
  }

  init_opt_reading();
}

//Motion Sensor
static void
check_change_in_mpu_reading()
{
  int x, y, z;
  printf("Curr State: %d", curr_state);

  x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  printf("MPU Acc: X=%d", x);
  
  y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  printf("MPU Acc: Y=%d", y);

  z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  printf("MPU Acc: Z=%d", z);
  

  if ((abs(x) > IMU_BOUND || abs(y) > IMU_BOUND || abs(z) > IMU_BOUND) && curr_state == _IDLE) {
    printf("Changing to Buzz State");
    curr_state = _BUZZ;
    buzzstate_countdown = 4; 
  }

  init_mpu_reading();
}

static void
init_mpu_reading(void)
{
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

PROCESS_THREAD(process_rtimer, ev, data)
{
  PROCESS_BEGIN();
  init_opt_reading();
  init_mpu_reading();
  buzzer_init();

  printf(" The value of RTIMER_SECOND is %d \n", RTIMER_SECOND);
  printf(" The value of timeout_rtimer is %d \n", timeout_rtimer);

  while (1) {
    rtimer_set(&timer_rtimer, RTIMER_NOW() + timeout_rtimer, 0, do_rtimer_timeout, NULL);
    PROCESS_YIELD();
  }
  PROCESS_END();
}