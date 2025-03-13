CONTIKI_PROJECT = etimer-buzzer rtimer-lightSensor rtimer-IMUSensor task2 task3
all: $(CONTIKI_PROJECT)
CONTIKI = ../..
include $(CONTIKI)/Makefile.include
CFLAGS += -w
