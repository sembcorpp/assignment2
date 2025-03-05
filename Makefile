CONTIKI_PROJECT = task2 task3
all: $(CONTIKI_PROJECT)
CONTIKI = ../..
include $(CONTIKI)/Makefile.include
CFLAGS += -w
