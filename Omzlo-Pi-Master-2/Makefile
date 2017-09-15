TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  = -O2 -g -Wall -pedantic
ASFLAGS = -g 

# object files

OBJS=  $(STARTUP) main.o
OBJS+= system.o systick.o gpio.o usart.o spi_slave.o can.o registers.o adc.o power.o 

# include common make file

include $(TEMPLATEROOT)/Makefile.common


