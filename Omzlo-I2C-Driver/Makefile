TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  = -O1 -g
ASFLAGS = -g 

# object files

OBJS=  $(STARTUP) main.o
OBJS+= stm32f0xx_misc.o stm32f0xx_gpio.o stm32f0xx_usart.o stm32f0xx_rcc.o stm32f0xx_can.o 
OBJS+= system.o usart.o can.o i2c_slave.o systick.o chip_options.o gpio.o nocan.o 

# include common make file

include $(TEMPLATEROOT)/Makefile.common


