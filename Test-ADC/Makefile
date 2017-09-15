TEMPLATEROOT = ../

# additional compilation flags

CFLAGS  += -O0 -g
ASFLAGS += -g 

# project files

OBJS=    $(STARTUP) main.o usart.o system.o
OBJS+=   stm32f0xx_gpio.o stm32f0xx_rcc.o stm32f0xx_usart.o

# include common make file

include $(TEMPLATEROOT)/Makefile.common
