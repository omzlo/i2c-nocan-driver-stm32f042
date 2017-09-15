SUBDIRS= BlinkingLight Omzlo-I2C-Driver

.PHONY:  clean 

all:  
	-for d in $(SUBDIRS); do (cd $$d; $(MAKE)); done

clean:
	-for d in $(SUBDIRS); do (cd $$d; $(MAKE) clean); done
