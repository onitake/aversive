TARGET = main

# repertoire des modules
AVERSIVE_DIR = ../../..

SRC  = $(TARGET).c cmdline.c commands_ax12.c commands_gen.c 
SRC += commands_cs.c commands_mainboard.c commands_traj.c commands.c
SRC += i2c_protocol.c sensor.c actuator.c cs.c ax12_user.c
SRC += strat_utils.c strat_base.c strat.c

ASRC = 

CFLAGS += -Wall -Werror
LDFLAGS = -T ../common/avr6.x

########################################

-include .aversive_conf
include $(AVERSIVE_DIR)/mk/aversive_project.mk

AVRDUDE_DELAY=50

program_noerase: $(TARGET).$(FORMAT_EXTENSION) $(TARGET).eep
	echo $(AVRDUDE) -D -V $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) ;\
	$(AVRDUDE) -D -V $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) ;\

