TARGET = main

# repertoire des modules
AVERSIVE_DIR = ../../..
# VALUE, absolute or relative path : example ../.. #

CFLAGS += -Werror
LDFLAGS = -T ../common/avr6.x

# List C source files here. (C dependencies are automatically generated.)
SRC  = $(TARGET).c cmdline.c commands_ax12.c commands_gen.c 
SRC += commands_cs.c commands_ballboard.c commands.c
SRC += i2c_protocol.c sensor.c actuator.c cs.c ax12_user.c


# List Assembler source files here.
# Make them always end in a capital .S.  Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC = 

########################################

-include .aversive_conf
include $(AVERSIVE_DIR)/mk/aversive_project.mk

scan_h_l.h:
	@gcc gen_scan_tab.c -o gen_scan_tab -lm; 		\
	if ./gen_scan_tab > /dev/null 2>&1; then 	\
		echo ok;				\
	else						\
		echo nok;				\
	fi

AVRDUDE_DELAY=50

program_noerase: $(TARGET).$(FORMAT_EXTENSION) $(TARGET).eep
	echo $(AVRDUDE) -D -V $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) ;\
	$(AVRDUDE) -D -V $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) ;\

