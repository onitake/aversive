TARGET = main

# repertoire des modules
AVERSIVE_DIR = ../../..
# VALUE, absolute or relative path : example ../.. #

CFLAGS += -Werror

# atm128
# address is 0xf000 (in words)
# LDFLAGS += -Wl,--section-start=.text=1e000
# UART_NUM = 0

# atm2560
# address is 0x1f800 (in words)
LDFLAGS += -Wl,--section-start=.text=3f000
UART_NUM = 1

CFLAGS += -DUART_NUM=$(UART_NUM)

# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).c 

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

program_noerase: $(TARGET).$(FORMAT_EXTENSION) $(TARGET).eep
	echo $(AVRDUDE) -D -V $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) ;\
	$(AVRDUDE) -D -V $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) ;\

