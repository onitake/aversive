TARGET = adc

# List C source files here. (C dependencies are automatically generated.)
ifeq ($(HOST),avr)
SRC = adc.c
else
SRC = adc_host.c
endif

include $(AVERSIVE_DIR)/mk/aversive_module.mk
