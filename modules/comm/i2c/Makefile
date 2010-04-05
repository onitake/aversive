TARGET = i2c

# List C source files here. (C dependencies are automatically generated.)
ifeq ($(HOST),avr)
SRC = i2c.c
else
SRC = i2c_host.c
endif

include $(AVERSIVE_DIR)/mk/aversive_module.mk
