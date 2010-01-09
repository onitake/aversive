
TARGET = spi

# List C source files here. (C dependencies are automatically generated.)
ifeq ($(HOST),avr)
SRC = spi.c
else
SRC = spi_host.c
endif

###########################################

include $(AVERSIVE_DIR)/mk/aversive_module.mk
