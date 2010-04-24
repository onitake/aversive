TARGET = diagnostic

# List C source files here. (C dependencies are automatically generated.)
ifeq ($(HOST),avr)
SRC = stack_space.c int_show.c stackdump.c
else
SRC = diag_host.c stackdump.c
endif

include $(AVERSIVE_DIR)/mk/aversive_module.mk

