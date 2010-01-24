TARGET = scheduler

# List C source files here. (C dependencies are automatically generated.)
SRC := scheduler_add.c scheduler_del.c scheduler_interrupt.c
SRC += scheduler_dump.c scheduler_stats.c
ifeq ($(H),1)
SRC += scheduler_host.c
else
SRC += scheduler.c
endif

include $(AVERSIVE_DIR)/mk/aversive_module.mk
