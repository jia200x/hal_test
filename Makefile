include ../Makefile.tests_common

DISABLE_MODULE += auto_init_at86rf2xx auto_init_nrf802154

USEMODULE += od
USEMODULE += shell
USEMODULE += ps
USEMODULE += event_thread_highest
USEMODULE += event_callback
CFLAGS += -DEVENT_THREAD_HIGHEST_STACKSIZE=1024

NETDEV ?= 0

CFLAGS += -DNETDEV=$(NETDEV)

# define the driver to be used for selected boards
ifneq (,$(filter samr21-xpro,$(BOARD)))
  DRIVER := at86rf233
endif
ifneq (,$(filter iotlab-m3 fox,$(BOARD)))
  DRIVER := at86rf231
endif
ifneq (,$(filter nrf52840dk,$(BOARD)))
  DRIVER := nrf802154
endif

# use the at86rf231 as fallback device
DRIVER ?= at86rf231

# include the selected driver
USEMODULE += $(DRIVER)

include $(RIOTBASE)/Makefile.include
