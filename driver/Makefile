# SPDX-License-Identifier: GPL-2.0-only
# Out-of-Tree Linux Kernel Module Makefile for BME280 I2C Sensor Driver

obj-m += sensor_driver.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

ccflags-y := -Wall -Wextra -Wno-unused-parameter

.PHONY: all modules clean check-style

all: modules

modules:
	@if [ -d "$(KDIR)" ]; then \
		$(MAKE) -C $(KDIR) M=$(PWD) modules; \
	else \
		echo "[WARN] Kernel build tree $(KDIR) not found on this host."; \
		echo "[INFO] Run './scripts/build.sh --check-only' in CI/container environments without kernel headers."; \
	fi

clean:
	@if [ -d "$(KDIR)" ]; then \
		$(MAKE) -C $(KDIR) M=$(PWD) clean; \
	else \
		rm -f *.o *.ko *.mod* *.symvers *.order .*.cmd; \
		rm -rf .tmp_versions; \
	fi
