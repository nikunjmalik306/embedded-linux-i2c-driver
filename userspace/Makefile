# SPDX-License-Identifier: MIT
# Userspace IIO Telemetry & Latency Analyzer Makefile

CC      ?= gcc
CFLAGS  := -std=c11 -O2 -Wall -Wextra -Werror -pedantic -D_POSIX_C_SOURCE=200809L
LDFLAGS := -lm

TARGET  := sensor_reader
SRC     := sensor_reader.c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o
