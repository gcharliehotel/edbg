COMPILER ?= gcc
UNAME ?= $(shell uname)

SRCS = \
  dap.c \
  edbg.c \
  target.c \
  target_atmel_cm0p.c \
  target_atmel_cm3.c \
  target_atmel_cm4.c \
  target_atmel_cm7.c

HDRS = \
  dap.h \
  dap_common.h \
  dbg.h \
  edbg.h \
  target.h

BIN = edbg
SRCS += dbg_sysfs.c

CFLAGS += -W -Wall -Wextra -O2 -std=gnu11

all: $(BIN)

$(BIN): $(SRCS) $(HDRS) $(HIDAPI)
	$(COMPILER) $(CFLAGS) $(SRCS) $(LIBS) -o $(BIN)

clean:
	rm -rvf $(BIN) hidapi

