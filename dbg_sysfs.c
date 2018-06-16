/*
 * Copyright (c) 2013-2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "edbg.h"
#include "dbg.h"

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  int num;
  int fd;
} gpio_t;

/*- Variables ---------------------------------------------------------------*/
static gpio_t swdio_gpio = {-1, -1};
static gpio_t swclk_gpio = {-1, -1};
static uint8_t req_buffer[1024];
static uint8_t res_buffer[1024];
static int report_size = sizeof(res_buffer); // must be same as above.

/*- Implementations ---------------------------------------------------------*/

void save_file2(char *path, char *buf) {
  save_file(path, (uint8_t *)buf, strlen(buf));
}

#if 0
void gpio_export(int num) {
  char buf[100];
  snprintf(buf, sizeof(buf), "%d", num);
  save_file2("/sys/class/gpio/export", buf);
}

void gpio_unexport(int num) {
  char buf[100];
  snprintf(buf, sizeof(buf), "%d", num);
  save_file2("/sys/class/gpio/unexport", buf);
}
#endif

void gpio_set_path(char *buf, size_t size, int num, char *attribute) {
  snprintf(buf, size, "/sys/class/gpio/gpio%d/%s", num, attribute);
}

void gpio_set_as_output(int num, int initial_value) {
  char path[PATH_MAX];
  gpio_set_path(path, sizeof(path), num, "direction");
  save_file2(path, initial_value ? "high" : "low");
}

void gpio_set_as_input(int num) {
  char path[PATH_MAX];
  gpio_set_path(path, sizeof(path), num, "direction");
  save_file2(path, "in");
}

void gpio_set_pullup(int num) {
  // This might not work.
  gpio_set_as_output(num, 1);
}


int gpio_open(int num) {
  char path[PATH_MAX];
  gpio_set_path(path, sizeof(path), num, "value");
  int fd = open(path, O_RDWR);
  if (fd < 0)
    perror_exit("open()");
  return fd;
}

int gpio_read(int fd) {
  check(fd >= 0, "gpio_read: fd < 0");
  if (lseek(fd, 0, SEEK_SET) < 0)
    perror_exit("lseek");
  char cc;
  int n = read(fd, &cc, 1);
  if (n != 1)
    perror_exit("read");
  if (cc == '0') {
    return 0;
  } else if (cc == '1') {
    return 1;
  } else
    error_exit("unexpected value read from gpio (0x%02X)", (int) cc);
  return -1;
}

void gpio_set(int fd, int value) {
  check(fd >= 0, "gpio_set: fd < 0");
  if (lseek(fd, 0, SEEK_SET) < 0)
    perror_exit("lseek");
  char cc = value ? '1' : '0';
  int n = write(fd, &cc, 1);
  if (n != 1)
    perror_exit("write");
}

//-----------------------------------------------------------------------------
void dbg_open(int swdio_gpio_num, int swclk_gpio_num)
{
  swdio_gpio.num = swdio_gpio_num;
  swclk_gpio.num = swclk_gpio_num;

  //gpio_export(swdio_gpio.num);
  gpio_set_as_input(swdio_gpio.num);
  swdio_gpio.fd = gpio_open(swdio_gpio.num);

  //gpio_export(swclk_gpio.num);
  gpio_set_as_output(swclk_gpio.num, 0);
  swclk_gpio.fd = gpio_open(swclk_gpio.num);
}

//-----------------------------------------------------------------------------
void dbg_close(void)
{
  if (swdio_gpio.fd >= 0) {
    close(swdio_gpio.fd);
    swdio_gpio.fd = -1;
  }
  if (swclk_gpio.fd >= 0) {
    close(swclk_gpio.fd);
    swclk_gpio.fd = -1;
  }
}

//-----------------------------------------------------------------------------
int dbg_get_report_size(void)
{
  return report_size;
}


//-----------------------------------------------------------------------------

#define DAP_CONFIG_ENABLE_SWD

#define DAP_CONFIG_DEFAULT_PORT DAP_PORT_SWD
#define DAP_CONFIG_DEFAULT_CLOCK 1000000 // Hz

#define DAP_CONFIG_PACKET_SIZE 64
#define DAP_CONFIG_PACKET_COUNT 1

#define DAP_CONFIG_VENDOR_STR "edbg"
#define DAP_CONFIG_PRODUCT_STR "sysfs CMSIS-DAP Adapter"
#define DAP_CONFIG_SER_NUM_STR "42"
#define DAP_CONFIG_FW_VER_STR "v0.1"
#define DAP_CONFIG_DEVICE_VENDOR_STR NULL
#define DAP_CONFIG_DEVICE_NAME_STR NULL

// A value at which dap_clock_test() produces 1 kHz output on the SWCLK pin
#define DAP_CONFIG_DELAY_CONSTANT      4700

// A threshold for switching to fast clock (no added delays)
// This is the frequency produced by dap_clock_test(1) on the SWCLK pin
#define DAP_CONFIG_FAST_CLOCK          3600000 // Hz

/*- Definitions -------------------------------------------------------------*/
enum
{
  ID_DAP_INFO               = 0x00,
  ID_DAP_LED                = 0x01,
  ID_DAP_CONNECT            = 0x02,
  ID_DAP_DISCONNECT         = 0x03,
  ID_DAP_TRANSFER_CONFIGURE = 0x04,
  ID_DAP_TRANSFER           = 0x05,
  ID_DAP_TRANSFER_BLOCK     = 0x06,
  ID_DAP_TRANSFER_ABORT     = 0x07,
  ID_DAP_WRITE_ABORT        = 0x08,
  ID_DAP_DELAY              = 0x09,
  ID_DAP_RESET_TARGET       = 0x0a,
  ID_DAP_SWJ_PINS           = 0x10,
  ID_DAP_SWJ_CLOCK          = 0x11,
  ID_DAP_SWJ_SEQUENCE       = 0x12,
  ID_DAP_SWD_CONFIGURE      = 0x13,
  ID_DAP_JTAG_SEQUENCE      = 0x14,
  ID_DAP_JTAG_CONFIGURE     = 0x15,
  ID_DAP_JTAG_IDCODE        = 0x16,
  ID_DAP_VENDOR_0           = 0x80,
  ID_DAP_VENDOR_31          = 0x9f,
  ID_DAP_INVALID            = 0xff,
};

enum
{
  DAP_INFO_VENDOR           = 0x01,
  DAP_INFO_PRODUCT          = 0x02,
  DAP_INFO_SER_NUM          = 0x03,
  DAP_INFO_FW_VER           = 0x04,
  DAP_INFO_DEVICE_VENDOR    = 0x05,
  DAP_INFO_DEVICE_NAME      = 0x06,
  DAP_INFO_CAPABILITIES     = 0xf0,
  DAP_INFO_PACKET_COUNT     = 0xfe,
  DAP_INFO_PACKET_SIZE      = 0xff,
};

enum
{
  DAP_TRANSFER_APnDP        = 1 << 0,
  DAP_TRANSFER_RnW          = 1 << 1,
  DAP_TRANSFER_A2           = 1 << 2,
  DAP_TRANSFER_A3           = 1 << 3,
  DAP_TRANSFER_MATCH_VALUE  = 1 << 4,
  DAP_TRANSFER_MATCH_MASK   = 1 << 5,
};

enum
{
  DAP_TRANSFER_INVALID      = 0,
  DAP_TRANSFER_OK           = 1 << 0,
  DAP_TRANSFER_WAIT         = 1 << 1,
  DAP_TRANSFER_FAULT        = 1 << 2,
  DAP_TRANSFER_ERROR        = 1 << 3,
  DAP_TRANSFER_MISMATCH     = 1 << 4,
};

enum
{
  DAP_PORT_DISABLED         = 0,
  DAP_PORT_AUTODETECT       = 0,
  DAP_PORT_SWD              = 1,
  DAP_PORT_JTAG             = 2,
};

enum
{
  DAP_SWJ_SWCLK_TCK         = 1 << 0,
  DAP_SWJ_SWDIO_TMS         = 1 << 1,
  DAP_SWJ_TDI               = 1 << 2,
  DAP_SWJ_TDO               = 1 << 3,
  DAP_SWJ_nTRST             = 1 << 5,
  DAP_SWJ_nRESET            = 1 << 7,
};

enum
{
  DAP_OK                    = 0x00,
  DAP_ERROR                 = 0xff,
};

enum
{
  SWD_DP_R_IDCODE           = 0x00,
  SWD_DP_W_ABORT            = 0x00,
  SWD_DP_R_CTRL_STAT        = 0x04,
  SWD_DP_W_CTRL_STAT        = 0x04, // When CTRLSEL == 0
  SWD_DP_W_WCR              = 0x04, // When CTRLSEL == 1
  SWD_DP_R_RESEND           = 0x08,
  SWD_DP_W_SELECT           = 0x08,
  SWD_DP_R_RDBUFF           = 0x0c,
};

/*- Constants ---------------------------------------------------------------*/
static const char * const dap_info_strings[] =
{
  [DAP_INFO_VENDOR]        = DAP_CONFIG_VENDOR_STR,
  [DAP_INFO_PRODUCT]       = DAP_CONFIG_PRODUCT_STR,
  [DAP_INFO_SER_NUM]       = DAP_CONFIG_SER_NUM_STR,
  [DAP_INFO_FW_VER]        = DAP_CONFIG_FW_VER_STR,
  [DAP_INFO_DEVICE_VENDOR] = DAP_CONFIG_DEVICE_VENDOR_STR,
  [DAP_INFO_DEVICE_NAME]   = DAP_CONFIG_DEVICE_NAME_STR,
};

/*- Variables ---------------------------------------------------------------*/
static int dap_port;
static bool dap_abort;
static uint32_t dap_match_mask;
static int dap_idle_cycles;
static int dap_retry_count;
static int dap_match_retry_count;
static int dap_clock_delay;

static void (*dap_swd_clock)(int);
static void (*dap_swd_write)(uint32_t, int);
static uint32_t (*dap_swd_read)(int);

#ifdef DAP_CONFIG_ENABLE_SWD
static int dap_swd_turnaround;
static bool dap_swd_data_phase;
#endif

/*- Implementations ---------------------------------------------------------*/

static inline void DAP_CONFIG_SWCLK_TCK_write(int value)
{
  gpio_set(swclk_gpio.fd, value);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_write(int value)
{
  gpio_set(swdio_gpio.fd, value);
}

#if 0
//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_TDO_write(int value)
{
  (void)value;
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_nTRST_write(int value)
{
  (void)value;
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_nRESET_write(int value)
{
  HAL_GPIO_nRESET_write(value);
}
#endif

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_SWCLK_TCK_read(void)
{
  return gpio_read(swclk_gpio.fd);
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_SWDIO_TMS_read(void)
{
  return gpio_read(swdio_gpio.fd);
}

#if 0
//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_TDI_read(void)
{
  return 0;
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_TDO_read(void)
{
  return 0;
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_nTRST_read(void)
{
  return 0;
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_nRESET_read(void)
{
  return HAL_GPIO_nRESET_read();
}
#endif

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_set(void)
{
  DAP_CONFIG_SWCLK_TCK_write(1);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_clr(void)
{
  DAP_CONFIG_SWCLK_TCK_write(0);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_in(void)
{
  gpio_set_as_input(swdio_gpio.num);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_out(void)
{
  gpio_set_as_output(swdio_gpio.num, 0);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SETUP(void)
{
  gpio_set_as_input(swclk_gpio.num);
  gpio_set_as_input(swdio_gpio.num);
  gpio_set_pullup(swclk_gpio.num);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_DISCONNECT(void)
{
  gpio_set_as_input(swclk_gpio.num);
  gpio_set_as_input(swdio_gpio.num);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_CONNECT_SWD(void)
{
  gpio_set_as_output(swdio_gpio.num, 1);
  gpio_set_as_output(swclk_gpio.num, 1);
}

//-----------------------------------------------------------------------------
#if 0
static inline void DAP_CONFIG_CONNECT_JTAG(void)
{
  HAL_GPIO_SWDIO_TMS_out();
  HAL_GPIO_SWDIO_TMS_set();

  HAL_GPIO_SWCLK_TCK_out();
  HAL_GPIO_SWCLK_TCK_set();

  HAL_GPIO_nRESET_out();
  HAL_GPIO_nRESET_set();
}
#endif

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_LED(int index, int state)
{
  (void) index; (void) state;
}


//-----------------------------------------------------------------------------
static inline void dap_delay_loop(int delay)
{
  while (--delay)
    asm("nop");
}

//-----------------------------------------------------------------------------
static void dap_delay_us(int delay)
{
  while (delay)
  {
    int del = (delay > 100000) ? 100000 : delay;

    dap_delay_loop((DAP_CONFIG_DELAY_CONSTANT * 2 * del) / 1000);

    delay -= del;
  }
}

//-----------------------------------------------------------------------------
static void dap_swd_clock_slow(int cycles)
{
  while (cycles--)
  {
    DAP_CONFIG_SWCLK_TCK_clr();
    dap_delay_loop(dap_clock_delay);
    DAP_CONFIG_SWCLK_TCK_set();
    dap_delay_loop(dap_clock_delay);
  }
}

//-----------------------------------------------------------------------------
static void dap_swd_write_slow(uint32_t value, int size)
{
  for (int i = 0; i < size; i++)
  {
    DAP_CONFIG_SWDIO_TMS_write(value & 1);
    DAP_CONFIG_SWCLK_TCK_clr();
    dap_delay_loop(dap_clock_delay);
    DAP_CONFIG_SWCLK_TCK_set();
    dap_delay_loop(dap_clock_delay);
    value >>= 1;
  }
}

//-----------------------------------------------------------------------------
static uint32_t dap_swd_read_slow(int size)
{
  uint32_t value = 0;

  for (int i = 0; i < size; i++)
  {
    DAP_CONFIG_SWCLK_TCK_clr();
    dap_delay_loop(dap_clock_delay);
    value |= ((uint32_t)DAP_CONFIG_SWDIO_TMS_read() << i);
    DAP_CONFIG_SWCLK_TCK_set();
    dap_delay_loop(dap_clock_delay);
  }

  return value;
}

//-----------------------------------------------------------------------------
static void dap_swd_clock_fast(int cycles)
{
  while (cycles--)
  {
    DAP_CONFIG_SWCLK_TCK_clr();
    DAP_CONFIG_SWCLK_TCK_set();
  }
}

//-----------------------------------------------------------------------------
static void dap_swd_write_fast(uint32_t value, int size)
{
  for (int i = 0; i < size; i++)
  {
    DAP_CONFIG_SWDIO_TMS_write(value & 1);
    DAP_CONFIG_SWCLK_TCK_clr();
    value >>= 1;
    DAP_CONFIG_SWCLK_TCK_set();
  }
}

//-----------------------------------------------------------------------------
static uint32_t dap_swd_read_fast(int size)
{
  uint32_t value = 0;
  uint32_t bit;

  for (int i = 0; i < size; i++)
  {
    DAP_CONFIG_SWCLK_TCK_clr();
    bit = DAP_CONFIG_SWDIO_TMS_read();
    DAP_CONFIG_SWCLK_TCK_set();
    value |= (bit << i);
  }

  return value;
}

//-----------------------------------------------------------------------------
static void dap_setup_clock(int freq)
{
  if (freq > DAP_CONFIG_FAST_CLOCK)
  {
    dap_clock_delay = 1;
    dap_swd_clock = dap_swd_clock_fast;
    dap_swd_write = dap_swd_write_fast;
    dap_swd_read = dap_swd_read_fast;
  }
  else
  {
    dap_clock_delay = (DAP_CONFIG_DELAY_CONSTANT * 1000) / freq;
    dap_swd_clock = dap_swd_clock_slow;
    dap_swd_write = dap_swd_write_slow;
    dap_swd_read = dap_swd_read_slow;
  }
}

//-----------------------------------------------------------------------------
static inline uint32_t dap_parity(uint32_t value)
{
  value ^= value >> 16;
  value ^= value >> 8;
  value ^= value >> 4;
  value &= 0x0f;

  return (0x6996 >> value) & 1;
}

//-----------------------------------------------------------------------------
static int dap_swd_operation(int req, uint32_t *data)
{
  uint32_t value;
  int ack = 0;

  dap_swd_write(0x81 | (dap_parity(req) << 5) | (req << 1), 8);

  DAP_CONFIG_SWDIO_TMS_in();

  dap_swd_clock(dap_swd_turnaround);

  ack = dap_swd_read(3);

  if (DAP_TRANSFER_OK == ack)
  {
    if (req & DAP_TRANSFER_RnW)
    {
      value = dap_swd_read(32);

      if (dap_parity(value) != dap_swd_read(1))
        ack = DAP_TRANSFER_ERROR;

      if (data)
        *data = value;

      dap_swd_clock(dap_swd_turnaround);

      DAP_CONFIG_SWDIO_TMS_out();
    }
    else
    {
      dap_swd_clock(dap_swd_turnaround);

      DAP_CONFIG_SWDIO_TMS_out();

      dap_swd_write(*data, 32);
      dap_swd_write(dap_parity(*data), 1);
    }

    DAP_CONFIG_SWDIO_TMS_write(0);
    dap_swd_clock(dap_idle_cycles);
  }

  else if (DAP_TRANSFER_WAIT == ack || DAP_TRANSFER_FAULT == ack)
  {
    if (dap_swd_data_phase && (req & DAP_TRANSFER_RnW))
      dap_swd_clock(32 + 1);

    dap_swd_clock(dap_swd_turnaround);

    DAP_CONFIG_SWDIO_TMS_out();

    if (dap_swd_data_phase && (0 == (req & DAP_TRANSFER_RnW)))
    {
      DAP_CONFIG_SWDIO_TMS_write(0);
      dap_swd_clock(32 + 1);
    }
  }

  else
  {
    dap_swd_clock(dap_swd_turnaround + 32 + 1);
  }

  DAP_CONFIG_SWDIO_TMS_write(1);

  return ack;
}

//-----------------------------------------------------------------------------
static int dap_swd_transfer_word(int req, uint32_t *data)
{
  int ack;

  req &= (DAP_TRANSFER_APnDP | DAP_TRANSFER_RnW | DAP_TRANSFER_A2 | DAP_TRANSFER_A3);

  for (int i = 0; i < dap_retry_count; i++)
  {
    ack = dap_swd_operation(req, data);

    if (DAP_TRANSFER_WAIT != ack || dap_abort)
      break;
  }

  return ack;
}

//-----------------------------------------------------------------------------
static void dap_swd_transfer(uint8_t *req, uint8_t *resp)
{
  int req_count, resp_count, request, ack;
  uint8_t *req_data, *resp_data;
  bool posted_read, verify_write;
  uint32_t data, match_value;

  req_count = req[1];
  req_data = &req[2];

  ack = DAP_TRANSFER_INVALID;
  resp_count = 0;
  resp_data = &resp[2];

  posted_read = false;
  verify_write = false;

  while (req_count && !dap_abort)
  {
    verify_write = false;
    request = req_data[0];
    req_data++;

    if (posted_read)
    {
      if ((request & DAP_TRANSFER_APnDP) && (request & DAP_TRANSFER_RnW))
      {
        ack = dap_swd_transfer_word(request, &data);
      }
      else
      {
        ack = dap_swd_transfer_word(SWD_DP_R_RDBUFF | DAP_TRANSFER_RnW, &data);
        posted_read = false;
      }

      if (ack != DAP_TRANSFER_OK)
        break;

      resp_data[0] = data;
      resp_data[1] = data >> 8;
      resp_data[2] = data >> 16;
      resp_data[3] = data >> 24;
      resp_data += 4;
    }

    if (request & DAP_TRANSFER_RnW)
    {
      if (request & DAP_TRANSFER_MATCH_VALUE)
      {
        match_value = ((uint32_t)req_data[3] << 24) | ((uint32_t)req_data[2] << 16) |
                      ((uint32_t)req_data[1] << 8) | req_data[0];
        req_data += 4;

        for (int i = 0; i < dap_match_retry_count; i++)
        {
          ack = dap_swd_transfer_word(request, &data);

          if (DAP_TRANSFER_OK != ack || (data & dap_match_mask) == match_value || dap_abort)
            break;
        };

        if ((data & dap_match_mask) != match_value)
          ack |= DAP_TRANSFER_MISMATCH;

        if (ack != DAP_TRANSFER_OK)
          break;
      }
      else
      {
        if (request & DAP_TRANSFER_APnDP)
        {
          if (!posted_read)
          {
            ack = dap_swd_transfer_word(request, NULL);

            if (ack != DAP_TRANSFER_OK)
              break;

            posted_read = true;
          }
        }
        else
        {
          ack = dap_swd_transfer_word(request, &data);

          if (DAP_TRANSFER_OK != ack)
            break;

          resp_data[0] = data;
          resp_data[1] = data >> 8;
          resp_data[2] = data >> 16;
          resp_data[3] = data >> 24;
          resp_data += 4;
        }
      }
    }
    else
    {
      data = ((uint32_t)req_data[3] << 24) | ((uint32_t)req_data[2] << 16) |
             ((uint32_t)req_data[1] << 8) | req_data[0];
      req_data += 4;

      if (request & DAP_TRANSFER_MATCH_MASK)
      {
        ack = DAP_TRANSFER_OK;
        dap_match_mask = data;
      }
      else
      {
        ack = dap_swd_transfer_word(request, &data);

        if (ack != DAP_TRANSFER_OK)
          break;

        verify_write = true;
      }
    }

    req_count--;
    resp_count++;
  }

  if (DAP_TRANSFER_OK == ack)
  {
    if (posted_read)
    {
      ack = dap_swd_transfer_word(SWD_DP_R_RDBUFF | DAP_TRANSFER_RnW, &data);

      // Save data regardless of the ACK status, at this point it does not matter
      resp_data[0] = data;
      resp_data[1] = data >> 8;
      resp_data[2] = data >> 16;
      resp_data[3] = data >> 24;
    }
    else if (verify_write)
    {
      ack = dap_swd_transfer_word(SWD_DP_R_RDBUFF | DAP_TRANSFER_RnW, NULL);
    }
  }

  resp[0] = resp_count;
  resp[1] = ack;
}

//-----------------------------------------------------------------------------
static void dap_swd_transfer_block(uint8_t *req, uint8_t *resp)
{
  int req_count, resp_count, request, ack;
  uint8_t *req_data, *resp_data;
  uint32_t data;

  req_count = ((int)req[2] << 8) | req[1];
  request = req[3];
  req_data = &req[4];

  ack = DAP_TRANSFER_INVALID;
  resp_count = 0;
  resp_data = &resp[3];

  resp[0] = 0;
  resp[1] = 0;
  resp[2] = DAP_TRANSFER_INVALID;

  if (0 == req_count)
    return;

  if (request & DAP_TRANSFER_RnW)
  {
    int transfers = (request & DAP_TRANSFER_APnDP) ? (req_count + 1) : req_count;

    for (int i = 0; i < transfers; i++)
    {
      if (i == req_count) // This will only happen for AP transfers
        request = SWD_DP_R_RDBUFF | DAP_TRANSFER_RnW;

      ack = dap_swd_transfer_word(request, &data);

      if (DAP_TRANSFER_OK != ack)
        break;

      if ((0 == i) && (request & DAP_TRANSFER_APnDP))
        continue;

      resp_data[0] = data;
      resp_data[1] = data >> 8;
      resp_data[2] = data >> 16;
      resp_data[3] = data >> 24;
      resp_data += 4;
      resp_count++;
    }
  }
  else
  {
    for (int i = 0; i < req_count; i++)
    {
      data = ((uint32_t)req_data[3] << 24) | ((uint32_t)req_data[2] << 16) |
             ((uint32_t)req_data[1] << 8) | ((uint32_t)req_data[0] << 0);
      req_data += 4;

      ack = dap_swd_transfer_word(request, &data);

      if (DAP_TRANSFER_OK != ack)
        break;

      resp_count++;
    }

    if (DAP_TRANSFER_OK == ack)
      ack = dap_swd_transfer_word(SWD_DP_R_RDBUFF | DAP_TRANSFER_RnW, NULL);
  }

  resp[0] = resp_count;
  resp[1] = resp_count >> 8;
  resp[2] = ack;
}

//-----------------------------------------------------------------------------
static void dap_info(uint8_t *req, uint8_t *resp)
{
  int index = req[0];

  if (DAP_INFO_VENDOR <= index && index <= DAP_INFO_DEVICE_NAME)
  {
    if (dap_info_strings[index])
    {
      resp[0] = strlen(dap_info_strings[index]) + 1;
      strcpy((char *)&resp[1], dap_info_strings[index]);
    }
    else
    {
      resp[0] = 0;
    }
  }
  else if (DAP_INFO_CAPABILITIES == index)
  {
    resp[0] = 1;
    resp[1] = 0;

#ifdef DAP_CONFIG_ENABLE_SWD
    resp[1] |= DAP_PORT_SWD;
#endif
#ifdef DAP_CONFIG_ENABLE_JTAG
    resp[1] |= DAP_PORT_JTAG;
#endif
  }
  else if (DAP_INFO_PACKET_COUNT == index)
  {
    resp[0] = 1;
    resp[1] = DAP_CONFIG_PACKET_COUNT;
  }
  else if (DAP_INFO_PACKET_SIZE == index)
  {
    resp[0] = 2;
    resp[1] = DAP_CONFIG_PACKET_SIZE;
    resp[2] = DAP_CONFIG_PACKET_SIZE >> 8;
  }
}

//-----------------------------------------------------------------------------
static void dap_led(uint8_t *req, uint8_t *resp)
{
  int index = req[0];
  int state = req[1];

  DAP_CONFIG_LED(index, state);

  resp[0] = DAP_OK;
}

//-----------------------------------------------------------------------------
static void dap_connect(uint8_t *req, uint8_t *resp)
{
  int port = req[0];

  if (DAP_PORT_AUTODETECT == port)
    port = DAP_CONFIG_DEFAULT_PORT;

  dap_port = DAP_PORT_DISABLED;

#ifdef DAP_CONFIG_ENABLE_SWD
  if (DAP_PORT_SWD == port)
  {
    DAP_CONFIG_CONNECT_SWD();
    dap_port = DAP_PORT_SWD;
  }
#endif

#ifdef DAP_CONFIG_ENABLE_JTAG
  if (DAP_PORT_JTAG == port)
  {
    DAP_CONFIG_CONNECT_JTAG();
    dap_port = DAP_PORT_JTAG;
  }
#endif

  resp[0] = dap_port;
}

//-----------------------------------------------------------------------------
static void dap_disconnect(uint8_t *req, uint8_t *resp)
{
  DAP_CONFIG_DISCONNECT();

  dap_port = DAP_PORT_DISABLED;

  resp[0] = DAP_OK;

  (void)req;
}

//-----------------------------------------------------------------------------
static void dap_transfer_configure(uint8_t *req, uint8_t *resp)
{
  dap_idle_cycles = req[0];
  dap_retry_count = ((int)req[2] << 8) | req[1];
  dap_match_retry_count = ((int)req[4] << 8) | req[3];

  resp[0] = DAP_OK;
}

//-----------------------------------------------------------------------------
static void dap_transfer(uint8_t *req, uint8_t *resp)
{
  resp[0] = 0;
  resp[1] = DAP_TRANSFER_INVALID;

#ifdef DAP_CONFIG_ENABLE_SWD
  if (DAP_PORT_SWD == dap_port)
    dap_swd_transfer(req, resp);
#endif

#ifdef DAP_CONFIG_ENABLE_JTAG
  if (DAP_PORT_JTAG == dap_port)
    dap_jtag_transfer(req, resp);
#endif
}

//-----------------------------------------------------------------------------
static void dap_transfer_block(uint8_t *req, uint8_t *resp)
{
  resp[0] = 0;
  resp[1] = 0;
  resp[2] = DAP_TRANSFER_INVALID;

#ifdef DAP_CONFIG_ENABLE_SWD
  if (DAP_PORT_SWD == dap_port)
    dap_swd_transfer_block(req, resp);
#endif

#ifdef DAP_CONFIG_ENABLE_JTAG
  if (DAP_PORT_JTAG == dap_port)
    dap_jtag_transfer_block(req, resp);
#endif
}

//-----------------------------------------------------------------------------
static void dap_transfer_abort(uint8_t *req, uint8_t *resp)
{
  // This request is handled outside of the normal queue
  // TODO: verify entire transfer abort mechanism
  resp[0] = DAP_OK;
  (void)req;
}

//-----------------------------------------------------------------------------
static void dap_write_abort(uint8_t *req, uint8_t *resp)
{
#ifdef DAP_CONFIG_ENABLE_SWD
  if (DAP_PORT_SWD == dap_port)
  {
    uint32_t data;

    data = ((uint32_t)req[4] << 24) | ((uint32_t)req[3] << 16) |
           ((uint32_t)req[2] << 8) | ((uint32_t)req[1] << 0);

    dap_swd_transfer_word(SWD_DP_W_ABORT, &data);

    resp[0] = DAP_OK;
  }
#endif

#ifdef DAP_CONFIG_ENABLE_JTAG
  if (DAP_PORT_JTAG == dap_port)
  {
    // TODO: implement
    resp[0] = DAP_OK;
  }
#endif
}

//-----------------------------------------------------------------------------
static void dap_delay(uint8_t *req, uint8_t *resp)
{
  int delay;

  delay = ((int)req[1] << 8) | req[0];

  dap_delay_us(delay);

  resp[0] = DAP_OK;
}

//-----------------------------------------------------------------------------
static void dap_reset_target(uint8_t *req, uint8_t *resp)
{
  resp[0] = DAP_OK;
#ifdef DAP_CONFIG_RESET_TARGET_FN
  resp[1] = 1;
  DAP_CONFIG_RESET_TARGET_FN();
#endif
  (void)req;
}

//-----------------------------------------------------------------------------
static void dap_swj_pins(uint8_t *req, uint8_t *resp)
{
  int value = req[0];
  int select = req[1];
  int wait;

  wait = ((int)req[5] << 24) | ((int)req[4] << 16) | ((int)req[3] << 8) | req[2];

  if (select & DAP_SWJ_SWCLK_TCK)
    DAP_CONFIG_SWCLK_TCK_write(value & DAP_SWJ_SWCLK_TCK);

  if (select & DAP_SWJ_SWDIO_TMS)
    DAP_CONFIG_SWDIO_TMS_write(value & DAP_SWJ_SWDIO_TMS);

#if 0
  if (select & DAP_SWJ_TDI)
    DAP_CONFIG_TDO_write(value & DAP_SWJ_TDI);

  if (select & DAP_SWJ_nTRST)
    DAP_CONFIG_nTRST_write(value & DAP_SWJ_nTRST);

  if (select & DAP_SWJ_nRESET)
    DAP_CONFIG_nRESET_write(value & DAP_SWJ_nRESET);
#endif

  dap_delay_us(wait * 1000);

  value =
      ((DAP_CONFIG_SWCLK_TCK_read() ? DAP_SWJ_SWCLK_TCK : 0)
       | (DAP_CONFIG_SWDIO_TMS_read() ? DAP_SWJ_SWDIO_TMS : 0)
#if 0
       | (DAP_CONFIG_TDI_read()       ? DAP_SWJ_TDI       : 0)
       | (DAP_CONFIG_TDO_read()       ? DAP_SWJ_TDO       : 0)
       | (DAP_CONFIG_nTRST_read()     ? DAP_SWJ_nTRST     : 0)
       | (DAP_CONFIG_nRESET_read()    ? DAP_SWJ_nRESET    : 0)
#endif
       );

  resp[0] = value;
}

//-----------------------------------------------------------------------------
static void dap_swj_clock(uint8_t *req, uint8_t *resp)
{
  uint32_t freq;

  freq = ((uint32_t)req[3] << 24) | ((uint32_t)req[2] << 16) |
         ((uint32_t)req[1] << 8) | req[0];

  dap_setup_clock(freq);

  resp[0] = DAP_OK;
}

//-----------------------------------------------------------------------------
static void dap_swj_sequence(uint8_t *req, uint8_t *resp)
{
  int size = req[0];
  uint8_t *data = &req[1];
  int offset = 0;

  while (size)
  {
    int sz = (size > 8) ? 8 : size;

    dap_swd_write(data[offset], sz);

    size -= sz;
    offset++;
  }

  resp[0] = DAP_OK;
}

//-----------------------------------------------------------------------------
static void dap_swd_configure(uint8_t *req, uint8_t *resp)
{
#ifdef DAP_CONFIG_ENABLE_SWD
  uint8_t data = req[0];

  dap_swd_turnaround  = (data & 3) + 1;
  dap_swd_data_phase  = (data & 4) ? 1 : 0;

  resp[0] = DAP_OK;
#endif

  (void)req;
  (void)resp;
}

//-----------------------------------------------------------------------------
static void dap_jtag_sequence(uint8_t *req, uint8_t *resp)
{
#ifdef DAP_CONFIG_ENABLE_JTAG
  // TODO: implement
  resp[0] = DAP_OK;
#endif

  (void)req;
  (void)resp;
}

//-----------------------------------------------------------------------------
static void dap_jtag_configure(uint8_t *req, uint8_t *resp)
{
#ifdef DAP_CONFIG_ENABLE_JTAG
  // TODO: implement
  resp[0] = DAP_OK;
#endif

  (void)req;
  (void)resp;
}

//-----------------------------------------------------------------------------
static void dap_jtag_idcode(uint8_t *req, uint8_t *resp)
{
#ifdef DAP_CONFIG_ENABLE_JTAG
  // TODO: implement
  resp[0] = DAP_OK;
#endif

  (void)req;
  (void)resp;
}

//-----------------------------------------------------------------------------
void dap_init(void)
{
  dap_port  = 0;
  dap_abort = false;
  dap_match_mask = 0;
  dap_idle_cycles = 0;
  dap_retry_count = 100;
  dap_match_retry_count = 100;

#ifdef DAP_CONFIG_ENABLE_SWD
  dap_swd_turnaround = 1;
  dap_swd_data_phase = false;
#endif

  dap_setup_clock(DAP_CONFIG_DEFAULT_CLOCK);

  DAP_CONFIG_SETUP();
}

//-----------------------------------------------------------------------------
void dap_filter_request(uint8_t *req)
{
  int cmd = req[0];

  if (ID_DAP_TRANSFER_ABORT == cmd)
  {
    dap_abort = true;
  }
}

//-----------------------------------------------------------------------------
void dap_process_request(uint8_t *req, uint8_t *resp)
{
  const struct
  {
    int    cmd;
    void   (*handler)(uint8_t *, uint8_t *);
  } handlers[] =
  {
    { ID_DAP_INFO,                      dap_info },
    { ID_DAP_LED,                       dap_led },
    { ID_DAP_CONNECT,                   dap_connect },
    { ID_DAP_DISCONNECT,                dap_disconnect },
    { ID_DAP_TRANSFER_CONFIGURE,        dap_transfer_configure },
    { ID_DAP_TRANSFER,                  dap_transfer },
    { ID_DAP_TRANSFER_BLOCK,            dap_transfer_block },
    { ID_DAP_TRANSFER_ABORT,            dap_transfer_abort },
    { ID_DAP_WRITE_ABORT,               dap_write_abort },
    { ID_DAP_DELAY,                     dap_delay },
    { ID_DAP_RESET_TARGET,              dap_reset_target },
    { ID_DAP_SWJ_PINS,                  dap_swj_pins },
    { ID_DAP_SWJ_CLOCK,                 dap_swj_clock },
    { ID_DAP_SWJ_SEQUENCE,              dap_swj_sequence },
    { ID_DAP_SWD_CONFIGURE,             dap_swd_configure },
    { ID_DAP_JTAG_SEQUENCE,             dap_jtag_sequence },
    { ID_DAP_JTAG_CONFIGURE,            dap_jtag_configure },
    { ID_DAP_JTAG_IDCODE,               dap_jtag_idcode },
    { -1, NULL },
  };
  int cmd = req[0];

  memset(resp, 0, DAP_CONFIG_PACKET_SIZE);

  dap_abort = false;

  resp[0] = cmd;
  resp[1] = DAP_ERROR;

  for (int i = 0; -1 != handlers[i].cmd; i++)
  {
    if (cmd == handlers[i].cmd)
    {
      handlers[i].handler(&req[1], &resp[1]);
      return;
    }
  }

  if (ID_DAP_VENDOR_0 <= cmd && cmd <= ID_DAP_VENDOR_31)
    return;

  resp[0] = ID_DAP_INVALID;
}

//-----------------------------------------------------------------------------
void dap_clock_test(int delay)
{
  DAP_CONFIG_CONNECT_SWD();

  if (delay)
  {
    while (1)
    {
      DAP_CONFIG_SWCLK_TCK_clr();
      dap_delay_loop(delay);
      DAP_CONFIG_SWCLK_TCK_set();
      dap_delay_loop(delay);
    }
  }
  else
  {
    while (1)
    {
      DAP_CONFIG_SWCLK_TCK_clr();
      DAP_CONFIG_SWCLK_TCK_set();
    }
  }
}

//-----------------------------------------------------------------------------
int dbg_dap_cmd(uint8_t *data, int size, int rsize)
{
#if 0
  char cmd = data[0];
  int res;

  memset(hid_buffer, 0xff, report_size + 1);
  memcpy(&hid_buffer[1], data, rsize);

  res = write(debugger_fd, hid_buffer, report_size + 1);
  if (res < 0)
    perror_exit("debugger write()");

  res = read(debugger_fd, hid_buffer, report_size + 1);
  if (res < 0)
    perror_exit("debugger read()");
  check(res, "empty response received");

  check(hid_buffer[0] == cmd, "invalid response received");

  res--;
  memcpy(data, &hid_buffer[1], (size < res) ? size : res);

  return res;
#endif
  memset(req_buffer, 0xff, sizeof(req_buffer));
  memset(res_buffer, 0xff, sizeof(res_buffer));
  memcpy(req_buffer, data, rsize);
  dap_process_request(req_buffer, res_buffer);
  memcpy(data, res_buffer, size);
  return 0;
}
