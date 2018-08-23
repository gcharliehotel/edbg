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
#include "dap_common.h"

#define DAP_CONFIG_ENABLE_SWD

#define DAP_CONFIG_DEFAULT_PORT DAP_PORT_SWD
#define DAP_CONFIG_DEFAULT_CLOCK 1000000 // Hz

#define DAP_CONFIG_PACKET_SIZE 128
#define DAP_CONFIG_PACKET_COUNT 1

#define DAP_CONFIG_VENDOR_STR "edbg"
#define DAP_CONFIG_PRODUCT_STR "sysfs CMSIS-DAP Adapter"
#define DAP_CONFIG_SER_NUM_STR "42"
#define DAP_CONFIG_FW_VER_STR "v0.1"
#define DAP_CONFIG_DEVICE_VENDOR_STR NULL
#define DAP_CONFIG_DEVICE_NAME_STR NULL

// A value at which dap_clock_test() produces 1 kHz output on the SWCLK pin
#define DAP_CONFIG_DELAY_CONSTANT      500

// A threshold for switching to fast clock (no added delays)
// This is the frequency produced by dap_clock_test(1) on the SWCLK pin
#define DAP_CONFIG_FAST_CLOCK         10000000 // Hz


/*- Types -------------------------------------------------------------------*/
typedef struct
{
  int num;
  bool invert;

  int value_fd;
  int direction_fd;
  int dir;
} gpio_t;

#define GPIO_DIR_UNDEFINED -1
#define GPIO_DIR_IN 0
#define GPIO_DIR_IN_PULLUP 1
#define GPIO_DIR_OUT 2

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
static gpio_t gpio_swdio;
static gpio_t gpio_swclk;
static gpio_t gpio_nreset;
static uint8_t req_buffer[1024];
static uint8_t res_buffer[1024];
static int report_size = sizeof(res_buffer); // must be same as above.
static int verbose_commands = 0;

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

/*- Forward */

void dap_init(void);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------

void gpio_init(gpio_t *gpio, gpio_config_t *config) {
  gpio->num = config->num;
  gpio->invert = config->invert;
  gpio->value_fd = -1;
  gpio->direction_fd = -1;
  gpio->dir = GPIO_DIR_UNDEFINED;
}

void gpio_set_path(gpio_t *gpio, char *buf, size_t size, char *attribute) {
  snprintf(buf, size, "/sys/class/gpio/gpio%d/%s", gpio->num, attribute);
}

void gpio_open(gpio_t *gpio) {
  check(gpio->value_fd == -1, "attempt to re-open gpio\n");
  char path[PATH_MAX];
  gpio_set_path(gpio, path, sizeof(path), "direction");
  int fd = open(path, O_RDWR);
  if (fd < 0)
    perror_exit("open()");
  gpio->direction_fd = fd;

  gpio_set_path(gpio, path, sizeof(path), "value");
  fd = open(path, O_RDWR);
  if (fd < 0)
    perror_exit("open()");
  gpio->value_fd = fd;
}

void gpio_close(gpio_t *gpio) {
  if (gpio->direction_fd >= 0) {
    close(gpio->direction_fd);
    gpio->direction_fd = -1;
  }
  if (gpio->value_fd >= 0) {
    close(gpio->value_fd);
    gpio->value_fd = -1;
  }
}

void gpio_set_direction(gpio_t *gpio, char *direction) {
  check(gpio->direction_fd >= 0, "gpio_set_direction: fd < 0");
  if (lseek(gpio->direction_fd, 0, SEEK_SET) < 0)
    perror_exit("lseek");
  int n = strlen(direction);
  int rc = write(gpio->direction_fd, direction, n);
  if (n != rc)
    perror_exit("failed to set gpio direction");
}

void _gpio_set_value(gpio_t *gpio, bool value) {
  check(gpio->value_fd >= 0, "gpio_set: fd < 0");
  if (lseek(gpio->value_fd, 0, SEEK_SET) < 0)
    perror_exit("lseek");
  char cc = value ? '1' : '0';
  int n = write(gpio->value_fd, &cc, 1);
  if (n != 1)
    error_exit("failed to set gpio %d to %d\n", gpio->num, value);
}

void gpio_make_output(gpio_t *gpio, int initial_value) {
  gpio->dir = GPIO_DIR_OUT;
  gpio_set_direction(gpio, initial_value ? "high" : "low");
}

void gpio_make_input(gpio_t *gpio) {
  gpio->dir = GPIO_DIR_IN;
  gpio_set_direction(gpio, "in");
}

void gpio_make_input_with_pullup(gpio_t *gpio) {
  // This doesn't activate any pullup -- that can't be done with the
  // sysfs interface.  Rather this makes the gpio work on GPIO port
  // that has a pullup.  That pullup must be established elsewhere
  // (device tree gpio config, external resistor, etc.).
  gpio->dir = GPIO_DIR_IN_PULLUP;
  gpio_set_direction(gpio, "in");
}

bool _gpio_read(gpio_t *gpio) {
  check(gpio->value_fd >= 0, "gpio_read: fd < 0");
  if (lseek(gpio->value_fd, 0, SEEK_SET) < 0)
    perror_exit("lseek");
  char cc;
  int n = read(gpio->value_fd, &cc, 1);
  if (n != 1)
    perror_exit("read");
  if (cc == '0') {
    return false;
  } else if (cc == '1') {
    return true;
  } else {
    error_exit("unexpected value read from gpio (0x%02X)", (int) cc);
    return false;
  }
}

bool gpio_read(gpio_t *gpio) {
  bool value = _gpio_read(gpio);
  if (gpio->invert) {
    return !value;
  } else {
    return value;
  }
}

void gpio_write(gpio_t *gpio, bool value) {
  if (gpio->invert)
    value = !value;
  switch (gpio->dir) {
  case GPIO_DIR_IN:
    // The code writes to inputs.  Not sure why.  It doesn't seem to matter, so
    // we (quietly) ignore these calls.
    // warning("gpio %d: cannot write %d to input", gpio->num, value);
    break;
  case GPIO_DIR_IN_PULLUP:
    if (value) {
      gpio_set_direction(gpio, "in");
    } else {
      gpio_set_direction(gpio, "low");
    }
    break;
  case GPIO_DIR_OUT:
    _gpio_set_value(gpio, value);
    break;
  default:
    error_exit("gpio %d: undefined direction", gpio->num);
  }
}

//-----------------------------------------------------------------------------

#define DEFINE_GPIO(name, gpio)                               \
  static inline void HAL_GPIO_##name##_set(void) {            \
    gpio_write(&gpio, 1);                                     \
  }                                                           \
  static inline void HAL_GPIO_##name##_clr(void) {            \
    gpio_write(&gpio, 0);                                     \
  }                                                           \
  static inline void HAL_GPIO_##name##_write(int value) {     \
    gpio_write(&gpio, !!value);                               \
  }                                                           \
  static inline int HAL_GPIO_##name##_read() {                \
    return gpio_read(&gpio);                                  \
  }                                                           \
  static inline void HAL_GPIO_##name##_in(void) {             \
    gpio_make_input(&gpio);                                   \
  }                                                           \
  static inline void HAL_GPIO_##name##_out(void) {            \
    gpio_make_output(&gpio, 0);  /* what inital value? */     \
  }                                                           \
  static inline void HAL_GPIO_##name##_pullup(void) {         \
    gpio_make_input_with_pullup(&gpio);                       \
  }

DEFINE_GPIO(SWDIO_TMS, gpio_swdio)
DEFINE_GPIO(SWCLK_TCK, gpio_swclk)
DEFINE_GPIO(nRESET, gpio_nreset)

#undef DEFINE_GPIO

//-----------------------------------------------------------------------------
void dbg_open(gpio_config_t *swdio_gpio_config,
              gpio_config_t *swclk_gpio_config,
              gpio_config_t *nreset_gpio_config)
{
  if (swdio_gpio_config->invert)
    error_exit("Inverting SWDIO is not supported.");

  gpio_init(&gpio_swdio, swdio_gpio_config);
  gpio_open(&gpio_swdio);

  gpio_init(&gpio_swclk, swclk_gpio_config);
  gpio_open(&gpio_swclk);

  gpio_init(&gpio_nreset, nreset_gpio_config);
  gpio_open(&gpio_nreset);

  dap_init();
}

//-----------------------------------------------------------------------------
void dbg_close(void)
{
  gpio_close(&gpio_swdio);
  gpio_close(&gpio_swclk);
  gpio_close(&gpio_nreset);
}

//-----------------------------------------------------------------------------
int dbg_get_report_size(void)
{
  return report_size;
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_write(int value)
{
  HAL_GPIO_SWCLK_TCK_write(value);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_write(int value)
{
  HAL_GPIO_SWDIO_TMS_write(value);
}

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

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_SWCLK_TCK_read(void)
{
  return HAL_GPIO_SWCLK_TCK_read();
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_SWDIO_TMS_read(void)
{
  return HAL_GPIO_SWDIO_TMS_read();
}

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

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_set(void)
{
  HAL_GPIO_SWCLK_TCK_set();
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_clr(void)
{
  HAL_GPIO_SWCLK_TCK_clr();
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_in(void)
{
  HAL_GPIO_SWDIO_TMS_in();
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_out(void)
{
  HAL_GPIO_SWDIO_TMS_out();
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SETUP(void)
{
  HAL_GPIO_SWCLK_TCK_in();
  HAL_GPIO_SWDIO_TMS_in();
  HAL_GPIO_nRESET_in();

  HAL_GPIO_SWDIO_TMS_pullup();
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_DISCONNECT(void)
{
  HAL_GPIO_SWCLK_TCK_in();
  HAL_GPIO_SWDIO_TMS_in();
  HAL_GPIO_nRESET_in();
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_CONNECT_SWD(void)
{
  HAL_GPIO_SWDIO_TMS_out();
  HAL_GPIO_SWDIO_TMS_set();

  HAL_GPIO_SWCLK_TCK_out();
  HAL_GPIO_SWCLK_TCK_set();

  HAL_GPIO_nRESET_out();
  HAL_GPIO_nRESET_set();
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_CONNECT_JTAG(void)
{
  HAL_GPIO_SWDIO_TMS_out();
  HAL_GPIO_SWDIO_TMS_set();

  HAL_GPIO_SWCLK_TCK_out();
  HAL_GPIO_SWCLK_TCK_set();

  HAL_GPIO_nRESET_out();
  HAL_GPIO_nRESET_set();
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_LED(int index, int state)
{
  (void) index; (void) state;
}


//-----------------------------------------------------------------------------
static inline void dap_delay_loop(int delay)
{
  usleep(delay);
}

//-----------------------------------------------------------------------------
static void dap_delay_us(int delay)
{
  usleep(delay);
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
    verbose("enabling fast clock\n");
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
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

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
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

  int index = req[0];
  int state = req[1];

  DAP_CONFIG_LED(index, state);

  resp[0] = DAP_OK;
}

//-----------------------------------------------------------------------------
static void dap_connect(uint8_t *req, uint8_t *resp)
{
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

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
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

  DAP_CONFIG_DISCONNECT();

  dap_port = DAP_PORT_DISABLED;

  resp[0] = DAP_OK;

  (void)req;
}

//-----------------------------------------------------------------------------
static void dap_transfer_configure(uint8_t *req, uint8_t *resp)
{
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

  dap_idle_cycles = req[0];
  dap_retry_count = ((int)req[2] << 8) | req[1];
  dap_match_retry_count = ((int)req[4] << 8) | req[3];

  resp[0] = DAP_OK;
}

//-----------------------------------------------------------------------------
static void dap_transfer(uint8_t *req, uint8_t *resp)
{
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

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
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

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
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

  // This request is handled outside of the normal queue
  // TODO: verify entire transfer abort mechanism
  resp[0] = DAP_OK;
  (void)req;
}

//-----------------------------------------------------------------------------
static void dap_write_abort(uint8_t *req, uint8_t *resp)
{
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

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
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

  int delay;

  delay = ((int)req[1] << 8) | req[0];

  dap_delay_us(delay);

  resp[0] = DAP_OK;
}

//-----------------------------------------------------------------------------
static void dap_reset_target(uint8_t *req, uint8_t *resp)
{
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

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
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

  int value = req[0];
  int select = req[1];
  int wait;

  wait = ((int)req[5] << 24) | ((int)req[4] << 16) | ((int)req[3] << 8) | req[2];

  if (select & DAP_SWJ_SWCLK_TCK)
    DAP_CONFIG_SWCLK_TCK_write(value & DAP_SWJ_SWCLK_TCK);

  if (select & DAP_SWJ_SWDIO_TMS)
    DAP_CONFIG_SWDIO_TMS_write(value & DAP_SWJ_SWDIO_TMS);

  if (select & DAP_SWJ_TDI)
    DAP_CONFIG_TDO_write(value & DAP_SWJ_TDI);

  if (select & DAP_SWJ_nTRST)
    DAP_CONFIG_nTRST_write(value & DAP_SWJ_nTRST);

  if (select & DAP_SWJ_nRESET)
    DAP_CONFIG_nRESET_write(value & DAP_SWJ_nRESET);

  dap_delay_us(wait * 1000);

  value =
    (DAP_CONFIG_SWCLK_TCK_read() ? DAP_SWJ_SWCLK_TCK : 0) |
    (DAP_CONFIG_SWDIO_TMS_read() ? DAP_SWJ_SWDIO_TMS : 0) |
    (DAP_CONFIG_TDI_read()       ? DAP_SWJ_TDI       : 0) |
    (DAP_CONFIG_TDO_read()       ? DAP_SWJ_TDO       : 0) |
    (DAP_CONFIG_nTRST_read()     ? DAP_SWJ_nTRST     : 0) |
    (DAP_CONFIG_nRESET_read()    ? DAP_SWJ_nRESET    : 0);

  resp[0] = value;
}

//-----------------------------------------------------------------------------
static void dap_swj_clock(uint8_t *req, uint8_t *resp)
{
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

  uint32_t freq;

  freq = ((uint32_t)req[3] << 24) | ((uint32_t)req[2] << 16) |
         ((uint32_t)req[1] << 8) | req[0];

  dap_setup_clock(freq);

  resp[0] = DAP_OK;
}

//-----------------------------------------------------------------------------
static void dap_swj_sequence(uint8_t *req, uint8_t *resp)
{
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

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
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

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
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

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
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

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
  if (verbose_commands) message("t/%s\n", __FUNCTION__);

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
    { ID_DAP_INFO,			dap_info },
    { ID_DAP_LED,			dap_led },
    { ID_DAP_CONNECT,			dap_connect },
    { ID_DAP_DISCONNECT,		dap_disconnect },
    { ID_DAP_TRANSFER_CONFIGURE,	dap_transfer_configure },
    { ID_DAP_TRANSFER,			dap_transfer },
    { ID_DAP_TRANSFER_BLOCK,		dap_transfer_block },
    { ID_DAP_TRANSFER_ABORT,		dap_transfer_abort },
    { ID_DAP_WRITE_ABORT,		dap_write_abort },
    { ID_DAP_DELAY,			dap_delay },
    { ID_DAP_RESET_TARGET,		dap_reset_target },
    { ID_DAP_SWJ_PINS,			dap_swj_pins },
    { ID_DAP_SWJ_CLOCK,			dap_swj_clock },
    { ID_DAP_SWJ_SEQUENCE,		dap_swj_sequence },
    { ID_DAP_SWD_CONFIGURE,		dap_swd_configure },
    { ID_DAP_JTAG_SEQUENCE,		dap_jtag_sequence },
    { ID_DAP_JTAG_CONFIGURE,		dap_jtag_configure },
    { ID_DAP_JTAG_IDCODE,		dap_jtag_idcode },
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
  char cmd = data[0];
  memset(req_buffer, 0xff, sizeof(req_buffer));
  memset(res_buffer, 0xff, sizeof(res_buffer));
  memcpy(req_buffer, data, rsize);
  dap_process_request(req_buffer, res_buffer);
  check(res_buffer[0] == cmd, "invalid rsponse received");
  memcpy(data, &res_buffer[1], size);
  return 0;
}
