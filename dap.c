/*
 * Copyright (c) 2013-2015, Alex Taradov <alex@taradov.com>
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
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "edbg.h"
#include "dap.h"
#include "dbg.h"
#include "dap_common.h"

/*- Definitions -------------------------------------------------------------*/

/*- Types -------------------------------------------------------------------*/

/*- Variables ---------------------------------------------------------------*/

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void dap_led(int index, int state)
{
  uint8_t buf[3];

  buf[0] = ID_DAP_LED;
  buf[1] = index;
  buf[2] = state;
  dbg_dap_cmd(buf, sizeof(buf), 3);

  check(DAP_OK == buf[0], "DAP_LED failed");
}

//-----------------------------------------------------------------------------
void dap_connect(void)
{
  uint8_t buf[2];

  buf[0] = ID_DAP_CONNECT;
  buf[1] = DAP_PORT_SWD;
  dbg_dap_cmd(buf, sizeof(buf), 2);

  check(DAP_PORT_SWD == buf[0], "DAP_CONNECT failed");
}

//-----------------------------------------------------------------------------
void dap_disconnect(void)
{
  uint8_t buf[1];

  buf[0] = ID_DAP_DISCONNECT;
  dbg_dap_cmd(buf, sizeof(buf), 1);
}

//-----------------------------------------------------------------------------
void dap_swj_clock(uint32_t clock)
{
  uint8_t buf[5];

  buf[0] = ID_DAP_SWJ_CLOCK;
  buf[1] = clock & 0xff;
  buf[2] = (clock >> 8) & 0xff;
  buf[3] = (clock >> 16) & 0xff;
  buf[4] = (clock >> 24) & 0xff;
  dbg_dap_cmd(buf, sizeof(buf), 5);

  check(DAP_OK == buf[0], "SWJ_CLOCK failed");
}

//-----------------------------------------------------------------------------
void dap_transfer_configure(uint8_t idle, uint16_t count, uint16_t retry)
{
  uint8_t buf[6];

  buf[0] = ID_DAP_TRANSFER_CONFIGURE;
  buf[1] = idle;
  buf[2] = count & 0xff;
  buf[3] = (count >> 8) & 0xff;
  buf[4] = retry & 0xff;
  buf[5] = (retry >> 8) & 0xff;
  dbg_dap_cmd(buf, sizeof(buf), 6);

  check(DAP_OK == buf[0], "TRANSFER_CONFIGURE failed");
}

//-----------------------------------------------------------------------------
void dap_swd_configure(uint8_t cfg)
{
  uint8_t buf[2];

  buf[0] = ID_DAP_SWD_CONFIGURE;
  buf[1] = cfg;
  dbg_dap_cmd(buf, sizeof(buf), 2);

  check(DAP_OK == buf[0], "SWD_CONFIGURE failed");
}

//-----------------------------------------------------------------------------
void dap_get_debugger_info(void)
{
  uint8_t buf[100];
  char str[500] = "";

  buf[0] = ID_DAP_INFO;
  buf[1] = DAP_INFO_VENDOR;
  dbg_dap_cmd(buf, sizeof(buf), 2);
  strncat(str, (char *)&buf[1], buf[0]);
  strcat(str, " ");

  buf[0] = ID_DAP_INFO;
  buf[1] = DAP_INFO_PRODUCT;
  dbg_dap_cmd(buf, sizeof(buf), 2);
  strncat(str, (char *)&buf[1], buf[0]);
  strcat(str, " ");

  buf[0] = ID_DAP_INFO;
  buf[1] = DAP_INFO_SER_NUM;
  dbg_dap_cmd(buf, sizeof(buf), 2);
  strncat(str, (char *)&buf[1], buf[0]);
  strcat(str, " ");

  buf[0] = ID_DAP_INFO;
  buf[1] = DAP_INFO_FW_VER;
  dbg_dap_cmd(buf, sizeof(buf), 2);
  strncat(str, (char *)&buf[1], buf[0]);
  strcat(str, " ");

  buf[0] = ID_DAP_INFO;
  buf[1] = DAP_INFO_CAPABILITIES;
  dbg_dap_cmd(buf, sizeof(buf), 2);

  strcat(str, "(");

  if (buf[1] & DAP_PORT_SWD)
    strcat(str, "S");

  if (buf[1] & DAP_PORT_JTAG)
    strcat(str, "J");

  strcat(str, ")");

  verbose("Debugger: %s\n", str);

  check(buf[1] & DAP_PORT_SWD, "SWD support required");
}

//-----------------------------------------------------------------------------
void dap_reset_target(void)
{
  uint8_t buf[1];

  buf[0] = ID_DAP_RESET_TARGET;
  dbg_dap_cmd(buf, sizeof(buf), 1);

  check(DAP_OK == buf[0], "RESET_TARGET failed");
}

//-----------------------------------------------------------------------------
void dap_reset_target_hw(int state)
{
  uint8_t buf[7];
  int value = state ? (DAP_SWJ_SWCLK_TCK | DAP_SWJ_SWDIO_TMS) : 0;

  //-------------
  buf[0] = ID_DAP_SWJ_PINS;
  buf[1] = value; // Value
  buf[2] = DAP_SWJ_nRESET | DAP_SWJ_SWCLK_TCK | DAP_SWJ_SWDIO_TMS; // Select
  buf[3] = 0; // Wait
  buf[4] = 0;
  buf[5] = 0;
  buf[6] = 0;
  dbg_dap_cmd(buf, sizeof(buf), 7);

  //-------------
  buf[0] = ID_DAP_SWJ_PINS;
  buf[1] = DAP_SWJ_nRESET | value; // Value
  buf[2] = DAP_SWJ_nRESET | DAP_SWJ_SWCLK_TCK | DAP_SWJ_SWDIO_TMS; // Select
  buf[3] = 0; // Wait
  buf[4] = 0;
  buf[5] = 0;
  buf[6] = 0;
  dbg_dap_cmd(buf, sizeof(buf), 7);
}

//-----------------------------------------------------------------------------
uint32_t dap_read_reg(uint8_t reg)
{
  uint8_t buf[8];

  buf[0] = ID_DAP_TRANSFER;
  buf[1] = 0x00; // DAP index
  buf[2] = 0x01; // Request size
  buf[3] = reg | DAP_TRANSFER_RnW;
  dbg_dap_cmd(buf, sizeof(buf), 4);

  if (1 != buf[0] || DAP_TRANSFER_OK != buf[1])
  {
    error_exit("invalid response while reading the register 0x%02x (count = %d, value = %d)",
        reg, buf[0], buf[1]);
  }

  return ((uint32_t)buf[5] << 24) | ((uint32_t)buf[4] << 16) |
         ((uint32_t)buf[3] << 8) | (uint32_t)buf[2];
}

//-----------------------------------------------------------------------------
void dap_write_reg(uint8_t reg, uint32_t data)
{
  uint8_t buf[8];

  buf[0] = ID_DAP_TRANSFER;
  buf[1] = 0x00; // DAP index
  buf[2] = 0x01; // Request size
  buf[3] = reg;
  buf[4] = data & 0xff;
  buf[5] = (data >> 8) & 0xff;
  buf[6] = (data >> 16) & 0xff;
  buf[7] = (data >> 24) & 0xff;
  dbg_dap_cmd(buf, sizeof(buf), 8);

  if (1 != buf[0] || DAP_TRANSFER_OK != buf[1])
  {
    error_exit("invalid response while writing the register 0x%02x (count = %d, value = %d)",
        reg, buf[0], buf[1]);
  }
}

//-----------------------------------------------------------------------------
uint32_t dap_read_word(uint32_t addr)
{
  dap_write_reg(SWD_AP_TAR, addr);
  return dap_read_reg(SWD_AP_DRW);
}

//-----------------------------------------------------------------------------
void dap_write_word(uint32_t addr, uint32_t data)
{
  dap_write_reg(SWD_AP_TAR, addr);
  dap_write_reg(SWD_AP_DRW, data);
}

//-----------------------------------------------------------------------------
void dap_read_block(uint32_t addr, uint8_t *data, int size)
{
  int max_size = (dbg_get_report_size() - 5) & ~3;
  int offs = 0;

  while (size)
  {
    int align, sz;
    uint8_t buf[1024];

    align = 0x400 - (addr - (addr & ~0x3ff));
    sz = (size > max_size) ? max_size : size;
    sz = (sz > align) ? align : sz;

    dap_write_reg(SWD_AP_TAR, addr);

    buf[0] = ID_DAP_TRANSFER_BLOCK;
    buf[1] = 0x00; // DAP index
    buf[2] = (sz / 4) & 0xff;
    buf[3] = ((sz / 4) >> 8) & 0xff;
    buf[4] = SWD_AP_DRW | DAP_TRANSFER_RnW | DAP_TRANSFER_APnDP;
    dbg_dap_cmd(buf, sizeof(buf), 5);

    if (DAP_TRANSFER_OK != buf[2])
    {
      error_exit("invalid response while reading the block at 0x%08x (value = %d)",
          addr, buf[2]);
    }

    memcpy(&data[offs], &buf[3], sz);

    size -= sz;
    addr += sz;
    offs += sz;
  }
}

//-----------------------------------------------------------------------------
void dap_write_block(uint32_t addr, uint8_t *data, int size)
{
  int max_size = (dbg_get_report_size() - 5) & ~3;
  int offs = 0;

  while (size)
  {
    int align, sz;
    uint8_t buf[1024];

    align = 0x400 - (addr - (addr & ~0x3ff));
    sz = (size > max_size) ? max_size : size;
    sz = (sz > align) ? align : sz;

    dap_write_reg(SWD_AP_TAR, addr);

    buf[0] = ID_DAP_TRANSFER_BLOCK;
    buf[1] = 0x00; // DAP index
    buf[2] = (sz / 4) & 0xff;
    buf[3] = ((sz / 4) >> 8) & 0xff;
    buf[4] = SWD_AP_DRW | DAP_TRANSFER_APnDP;
    memcpy(&buf[5], &data[offs], sz);
    dbg_dap_cmd(buf, sizeof(buf), 5 + sz);

    if (DAP_TRANSFER_OK != buf[2])
    {
      error_exit("invalid response while writing the block at 0x%08x (value = %d)",
          addr, buf[2]);
    }

    size -= sz;
    addr += sz;
    offs += sz;
  }
}

//-----------------------------------------------------------------------------
void dap_reset_link(void)
{
  uint8_t buf[128];

  //-------------
  buf[0] = ID_DAP_SWJ_SEQUENCE;
  buf[1] = (7 + 2 + 7 + 1) * 8;
  buf[2] = 0xff;
  buf[3] = 0xff;
  buf[4] = 0xff;
  buf[5] = 0xff;
  buf[6] = 0xff;
  buf[7] = 0xff;
  buf[8] = 0xff;
  buf[9] = 0x9e;
  buf[10] = 0xe7;
  buf[11] = 0xff;
  buf[12] = 0xff;
  buf[13] = 0xff;
  buf[14] = 0xff;
  buf[15] = 0xff;
  buf[16] = 0xff;
  buf[17] = 0xff;
  buf[18] = 0x00;

  dbg_dap_cmd(buf, sizeof(buf), 19);
  check(DAP_OK == buf[0], "SWJ_SEQUENCE failed");

  //-------------
  buf[0] = ID_DAP_TRANSFER;
  buf[1] = 0; // DAP index
  buf[2] = 1; // Request size
  buf[3] = SWD_DP_R_IDCODE | DAP_TRANSFER_RnW;
  dbg_dap_cmd(buf, sizeof(buf), 4);
}

//-----------------------------------------------------------------------------
uint32_t dap_read_idcode(void)
{
  return dap_read_reg(SWD_DP_R_IDCODE);
}

//-----------------------------------------------------------------------------
void dap_target_prepare(void)
{
  dap_write_reg(SWD_DP_W_ABORT, 0x00000016);
  dap_write_reg(SWD_DP_W_SELECT, 0x00000000);
  dap_write_reg(SWD_DP_W_CTRL_STAT, 0x50000f00);
  dap_write_reg(SWD_AP_CSW, 0x23000052);
}
