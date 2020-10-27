/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef _TUSB_CDC_DEVICE_H_
#define _TUSB_CDC_DEVICE_H_

#include "tusb_common.h"
#include "usbd.h"
#include "cdc.h"

//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+
#if !defined(CFG_TUD_CDC_EP_BUFSIZE) && defined(CFG_TUD_CDC_EPSIZE)
  #warning CFG_TUD_CDC_EPSIZE is renamed to CFG_TUD_CDC_EP_BUFSIZE, please update to use the new name
  #define CFG_TUD_CDC_EP_BUFSIZE    CFG_TUD_CDC_EPSIZE
#endif

#ifndef CFG_TUD_CDC_EP_BUFSIZE
  #define CFG_TUD_CDC_EP_BUFSIZE    (TUD_OPT_HIGH_SPEED ? 512 : 64)
#endif

#ifdef __cplusplus
 extern "C" {
#endif

/** \addtogroup CDC_Serial Serial
 *  @{
 *  \defgroup   CDC_Serial_Device Device
 *  @{ */

//--------------------------------------------------------------------+
// Application API (Multiple Ports)
// CFG_TUD_CDC > 1
//--------------------------------------------------------------------+

// Check if terminal is connected to this port
bool     tud_cdc_n_connected       (uint8_t itf);

// Get current line state. Bit 0:  DTR (Data Terminal Ready), Bit 1: RTS (Request to Send)
uint8_t  tud_cdc_n_get_line_state  (uint8_t itf);

// Get current line encoding: bit rate, stop bits parity etc ..
void     tud_cdc_n_get_line_coding (uint8_t itf, cdc_line_coding_t* coding);

// Set special character that will trigger tud_cdc_rx_wanted_cb() callback on receiving
void     tud_cdc_n_set_wanted_char (uint8_t itf, char wanted);

// Force sending data if possible, return number of forced bytes
uint32_t tud_cdc_n_write_flush     (uint8_t itf);


//--------------------------------------------------------------------+
// Application API (Single Port)
//--------------------------------------------------------------------+
static inline bool     tud_cdc_connected       (void);
static inline uint8_t  tud_cdc_get_line_state  (void);
static inline void     tud_cdc_get_line_coding (cdc_line_coding_t* coding);
static inline void     tud_cdc_set_wanted_char (char wanted);


//--------------------------------------------------------------------+
// Application Callback API (weak is optional)
//--------------------------------------------------------------------+

// Invoked when received new data
void tud_cdc_rx_cb(uint8_t itf,const void* p_data,uint16_t count);

// Invoked when received `wanted_char`
TU_ATTR_WEAK void tud_cdc_rx_wanted_cb(uint8_t itf, char wanted_char);

// Invoked when line state DTR & RTS are changed via SET_CONTROL_LINE_STATE
TU_ATTR_WEAK void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts);

// Invoked when line coding is change via SET_LINE_CODING
TU_ATTR_WEAK void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding);


//--------------------------------------------------------------------+
// INTERNAL USBD-CLASS DRIVER API
//--------------------------------------------------------------------+
void     cdcd_init             (void);
void     cdcd_reset            (uint8_t rhport);
uint16_t cdcd_open             (uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len);
bool     cdcd_control_request  (uint8_t rhport, tusb_control_request_t const * request);
bool     cdcd_control_complete (uint8_t rhport, tusb_control_request_t const * request);
bool     cdcd_xfer_cb          (uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CDC_DEVICE_H_ */
