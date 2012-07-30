//=============================================================================
// File       : Tdmb_i2c.h
//
// Description: 
//
// Revision History:
//
// Version         Date           Author        Description of Changes
//-----------------------------------------------------------------------------
//  1.0.0       2009/05/06       yschoi         Create
//=============================================================================

#ifndef _TDMB_I2C_INCLUDES_H_
#define _TDMB_I2C_INCLUDES_H_

#include "tdmb_type.h"


/* ========== Message ID for TDMB ========== */

#define TDMB_MSG_I2C(fmt, arg...) \
  TDMB_KERN_MSG_ALERT(fmt, ##arg)



void tdmb_i2c_api_Init(void);
void tdmb_i2c_api_DeInit(void);

uint8 tdmb_i2c_write(uint8 chip_id, uint16 reg, uint8 reg_len, uint16 data, uint8 data_len);
uint16 tdmb_i2c_read(uint8 chip_id, uint16 reg, uint8 reg_len, uint16 *data, uint8 data_len);
void tdmb_i2c_read_len(uint8 chip_id, unsigned int uiAddr, uint8 ucRecvBuf[], uint16 ucCount, uint8 width);

#endif /* _TDMB_I2C_INCLUDES_H_ */
