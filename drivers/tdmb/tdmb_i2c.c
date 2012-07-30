//=============================================================================
// File       : T3700_i2c.c
//
// Description: 
//
// Revision History:
//
// Version         Date           Author        Description of Changes
//-----------------------------------------------------------------------------
//  1.0.0       2009/05/06       yschoi         Create
//=============================================================================

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "tdmb_comdef.h"
#include "tdmb_dev.h"
#include "tdmb_i2c.h"
#include "tdmb_type.h"



/*================================================================== */
/*================      TDMB I2C Definition        ================= */
/*================================================================== */

#define MAX_REG_LEN 2
#define MAX_DATA_LEN 16


static struct i2c_client *tdmb_i2c_client = NULL;

static int tdmb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);

/*================================================================== */
/*==============       TDMB i2c Driver Function      =============== */
/*================================================================== */

static int tdmb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  int rc = 0;

  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
  {
    tdmb_i2c_client = NULL;
    rc = -1;
    TDMB_MSG_I2C("[%s] failed!!!\n", __func__);
  }
  else
  {
  tdmb_i2c_client = client;
  }

  TDMB_MSG_I2C("[%s] succeed!!!\n", __func__);

  return rc;
}


static int tdmb_i2c_remove(struct i2c_client *client)
{
#if 0 // 20101102 cys
  int rc;

  tdmb_i2c_client = NULL;
#if 1
  rc = i2c_detach_client(client);

  return rc;
#endif
#endif // 0
  TDMB_MSG_I2C("[%s] removed!!!\n", __func__);
  return 0;
}


static const struct i2c_device_id tdmb_i2c_id[] = {
  {TDMB_I2C_DEV_NAME, 0},
};

static struct i2c_driver tdmb_i2c_driver = {
  .id_table = tdmb_i2c_id,
  .probe  = tdmb_i2c_probe,
  .remove = tdmb_i2c_remove,
  .driver = {
    .name = TDMB_I2C_DEV_NAME,
  },
};

void tdmb_i2c_api_Init(void)
{
  int result = 0;

  result = i2c_add_driver(&tdmb_i2c_driver);
  TDMB_MSG_I2C("[%s] i2c_add_driver!\n", __func__);

  if(result){
    TDMB_MSG_I2C("[%s] error!!!\n", __func__);
  }
}

void tdmb_i2c_api_DeInit(void)
{
  i2c_del_driver(&tdmb_i2c_driver);
}



/*================================================================== */
/*=================       TDMB i2c Function       ================== */
/*================================================================== */
/*====================================================================
FUNCTION       tdmb_i2c_check_id
DESCRIPTION 
DEPENDENCIES
RETURN VALUE
SIDE EFFECTS
======================================================================*/
static int tdmb_i2c_check_id(uint8 chipid)
{
  if(tdmb_i2c_client->addr != (chipid >> 1))
  {
    TDMB_MSG_I2C("[%s] chipid error addr[0x%02x], chipid[0x%02x]!!!\n", __func__, tdmb_i2c_client->addr, (chipid>>1));
    return FALSE;
  }
  return TRUE;
}

/*====================================================================
FUNCTION       tdmb_i2c_write_data
DESCRIPTION 
DEPENDENCIES
RETURN VALUE
SIDE EFFECTS
======================================================================*/
uint8 tdmb_i2c_write_data(uint16 reg, uint8 *data, uint8 width)
{
  static int ret = 0;
  struct i2c_msg msgs[] = 
  {
    {
      .addr = tdmb_i2c_client->addr,  .flags = 0, .len = width,  .buf = data,
    },
  };

  if(!tdmb_i2c_client)
  {
    TDMB_MSG_I2C("[%s] tdmb_i2c_client is null!!!\n", __func__);
    return -1;
  }

#ifdef FEATURE_I2C_DBG_MSG
    TDMB_MSG_I2C("[%s] ID[0x%02x] reg[0x%04x] data[0x%04x]\n", __func__, tdmb_i2c_client->addr, reg, *data);
#endif /* FEATURE_I2C_DBG_MSG */

  ret = i2c_transfer(tdmb_i2c_client->adapter, msgs, 1);
  if(ret < 0)
  {
    TDMB_MSG_I2C("[%s] write error!!![%d]\n", __func__,ret);
    return FALSE;
  }
  else
  {
#ifdef FEATURE_I2C_DBG_MSG
    TDMB_MSG_I2C("[%s] write OK!!!\n", __func__);
#endif /* FEATURE_I2C_DBG_MSG */
    return TRUE;
  }
}

/*====================================================================
FUNCTION       tdmb_i2c_write
DESCRIPTION 
DEPENDENCIES
RETURN VALUE
SIDE EFFECTS
======================================================================*/
uint8 tdmb_i2c_write(uint8 chip_id, uint16 reg, uint8 reg_len, uint16 data, uint8 data_len)
{
  static int ret = 0;
  unsigned char wlen=0;
  unsigned char buf[4];
#ifdef FEATURE_I2C_WRITE_CHECK
  uint16 *rData;
#endif /* FEATURE_I2C_WRITE_CHECK */

  if(!tdmb_i2c_client)
  {
    TDMB_MSG_I2C("[%s] tdmb_i2c_client is null!!!\n", __func__);
    return FALSE;
  }

  if(!tdmb_i2c_check_id(chip_id))
  {
    TDMB_MSG_I2C("[%s] tdmb i2c chipID error !!!\n", __func__);
    return FALSE;
  }

  memset(buf, 0, sizeof(buf));

  switch(reg_len) //현재는 byte, word write 만 사용 
  {
    case 2:
      buf[0] = (reg & 0xFF00) >> 8;
      buf[1] = (reg & 0x00FF);
      buf[2] = (data & 0xFF00) >> 8;
      buf[3] = (data & 0x00FF);
      wlen = data_len*2;
    break;

    case 1:
      buf[0] = (reg & 0x00FF);
      buf[1] = (data & 0x00FF);
      wlen = data_len*2;
    break;

    default:
    break;
  }

#ifdef FEATURE_I2C_DBG_MSG
    TDMB_MSG_I2C("[%s] ID[0x%02x] reg[0x%04x] data[0x%04x]\n", __func__, tdmb_i2c_client->addr, reg, data);
#endif /* FEATURE_I2C_DBG_MSG */

  ret = tdmb_i2c_write_data(reg, buf, wlen);

  if(ret > 0)
  {   
#ifdef FEATURE_I2C_WRITE_CHECK
    tdmb_i2c_read(reg, rData,2);
    if (rData != data)
    {
      TDMB_MSG_I2C("[%s] r/w check error! reg[0x%04x], data[0x%04x]\n", __func__, reg, rData);
    }
#endif /* FEATURE_I2C_WRITE_CHECK */
  }
  return TRUE;
}

/*====================================================================
FUNCTION       tdmb_i2c_read_data
DESCRIPTION 
DEPENDENCIES
RETURN VALUE
SIDE EFFECTS
======================================================================*/
uint16 tdmb_i2c_read_data(uint16 reg, uint8 reg_len, uint8 *data, uint8 *read_buf, uint16 data_len)
{
  static int ret = 0;
  struct i2c_msg msgs[] = 
  {
    {
      .addr = tdmb_i2c_client->addr,  .flags = 0, .len = reg_len,  .buf = data,
    },
    {
      .addr = tdmb_i2c_client->addr,  .flags = I2C_M_RD,  .len = data_len,  .buf = read_buf,
    },
  };

#ifdef FEATURE_I2C_DBG_MSG
  TDMB_MSG_I2C("[%s] ID[0x%02x] reg[0x%04x]\n", __func__, tdmb_i2c_client->addr, reg);
#endif

//EF33S QUP I2C 사용시 두번 보내야함
#if 1 // android 3145 버전에서 msgs 2개 한번에 보내면 에러리턴됨
  ret = i2c_transfer(tdmb_i2c_client->adapter, msgs, 1);
  if (ret < 0)
  {
    TDMB_MSG_I2C("[%s] write error!!!\n", __func__);
    return FALSE;
  }
  ret = i2c_transfer(tdmb_i2c_client->adapter, &msgs[1], 1);
#else
  ret = i2c_transfer(tdmb_i2c_client->adapter, msgs, 2);
#endif

  if(ret < 0)
  {
    TDMB_MSG_I2C("[%s] read error!!![%d]\n", __func__, ret);
    return FALSE;
  }
  else
  {
#ifdef FEATURE_I2C_DBG_MSG
    TDMB_MSG_I2C("[%s] read OK!!!\n", __func__);
#endif /* FEATURE_I2C_DBG_MSG */
    return TRUE;
  }
}

/*====================================================================
FUNCTION       tdmb_i2c_read
DESCRIPTION 
DEPENDENCIES
RETURN VALUE
SIDE EFFECTS
======================================================================*/
uint16 tdmb_i2c_read(uint8 chip_id, uint16 reg, uint8 reg_len, uint16 *data, uint8 data_len)
{
  static int ret = 0;
  unsigned char rlen=0;
  unsigned char buf[2] = {0,0};

  if(!tdmb_i2c_client)
  {
    TDMB_MSG_I2C("[%s] tdmb_i2c_client is null!!!\n", __func__);
    return FALSE;
  }

#ifdef FEATURE_I2C_DBG_MSG
    TDMB_MSG_I2C("[%s] ID[0x%02x] reg[0x%04x]\n", __func__, tdmb_i2c_client->addr, reg);
#endif

  if(!tdmb_i2c_check_id(chip_id))
  {
    TDMB_MSG_I2C("[%s] tdmb i2c chipID error !!!\n", __func__);
    return FALSE;
  }

  memset(buf, 0, sizeof(buf));

  switch(reg_len)
  {
    case 2:
      buf[0] = (reg & 0xFF00) >> 8;
      buf[1] = (reg & 0x00FF);
      rlen = 2;
      ret = tdmb_i2c_read_data(reg, reg_len, buf, buf,  rlen);
      if(ret < 0)
      {
        return FALSE;
      }
      *data = (buf[0] << 8) | buf[1];
    break;

    case 1:
      buf[0] = (reg & 0x00FF);
      rlen = 1;
      ret = tdmb_i2c_read_data(reg, reg_len, buf, buf,  rlen);
      if(ret < 0)
      {
        return FALSE;
      }
      *data = buf[0];
    break;

    default:
    break;
  }

  if(ret > 0)
  {
#ifdef FEATURE_I2C_DBG_MSG
    TDMB_MSG_I2C("[%s] read OK!!!\n", __func__);
    TDMB_MSG_I2C("[%s] reg[0x%04x], data[0x%04x]\n", __func__, reg, *data);
#endif
  }

  return ret;
}

/*====================================================================
FUNCTION       tdmb_i2c_read_len
DESCRIPTION 
DEPENDENCIES
RETURN VALUE
SIDE EFFECTS
======================================================================*/
void tdmb_i2c_read_len(uint8 chip_id, unsigned int uiAddr, uint8 ucRecvBuf[], uint16 ucCount, uint8 width)
{
  static int ret = 0;
  unsigned char reg_len=0;
  unsigned char buf[2];
  //struct i2c_msg msgs[2];
#ifdef FEATURE_I2C_DBG_MSG
  uint16 uiData;
#endif /* FEATURE_I2C_DBG_MSG */

  if(!tdmb_i2c_client)
  {
    TDMB_MSG_I2C("[%s] tdmb_i2c_client is null!!!\n", __func__);
    return;
  }

  //QUP I2C read시 data len 256byte 넘으면 에러.
  //if(ucCount > 256)
  //{
  //  ucCount = 256;
  //}

  memset(buf, 0, sizeof(buf));

  switch(width)
  {
    case 2:
      buf[0] = (uiAddr & 0xFF00) >> 8;
      buf[1] = (uiAddr & 0x00FF);
      reg_len = 2;
    break;

    case 1:
      buf[0] = (uiAddr & 0x00FF);
      reg_len = 1;
    break;

    default:
    break;
  }

#ifdef FEATURE_I2C_DBG_MSG
    TDMB_MSG_I2C("[%s] ID[0x%02x] reg[0x%04x] cnt[%d]\n", __func__, tdmb_i2c_client->addr, uiAddr, ucCount);
#endif

  ret = tdmb_i2c_read_data(uiAddr, reg_len, buf, ucRecvBuf, ucCount);

  if(ret == 0)
  {
    TDMB_MSG_I2C("[%s] read len error!!!\n", __func__);
  }
  else
  {    
#ifdef FEATURE_I2C_DBG_MSG
    uiData = (buf[0] << 8) | buf[1];
    TDMB_MSG_I2C("[%s] read OK!!!\n", __func__);
    TDMB_MSG_I2C("[%s] reg[0x%04x], data[0x%04x]\n", __func__, uiAddr, uiData);
#endif
  }
  return;
}

