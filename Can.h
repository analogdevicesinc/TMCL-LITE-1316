/*******************************************************************************
* Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file provides functions for intializing and
  using the CAN interface.
*/

#ifndef __CAN_H
#define __CAN_H

//! CAN Frame
typedef struct
{
  unsigned char Dlc;      //!< data length code (0..8)
  unsigned char Ext;      //!< extended ID flag (FALSE / TRUE)
  unsigned char Rtr;      //!< RTR flag (FALSE / TRUE)
  unsigned long Id;       //!< CAN frame ID
  unsigned char Data[8];  //!< CAN frame data
} TCanFrame;

void InitCan(UCHAR Baudrate, USHORT ReceiveID, USHORT SecondaryID);
int CanSendMessage(TCanFrame *Msg);
int CanGetMessage(TCanFrame *Msg);

#endif
