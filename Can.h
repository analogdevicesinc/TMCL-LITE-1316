/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  Can.h
           CAN routines

   Copyright (C) 2011 TRINAMIC Motion Control GmbH & Co KG
                      Waterloohain 5
                      D - 22769 Hamburg, Germany
                      http://www.trinamic.com/

   This program is free software; you can redistribute it and/or modify it
   freely.

   This program is distributed "as is" in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/

/**
  \file Can.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief CAN

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
