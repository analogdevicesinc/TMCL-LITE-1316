/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker servo (TMCM-1111))

  Module:  TMC4361.c
           TMC4361 library

   Copyright (C) 2018 TRINAMIC Motion Control GmbH & Co KG
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
  \file TMC4361.c
  \author Trinamic Motion Control GmbH & Co KG
  \version 1.00

  \brief TMC4361 Ramp generator functions

  This file provides all functions needed for easy
  access to the TMC4361 stepper motor driver IC.
*/


#include <stdlib.h>
#if defined(MK20DX128)
  #include "derivative.h"
#elif defined(GD32F425)
  #include "gd32f4xx.h"
#endif
#include "bits.h"
#include "stealthRocker.h"
#include "Globals.h"
#include "SPI.h"
#include "TMC4361.h"

USHORT TMC43xxTable[1]={SPI_DEV_TMC43xx_0};


/***************************************************************//**
   \fn WriteTMC43xxBytes(UCHAR Axis, UCHAR Address, UCHAR x1, UCHAR x2, UCHAR x3, UCHAR x4)
   \brief Write bytes to a TMC4361 register
   \param Axis  Index of TMC4361 to be used (with stepRocker always 0)
   \param Address   TMC4361 register address
   \param x1        First byte to write (MSB)
   \param x2        Second byte to write
   \param x3        Third byte to write
   \param x4        Fourth byte to write (LSB)

  This is a low level function for writing data to a TMC4361 register
  (32 bit value split up into four bytes).
********************************************************************/
void WriteTMC43xxBytes(UCHAR Axis, UCHAR Address, UCHAR x1, UCHAR x2, UCHAR x3, UCHAR x4)
{
  USHORT Index;

  Index=TMC43xxTable[Axis];
  ReadWriteSPI(Index, Address|TMC43xx_WRITE, FALSE);
  ReadWriteSPI(Index, x1, FALSE);
  ReadWriteSPI(Index, x2, FALSE);
  ReadWriteSPI(Index, x3, FALSE);
  ReadWriteSPI(Index, x4, TRUE);
}


/***************************************************************//**
   \fn WriteTMC43xxInt(UCHAR Axis, UCHAR Address, int Value)
   \brief Write a 32 bit integer value to a TMC4361 register
   \param Axis  Index of TMC4361 to be used (with stepRocker always 0)
   \param Address   TMC4361 register address
   \param Value     Value to be written

  This is a low level function for writing data to a TMC4361 register
  (32 bit value).
********************************************************************/
void WriteTMC43xxInt(UCHAR Axis, UCHAR Address, int Value)
{
  USHORT Index;

  Index=TMC43xxTable[Axis];
  ReadWriteSPI(Index, Address|TMC43xx_WRITE, FALSE);
  ReadWriteSPI(Index, Value >> 24, FALSE);
  ReadWriteSPI(Index, Value >> 16, FALSE);
  ReadWriteSPI(Index, Value >> 8,  FALSE);
  ReadWriteSPI(Index, Value & 0xff, TRUE);
}


/***************************************************************//**
   \fn ReadTMC43xxInt(UCHAR Axis, UCHAR Address)
   \brief Read from a TMC4361 register
   \param Axis      Index of TMC4361 to be used (with stepRocker always 0)
   \param Address   TMC4361 register address
   \return          Value read from the register

  This is a low level function for reading data from a TMC4361 register.
********************************************************************/
int ReadTMC43xxInt(UCHAR Axis, UCHAR Address)
{
  USHORT Index;
  int x;

  Index=TMC43xxTable[Axis];
  ReadWriteSPI(Index, Address, FALSE);
  ReadWriteSPI(Index, 0, FALSE);
  ReadWriteSPI(Index, 0, FALSE);
  ReadWriteSPI(Index, 0, FALSE);
  ReadWriteSPI(Index, 0, TRUE);

  ReadWriteSPI(Index, Address, FALSE);
  x=ReadWriteSPI(Index, 0, FALSE);
  x<<=8;
  x|=ReadWriteSPI(Index, 0, FALSE);
  x<<=8;
  x|=ReadWriteSPI(Index, 0, FALSE);
  x<<=8;
  x|=ReadWriteSPI(Index, 0, TRUE);

  return x;
}


/***************************************************************//**
   \fn TMC43xxSetBits(UCHAR Axis, UCHAR Address, UINT BitMask)
   \brief Set bits in a TMC4361 register
   \param Axis      Index of TMC4361 to be used (with stepRocker always 0)
   \param Address   TMC4361 register address
   \param BitMask   Bits to be set

  This function sets specified bits in a specified register. Bits that
  are set in the bit mask will also be set in the register. All other
  bits in the register remain untouched.
********************************************************************/
void TMC43xxSetBits(UCHAR Axis, UCHAR Address, UINT BitMask)
{
  UINT Value;

  Value=ReadTMC43xxInt(Axis, Address);
  Value|=BitMask;
  WriteTMC43xxInt(Axis, Address, Value);
}


/***************************************************************//**
   \fn TMC43xxClearBits(UCHAR Axis, UCHAR Address, UINT BitMask)
   \brief Clear bits in a TMC4361 register
   \param Axis      Index of TMC4361 to be used (with stepRocker always 0)
   \param Address   TMC4361 register address
   \param BitMask   Bits to be cleared

  This function clears specified bits in a specified register. Bits that
  are set in the bit mask will be cleared in the register, too. All other
  bits in the register remain untouched.
********************************************************************/
void TMC43xxClearBits(UCHAR Axis, UCHAR Address, UINT BitMask)
{
  UINT Value;

  Value=ReadTMC43xxInt(Axis, Address);
  Value&= ~BitMask;
  WriteTMC43xxInt(Axis, Address, Value);
}


/***************************************************************//**
   \fn TMC43xxWriteBits(UCHAR Axis, UCHAR Address, UINT Value, UCHAR Start, UCHAR Size)
   \brief Write bits in a TMC4361 register
   \param Axis      Index of TMC4361 to be used (with stepRocker always 0)
   \param Address   TMC4361 register address
   \param Value     Value to be written
   \param Start     First bit to be written
   \param Size      Number of bits to be written

  This function writes to specified bits in a specified register. The
  value given will be copied to the specified register, limited to
  the bits specified by Start and Size. All other bits remain unchanged.
********************************************************************/
void TMC43xxWriteBits(UCHAR Axis, UCHAR Address, UINT Value, UCHAR Start, UCHAR Size)
{
  UINT RegVal;
  UINT Mask;
  UINT i;

  RegVal=ReadTMC43xxInt(Axis, Address);

  Mask=0;
  for(i=Start; i<Start+Size; i++) Mask|=(1<<i);
  RegVal&=~Mask;

  RegVal|=(Value << Start) & Mask;

  WriteTMC43xxInt(Axis, Address, RegVal);
}


/***************************************************************//**
   \fn PeekTMC43xxEvents(UCHAR Axis)
   \brief Read events without clearing them
   \param Axis      Index of TMC4361 to be used (with stepRocker always 0)
   \return Value of the event register

  This function reads the TMC4361 event register, without clearing the
  event bits (by setting the event clear mask to 0xffffffff before
  reading the event register).
********************************************************************/
UINT PeekTMC43xxEvents(UCHAR Axis)
{
  WriteTMC43xxInt(Axis, TMC43xx_EVENT_CLR_CONF, 0xffffffff);
  return ReadTMC43xxInt(Axis, TMC43xx_EVENTS);
}


/***************************************************************//**
   \fn ReadAndClearTMC43xxEvents(UCHAR Axis, UINT EventMask)
   \brief Read events and clear specific events
   \param Axis      Index of TMC4361 to be used (with stepRocker always 0)
   \param EventMask Clear all event bits that are set here
   \return Value of the event register

  This function reads the TMC4361 event register and clears all
  event bits specified by EventMask (all bits that are set there).
********************************************************************/
UINT ReadAndClearTMC43xxEvents(UCHAR Axis, UINT EventMask)
{
  WriteTMC43xxInt(Axis, TMC43xx_EVENT_CLR_CONF, ~EventMask);
  return ReadTMC43xxInt(Axis, TMC43xx_EVENTS);
}


/***************************************************************//**
   \fn HardStop(UINT Axis)
   \brief Stop the motor immediately
   \param Axis      Index of TMC4361 to be used (with stepRocker always 0)

  This function stops the motor immediately, without using any
  deceleration ramp.
********************************************************************/
void HardStop(UINT Axis)
{
  VMaxModified[Axis]=TRUE;
  WriteTMC43xxInt(Axis, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_VEL_HOLD);
  WriteTMC43xxInt(Axis, TMC43xx_VMAX, 0);
}


/***************************************************************//**
   \fn InitTMC43xx
   \brief Initialize the TMC4361

  This function initializes the TMC4361.
********************************************************************/
void InitTMC43xx(void)
{
  int i;

  for(i=0; i<N_O_MOTORS; i++)
  {
    WriteTMC43xxInt(i, TMC43xx_GENERAL_CONF, TMC43xx_GCONF_ENC_INC|TMC43xx_GCONF_ENC_DIFF_DIS);
    WriteTMC43xxInt(i, TMC43xx_STEP_DIR_TIME, 0x00050005);
    WriteTMC43xxInt(i, TMC43xx_SPI_OUT_CONF, 0x84400000|TMC43xx_SPIOUT_TMC21xx
                      |TMC43xx_SPIOUT_POLL_BLOCK_MULTI(2)
                      //|TMC43xx_SPIOUT_DISABLE_POLLING
                      |TMC43xx_SPIOUT_ENABLE_SHADOW_DATAGRAMS
                      |TMC43xx_SPIOUT_COVER_DONE_NOT_FOR_CURRENT);

    WriteTMC43xxInt(i, TMC43xx_EVENT_CLR_CONF, ~TMC43xx_EV_COVER_DONE);  //Lesen des Event-Registers löscht nur das Cover-Done-Bit
    ReadTMC43xxInt(i, TMC43xx_EVENTS);
    WriteTMC43xxBytes(i, TMC43xx_COVER_HIGH, 0, 0, 0, 0);
    WriteTMC43xxInt(i, TMC43xx_COVER_LOW, 0);

    WriteTMC43xxInt(i, TMC43xx_CURRENT_CONF, TMC43xx_CURCONF_HOLD_EN|TMC43xx_CURCONF_DRIVE_EN);
    WriteTMC43xxInt(i, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_POS_HOLD);
    WriteTMC43xxInt(i, TMC43xx_XTARGET, 0);
    WriteTMC43xxInt(i, TMC43xx_XACTUAL, 0);

    WriteTMC43xxInt(i, TMC43xx_VMAX, 51200 << 8);
    WriteTMC43xxInt(i, TMC43xx_AMAX, 51200 << 2);
    WriteTMC43xxInt(i, TMC43xx_DMAX, 51200 << 2);

    WriteTMC43xxBytes(i, TMC43xx_SCALE_VALUES, MotorConfig[i].IStandby, 0, MotorConfig[i].IRun, MotorConfig[i].BoostCurrent);
    WriteTMC43xxInt(i, TMC43xx_STDBY_DELAY, MotorConfig[i].SettingDelay*160000);

    WriteTMC43xxInt(i, TMC43xx_CL_VMIN_EMF, ClosedLoopConfig[i].GammaVMin);
    WriteTMC43xxInt(i, TMC43xx_CL_VADD_EMF, ClosedLoopConfig[i].GammaVAdd);
    WriteTMC43xxInt(i, TMC43xx_CL_BETA_GAMMA, (ClosedLoopConfig[i].Gamma<<16)|ClosedLoopConfig[i].Beta);
    WriteTMC43xxInt(i, TMC43xx_CL_OFFSET, ClosedLoopConfig[i].Offset);
    WriteTMC43xxInt(i, TMC43xx_CL_VMAX_P_W, ClosedLoopConfig[i].CorrectionVelocityP);
    WriteTMC43xxInt(i, TMC43xx_CL_VMAX_CALC_I_W, ClosedLoopConfig[i].CorrectionVelocityI);
    WriteTMC43xxBytes(i, TMC43xx_PID_I_CLIP_D_CLK_W, 0, ClosedLoopConfig[i].CorrectionVelocityDClk,
      ClosedLoopConfig[i].CorrectionVelocityIClip >> 8, ClosedLoopConfig[i].CorrectionVelocityIClip & 0xff);
    WriteTMC43xxInt(i, TMC43xx_PID_DV_CLIP, ClosedLoopConfig[i].CorrectionVelocityDClip);
    WriteTMC43xxInt(i, TMC43xx_CL_UPSCALE_DELAY, ClosedLoopConfig[i].UpscaleDelay);
    WriteTMC43xxInt(i, TMC43xx_CL_DOWNSCALE_DELAY, ClosedLoopConfig[i].DownscaleDelay);
    WriteTMC43xxInt(i, TMC43xx_CL_DELTA_P_W, ClosedLoopConfig[i].PositionCorrectionP);
    WriteTMC43xxInt(i, TMC43xx_CL_TOLERANCE, ClosedLoopConfig[i].PositionCorrectionTolerance);
    WriteTMC43xxInt(i, TMC43xx_CL_TR_TOLERANCE_W, ClosedLoopConfig[i].PositionWindow);
    WriteTMC43xxBytes(i, TMC43xx_ENC_VMEAN_WAIT_FILT, ClosedLoopConfig[i].EncVMeanInt >> 8,
          ClosedLoopConfig[i].EncVMeanInt & 0xff, ClosedLoopConfig[i].EncVMeanFilter,
          ClosedLoopConfig[i].EncVMeanWait);

    WriteTMC43xxBytes(i, TMC43xx_ENC_COMP_OFFSET, 0, ClosedLoopConfig[i].EncoderCorrectionYOffset, 0, 0);

    WriteTMC43xxInt(i, TMC43xx_REFERENCE_CONF, BIT17|BIT16);
    WriteTMC43xxInt(i, TMC43xx_XHOME, 0x7fffffff);
  }
}


/***************************************************************//**
   \fn ResetTMC43xx
   \brief Reset the TMC4361

  This function resets the TMC4361 (by pulling its reset pin low
  for some clock periods).
********************************************************************/
void ResetTMC43xx(void)
{
  #if defined(MK20DX128)
  GPIOD_PCOR=BIT4;  //Reset-Pin low
  asm volatile("nop\n");
  asm volatile("nop\n");
  asm volatile("nop\n");
  GPIOD_PSOR=BIT4;  //Reset-Pin high
  
  #elif defined(GD32F425)
  GPIO_BC(GPIOB)=BIT7;  //Reset pin low
  asm volatile("nop\n");
  asm volatile("nop\n");
  asm volatile("nop\n");
  GPIO_BOP(GPIOB)=BIT7;  //Reset pin high
  #endif
}


/***************************************************************//**
   \fn GetHomeInput(UCHAR Motor)
   \brief Read TMC4361 home input
   \param Motor  Index of TMC4361 to be used (with stepRocker always 0)
   \return State of the home input (TRUE or FALSE)

  This function reads the TMC4361 home input. This works using a little
  trick: Bit 17 and bit 16 in the REFERENCE_CONF register must be
  set and the XHOME register must be set to INT_MAX.
********************************************************************/
UCHAR GetHomeInput(UCHAR Motor)
{
  if(ReadTMC43xxInt(Motor, TMC43xx_STATUS) & TMC43xx_ST_HOME_ERROR)
    return TRUE;
  else
    return FALSE;
}


/***************************************************************//**
   \fn ConvertVelocityUserToInternal(int UserVelocity)
   \brief Convert from pps to internal unit
   \param UserVelocity: Velocity as pps value
   \return Internal velocity value

  This function converts a velocity value given in pps for use
  with most TMC4361 velocity registers.
********************************************************************/
int ConvertVelocityUserToInternal(int UserVelocity)
{
  return UserVelocity << 8;
}


/***************************************************************//**
   \fn ConvertAccelerationUserToInternal(int UserAcceleration)
   \brief Convert from pps/s to internal unit
   \param UserAcceleration: Acceleration/Deceleration as pps/s value
   \return Internal acceleration/deceleration value

  This function converts an acceleration value or a deceleration value
  given in pps/s for use with most TMC4361 acceleration/deceleration
  registers.
********************************************************************/
int ConvertAccelerationUserToInternal(int UserAcceleration)
{
  return UserAcceleration << 2;
}


/***************************************************************//**
   \fn ConvertVelocityInternalToUser(int InternalVelocity)
   \brief Convert from internal unit to pps
   \param InternalVelocity: Velocity as internal value
   \return PPS velocity value

  This function converts a velocity value given in internal units
  of the TMC4361 back into pps.
********************************************************************/
int ConvertVelocityInternalToUser(int InternalVelocity)
{
  return InternalVelocity >> 8;
}

/***************************************************************//**
   \fn ConvertAccelerationInternalToUser(int InternalAcceleration)
   \brief Convert from internal unit to pps/s
   \param InternalAcceleration: Accleration/Deceleration as internal value
   \return PPS/S acceleration/deceleration value

  This function converts an acceleration/deceleration value given
  in internal units of the TMC4361 back into pps/s.
********************************************************************/
int ConvertAccelerationInternalToUser(int InternalAcceleration)
{
  return InternalAcceleration >> 2;
}

/***************************************************************//**
   \fn ConvertInternalToInternal(int Internal)
   \brief Dummy function used when unit conversion is switched off
   \param Internal: Input value
   \return Unchanged value

   This is a dummy function which is used when unit conversion is
   switched off.
********************************************************************/
int ConvertInternalToInternal(int Internal)
{
  return Internal;
}
