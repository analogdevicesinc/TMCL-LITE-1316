/*******************************************************************************
* Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file provides all functions needed for easy
  access to the TMC5160 stepper motor driver IC.
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
#include "SysTick.h"
#include "TMC4361.h"
#include "TMC5160.h"

//This table shows which TMC5160 register can be read back (0=no, 1=yes).
static const UCHAR TMC5160RegisterReadable[128]={
0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,    //00..0f  GCONF set to "not readable" because of problems when reading back via TMC4361
0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    //10..1f
1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,    //20..2f
0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0,    //30..3f
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    //40..4f
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    //50..5f
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1,    //60..6f  CHOPCONF set to "not readable" because of problems when reading back via TMC4361
0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    //70..7f
};


static int TMC5160SoftwareCopy[128][N_O_MOTORS];    //!< Software copy of all registers
static UCHAR DriverDisableFlag[N_O_MOTORS];         //!< Flags used for switching off a motor driver via TOff
static UCHAR LastTOffSetting[N_O_MOTORS];           //!< Last TOff setting before switching off the driver


/***************************************************************//**
   \fn WriteTMC5160Datagram(UCHAR Which5160, UCHAR Address, UCHAR x1, UCHAR x2, UCHAR x3, UCHAR x4)
   \brief Write bytes to a TMC5160 register
   \param Which5160  Index of TMC5160 to be used (with stepRocker always 0)
   \param Address    Registeradresse (0x00..0x7f)
   \param x1        First byte to write (MSB)
   \param x2        Second byte to write
   \param x3        Third byte to write
   \param x4        Fourth byte to write (LSB)

  This is a low level function for writing data to a TMC5160 register
  (32 bit value split up into four bytes).
********************************************************************/
void WriteTMC5160Datagram(UCHAR Which5160, UCHAR Address, UCHAR x1, UCHAR x2, UCHAR x3, UCHAR x4)
{
  int Value;

  //Write to TMC5160 via TMC43xx cover datagram
  WriteTMC43xxInt(Which5160, TMC43xx_EVENT_CLR_CONF, ~TMC43xx_EV_COVER_DONE);
  ReadTMC43xxInt(Which5160, TMC43xx_EVENTS);
  WriteTMC43xxInt(Which5160, TMC43xx_EVENT_CLR_CONF, 0xffffffff);
  WriteTMC43xxBytes(Which5160, TMC43xx_COVER_HIGH, 0, 0, 0, Address|0x80);
  WriteTMC43xxBytes(Which5160, TMC43xx_COVER_LOW, x1, x2, x3, x4);
  while(!(ReadTMC43xxInt(Which5160, TMC43xx_EVENTS) & TMC43xx_EV_COVER_DONE));

  //Update software copy
  Value=x1;
  Value<<=8;
  Value|=x2;
  Value<<=8;
  Value|=x3;
  Value<<=8;
  Value|=x4;
  TMC5160SoftwareCopy[Address & 0x7f][Which5160]=Value;
}


/***************************************************************//**
   \fn WriteTMC5160Int(UCHAR Which5160, UCHAR Address, int Value)
   \brief Write a 32 bit value to a TMC5160 register
   \param Which5160  Index of TMC5160 to be used (with stepRocker always 0)
   \param Address    Registeradresse (0x00..0x7f)
   \param Value      Value to be written

  This is a low level function for writing data to a TMC5160 register
  (32 bit value).
********************************************************************/
void WriteTMC5160Int(UCHAR Which5160, UCHAR Address, int Value)
{
  //Write to TMC5160 register via TMC43xx cover datagram
  WriteTMC43xxInt(Which5160, TMC43xx_EVENT_CLR_CONF, ~TMC43xx_EV_COVER_DONE);
  ReadTMC43xxInt(Which5160, TMC43xx_EVENTS);
  WriteTMC43xxInt(Which5160, TMC43xx_EVENT_CLR_CONF, 0xffffffff);
  WriteTMC43xxBytes(Which5160, TMC43xx_COVER_HIGH, 0, 0, 0, Address|0x80);
  WriteTMC43xxInt(Which5160, TMC43xx_COVER_LOW, Value);
  while(!(ReadTMC43xxInt(Which5160, TMC43xx_EVENTS) & TMC43xx_EV_COVER_DONE));

  //Update software copy
  TMC5160SoftwareCopy[Address & 0x7f][Which5160]=Value;
}


/***************************************************************//**
   \fn ReadTMC5160Int(UCHAR Which5160, UCHAR Address)
   \brief Write a 32 bit value to a TMC5160 register
   \param Which5160  Index of TMC5160 to be used (with stepRocker always 0)
   \param Address    Registeradresse (0x00..0x7f)
   \return           Value read from the register

  This is a low level function for writing data to a TMC5160 register
  (32 bit value).
********************************************************************/
int ReadTMC5160Int(UCHAR Which5160, UCHAR Address)
{
  int Value;

  Address&=0x7f;
  if(TMC5160RegisterReadable[Address])
  {
    //Register readable => read via TMC4361 cover datagram mechanism.
    WriteTMC43xxInt(Which5160, TMC43xx_EVENT_CLR_CONF, ~TMC43xx_EV_COVER_DONE);
    ReadTMC43xxInt(Which5160, TMC43xx_EVENTS);
    WriteTMC43xxInt(Which5160, TMC43xx_EVENT_CLR_CONF, 0xffffffff);
    WriteTMC43xxBytes(Which5160, TMC43xx_COVER_HIGH, 0, 0, 0, Address);
    WriteTMC43xxInt(Which5160, TMC43xx_COVER_LOW, 0);
    while(!(ReadTMC43xxInt(Which5160, TMC43xx_EVENTS) & TMC43xx_EV_COVER_DONE));

    Value=ReadTMC43xxInt(Which5160, TMC43xx_COVER_DRV_LOW);

    return Value;
  }
  else
  {
    //Register noot readable => use software copy.
    return TMC5160SoftwareCopy[Address][Which5160];
  }
}


/***************************************************************//**
   \fn SetTMC5160ChopperTOff(UCHAR Motor, UCHAR TOff)
   \brief Set the TOff parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \param TOff    TOff parameter

  This function sets the TOff parameter.
********************************************************************/
void SetTMC5160ChopperTOff(UCHAR Motor, UCHAR TOff)
{
  UINT Value;

  if(!DriverDisableFlag[Motor])
  {
    Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & 0xfffffff0;
    WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value | (TOff & 0x0f));
  }
  LastTOffSetting[Motor]=TOff;
}

/***************************************************************//**
   \fn SetTMC5160ChopperHysteresisStart(UCHAR Motor, UCHAR HysteresisStart)
   \brief Set the HSTART parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \param HysteresisStart    Hysteresis start parameter.

  This function sets the HSTART parameter.
********************************************************************/
void SetTMC5160ChopperHysteresisStart(UCHAR Motor, UCHAR HysteresisStart)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & 0xffffff8f;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value | ((HysteresisStart & 0x07) << 4));
}

/***************************************************************//**
   \fn SetTMC5160ChopperHysteresisEnd(UCHAR Motor, UCHAR HysteresisEnd)
   \brief Set the HEND parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \param HysteresisEnd    Hysteresis end parameter.

  This function sets the HEND parameter.
********************************************************************/
void SetTMC5160ChopperHysteresisEnd(UCHAR Motor, UCHAR HysteresisEnd)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & 0xfffff87f;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value | ((HysteresisEnd & 0x0f) << 7));
}

/***************************************************************//**
   \fn SetTMC5160ChopperBlankTime(UCHAR Motor, UCHAR BlankTime)
   \brief Set the chopper blank time parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \param BlankTime    Chopper blank time.

  This function sets the chopper blank time parameter.
********************************************************************/
void SetTMC5160ChopperBlankTime(UCHAR Motor, UCHAR BlankTime)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & 0xfffe7fff;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value | ((BlankTime & 0x03) << 15));
}

/***************************************************************//**
   \fn SetTMC5160ChopperSync(UCHAR Motor, UCHAR Sync)
   \brief Set the chopper synchronization parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \param Sync    Chopper sync time.

  This function sets the chopper synchronization parameter.
********************************************************************/
void SetTMC5160ChopperSync(UCHAR Motor, UCHAR Sync)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & 0xff0fffff;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value | ((Sync & 0x0f) << 20));
}

/***************************************************************//**
   \fn SetTMC5160ChopperMStepRes(UCHAR Motor, UCHAR MRes)
   \brief Set microstep resolution.
   \param Motor   Axis number (with stepRocker always 0)
   \param MRes    Microstep resolution (0..7).

  This function sets the microstep resolution.
********************************************************************/
void SetTMC5160ChopperMStepRes(UCHAR Motor, UCHAR MRes)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & 0xf0ffffff;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value | ((MRes & 0x0f) << 24));
}

/***************************************************************//**
   \fn SetTMC5160ChopperDisableShortToGround(UCHAR Motor, UCHAR Disable)
   \brief Disable the short to ground detection.
   \param Motor   Axis number (with stepRocker always 0)
   \param Disable   TRUE: short to ground detection off.\n
                    FALSE: short to ground detection on.

  This function disables or enables the short to ground detection.
********************************************************************/
void SetTMC5160ChopperDisableShortToGround(UCHAR Motor, UCHAR Disable)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF);
  if(Disable)
    Value|=BIT30;
  else
    Value&= ~BIT30;

  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5160ChopperVHighChm(UCHAR Motor, UCHAR VHighChm)
   \brief Switch off/on VHIGHCHM flag.
   \param Motor   Axis number (with stepRocker always 0)
   \param VHighChm  TRUE/FALSE.

  This function disables or enables the VHIGHCHM flag.
********************************************************************/
void SetTMC5160ChopperVHighChm(UCHAR Motor, UCHAR VHighChm)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF);
  if(VHighChm)
    Value|=BIT19;
  else
    Value&= ~BIT19;

  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5160ChopperVHighFs(UCHAR Motor, UCHAR VHighFs)
   \brief Switch off/on VHIGHFS flag.
   \param Motor   Axis number (with stepRocker always 0)
   \param VHighFs   TRUE/FALSE.

  This function disables or enables the VHIGHFS flag.
********************************************************************/
void SetTMC5160ChopperVHighFs(UCHAR Motor, UCHAR VHighFs)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF);
  if(VHighFs)
    Value|=BIT18;
  else
    Value&= ~BIT18;

  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5160ChopperConstantTOffMode(UCHAR Motor, UCHAR ConstantTOff)
   \brief Switch off/on constant chopper mode.
   \param Motor   Axis number (with stepRocker always 0)
   \param ConstantTOff  Constant TOff mode switch: TRUE/FALSE.

  This function disables or enables the constant chopper mode.
********************************************************************/
void SetTMC5160ChopperConstantTOffMode(UCHAR Motor, UCHAR ConstantTOff)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF);
  if(ConstantTOff)
    Value|=BIT14;
  else
    Value&= ~BIT14;

  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5160ChopperRandomTOff(UCHAR Motor, UCHAR RandomTOff)
   \brief Switch off/on random TOff mode.
   \param Motor   Axis number (with stepRocker always 0)
   \param RandomTOff  TRUE/FALSE.

  This function disables or enables the random chopper mode.
********************************************************************/
void SetTMC5160ChopperRandomTOff(UCHAR Motor, UCHAR RandomTOff)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF);
  if(RandomTOff)
    Value|=BIT13;
  else
    Value&= ~BIT13;

  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5160ChopperDisableFastDecayComp(UCHAR Motor, UCHAR Disable)
   \brief Switch off/on FastDecayComp.
   \param Motor   Axis number (with stepRocker always 0)
   \param Disable  TRUE/FALSE.

  This function disables or enables the fast decay comparator.
********************************************************************/
void SetTMC5160ChopperDisableFastDecayComp(UCHAR Motor, UCHAR Disable)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF);
  if(Disable)
    Value|=BIT12;
  else
    Value&= ~BIT12;

  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5160ChopperFastDecayTime(UCHAR Motor, UCHAR Time)
   \brief Set the fast decay time.
   \param Motor   Axis number (with stepRocker always 0)
   \param Time    Fast decay time (0..15).

  This function sets the fast decay time.
********************************************************************/
void SetTMC5160ChopperFastDecayTime(UCHAR Motor, UCHAR Time)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & 0xffffff8f;

  if(Time & BIT3)
    Value|=BIT11;
  else
    Value&= ~BIT11;

  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value | ((Time & 0x07) << 4));
}

/***************************************************************//**
   \fn SetTMC5160ChopperSineWaveOffset(UCHAR Motor, UCHAR Offset)
   \brief Set the sine offset value.
   \param Motor   Axis number (with stepRocker always 0)
   \param Offset  Sine wave offset (0..15).

  This function sets the sine wave offset.
********************************************************************/
void SetTMC5160ChopperSineWaveOffset(UCHAR Motor, UCHAR Offset)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & 0xfffff87f;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF, Value | ((Offset & 0x0f) << 7));
}

/***************************************************************//**
   \fn GetTMC5160ChopperTOff(UCHAR Motor)
   \brief Read the TOff parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \return        TOff parameter

  This function reads the TOff parameter.
********************************************************************/
UCHAR GetTMC5160ChopperTOff(UCHAR Motor)
{
  if(!DriverDisableFlag[Motor])
    return ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & 0x0000000f;
  else
    return LastTOffSetting[Motor];
}

/***************************************************************//**
   \fn GetTMC5160ChopperHysteresisStart(UCHAR Motor)
   \brief Read the HSTART parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \return        Hysteresis start parameter.

  This function reads the HSTART parameter.
********************************************************************/
UCHAR GetTMC5160ChopperHysteresisStart(UCHAR Motor)
{
  return (ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) >> 4) & 0x07;
}

/***************************************************************//**
   \fn GetTMC5160ChopperHysteresisEnd(UCHAR Motor)
   \brief Read the HEND parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \return        Hysteresis end parameter.

  This function sets the HEND parameter.
********************************************************************/
UCHAR GetTMC5160ChopperHysteresisEnd(UCHAR Motor)
{
  return (ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) >> 7) & 0x0f;
}

/***************************************************************//**
   \fn GetTMC5160ChopperBlankTime(UCHAR Motor)
   \brief Read the chopper blank time parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \return        Chopper blank time.

  This function reads the chopper blank time parameter.
********************************************************************/
UCHAR GetTMC5160ChopperBlankTime(UCHAR Motor)
{
  return (ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) >> 15) & 0x03;
}

/***************************************************************//**
   \fn GetTMC5160ChopperSync(UCHAR Motor)
   \brief Read the chopper synchronization parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \return        Chopper synchronization.

  This function reads the chopper synchronization parameter.
********************************************************************/
UCHAR GetTMC5160ChopperSync(UCHAR Motor)
{
  return (ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) >> 20) & 0x0f;
}

/***************************************************************//**
   \fn GetTMC5160ChopperMStepRes(UCHAR Motor)
   \brief Read microstep resolution.
   \param Motor   Axis number (with stepRocker always 0)
   \return        Microstep resolution (0..7).

  This function reads the microstep resolution.
********************************************************************/
UCHAR GetTMC5160ChopperMStepRes(UCHAR Motor)
{
  return (ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) >> 24) & 0x0f;
}

/***************************************************************//**
   \fn GetTMC5160ChopperDisableShortToGround(UCHAR Motor)
   \brief Check if short to ground detection is disabled.
   \param Motor   Axis number (with stepRocker always 0)
   \return      TRUE: short to ground detection off.\n
                FALSE: short to ground detection on.

  This function checks if the short to ground detection is enabled or
  disabled.
********************************************************************/
UCHAR GetTMC5160ChopperDisableShortToGround(UCHAR Motor)
{
  return ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & BIT30 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5160ChopperVHighChm(UCHAR Motor)
   \brief Check VHIGHCHM flag.
   \param Motor   Axis number (with stepRocker always 0)
   \return   State of VHighChm bit.

  This function reads back the VHIGHCHM flag.
********************************************************************/
UCHAR GetTMC5160ChopperVHighChm(UCHAR Motor)
{
  return ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & BIT19 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5160ChopperVHighFs(UCHAR Motor)
   \brief Check VHIGHFS flag.
   \param Motor   Axis number (with stepRocker always 0)
   \return   State of VHighFs bit.

  This function reads back the VHIGHFS flag.
********************************************************************/
UCHAR GetTMC5160ChopperVHighFs(UCHAR Motor)
{
  return ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & BIT18 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5160ChopperConstantTOffMode(UCHAR Motor)
   \brief Check if constant chopper mode is switched on or off.
   \param Motor   Axis number (with stepRocker always 0)
   \return        TRUE/FALSE

  This function checks if the constant chopper mode is selected.
********************************************************************/
UCHAR GetTMC5160ChopperConstantTOffMode(UCHAR Motor)
{
  return ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & BIT14 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5160ChopperRandomTOff(UCHAR Motor)
   \brief Check if random TOff mode is switched on.
   \param Motor   Axis number (with stepRocker always 0)
   \return        TRUE/FALSE.

  This function disables or enables the random chopper mode.
********************************************************************/
UCHAR GetTMC5160ChopperRandomTOff(UCHAR Motor)
{
  return ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & BIT13 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5160ChopperDisableFastDecayComp(UCHAR Motor)
   \brief Read back the fast decay comparator setting.
   \param Motor   Axis number (with stepRocker always 0)
   \return   Fast decay comparator setting.

  This function reads back the fast decay comparator disable setting.
********************************************************************/
UCHAR GetTMC5160ChopperDisableFastDecayComp(UCHAR Motor)
{
  return ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) & BIT12 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5160ChopperFastDecayTime(UCHAR Motor)
   \brief Read back the fast decay time.
   \param Motor   Axis number (with stepRocker always 0)
   \return   Fast decay time (0..15).

  This function reads back the fast decay time.
********************************************************************/
UCHAR GetTMC5160ChopperFastDecayTime(UCHAR Motor)
{
  UINT Value;
  UCHAR Time;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF);
  Time=(Value >> 4) & 0x07;
  if(Value & BIT11) Time|=BIT3;

  return Time;
}

/***************************************************************//**
   \fn GetTMC5160ChopperSineWaveOffset(UCHAR Motor)
   \brief Read the sine offset value.
   \param Motor   Axis number (with stepRocker always 0)
   \return   Sine Wave Offset (0..15).

  This function reads back the sine wave offset.
********************************************************************/
UCHAR GetTMC5160ChopperSineWaveOffset(UCHAR Motor)
{
  return (ReadTMC5160Int(WHICH_5160(Motor), TMC5160_CHOPCONF) >> 7) & 0x0f;
}


/*************************************************************************//**
  \fn SetTMC5160SmartEnergyUpStep(UCHAR Motor, UCHAR UpStep)
  \brief Set smart energy up step
  \param Motor        Axis number (with stepRocker always 0)
  \param UpStep       up step width (0..3)

  This function sets the current up step width used with coolStep, where 0 ist
  the lowest and 3 is the highest up step width.
*****************************************************************************/
void SetTMC5160SmartEnergyUpStep(UCHAR Motor, UCHAR UpStep)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) & 0xffffff9f;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF, Value | ((UpStep & 0x03) << 5));
}

/*************************************************************************//**
  \fn SetTMC5160SmartEnergyDownStep(UCHAR Motor, UCHAR DownStep)
  \brief Set smart energy down step
  \param Motor          Axis number (with stepRocker always 0)
  \param DownStep       down step speed (0..3)

  This function sets the current down step speed used with coolStep, where 0 ist
  the highest and 3 is the lowest speed.
*****************************************************************************/
void SetTMC5160SmartEnergyDownStep(UCHAR Motor, UCHAR DownStep)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) & 0xffff9fff;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF, Value | ((DownStep & 0x03) << 13));
}

/*************************************************************************//**
  \fn SetTMC5160SmartEnergyStallLevelMax(UCHAR Motor, UCHAR Max)
  \brief Set smart enery hysteresis width
  \param Motor          Axis number (with stepRocker always 0)
  \param Max            hysteresis width (0..15)

  This function sets the SEMAX parameter which defines the width of the
  smart energy stall level hysteresis.
*****************************************************************************/
void SetTMC5160SmartEnergyStallLevelMax(UCHAR Motor, UCHAR Max)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) & 0xfffff0ff;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF, Value | ((Max & 0x0f) << 8));
}

/*************************************************************************//**
  \fn SetTMC5160SmartEnergyStallLevelMin(UCHAR Motor, UCHAR Min)
  \brief Set smart energy hysteresis start
  \param Motor          Axis number (with stepRocker always 0)
  \param Min            minimum stall level (0..15)

  This function sets the start point of the hysteresis used for coolStep.
  A value of 0 completely turns off coolStep.
*****************************************************************************/
void SetTMC5160SmartEnergyStallLevelMin(UCHAR Motor, UCHAR Min)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) & 0xfffffff0;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF, Value | (Min & 0x0f));
}

/*************************************************************************//**
  \fn SetTMC5160SmartEnergyStallThreshold(UCHAR Motor, char Threshold)
  \brief Set stallGuard threshold value
  \param Motor       Axis number (with stepRocker always 0)
  \param Threshold   stallGuard threshold (-63..+63)

  This function sets the stallGuard threshold value.
*****************************************************************************/
void SetTMC5160SmartEnergyStallThreshold(UCHAR Motor, char Threshold)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) & 0xff00ffff;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF, Value | ((Threshold & 0xff) << 16));
}

/*************************************************************************//**
  \fn SetTMC5160SmartEnergyIMin(UCHAR Motor, UCHAR IMin)
  \brief Set smart energy minimum current
  \param Motor      Axis number (with stepRocker always 0)
  \param IMin       Minimum current (0=1/2, 1=1/4 of current setting)

  This function sets the minimum current used with coolStep, which can
  be either 1/2 or 1/4 of the normal current setting.
*****************************************************************************/
void SetTMC5160SmartEnergyIMin(UCHAR Motor, UCHAR IMin)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF);
  if(IMin)
    Value|=BIT15;
  else
    Value&= ~BIT15;

  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5160SmartEnergyFilter(UCHAR Motor, UCHAR Filter)
  \brief Set stallGuard filter
  \param Motor             Axis number (with stepRocker always 0)
  \param Filter            stallGuard filter (0=off, 1=on)

  This function turns the stallGuard filter on or off.
*****************************************************************************/
void SetTMC5160SmartEnergyFilter(UCHAR Motor, UCHAR Filter)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF);
  if(Filter)
    Value|=BIT24;
  else
    Value&= ~BIT24;

  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF, Value);
}


/*************************************************************************//**
  \fn GetTMC5160SmartEnergyUpStep(UCHAR Motor)
  \brief Get current up step width
  \param Motor  Axis number (with stepRocker always 0)
  \return SEUP value

  This function reads back the current up step setting.
*****************************************************************************/
UCHAR GetTMC5160SmartEnergyUpStep(UCHAR Motor)
{
  return (ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) >> 5) & 0x03;
}

/*************************************************************************//**
  \fn GetTMC5160SmartEnergyDownStep(UCHAR Motor)
  \brief Get current down step speed
  \param Motor  Axis number (with stepRocker always 0)
  \return Current down step speed

  This function reads back the smart energy current down step speed setting.
*****************************************************************************/
UCHAR GetTMC5160SmartEnergyDownStep(UCHAR Motor)
{
  return (ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) >> 13) & 0x03;
}

/*************************************************************************//**
  \fn GetTMC5160SmartEnergyStallLevelMax(UCHAR Motor)
  \brief Get hystersis width
  \param Motor  Axis number (with stepRocker always 0)
  \return SEMAX value

  This function reads back the stall level maximum value (which is the coolStep
  hysteresis width).
*****************************************************************************/
UCHAR GetTMC5160SmartEnergyStallLevelMax(UCHAR Motor)
{
  return (ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) >> 8) & 0x0f;
}

/*************************************************************************//**
  \fn GetTMC5160SmartEnergyStallLevelMin(UCHAR Motor)
  \brief Get hysteresis start
  \param Motor  Axis number (with stepRocker always 0)
  \return hysteresis start

  This function reads back the smart energy minimum stall level (which is the
  start of the coolStep hystetesis).
*****************************************************************************/
UCHAR GetTMC5160SmartEnergyStallLevelMin(UCHAR Motor)
{
  return ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) & 0x0f;
}

/*************************************************************************//**
  \fn GetTMC5160SmartEnergyStallThreshold(UCHAR Motor)
  \brief Get stallGuard threshold setting
  \param Motor  Axis number (with stepRocker always 0)
  \return stallGuard threshold value

  This function reads back the stallGuard threshold value.
*****************************************************************************/
int GetTMC5160SmartEnergyStallThreshold(UCHAR Motor)
{
  int Value;

  Value=(ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) >> 16) & 0xff;
  if(Value & BIT7) Value|=0xffffff00;

  return Value;
}

/*************************************************************************//**
  \fn GetTMC5160SmartEnergyIMin(UCHAR Motor)
  \brief Get minimum current
  \param Motor  Axis number (with stepRocker always 0)
  \return Minimum currren

  This function reads back the smart energy minimum current setting.
*****************************************************************************/
UCHAR GetTMC5160SmartEnergyIMin(UCHAR Motor)
{
  if(ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) & BIT15)
    return 1;
  else
    return 0;
}

/*************************************************************************//**
  \fn GetTMC5160SmartEnergyFilter(UCHAR Motor)
  \brief Get stallGuard filter
  \param Motor  Axis number (with stepRocker always 0)
  \return stallGuard filter (0=off, 1=on)

  This function reads back the stallGuard filter setting.
*****************************************************************************/
UCHAR GetTMC5160SmartEnergyFilter(UCHAR Motor)
{
  if(ReadTMC5160Int(WHICH_5160(Motor), TMC5160_COOLCONF) & BIT24)
    return 1;
  else
    return 0;
}

/*************************************************************************//**
  \fn SetTMC5160PWMFreewheelMode(UCHAR Motor, UCHAR Mode)
  \brief Set freewheeling mode
  \param Motor  Axis number (with stepRocker always 0)
  \param Mode  Freeheeling mode (0=off, 1=LS short, 2=HS short, 3=freewheel)

  This function selects the freewheeling mode. StealthChop has to be active
  to make this setting become active.
*****************************************************************************/
void SetTMC5160PWMFreewheelMode(UCHAR Motor, UCHAR Mode)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF) & 0xffcfffff;
  Mode&=0x03;
  Value|=Mode << 20;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5160PWMSymmetric(UCHAR Motor, UCHAR Symmetric)
  \brief Set symmetric PWM mode
  \param Motor  Axis number (with stepRocker always 0)
  \param Symmetric  TRUE or FALSE

  This function switches PWM symmetric mode on or off.
*****************************************************************************/
void SetTMC5160PWMSymmetric(UCHAR Motor, UCHAR Symmetric)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF);
  if(Symmetric)
    Value|=BIT19;
  else
    Value&= ~BIT19;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5160PWMAutoscale(UCHAR Motor, UCHAR Autoscale)
  \brief Set autoscale mode for stealthChop
  \param Motor       Axis number (with stepRocker always 0)
  \param Autoscale   1=on,  0=off

  This function switches on or off the autoscale option for stealthChop.
*****************************************************************************/
void SetTMC5160PWMAutoscale(UCHAR Motor, UCHAR Autoscale)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF);
  if(Autoscale)
    Value|=BIT18;
  else
    Value&= ~BIT18;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5160PWMFrequency(UCHAR Motor, UCHAR Frequency)
  \brief Set PWM frequency
  \param Motor  Axis number (with stepRocker always 0)
  \param Frequency  PWM frequency setting (0..3)

  This function sets the TMC5160 PWM frequency.
*****************************************************************************/
void SetTMC5160PWMFrequency(UCHAR Motor, UCHAR Frequency)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF) & 0xfffcffff;
  Frequency&=0x03;
  Value|=Frequency << 16;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5160PWMGrad(UCHAR Motor, UCHAR PWMGrad)
  \brief Set stealthChop PWM gradient
  \param Motor  Axis number (with stepRocker always 0)
  \param PWMGrad  PWM gradient (0..15)

  This function sets the PWM gradient used for stealthChop.
*****************************************************************************/
void SetTMC5160PWMGrad(UCHAR Motor, UCHAR PWMGrad)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF) & 0xffff00ff;
  Value|=PWMGrad << 8;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5160PWMAmpl(UCHAR Motor, UCHAR PWMAmpl)
  \brief Set stealthChop PWM amplitude
  \param Motor  Axis number (with stepRocker always 0)
  \param PWMAmpl   PWM amplitude (0..255)

  This function sets the PWM amplitude used for stealthChop.
*****************************************************************************/
void SetTMC5160PWMAmpl(UCHAR Motor, UCHAR PWMAmpl)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF) & 0xffffff00;
  Value|=PWMAmpl;
  WriteTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF, Value);
}

/*************************************************************************//**
  \fn GetTMC5160PWMFreewheelMode(UCHAR Motor)
  \brief Read back freewheeling mode
  \param Motor  Axis number (with stepRocker always 0)
  \return       Freeheeling mode (0=off, 1=LS short, 2=HS short, 3=freewheel)

  This function reads the selected freewheeling mode.
*****************************************************************************/
UCHAR GetTMC5160PWMFreewheelMode(UCHAR Motor)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF);
  Value>>=20;

  return Value & 0x03;
}

/*************************************************************************//**
  \fn GetTMC5160PWMSymmetric(UCHAR Motor)
  \brief Read back symmetric PWM mode
  \param Motor  Axis number (with stepRocker always 0)
  \return       TRUE or FALSE

  This function reads back the PWM symmetric mode.
*****************************************************************************/
UCHAR GetTMC5160PWMSymmetric(UCHAR Motor)
{
  if(ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF) & BIT19)
    return TRUE;
  else
    return FALSE;
}

/*************************************************************************//**
  \fn GetTMC5160PWMAutoscale(UCHAR Motor)
  \brief Read back stealthChop autoscale mode
  \param Motor  Axis number (with stepRocker always 0)
  \return       1=on\n
                0=off

  This function reads back the state of the autoscale option for stealthChop.
*****************************************************************************/
UCHAR GetTMC5160PWMAutoscale(UCHAR Motor)
{
  if(ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF) & BIT18)
    return TRUE;
  else
    return FALSE;
}

/*************************************************************************//**
  \fn GetTMC5160PWMFrequency(UCHAR Motor)
  \brief Read back PWM frequency
  \param Motor  Axis number (with stepRocker always 0)
  \return       PWM frequency setting (0..3)

  This function reads back the TMC5160 PWM frequency.
*****************************************************************************/
UCHAR GetTMC5160PWMFrequency(UCHAR Motor)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF);
  Value>>=16;

  return Value & 0x03;
}

/*************************************************************************//**
  \fn GetTMC5160PWMGrad(UCHAR Motor)
  \brief Read back stealthChop PWM gradient
  \param Motor  Axis number (with stepRocker always 0)
  \return       PWM gradient (0..15)

  This function reads back the PWM gradient used for stealthChop.
*****************************************************************************/
UCHAR GetTMC5160PWMGrad(UCHAR Motor)
{
  UINT Value;

  Value=ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF);
  Value>>=8;

  return Value & 0xff;
}

/*************************************************************************//**
  \fn GetTMC5160PWMAmpl(UCHAR Motor)
  \brief Set stealthChop PWM amplitude
  \param Motor  Axis number (with stepRocker always 0)
  \return       PWM amplitude (0..255)

  This function reads back the PWM amplitude used for stealthChop.
*****************************************************************************/
UCHAR GetTMC5160PWMAmpl(UCHAR Motor)
{
   return ReadTMC5160Int(WHICH_5160(Motor), TMC5160_PWMCONF) & 0xff;
}


/***************************************************************//**
   \fn Read5160State(UCHAR Which5160, UINT *StallGuard, UCHAR *SmartEnergy, UCHAR *Flags)
   \brief Get TMC5160 status from TMC4361 polling mechanism
   \param Which5160  Index of TMC5160 to be used (with stepRocker always 0)
   \param StallGuard   Pointer at UINT for stallGuard value
   \param SmartEnergy  Pointer at UCHAR for smartEnergy value
   \param Flags   Pointer at UCHAR for driver error flags

   This function reads status information from the TMC5160 motor driver
   without using extra cover datagrams. For this purpose the automatic
   polling mechanism of the TMC4361 is used, so this must have been
   activated during initializtion of the TMC4361.
   The values are extracted from the datagrams.

   NULL pointers can be used for values that are not needed.
********************************************************************/
void Read5160State(UCHAR Which5160, UINT *StallGuard, UCHAR *SmartEnergy, UCHAR *Flags)
{
  UINT DrvStatus;

  DrvStatus=ReadTMC43xxInt(Which5160, TMC43xx_POLLING_1_R);
  if(StallGuard!=NULL) *StallGuard=DrvStatus & 0x3ff;
  if(SmartEnergy!=NULL) *SmartEnergy=(DrvStatus>>16) & 0x1f;
  if(Flags!=NULL) *Flags=(DrvStatus>>24) & 0xff;
}


/***************************************************************//**
   \fn InitMotorDrivers(void)
   \brief Initialise all motor drivers

   This function initalizes the software copies of all TMC5160
   registers and sends this basic initialization data to all
   TMC5160 ICs.
********************************************************************/
void InitMotorDrivers(void)
{
  int Delay;
  int i;

  //Short delay (TMC5160 power-on reset)
  Delay=GetSysTimer();
  while(abs(GetSysTimer()-Delay)<10);

  for(i=0; i<N_O_MOTORS; i++)
  {
    WriteTMC5160Int(i, TMC5160_GCONF, TMC5160_GCONF_DIRECT_MODE|TMC5160_GCONF_RECALIBRATE);
    WriteTMC5160Datagram(i, TMC5160_CHOPCONF, 0x00, 0x41, 0x02, 0x53);
    WriteTMC5160Datagram(i, TMC5160_SHORT_CONF, 0x00, 0x00, 0x03, 0x06);
    WriteTMC5160Datagram(i, TMC5160_DRV_CONF, 0x00, 0x01, 0x00, 0x0A);
    WriteTMC5160Datagram(i, TMC5160_IHOLD_IRUN, 0x00, 0x00, 0x1f, 0x1f);
    WriteTMC5160Int(i, TMC5160_PWMCONF, 0xC40C001E);  //Reset default value
    LastTOffSetting[i]=GetTMC5160ChopperTOff(i);
    DriverDisableFlag[i]=FALSE;
  }
}

/***************************************************************//**
   \fn DisableTMC5160(UCHAR Motor)
   \brief Disable a motor driver
   \param Motor  Axis number (always 0 on stepRocker)

   Completely switch off a motor driver (by setting its TOff value
   to zero).
********************************************************************/
void DisableTMC5160(UCHAR Motor)
{
  UCHAR TOff;

  if(!DriverDisableFlag[Motor])
  {
    TOff=LastTOffSetting[Motor];
    SetTMC5160ChopperTOff(Motor, 0);
    DriverDisableFlag[Motor]=TRUE;
    LastTOffSetting[Motor]=TOff;
  }
}

/***************************************************************//**
   \fn EnableTMC5160(UCHAR Motor)
   \brief Enable a motor driver
   \param Motor  Axis number (always 0 on stepRocker)

   Re-enable a motor driver (by setting its TOff value back to the
   original value).
********************************************************************/
void EnableTMC5160(UCHAR Motor)
{
  if(DriverDisableFlag[Motor])
  {
    DriverDisableFlag[Motor]=FALSE;
    SetTMC5160ChopperTOff(Motor, LastTOffSetting[Motor]);
  }
}
