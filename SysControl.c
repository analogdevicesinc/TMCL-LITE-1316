/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  SysControl.c
           Motor monitoring (automatic current switching etc.)

   Copyright (C) 2016 TRINAMIC Motion Control GmbH & Co KG
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
  \file SysControl.c
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Motor monitoring

  This file contains the SystemControl function which does all necessary motor
  monitoring tasks.
*/

#include <limits.h>
#include <stdlib.h>
#if defined(MK20DX128)
  #include "derivative.h"
#elif defined(GD32F425)
  #include "gd32f4xx.h"
#endif
#include "bits.h"
#include "stealthRocker.h"
#include "Globals.h"
#include "IO.h"
#include "SysTick.h"
#include "TMC4361.h"
#include "TMC5160.h"

typedef enum
{CS_OPENLOOP, CS_START, CS_WAIT_FS, CS_CALIBRATE, CS_WAIT_CALIBRATE, CS_CLOSEDLOOP} TClosedLoopInitState;

typedef enum
{TQ_MODE_OFF, TQ_MODE_ROL, TQ_MODE_ROR} TTorqueMode;

static UCHAR ActualAxis;                      //!< monitored axis
static UINT Delay;                            //!< Delay timer
static UCHAR StopOnStallState[N_O_MOTORS];    //!< status of the stop-on-stall functionality
static TClosedLoopInitState ClosedLoopInitState[N_O_MOTORS];  //!< closed-loop initialization status
static UINT ClosedLoopTimer[N_O_MOTORS];      //!< timer needed for closed-loop initiaization
static TTorqueMode TorqueMode[N_O_MOTORS];    //!< torque mode (FALSE=off, TRUE=on)
static int TorqueModeCurrent[N_O_MOTORS];     //!< desired current in torque mode

void StopTorqueMode(UCHAR Axis);


/***************************************************************//**
   \fn GetClosedLoopInitFlag(UCHAR Axis)
   \brief Check status of closed-loop initialization

   \param Axis   Axis number (always 0 with the stepRocker)
   \return       TRUE: closed loop is initalized\n
                 FALSE: closed loop not initialized

   Check if the closed loop function has been initialized.
********************************************************************/
UCHAR GetClosedLoopInitFlag(UCHAR Axis)
{
  if(ClosedLoopInitState[Axis]==CS_CLOSEDLOOP)
    return TRUE;
  else
    return FALSE;
}


/***************************************************************//**
   \fn InitClosedLoop(UCHAR Axis)
   \brief Initialize closed loop
   \param Axis Motor number (always 0 with the stepRocker)

   Initalizes or de-initializes the closed loop function of the
   given axis (depending on MotorConfig.ClosedLoopMode). This
   function gets called periodically by SystemControl().
********************************************************************/
static void InitClosedLoop(UCHAR Axis)
{
  int x;

  switch(ClosedLoopInitState[Axis])
  {
    case CS_OPENLOOP:
      //Switch to closed loop
      if(ClosedLoopConfig[Axis].ClosedLoopMode)
      {
      	//Current scaling also in the driver.
        WriteTMC5160Int(Axis, TMC5160_IHOLD_IRUN, (ReadTMC5160Int(Axis, TMC5160_IHOLD_IRUN) & 0xffffe0ff) | ((MotorConfig[Axis].IRun/8)<<8));
        ClosedLoopInitState[Axis]=CS_START;
      }
      break;

    case CS_START:
      //Turn off current scaling, start moving to a full step
      WriteTMC43xxInt(Axis, TMC43xx_CURRENT_CONF, 0);
      WriteTMC43xxBytes(Axis, TMC43xx_SCALE_VALUES, 0, ClosedLoopConfig[Axis].CurrentScalerStartUp,
                        ClosedLoopConfig[Axis].CurrentScalerMaximum,
                        ClosedLoopConfig[Axis].CurrentScalerMinimum);
      //Number of microstepd until next full step
      x=ReadTMC43xxInt(Axis, TMC43xx_MSCNT);
      if(x<128) x=128-x;
      else if(x<384) x=384-x;
      else if(x<640) x=640-x;
      else if(x<896) x=896-x;
      else x=1152-x;  //Overflow: 1152 & 0xff = 128
      WriteTMC43xxInt(Axis, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_POS_HOLD);
      WriteTMC43xxInt(Axis, TMC43xx_VMAX, 1000000);
      WriteTMC43xxInt(Axis, TMC43xx_XTARGET, ReadTMC43xxInt(Axis, TMC43xx_XACTUAL)+x);
      ClosedLoopTimer[Axis]=GetSysTimer();
      ClosedLoopInitState[Axis]=CS_WAIT_FS;
      break;

    case CS_WAIT_FS:
      //Wait until full step has been reached.
      if(abs(GetSysTimer()-ClosedLoopTimer[Axis])>1000)
      {
        if(MotorConfig[Axis].EncoderResolution!=0)
        {
          WriteTMC43xxInt(Axis, TMC43xx_ENC_POS, ReadTMC43xxInt(Axis, TMC43xx_XACTUAL));
        }
        else
        {
          WriteTMC43xxInt(Axis, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_VEL_HOLD);
          WriteTMC43xxInt(Axis, TMC43xx_VMAX, 0);
          WriteTMC43xxInt(Axis, TMC43xx_XACTUAL, ReadTMC43xxInt(Axis, TMC43xx_ENC_POS));
          WriteTMC43xxInt(Axis, TMC43xx_XTARGET, ReadTMC43xxInt(Axis, TMC43xx_XACTUAL));
          WriteTMC43xxInt(Axis, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_POS_HOLD);
          WriteTMC43xxInt(Axis, TMC43xx_VMAX, 1000000);
          VMaxModified[Axis]=TRUE;
        }
        ClosedLoopInitState[Axis]=CS_CALIBRATE;
      }
      break;

    case CS_CALIBRATE:
      //Start closed loop calibration of the TMC4361
      TMC43xxClearBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VLIMIT_EN|TMC43xx_ENC_IN_CL_VELOCITY_EN);
      TMC43xxSetBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_MODE_CL|TMC43xx_ENC_IN_CL_CALIBRATION_EN|TMC43xx_ENC_IN_SER_VAR_LIMIT);
      ClosedLoopTimer[Axis]=GetSysTimer();
      ClosedLoopInitState[Axis]=CS_WAIT_CALIBRATE;
      break;

    case CS_WAIT_CALIBRATE:
      //Wait until calibration finished
      if(abs(GetSysTimer()-ClosedLoopTimer[Axis])>1000)
      {
        TMC43xxClearBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_CALIBRATION_EN|TMC43xx_ENC_IN_SER_VAR_LIMIT|
                         TMC43xx_ENC_IN_CL_VELOCITY_EN|TMC43xx_ENC_IN_CL_VLIMIT_EN);

        if(MotorConfig[Axis].StepDirMode==0 || (MotorConfig[Axis].StepDirMode & BIT3))
          TMC43xxSetBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_EMF_EN|TMC43xx_ENC_IN_CL_VLIMIT_EN);
        else
          TMC43xxSetBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_EMF_EN);

        WriteTMC43xxInt(Axis, TMC43xx_CURRENT_CONF, TMC43xx_CURCONF_CL_SCALE_EN);
        ClosedLoopConfig[Axis].Offset=ReadTMC43xxInt(Axis, TMC43xx_CL_OFFSET);
        ClosedLoopPositionOffset[Axis]=0;
        EncoderOffset[Axis]=ReadTMC43xxInt(Axis, TMC43xx_ENC_POS)-ReadTMC43xxInt(Axis, TMC43xx_XACTUAL);
        ClosedLoopInitState[Axis]=CS_CLOSEDLOOP;
      }
      break;

    case CS_CLOSEDLOOP:
      //Switch back to open loop
      if(!ClosedLoopConfig[Axis].ClosedLoopMode)
      {
        StopTorqueMode(Axis);
        TMC43xxClearBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_MODE_CL|TMC43xx_ENC_IN_CL_VLIMIT_EN);
        WriteTMC43xxInt(Axis, TMC43xx_CURRENT_CONF, TMC43xx_CURCONF_HOLD_EN|TMC43xx_CURCONF_DRIVE_EN);
        WriteTMC43xxBytes(Axis, TMC43xx_SCALE_VALUES, MotorConfig[Axis].IStandby, 0,
                          MotorConfig[Axis].IRun, MotorConfig[Axis].BoostCurrent);
        WriteTMC5160Int(Axis, TMC5160_IHOLD_IRUN, (ReadTMC5160Int(Axis, TMC5160_IHOLD_IRUN) & 0xffffe0ff) | (31<<8));  //Scaling only happens in the TMC4361
        ClosedLoopInitState[Axis]=CS_OPENLOOP;
        if(MotorConfig[Axis].RampType==RAMP_TRAPEZ)
          WriteTMC43xxInt(Axis, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_VEL_TRAPEZ);
        else
          WriteTMC43xxInt(Axis, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_VEL_SSHAPE);
        WriteTMC43xxInt(Axis, TMC43xx_VMAX, 0);
        WriteTMC43xxInt(Axis, TMC43xx_XACTUAL, ReadTMC43xxInt(Axis, TMC43xx_XACTUAL)-ClosedLoopPositionOffset[Axis]);
        VMaxModified[Axis]=TRUE;
      }
      break;
  }
}


/***************************************************************//**
   \fn SystemControl(void)
   \brief Motor monitoring

   This function must be called periodically from the main loop and
   does some monitoring tasks, e.g. lowering the current after the
   motor has not been moving for some time.
********************************************************************/
void SystemControl(void)
{
  UINT Events;
  int ActualVelocity;

  ActualVelocity=ReadTMC43xxInt(ActualAxis, TMC43xx_VACTUAL)<<8;
  Events=PeekTMC43xxEvents(ActualAxis);

  //Switch stop-on-stall depending on velocity
  if(MotorConfig[ActualAxis].StallVMin>0 && abs(ActualVelocity)>abs(MotorConfig[ActualAxis].StallVMin))
  {
    if(!StopOnStallState[ActualAxis])
    {
      TMC43xxSetBits(ActualAxis, TMC43xx_REFERENCE_CONF, TMC43xx_REFCONF_STOP_ON_STALL);
      StopOnStallState[ActualAxis]=TRUE;
    }
  }
  else
  {
    if(StopOnStallState[ActualAxis])
    {
      TMC43xxClearBits(ActualAxis, TMC43xx_REFERENCE_CONF, TMC43xx_REFCONF_STOP_ON_STALL);
      StopOnStallState[ActualAxis]=FALSE;
    }
  }

  //Reset end switch events when needed
  if(Events & TMC43xx_EV_STOP_LEFT)
  {
    if(ActualVelocity==0)
    {
      ReadAndClearTMC43xxEvents(ActualAxis, TMC43xx_EV_STOP_LEFT);
    }
  }

  if(Events & TMC43xx_EV_STOP_RIGHT)
  {
    if(ActualVelocity==0)
    {
      ReadAndClearTMC43xxEvents(ActualAxis, TMC43xx_EV_STOP_RIGHT);
    }
  }

  //Reset soft limit events when needed
  if(Events & TMC43xx_EV_VIRT_STOP_LEFT)
  {
    if(ActualVelocity==0)
    {
      ReadAndClearTMC43xxEvents(ActualAxis, TMC43xx_EV_VIRT_STOP_LEFT);
    }
  }

  if(Events & TMC43xx_EV_VIRT_STOP_RIGHT)
  {
    if(ActualVelocity==0)
    {
      ReadAndClearTMC43xxEvents(ActualAxis, TMC43xx_EV_VIRT_STOP_RIGHT);
    }
  }

  //Deviation error handling
  if(MotorConfig[ActualAxis].MaxPositionDeviation>0 && (Events & TMC43xx_EV_ENC_FAIL))
  {
    HardStop(ActualAxis);
    WriteTMC43xxInt(ActualAxis, TMC43xx_XACTUAL, ReadTMC43xxInt(ActualAxis, TMC43xx_ENC_POS)-EncoderOffset[ActualAxis]);
    ReadAndClearTMC43xxEvents(ActualAxis, TMC43xx_EV_ENC_FAIL);
    DeviationFlag[ActualAxis]=TRUE;
  }

  //10ms tasks
  if(abs(GetSysTimer()-Delay)>10)
  {
    //Monitoring of the motor driver
    Read5160State(WHICH_5160(ActualAxis), &StallLevel[ActualAxis], &SmartEnergy[ActualAxis], &DriverFlags[ActualAxis]);

    //Closed loop initialization
    InitClosedLoop(ActualAxis);

    //Torque mode
    if(ClosedLoopConfig[ActualAxis].ClosedLoopMode)
    {
      switch(TorqueMode[ActualAxis])
      {
        case TQ_MODE_ROL:
          WriteTMC43xxInt(ActualAxis, TMC43xx_XACTUAL, ReadTMC43xxInt(ActualAxis, TMC43xx_ENC_POS)-100000);
          break;

        case TQ_MODE_ROR:
          WriteTMC43xxInt(ActualAxis, TMC43xx_XACTUAL, ReadTMC43xxInt(ActualAxis, TMC43xx_ENC_POS)+100000);
          break;

        default:
          break;
      }
    }

    //next motor (stepRocker servo only has one)
    ActualAxis++;
    if(ActualAxis>=N_O_MOTORS) ActualAxis=0;

    Delay=GetSysTimer();
  }
}

/***************************************************************//**
   \fn SetStepDirMode(UCHAR Axis, UCHAR Mode)
   \brief Switch to step/direction mode
   \param Axis  Axis number (always 0 with the stepRocker)
   \param Mode  Desired mode


   Switch to step/direction mode or back to normal ramp generator
   mode.\n

   Possible Modes:\n
    0: Ramp generator mode\n
    1: step/direction mode, step pulse are high\n
    2: step/direction mode, step pulse are low\n
    3: step/direction mode, both edges of each step pulse (double speed)\n
    Bit 2: direction signal polarity\n
    Bit 3: indirect mode (ramp generator controlled by step pulses)
********************************************************************/
void SetStepDirMode(UCHAR Axis, UCHAR Mode)
{
  UINT Value;

  if(Mode==0)
  {
    Value=ReadTMC43xxInt(Axis, TMC43xx_GENERAL_CONF);
    WriteTMC43xxInt(Axis, TMC43xx_GENERAL_CONF,
      Value & ~(TMC43xx_GCONF_EXT_SD_TOGGLE|TMC43xx_GCONF_DIR_IN_POL));

    if(ClosedLoopConfig[Axis].ClosedLoopMode)
      TMC43xxSetBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VLIMIT_EN);
  }
  else
  {
    Value=ReadTMC43xxInt(Axis, TMC43xx_GENERAL_CONF);
    Value&= ~(TMC43xx_GCONF_EXT_SD_TOGGLE|TMC43xx_GCONF_DIR_IN_POL);
    switch(Mode & 0x03)
    {
      case 1:
        Value|=TMC43xx_GCONF_EXT_SD_HIGH;
        break;

      case 2:
        Value|=TMC43xx_GCONF_EXT_SD_LOW;
        break;

      case 3:
        Value|=TMC43xx_GCONF_EXT_SD_TOGGLE;
        break;
    }
    if(Mode & BIT2) Value|=TMC43xx_GCONF_DIR_IN_POL;
    if(Mode & BIT3) Value|=TMC43xx_GCONF_SD_INDIRECT;
    WriteTMC43xxInt(Axis, TMC43xx_GENERAL_CONF, Value);

    if(!(Mode & BIT3))
      TMC43xxClearBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VLIMIT_EN);
    else
      TMC43xxSetBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VLIMIT_EN);
  }
}


/***************************************************************//**
   \fn StartTorqueMode(UCHAR Axis, int Torque)
   \brief Run motor in torque mode.
   \param Axis  Axis number (always 0 with stepRocker)
   \param Torque  Torque mode direction and current (-255..+255)

   Switch on torque mode and set the torque mode current to be used.
   This only works in closed loop mode.
********************************************************************/
void StartTorqueMode(UCHAR Axis, int Torque)
{
  if(TorqueMode[Axis]==TQ_MODE_OFF)
  {
    TMC43xxClearBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VELOCITY_EN|TMC43xx_ENC_IN_CL_VLIMIT_EN);
  }

  WriteTMC43xxBytes(Axis, TMC43xx_SCALE_VALUES, 0, ClosedLoopConfig[Axis].CurrentScalerStartUp, abs(Torque), 1);
  TorqueMode[Axis]=(Torque<0) ? TQ_MODE_ROL:TQ_MODE_ROR;
  TorqueModeCurrent[Axis]=Torque;
}


/***************************************************************//**
   \fn StopTorqueMode(UCHAR Axis)
   \brief Turn off torque mode
   \param Axis  Axis number (alwaxs 0 with the stepRocker)

   Switch off the torque mode. Afterwards the motr will be in
   velocity mode with velocity=0 (stopped).
********************************************************************/
void StopTorqueMode(UCHAR Axis)
{
  if(TorqueMode[Axis]!=TQ_MODE_OFF)
  {
    TorqueMode[Axis]=TQ_MODE_OFF;
    TorqueModeCurrent[Axis]=0;
    TMC43xxSetBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VELOCITY_EN);
    TMC43xxClearBits(Axis, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VLIMIT_EN);

    if(ClosedLoopConfig[Axis].ClosedLoopMode)
    {
      WriteTMC43xxInt(Axis, TMC43xx_XACTUAL, ReadTMC43xxInt(Axis, TMC43xx_ENC_POS));
      WriteTMC43xxBytes(Axis, TMC43xx_SCALE_VALUES, 0,
                        ClosedLoopConfig[Axis].CurrentScalerStartUp,
                        ClosedLoopConfig[Axis].CurrentScalerMaximum,
                        ClosedLoopConfig[Axis].CurrentScalerMinimum);
    }
  }
}


/***************************************************************//**
   \fn GetTorqueModeCurrent(UCHAR Axis)
   \brief Read actual torque mode current setting
   \param Axis  Axis number (always 0 with the stepRocker)
   \return -255..+255 when in torque mode\n
           0 when not in torque mode

   Read back the actual torque mode current setting.
********************************************************************/
int GetTorqueModeCurrent(UCHAR Axis)
{
  return TorqueModeCurrent[Axis];
}
