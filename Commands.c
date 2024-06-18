/*******************************************************************************
* Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file all functions necessary to implement a small TMCL interpreter.
*/

#include <stdlib.h>
#include <stddef.h>
#if defined(MK20DX128)
  #include "derivative.h"
#elif defined(GD32F425)
  #include "gd32f4xx.h"
#endif
#include "bits.h"
#include "stealthRocker.h"
#include "Commands.h"
#include "Globals.h"
#include "RS485.h"
#include "SysTick.h"
#include "SysControl.h"
#include "TMC4361.h"
#include "TMC5160.h"
#include "Eeprom.h"
#include "IO.h"
#include "Can.h"
#include "USB.h"


#ifdef BOOTLOADER
extern UINT BLMagic;                        //!< Magic code for bootloader
#endif

#define MAX_PPS_ACC   4194303               //!< Maximum possible acceleration/deceleration in PPS/S

//Variables
static UCHAR UARTCmd[9];                    //!< RS485 command buffer
static UCHAR UARTCount;                     //!< RS485 commnd byte counter
static UCHAR TMCLCommandState;              //!< State of the interpreter
static TTMCLCommand ActualCommand;          //!< TMCL command to be executed (with all parameters)
static TTMCLReply ActualReply;              //!< Reply of last executed TMCL command
static UCHAR TMCLReplyFormat;               //!< format of next reply (RF_NORMAL or RF_SPECIAL)
static UCHAR SpecialReply[9];               //!< buffer for special replies
static UCHAR ResetRequested;                //!< TRUE after executing the software reset command
static UCHAR ExtendedCANFrame;              //!< TRUE when extended CAN frame used

static UCHAR RelativePositioningOptionCode[N_O_MOTORS];  //!< Option code for MVP REL command
static int VMax[N_O_MOTORS];                             //!< Maximum positioning speed

typedef int (*PConvertFunction)(int);                    //!< Function pointer type definition for velocity and acceleration conversion functions
static PConvertFunction VelocityToInternal[N_O_MOTORS];      //!< Pointer to velocity conversion function from PPS to TMC4361
static PConvertFunction VelocityToUser[N_O_MOTORS];          //!< Pointer to velocity conversion function from TMC4361 to PPS
static PConvertFunction AccelerationToInternal[N_O_MOTORS];  //!< Pointer to acceleration conversion function from PPS to TMC4361
static PConvertFunction AccelerationToUser[N_O_MOTORS];      //!< Pointer to acceleration conversion function from TMC4361 to PPS

//Prototypes
static void RotateRight(void);
static void RotateLeft(void);
static void MotorStop(void);
static void MoveToPosition(void);
static void SetAxisParameter(void);
static void GetAxisParameter(void);
static void GetVersion(void);
static void Boot(void);
static void SoftwareReset(void);


//Imported variables
extern char VersionString[];   //!< Imported version string


//Functions

/***************************************************************//**
   \fn ExecuteActualCommand()
   \brief Execute actual TMCL command

   Execute the TMCL command that must have been written
   to the global variable "ActualCommand" before.
********************************************************************/
static void ExecuteActualCommand(void)
{
  //Prepare answer
  ActualReply.Opcode=ActualCommand.Opcode;
  ActualReply.Status=REPLY_OK;
  ActualReply.Value.Int32=ActualCommand.Value.Int32;

  //Execute command
  switch(ActualCommand.Opcode)
  {
    case TMCL_ROR:
      RotateRight();
      break;

    case TMCL_ROL:
      RotateLeft();
      break;

    case TMCL_MST:
      MotorStop();
      break;

    case TMCL_MVP:
      MoveToPosition();
      break;

    case TMCL_SAP:
      SetAxisParameter();
      break;

    case TMCL_GAP:
      GetAxisParameter();
      break;

    case TMCL_GetVersion:
      GetVersion();
      break;

    case TMCL_Boot:
      Boot();
      break;

    case TMCL_SoftwareReset:
      SoftwareReset();
      break;

    default:
      ActualReply.Status=REPLY_INVALID_CMD;
      break;
  }
}


/***************************************************************//**
   \fn InitTMCL(void)
   \brief Initialize TMCL interpreter

   Intialise the TMCL interpreter. Must be called once at startup.
********************************************************************/
void InitTMCL(void)
{
  UINT i;

  TMCLCommandState=TCS_IDLE;
  for(i=0; i<N_O_MOTORS; i++)
  {
    VMax[i]=ReadTMC43xxInt(i, TMC43xx_VMAX);
    VMaxModified[i]=FALSE;
    if(MotorConfig[i].UnitMode==UNIT_MODE_INTERNAL)
    {
      VelocityToInternal[i]=ConvertInternalToInternal;
      VelocityToUser[i]=ConvertInternalToInternal;
      AccelerationToInternal[i]=ConvertInternalToInternal;
      AccelerationToUser[i]=ConvertInternalToInternal;
    }
    else
    {
      VelocityToInternal[i]=ConvertVelocityUserToInternal;
      VelocityToUser[i]=ConvertVelocityInternalToUser;
      AccelerationToInternal[i]=ConvertAccelerationUserToInternal;
      AccelerationToUser[i]=ConvertAccelerationInternalToUser;
    }

    MotorConfig[i].AMax=AccelerationToUser[i](ReadTMC43xxInt(i, TMC43xx_AMAX));
    MotorConfig[i].DMax=AccelerationToUser[i](ReadTMC43xxInt(i, TMC43xx_DMAX));
    MotorConfig[i].AStart=AccelerationToUser[i](ReadTMC43xxInt(i, TMC43xx_ASTART));
    MotorConfig[i].DFinal=AccelerationToUser[i](ReadTMC43xxInt(i, TMC43xx_DFINAL));
    MotorConfig[i].DStop=AccelerationToUser[i](ReadTMC43xxInt(i, TMC43xx_DSTOP));
  }
}


/***************************************************************//**
   \fn ProcessCommand(void)
   \brief Fetch and execute TMCL commands

   This is the main function for fetching and executing TMCL commands
   and has to be called periodically from the main loop.
********************************************************************/
void ProcessCommand(void)
{
  UCHAR Byte;
  UCHAR Checksum;
  UCHAR i;
  TCanFrame CanFrame;
  UCHAR USBCmd[9];
  UCHAR USBReply[9];

  //**Send answer for the last command**
  if(TMCLCommandState==TCS_CAN7 || TMCLCommandState==TCS_CAN8)  //via CAN
  {
    CanFrame.Id=ModuleConfig.CANSendID;
    CanFrame.Dlc=(TMCLCommandState==TCS_CAN7 ? 7:8);
    CanFrame.Ext=ExtendedCANFrame;
    CanFrame.Rtr=FALSE;

    if(TMCLReplyFormat==RF_STANDARD)
    {
      CanFrame.Data[0]=ModuleConfig.CANReceiveID & 0xff;
      CanFrame.Data[1]=ActualReply.Status;
      CanFrame.Data[2]=ActualReply.Opcode;
      CanFrame.Data[3]=ActualReply.Value.Byte[3];
      CanFrame.Data[4]=ActualReply.Value.Byte[2];
      CanFrame.Data[5]=ActualReply.Value.Byte[1];
      CanFrame.Data[6]=ActualReply.Value.Byte[0];
      CanFrame.Data[7]=0;
    }
    else if(TMCLReplyFormat==RF_SPECIAL)
    {
      for(i=0; i<8; i++)
        CanFrame.Data[i]=SpecialReply[i+1];
    }

    //Antwort senden
    if(!CanSendMessage(&CanFrame)) return;
  }
  else if(TMCLCommandState==TCS_UART)  //via UART
  {
    if(TMCLReplyFormat==RF_STANDARD)
    {
      Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
               ActualReply.Status+ActualReply.Opcode+
               ActualReply.Value.Byte[3]+
               ActualReply.Value.Byte[2]+
               ActualReply.Value.Byte[1]+
               ActualReply.Value.Byte[0];

      WriteRS485(ModuleConfig.SerialHostAddress);
      WriteRS485(ModuleConfig.SerialModuleAddress);
      WriteRS485(ActualReply.Status);
      WriteRS485(ActualReply.Opcode);
      WriteRS485(ActualReply.Value.Byte[3]);
      WriteRS485(ActualReply.Value.Byte[2]);
      WriteRS485(ActualReply.Value.Byte[1]);
      WriteRS485(ActualReply.Value.Byte[0]);
      WriteRS485(Checksum);
    }
    else if(TMCLReplyFormat==RF_SPECIAL)
    {
      for(i=0; i<9; i++)
      {
        WriteRS485(SpecialReply[i]);
      }
    }
  }
  else if(TMCLCommandState==TCS_UART_ERROR)  //check sum of the last command has been wrong
  {
    ActualReply.Opcode=0;
    ActualReply.Status=REPLY_CHKERR;
    ActualReply.Value.Int32=0;

    Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
             ActualReply.Status+ActualReply.Opcode+
             ActualReply.Value.Byte[3]+
             ActualReply.Value.Byte[2]+
             ActualReply.Value.Byte[1]+
             ActualReply.Value.Byte[0];

    WriteRS485(ModuleConfig.SerialHostAddress);
    WriteRS485(ModuleConfig.SerialModuleAddress);
    WriteRS485(ActualReply.Status);
    WriteRS485(ActualReply.Opcode);
    WriteRS485(ActualReply.Value.Byte[3]);
    WriteRS485(ActualReply.Value.Byte[2]);
    WriteRS485(ActualReply.Value.Byte[1]);
    WriteRS485(ActualReply.Value.Byte[0]);
    WriteRS485(Checksum);
  }
  else if(TMCLCommandState==TCS_USB)  //via USB
  {
    if(TMCLReplyFormat==RF_STANDARD)
    {
      Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
               ActualReply.Status+ActualReply.Opcode+
               ActualReply.Value.Byte[3]+
               ActualReply.Value.Byte[2]+
               ActualReply.Value.Byte[1]+
               ActualReply.Value.Byte[0];

      USBReply[0]=ModuleConfig.SerialHostAddress;
      USBReply[1]=ModuleConfig.SerialModuleAddress;
      USBReply[2]=ActualReply.Status;
      USBReply[3]=ActualReply.Opcode;
      USBReply[4]=ActualReply.Value.Byte[3];
      USBReply[5]=ActualReply.Value.Byte[2];
      USBReply[6]=ActualReply.Value.Byte[1];
      USBReply[7]=ActualReply.Value.Byte[0];
      USBReply[8]=Checksum;
    }
    else if(TMCLReplyFormat==RF_SPECIAL)
    {
      for(i=0; i<9; i++)
      {
        USBReply[i]=SpecialReply[i];
      }
    }

    SendUSBReply(USBReply);
  }
  else if(TMCLCommandState==TCS_USB_ERROR)  //Check sum of last USB command was wrong
  {
    ActualReply.Opcode=0;
    ActualReply.Status=REPLY_CHKERR;
    ActualReply.Value.Int32=0;

    Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
             ActualReply.Status+ActualReply.Opcode+
             ActualReply.Value.Byte[3]+
             ActualReply.Value.Byte[2]+
             ActualReply.Value.Byte[1]+
             ActualReply.Value.Byte[0];

    USBReply[0]=ModuleConfig.SerialHostAddress;
    USBReply[1]=ModuleConfig.SerialModuleAddress;
    USBReply[2]=ActualReply.Status;
    USBReply[3]=ActualReply.Opcode;
    USBReply[4]=ActualReply.Value.Byte[3];
    USBReply[5]=ActualReply.Value.Byte[2];
    USBReply[6]=ActualReply.Value.Byte[1];
    USBReply[7]=ActualReply.Value.Byte[0];
    USBReply[8]=Checksum;

    //Antwort senden
    SendUSBReply(USBReply);
  }


  //Reset state (answer has been sent now)
  TMCLCommandState=TCS_IDLE;
  TMCLReplyFormat=RF_STANDARD;

  //Generate a system reset if requested by the host
  #if defined(MK20DX128)
  if(ResetRequested) ResetCPU(TRUE);
  #elif defined(GD32F425)
  if(ResetRequested) NVIC_SystemReset();
  #endif

  //**Try to get a new command**
  if(CanGetMessage(&CanFrame))  //From CAN?
  {
    ActualCommand.Opcode=CanFrame.Data[0];
    ActualCommand.Type=CanFrame.Data[1];
    ActualCommand.Motor=CanFrame.Data[2];
    ActualCommand.Value.Byte[3]=CanFrame.Data[3];
    ActualCommand.Value.Byte[2]=CanFrame.Data[4];
    ActualCommand.Value.Byte[1]=CanFrame.Data[5];
    ActualCommand.Value.Byte[0]=CanFrame.Data[6];
    ExtendedCANFrame=CanFrame.Ext;

    if(CanFrame.Dlc==7)
      TMCLCommandState=TCS_CAN7;
    else
      TMCLCommandState=TCS_CAN8;
  }
  else if(ReadRS485(&Byte))  //Get data from UART
  {
    if(CheckUARTTimeout()) UARTCount=0;  //discard everything when there has been a command timeout
    UARTCmd[UARTCount++]=Byte;

    if(UARTCount==9)  //Nine bytes have been received without timeout
    {
      UARTCount=0;

      if(UARTCmd[0]==ModuleConfig.SerialModuleAddress)  //Is this our addresss?
      {
        Checksum=0;
        for(i=0; i<8; i++) Checksum+=UARTCmd[i];

        if(Checksum==UARTCmd[8])  //Is the checksum correct?
        {
          ActualCommand.Opcode=UARTCmd[1];
          ActualCommand.Type=UARTCmd[2];
          ActualCommand.Motor=UARTCmd[3];
          ActualCommand.Value.Byte[3]=UARTCmd[4];
          ActualCommand.Value.Byte[2]=UARTCmd[5];
          ActualCommand.Value.Byte[1]=UARTCmd[6];
          ActualCommand.Value.Byte[0]=UARTCmd[7];

          TMCLCommandState=TCS_UART;

          UARTCount=0;
        }
        else TMCLCommandState=TCS_UART_ERROR;  //Checksum wrong
      }
    }
  }
  else if(GetUSBCmd(USBCmd))
  {
    //Ignore address byte
    Checksum=0;
    for(i=0; i<8; i++) Checksum+=USBCmd[i];

    if(Checksum==USBCmd[8])  //Checksum correct?
    {
      ActualCommand.Opcode=USBCmd[1];
      ActualCommand.Type=USBCmd[2];
      ActualCommand.Motor=USBCmd[3];
      ActualCommand.Value.Byte[3]=USBCmd[4];
      ActualCommand.Value.Byte[2]=USBCmd[5];
      ActualCommand.Value.Byte[1]=USBCmd[6];
      ActualCommand.Value.Byte[0]=USBCmd[7];

      TMCLCommandState=TCS_USB;

    } else TMCLCommandState=TCS_USB_ERROR;  //Checksum wrong
  }

  //**Execute the command**
  //Check if a command could be fetched and execute it.
  if(TMCLCommandState!=TCS_IDLE && TMCLCommandState!=TCS_UART_ERROR) ExecuteActualCommand();
}


/**********************************************************************//**
   \fn SetAccelerationDecelerationParameters(UCHAR Axis)
   \brief Set the acceleration and deceleration parameters of the TMC4361
   \param Axis  Axis number (always 0 with the stepRocker)

   This function adapts the acceleration and deceleration parameters in
   a way that the entire range for accleration and deceleration of the
   TMC4361 can be used.
   It bypasses a limitation of the internal parameter conversion of the 
   TMC4361. When one of these parameters is higher than MAX_PPS_ACC we
   switch to direct acceleration values and do the conversions ourselves
   here. Otherwise we can turn on the built-in conversion of the TMC4361
   and so do not have to do those calculations ourselves.
   This function gets called by the SetAxisParameter function (SAP command)
   each time an acceleration or deceleration parameter has been changed
   by an SAP command.
***************************************************************************/
void SetAccelerationDecelerationParameters(UCHAR Axis)
{
  if(MotorConfig[Axis].AMax>MAX_PPS_ACC || MotorConfig[Axis].DMax>MAX_PPS_ACC ||
     MotorConfig[Axis].AStart>MAX_PPS_ACC || MotorConfig[Axis].DFinal>MAX_PPS_ACC ||
     MotorConfig[Axis].DStop>MAX_PPS_ACC)
  {
    TMC43xxSetBits(Axis, TMC43xx_GENERAL_CONF, TMC43xx_GCONF_DIRECT_ACC_EN);
    WriteTMC43xxInt(Axis, TMC43xx_AMAX, (int) ((float) MotorConfig[Axis].AMax*137438953472.0/2.56E14));  //137438953472 = 2^37, 2.56E14 = 16E6 ^ 2
    WriteTMC43xxInt(Axis, TMC43xx_DMAX, (int) ((float) MotorConfig[Axis].DMax*137438953472.0/2.56E14));
    WriteTMC43xxInt(Axis, TMC43xx_ASTART, (int) ((float) MotorConfig[Axis].AStart*137438953472.0/2.56E14));
    WriteTMC43xxInt(Axis, TMC43xx_DFINAL, (int) ((float) MotorConfig[Axis].DFinal*137438953472.0/2.56E14));
    WriteTMC43xxInt(Axis, TMC43xx_DSTOP, (int) ((float) MotorConfig[Axis].DStop*137438953472.0/2.56E14));
  }
  else
  {
    TMC43xxClearBits(Axis, TMC43xx_GENERAL_CONF, TMC43xx_GCONF_DIRECT_ACC_EN);
    WriteTMC43xxInt(Axis, TMC43xx_AMAX, AccelerationToInternal[Axis](MotorConfig[Axis].AMax));
    WriteTMC43xxInt(Axis, TMC43xx_DMAX, AccelerationToInternal[Axis](MotorConfig[Axis].DMax));
    WriteTMC43xxInt(Axis, TMC43xx_ASTART, AccelerationToInternal[Axis](MotorConfig[Axis].AStart));
    WriteTMC43xxInt(Axis, TMC43xx_DFINAL, AccelerationToInternal[Axis](MotorConfig[Axis].DFinal));
    WriteTMC43xxInt(Axis, TMC43xx_DSTOP, AccelerationToInternal[Axis](MotorConfig[Axis].DStop));
  }
}

//** TMCL Commands **

/***************************************************************//**
  \fn RotateRight(void)
  \brief Command ROR (see TMCL manual)

  ROR (ROtate Right) command (see TMCL manual).
********************************************************************/
static void RotateRight(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    StopTorqueMode(ActualCommand.Motor);
    StallFlag[ActualCommand.Motor]=FALSE;
    DeviationFlag[ActualCommand.Motor]=FALSE;
    VMaxModified[ActualCommand.Motor]=TRUE;
    TMC43xxSetBits(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, TMC43xx_REFCONF_DRV_AFTER_STALL);
    if(MotorConfig[ActualCommand.Motor].RampType==RAMP_TRAPEZ)
      WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_VEL_TRAPEZ);
    else
      WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_VEL_SSHAPE);

    WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VMAX, VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
    TMC43xxSetBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VELOCITY_EN);
    TMC43xxClearBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VLIMIT_EN);
    TMC43xxClearBits(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, TMC43xx_REFCONF_DRV_AFTER_STALL);
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn RotateLeft(void)
  \brief Command ROL

  ROL (ROtate Left) command (see TMCL manual).
********************************************************************/
static void RotateLeft(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    StopTorqueMode(ActualCommand.Motor);
    StallFlag[ActualCommand.Motor]=FALSE;
    DeviationFlag[ActualCommand.Motor]=FALSE;
    VMaxModified[ActualCommand.Motor]=TRUE;
    TMC43xxSetBits(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, TMC43xx_REFCONF_DRV_AFTER_STALL);
    if(MotorConfig[ActualCommand.Motor].RampType==RAMP_TRAPEZ)
      WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_VEL_TRAPEZ);
    else
      WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_VEL_SSHAPE);

    WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VMAX, VelocityToInternal[ActualCommand.Motor](-ActualCommand.Value.Int32));
    TMC43xxSetBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VELOCITY_EN);
    TMC43xxClearBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VLIMIT_EN);
    TMC43xxClearBits(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, TMC43xx_REFCONF_DRV_AFTER_STALL);
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn MotorStop(void)
  \brief Command MST

  MST (Motor StoP) command (see TMCL manual).
********************************************************************/
static void MotorStop(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    VMaxModified[ActualCommand.Motor]=TRUE;
    if(MotorConfig[ActualCommand.Motor].RampType==RAMP_TRAPEZ)
      WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_VEL_TRAPEZ);
    else
      WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_VEL_SSHAPE);
    WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VMAX, 0);
    StopTorqueMode(ActualCommand.Motor);
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn MoveToPosition(void)
  \brief Command MVP

  MVP (Move To Position) command (see TMCL manual).
********************************************************************/
static void MoveToPosition(void)
{
  int NewPosition;

  if(ActualCommand.Motor<N_O_MOTORS)
  {
    switch(ActualCommand.Type)
    {
      case MVP_ABS:
        StopTorqueMode(ActualCommand.Motor);
        StallFlag[ActualCommand.Motor]=FALSE;
        DeviationFlag[ActualCommand.Motor]=FALSE;
        TMC43xxSetBits(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, TMC43xx_REFCONF_DRV_AFTER_STALL);
        if(MotorConfig[ActualCommand.Motor].RampType==RAMP_TRAPEZ)
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_POS_TRAPEZ);
        else
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_POS_SSHAPE);
        if(VMaxModified[ActualCommand.Motor])
        {
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VMAX, VMax[ActualCommand.Value.Int32]);
          VMaxModified[ActualCommand.Motor]=FALSE;
        }
        TMC43xxClearBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VELOCITY_EN);
        TMC43xxSetBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VLIMIT_EN);
        if(!ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_XTARGET, ActualCommand.Value.Int32);
        else
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_XTARGET, ActualCommand.Value.Int32+ClosedLoopPositionOffset[ActualCommand.Motor]);
        TMC43xxClearBits(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, TMC43xx_REFCONF_DRV_AFTER_STALL);
        break;

      case MVP_REL:
        StopTorqueMode(ActualCommand.Motor);
        StallFlag[ActualCommand.Motor]=FALSE;
        DeviationFlag[ActualCommand.Motor]=FALSE;
        TMC43xxSetBits(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, TMC43xx_REFCONF_DRV_AFTER_STALL);
        if(MotorConfig[ActualCommand.Motor].RampType==RAMP_TRAPEZ)
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_POS_TRAPEZ);
        else
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE, TMC43xx_RAMPMODE_POS_SSHAPE);

        switch(RelativePositioningOptionCode[ActualCommand.Motor])
        {
          case RMO_TARGET:
            NewPosition=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_XTARGET)+ActualCommand.Value.Int32;
            break;

          case RMO_ACTINT:
            NewPosition=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_XACTUAL)+ActualCommand.Value.Int32;
            break;

          case RMO_ACTENC:
            NewPosition=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_ENC_POS)-EncoderOffset[ActualCommand.Motor]+ActualCommand.Value.Int32;
            break;

          default:
            ActualReply.Status=REPLY_INVALID_VALUE;
            return;
            break;
        }

        if(VMaxModified[ActualCommand.Motor])
        {
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VMAX, VMax[ActualCommand.Motor]);
          VMaxModified[ActualCommand.Motor]=FALSE;
        }
        TMC43xxClearBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VELOCITY_EN);
        TMC43xxSetBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, TMC43xx_ENC_IN_CL_VLIMIT_EN);
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_XTARGET, NewPosition);
        TMC43xxClearBits(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, TMC43xx_REFCONF_DRV_AFTER_STALL);
        break;

      default:
        ActualReply.Status=REPLY_WRONG_TYPE;
        break;
    }
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn SetAxisParameter(void)
   \brief Command SAP

  SAP (Set Axis Parameter) command (see TMCL manual).
********************************************************************/
static void SetAxisParameter(void)
{
  int Value;

  if(ActualCommand.Motor<N_O_MOTORS)
  {
    switch(ActualCommand.Type)
    {
      case 0:
        if(!ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_XTARGET, ActualCommand.Value.Int32);
        else
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_XTARGET, ActualCommand.Value.Int32+ClosedLoopPositionOffset[ActualCommand.Motor]);
        break;

      case 1:
        if(!ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
        {
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_XACTUAL, ActualCommand.Value.Int32);
        }
        else
        {
          ClosedLoopPositionOffset[ActualCommand.Motor]=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_XACTUAL)-ActualCommand.Value.Int32;
          EncoderOffset[ActualCommand.Motor]=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_ENC_POS)-ActualCommand.Value.Int32;
        }
        break;

      case 2:
        VMaxModified[ActualCommand.Motor]=TRUE;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VMAX, VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
        break;

      case 3:
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VACTUAL, VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
        break;

      case 4:
        VMax[ActualCommand.Motor]=abs(VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
        if(ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE) & BIT2) //wenn Positionierungsmodus
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VMAX, VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
        break;

      case 5:
        MotorConfig[ActualCommand.Motor].AMax=ActualCommand.Value.Int32;
        SetAccelerationDecelerationParameters(ActualCommand.Motor);
        break;

      case 6:
        MotorConfig[ActualCommand.Motor].IRun=ActualCommand.Value.Byte[0];
        if(!ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
        {
          WriteTMC43xxBytes(ActualCommand.Motor, TMC43xx_SCALE_VALUES, MotorConfig[ActualCommand.Motor].IStandby, 0,
                            MotorConfig[ActualCommand.Motor].IRun, MotorConfig[ActualCommand.Motor].BoostCurrent);
        }
        else
        {
          WriteTMC5160Int(ActualCommand.Motor, TMC5160_IHOLD_IRUN, (ReadTMC5160Int(ActualCommand.Motor, TMC5160_IHOLD_IRUN) & 0xffffe0ff) | ((ActualCommand.Value.Byte[0]/8) << 8));
        }
        break;

      case 7:
        MotorConfig[ActualCommand.Motor].IStandby=ActualCommand.Value.Byte[0];
        if(!ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
        {
          WriteTMC43xxBytes(ActualCommand.Motor, TMC43xx_SCALE_VALUES, MotorConfig[ActualCommand.Motor].IStandby, 0,
                            MotorConfig[ActualCommand.Motor].IRun, MotorConfig[ActualCommand.Motor].BoostCurrent);
        }
        break;

      case 12:
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF) & ~(TMC43xx_REFCONF_STOP_RIGHT_EN|TMC43xx_REFCONF_POL_STOP_RIGHT);
        if(ActualCommand.Value.Int32 & BIT0) Value|=TMC43xx_REFCONF_STOP_RIGHT_EN;
        if(ActualCommand.Value.Int32 & BIT1) Value|=TMC43xx_REFCONF_POL_STOP_RIGHT;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, Value);
        break;

      case 13:
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF) & ~(TMC43xx_REFCONF_STOP_LEFT_EN|TMC43xx_REFCONF_POL_STOP_LEFT);
        if(ActualCommand.Value.Int32 & BIT0) Value|=TMC43xx_REFCONF_STOP_LEFT_EN;
        if(ActualCommand.Value.Int32 & BIT1) Value|=TMC43xx_REFCONF_POL_STOP_LEFT;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, Value);
        break;

      case 14:
        if(ActualCommand.Value.Int32==RAMP_TRAPEZ || ActualCommand.Value.Int32==RAMP_SSHAPE)
          MotorConfig[ActualCommand.Motor].RampType=ActualCommand.Value.Int32;
        else
          ActualReply.Status=REPLY_INVALID_VALUE;
        break;

      case 15:
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VSTART, VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
        break;

      case 16:
        MotorConfig[ActualCommand.Motor].AStart=ActualCommand.Value.Int32;
        SetAccelerationDecelerationParameters(ActualCommand.Motor);
        break;

      case 17:
        MotorConfig[ActualCommand.Motor].DMax=ActualCommand.Value.Int32;
        SetAccelerationDecelerationParameters(ActualCommand.Motor);
        break;

      case 18:
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VBREAK, VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
        break;

      case 19:
        MotorConfig[ActualCommand.Motor].DFinal=ActualCommand.Value.Int32;
        SetAccelerationDecelerationParameters(ActualCommand.Motor);
        break;

      case 20:
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VSTOP, VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
        break;

      case 21:
        MotorConfig[ActualCommand.Motor].DStop=ActualCommand.Value.Int32;
        SetAccelerationDecelerationParameters(ActualCommand.Motor);
        break;

      case 22:
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_BOW1, ActualCommand.Value.Int32);
        break;

      case 23:
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_BOW2, ActualCommand.Value.Int32);
        break;

      case 24:
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_BOW3, ActualCommand.Value.Int32);
        break;

      case 25:
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_BOW4, ActualCommand.Value.Int32);
        break;

      case 26:
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VIRT_STOP_LEFT, ActualCommand.Value.Int32);
        break;

      case 27:
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_VIRT_STOP_RIGHT, ActualCommand.Value.Int32);
        break;

      case 28:
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF);
        Value&= ~(TMC43xx_REFCONF_VIRT_LEFT_LIM_EN|TMC43xx_REFCONF_VIRT_RIGHT_LIM_EN);
        if(ActualCommand.Value.Int32 & BIT0) Value|=TMC43xx_REFCONF_VIRT_LEFT_LIM_EN;
        if(ActualCommand.Value.Int32 & BIT1) Value|=TMC43xx_REFCONF_VIRT_RIGHT_LIM_EN;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, Value);
        break;

      case 29:
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF);
        Value&= ~(TMC43xx_REFCONF_VIRT_STOP_HARD|TMC43xx_REFCONF_VIRT_STOP_LINEAR);
        Value|=(ActualCommand.Value.Int32 & 0x03) << 8;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, Value);
        break;

      case 33:
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF);
        Value&= ~TMC43xx_REFCONF_INV_STOP_DIR;
        if(ActualCommand.Value.Int32!=0) Value|=TMC43xx_REFCONF_INV_STOP_DIR;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, Value);
        break;

      case 34:
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF);
        Value&= ~TMC43xx_REFCONF_SOFT_STOP_EN;
        if(ActualCommand.Value.Int32!=0) Value|=TMC43xx_REFCONF_SOFT_STOP_EN;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF, Value);
        break;

      case 50:
        if(ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
        {
          StartTorqueMode(ActualCommand.Motor, ActualCommand.Value.Int32);
        }
        break;

      case 108:
        ClosedLoopConfig[ActualCommand.Motor].GammaVMin=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_VMIN_EMF, ActualCommand.Value.Int32);
        break;

      case 109:
        ClosedLoopConfig[ActualCommand.Motor].GammaVAdd=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_VADD_EMF, ActualCommand.Value.Int32);
        break;

      case 110:
        ClosedLoopConfig[ActualCommand.Motor].Gamma=ActualCommand.Value.Int32;
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_BETA_GAMMA) & 0x000001ff;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_BETA_GAMMA, Value|(ActualCommand.Value.Int32<<16));
        break;

      case 111:
        ClosedLoopConfig[ActualCommand.Motor].Beta=ActualCommand.Value.Int32;
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_BETA_GAMMA) & 0x00ff0000;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_BETA_GAMMA, Value|(ActualCommand.Value.Int32 & 0x1ff));
        break;

      case 112:
        ClosedLoopConfig[ActualCommand.Motor].Offset=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_OFFSET, ActualCommand.Value.Int32);
        break;

      case 113:
        ClosedLoopConfig[ActualCommand.Motor].CurrentScalerMinimum=ActualCommand.Value.Int32;
        if(ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
        {
          WriteTMC43xxBytes(ActualCommand.Motor, TMC43xx_SCALE_VALUES, 0, ClosedLoopConfig[ActualCommand.Motor].CurrentScalerStartUp,
                            ClosedLoopConfig[ActualCommand.Motor].CurrentScalerMaximum,
                            ClosedLoopConfig[ActualCommand.Motor].CurrentScalerMinimum);
        }
        break;

      case 114:
        ClosedLoopConfig[ActualCommand.Motor].CurrentScalerMaximum=ActualCommand.Value.Int32;
        if(ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
        {
          WriteTMC43xxBytes(ActualCommand.Motor, TMC43xx_SCALE_VALUES, 0, ClosedLoopConfig[ActualCommand.Motor].CurrentScalerStartUp,
                            ClosedLoopConfig[ActualCommand.Motor].CurrentScalerMaximum,
                            ClosedLoopConfig[ActualCommand.Motor].CurrentScalerMinimum);
        }
        break;

      case 115:
        ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityP=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_VMAX_P_W, ActualCommand.Value.Int32);
        break;

      case 116:
        ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityI=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_VMAX_CALC_I_W, ActualCommand.Value.Int32);
        break;

      case 117:
        ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityIClip=ActualCommand.Value.Int32;
        WriteTMC43xxBytes(ActualCommand.Motor, TMC43xx_PID_I_CLIP_D_CLK_W, 0, ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityDClk,
          ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityIClip >> 8, ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityIClip & 0xff);
        break;

      case 118:
        ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityDClk=ActualCommand.Value.Int32;
        WriteTMC43xxBytes(ActualCommand.Motor, TMC43xx_PID_I_CLIP_D_CLK_W, 0, ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityDClk,
          ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityIClip >> 8, ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityIClip & 0xff);
        break;

      case 119:
        ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityDClip=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_PID_DV_CLIP, ActualCommand.Value.Int32);
        break;

      case 120:
        ClosedLoopConfig[ActualCommand.Motor].UpscaleDelay=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_UPSCALE_DELAY, ActualCommand.Value.Int32);
        break;

      case 121:
        ClosedLoopConfig[ActualCommand.Motor].DownscaleDelay=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_DOWNSCALE_DELAY, ActualCommand.Value.Int32);
        break;

      case 124:
        ClosedLoopConfig[ActualCommand.Motor].PositionCorrectionP=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_DELTA_P_W, ActualCommand.Value.Int32);
        break;

      case 125:
        ClosedLoopConfig[ActualCommand.Motor].PositionCorrectionTolerance=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_TOLERANCE, ActualCommand.Value.Int32);
        break;

      case 126:
        ClosedLoopConfig[ActualCommand.Motor].CurrentScalerStartUp=ActualCommand.Value.Int32;
        if(ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
        {
          WriteTMC43xxBytes(ActualCommand.Motor, TMC43xx_SCALE_VALUES, 0, ClosedLoopConfig[ActualCommand.Motor].CurrentScalerStartUp,
                            ClosedLoopConfig[ActualCommand.Motor].CurrentScalerMaximum,
                            ClosedLoopConfig[ActualCommand.Motor].CurrentScalerMinimum);
        }
        break;

      case 127:
        if(ActualCommand.Value.Int32==RMO_TARGET || ActualCommand.Value.Int32==RMO_ACTINT ||
           ActualCommand.Value.Int32==RMO_ACTENC)
        {
          RelativePositioningOptionCode[ActualCommand.Motor]=ActualCommand.Value.Byte[0];
        }
        else
        {
          ActualReply.Status=REPLY_INVALID_VALUE;
        }
        break;

      case 128:
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE, ActualCommand.Value.Int32);
        break;

      case 129:  //Closed loop on/off
        if(ActualCommand.Value.Int32==1)
        {
          VMaxModified[ActualCommand.Motor]=TRUE;
          ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode=TRUE;
        }
        else
        {
          ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode=FALSE;
        }
        break;

      case 134:
        ClosedLoopConfig[ActualCommand.Motor].PositionWindow=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_CL_TR_TOLERANCE_W, ActualCommand.Value.Int32);
        break;

      case 136:
        ClosedLoopConfig[ActualCommand.Motor].EncVMeanWait=ActualCommand.Value.Byte[0];
        WriteTMC43xxBytes(ActualCommand.Motor, TMC43xx_ENC_VMEAN_WAIT_FILT, ClosedLoopConfig[ActualCommand.Motor].EncVMeanInt >> 8,
          ClosedLoopConfig[ActualCommand.Motor].EncVMeanInt & 0xff, ClosedLoopConfig[ActualCommand.Motor].EncVMeanFilter,
          ClosedLoopConfig[ActualCommand.Motor].EncVMeanWait);
        break;

      case 137:
        ClosedLoopConfig[ActualCommand.Motor].EncVMeanFilter=ActualCommand.Value.Byte[0];
        WriteTMC43xxBytes(ActualCommand.Motor, TMC43xx_ENC_VMEAN_WAIT_FILT, ClosedLoopConfig[ActualCommand.Motor].EncVMeanInt >> 8,
          ClosedLoopConfig[ActualCommand.Motor].EncVMeanInt & 0xff, ClosedLoopConfig[ActualCommand.Motor].EncVMeanFilter,
          ClosedLoopConfig[ActualCommand.Motor].EncVMeanWait);
        break;

      case 138:
        ClosedLoopConfig[ActualCommand.Motor].EncVMeanInt=ActualCommand.Value.Int32;
        WriteTMC43xxBytes(ActualCommand.Motor, TMC43xx_ENC_VMEAN_WAIT_FILT, ClosedLoopConfig[ActualCommand.Motor].EncVMeanInt >> 8,
          ClosedLoopConfig[ActualCommand.Motor].EncVMeanInt & 0xff, ClosedLoopConfig[ActualCommand.Motor].EncVMeanFilter,
          ClosedLoopConfig[ActualCommand.Motor].EncVMeanWait);
        break;

      case 140:
        if(ActualCommand.Value.Int32>=0 && ActualCommand.Value.Int32<=8)
        {
          Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_STEP_CONF) & 0xfffffff0;
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_STEP_CONF, Value|(8-ActualCommand.Value.Int32));
        }
        else ActualReply.Status=REPLY_INVALID_VALUE;
        break;

      case 200:
        MotorConfig[ActualCommand.Motor].BoostCurrent=ActualCommand.Value.Byte[0];
        if(!ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
        {
          WriteTMC43xxBytes(ActualCommand.Motor, TMC43xx_SCALE_VALUES, MotorConfig[ActualCommand.Motor].IStandby, 0,
                          MotorConfig[ActualCommand.Motor].IRun, MotorConfig[ActualCommand.Motor].BoostCurrent);
        }
        break;

      case 202:
        if(ActualCommand.Value.Int32!=MotorConfig[ActualCommand.Motor].MotorResolution)
        {
          if(!ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
          {
            MotorConfig[ActualCommand.Motor].MotorResolution=ActualCommand.Value.Int32;
            TMC43xxWriteBits(ActualCommand.Motor, TMC43xx_STEP_CONF, MotorConfig[ActualCommand.Motor].MotorResolution, 4, 12);
            WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_ENC_IN_RES_W, abs(MotorConfig[ActualCommand.Motor].EncoderResolution));
            if(MotorConfig[ActualCommand.Motor].EncoderResolution<0)
              TMC43xxSetBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, BIT29);
            else
              TMC43xxClearBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, BIT29);
          }
          else ActualReply.Status=REPLY_CMD_NOT_AVAILABLE;
        }
        break;

      case 209:
        if(ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
        {
          EncoderOffset[ActualCommand.Motor]=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_ENC_POS)-ActualCommand.Value.Int32;
          ClosedLoopPositionOffset[ActualCommand.Motor]=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_XACTUAL)-ActualCommand.Value.Int32;
        }
        else
        {
          WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_ENC_POS, ActualCommand.Value.Int32);
          EncoderOffset[ActualCommand.Motor]=0;
        }
        break;

      case 210:
        if(ActualCommand.Value.Int32!=MotorConfig[ActualCommand.Motor].EncoderResolution)
        {
          if(!ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
          {
            WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_ENC_IN_RES_W, abs(ActualCommand.Value.Int32));
            MotorConfig[ActualCommand.Motor].EncoderResolution=ActualCommand.Value.Int32;
            if(ActualCommand.Value.Int32<0)
              TMC43xxSetBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, BIT29);
            else
              TMC43xxClearBits(ActualCommand.Motor, TMC43xx_ENC_IN_CONF, BIT29);
          }
          else ActualReply.Status=REPLY_CMD_NOT_AVAILABLE;
        }
        break;

      case 212:
        MotorConfig[ActualCommand.Motor].MaxPositionDeviation=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_ENC_POS_DEV_TOL, ActualCommand.Value.Int32);
        break;

      case 213:
        MotorConfig[ActualCommand.Motor].MaxVelocityDeviation=ActualCommand.Value.Int32;
        break;

      case 214:
        MotorConfig[ActualCommand.Motor].SettingDelay=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_STDBY_DELAY, ActualCommand.Value.Int32*160000);
        break;

      case 251:
        if(ActualCommand.Value.Int32!=0)
          TMC43xxSetBits(ActualCommand.Motor, TMC43xx_GENERAL_CONF, TMC43xx_GCONF_REV_DIR);
        else
          TMC43xxClearBits(ActualCommand.Motor, TMC43xx_GENERAL_CONF, TMC43xx_GCONF_REV_DIR);
        break;

      case 253:
        GearRatio[ActualCommand.Motor]=ActualCommand.Value.Int32;
        WriteTMC43xxInt(ActualCommand.Motor, TMC43xx_GEAR_RATIO, ActualCommand.Value.Int32);
        break;

      case 254:
        if(ActualCommand.Value.Int32>=0 && ActualCommand.Value.Int32<=0x0f)
        {
          MotorConfig[ActualCommand.Motor].StepDirMode=ActualCommand.Value.Byte[0];
          SetStepDirMode(ActualCommand.Motor, ActualCommand.Value.Byte[0]);
        }
        else ActualReply.Status=REPLY_INVALID_VALUE;
        break;

      case 255:
        switch(ActualCommand.Value.Int32)
        {
          case UNIT_MODE_INTERNAL:
            MotorConfig[ActualCommand.Motor].UnitMode=0;
            VelocityToInternal[ActualCommand.Motor]=ConvertInternalToInternal;
            VelocityToUser[ActualCommand.Motor]=ConvertInternalToInternal;
            AccelerationToInternal[ActualCommand.Motor]=ConvertInternalToInternal;
            AccelerationToUser[ActualCommand.Motor]=ConvertInternalToInternal;
            break;

          case UNIT_MODE_PPS:
            MotorConfig[ActualCommand.Motor].UnitMode=1;
            VelocityToInternal[ActualCommand.Motor]=ConvertVelocityUserToInternal;
            VelocityToUser[ActualCommand.Motor]=ConvertVelocityInternalToUser;
            AccelerationToInternal[ActualCommand.Motor]=ConvertAccelerationUserToInternal;
            AccelerationToUser[ActualCommand.Motor]=ConvertAccelerationInternalToUser;
            break;

           default:
            ActualReply.Status=REPLY_INVALID_VALUE;
            break;
        }
        break;

      //TMC5160 specific parameters
      case 162:
        SetTMC5160ChopperBlankTime(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 163:
        SetTMC5160ChopperConstantTOffMode(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 164:
        SetTMC5160ChopperDisableFastDecayComp(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 165:
        if(GetTMC5160ChopperConstantTOffMode(ActualCommand.Motor)==0)
          SetTMC5160ChopperHysteresisEnd(ActualCommand.Motor, ActualCommand.Value.Int32);
        else
          SetTMC5160ChopperFastDecayTime(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 166:
        if(GetTMC5160ChopperConstantTOffMode(ActualCommand.Motor)==0)
          SetTMC5160ChopperHysteresisStart(ActualCommand.Motor, ActualCommand.Value.Int32);
        else
          SetTMC5160ChopperSineWaveOffset(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 167:
        SetTMC5160ChopperTOff(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 168:
        SetTMC5160SmartEnergyIMin(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 169:
        SetTMC5160SmartEnergyDownStep(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 170:
        SetTMC5160SmartEnergyStallLevelMax(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 171:
        SetTMC5160SmartEnergyUpStep(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 172:
        SetTMC5160SmartEnergyStallLevelMin(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 173:
        SetTMC5160SmartEnergyFilter(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 174:
        SetTMC5160SmartEnergyStallThreshold(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 177:
        SetTMC5160ChopperDisableShortToGround(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 181:
        if(ActualCommand.Value.Int32>=0)
          MotorConfig[ActualCommand.Motor].StallVMin=VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
        else
          ActualReply.Status=REPLY_INVALID_VALUE;
        break;
  
      case 182:
        if(ActualCommand.Value.Int32>=0)
        {
          if(MotorConfig[ActualCommand.Motor].UnitMode==UNIT_MODE_PPS)
            Value=ActualCommand.Value.Int32;
          else
            Value=ConvertVelocityInternalToUser(ActualCommand.Value.Int32);
  
          if(Value>0)
            WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TCOOLTHRS, 16000000 / Value);
          else
            WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TCOOLTHRS, 1048757);
        }
        else ActualReply.Status=REPLY_INVALID_VALUE;
        break;
  
      case 184:
        SetTMC5160ChopperRandomTOff(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 185:
        SetTMC5160ChopperSync(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 186:
        if(ActualCommand.Value.Int32>=0)
        {
          if(MotorConfig[ActualCommand.Motor].UnitMode==UNIT_MODE_PPS)
            Value=ActualCommand.Value.Int32;
          else
            Value=ConvertVelocityInternalToUser(ActualCommand.Value.Int32);
  
          if(Value>0)
            WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TPWMTHRS, 16000000 / Value);
          else
            WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TPWMTHRS, 1048757);
        }
        else ActualReply.Status=REPLY_INVALID_VALUE;
        break;
  
      case 187:
        SetTMC5160PWMGrad(ActualCommand.Motor, ActualCommand.Value.Int32);  //PWMGrad=0 => stealthChop ausgeschaltet
        Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_GCONF);
        if(ActualCommand.Value.Int32!=0)
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_GCONF, Value|BIT2);
        else
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_GCONF, Value& ~BIT2);
        break;
  
      case 188:
        SetTMC5160PWMAmpl(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 191:
        SetTMC5160PWMFrequency(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 192:
        SetTMC5160PWMAutoscale(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
  
      case 204:
        SetTMC5160PWMFreewheelMode(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      default:
        ActualReply.Status=REPLY_WRONG_TYPE;
        break;
    }
  } else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn GetAxisParameter(void)
  \brief Command GAP

  GAP (Get Axis Parameter) command (see TMCL manual).
********************************************************************/
static void GetAxisParameter(void)
{
  int Value;

  if(ActualCommand.Motor<N_O_MOTORS)
  {
    ActualReply.Value.Int32=0;

    switch(ActualCommand.Type)
    {
      case 0:
        if(!GetClosedLoopInitFlag(ActualCommand.Motor))
          ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_XTARGET);
        else
          ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_XTARGET)-ClosedLoopPositionOffset[ActualCommand.Motor];
        break;

      case 1:
        if(!ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode)
          ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_XACTUAL);
        else
          ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_XACTUAL)-ClosedLoopPositionOffset[ActualCommand.Motor];
        break;

      case 2:
        ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_VMAX));
        break;

      case 3:
        ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_VACTUAL)<<8);
        break;

      case 4:
        ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](VMax[ActualCommand.Motor]);
        break;

      case 5:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].AMax;
        break;

      case 6:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].IRun;
        break;

      case 7:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].IStandby;
        break;

      case 8:
        ActualReply.Value.Int32=(ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_STATUS) & TMC43xx_ST_TARGET_REACHED) ? 1:0;
        break;

      case 9:
        ActualReply.Value.Int32=GetHomeInput(ActualCommand.Motor);
        break;

      case 10:
        ActualReply.Value.Int32=(ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_STATUS) & TMC43xx_ST_STOP_RIGHT_ACTIVE) ? 1:0;
        break;

      case 11:
        ActualReply.Value.Int32=(ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_STATUS) & TMC43xx_ST_STOP_LEFT_ACTIVE) ? 1:0;
        break;

      case 12:
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF);
        if(Value & TMC43xx_REFCONF_STOP_RIGHT_EN) ActualReply.Value.Int32|=BIT0;
        if(Value & TMC43xx_REFCONF_POL_STOP_RIGHT) ActualReply.Value.Int32|=BIT1;
        break;

      case 13:
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF);
        if(Value & TMC43xx_REFCONF_STOP_LEFT_EN) ActualReply.Value.Int32|=BIT0;
        if(Value & TMC43xx_REFCONF_POL_STOP_LEFT) ActualReply.Value.Int32|=BIT1;
        break;

      case 14:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].RampType;
        break;

      case 15:
        ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_VSTART));
        break;

      case 16:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].AStart;
        break;

      case 17:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].DMax;
        break;

      case 18:
        ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_VBREAK));
        break;

      case 19:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].DFinal;
        break;

      case 20:
        ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_VSTOP));
        break;

      case 21:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].DStop;
        break;

      case 22:
        ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_BOW1);
        break;

      case 23:
        ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_BOW2);
        break;

      case 24:
        ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_BOW3);
        break;

      case 25:
        ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_BOW4);
        break;

      case 26:
        ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_VIRT_STOP_LEFT);
        break;

      case 27:
        ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_VIRT_STOP_RIGHT);
        break;

      case 28:
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF);
        if(Value & TMC43xx_REFCONF_VIRT_LEFT_LIM_EN) ActualReply.Value.Int32|=BIT0;
        if(Value & TMC43xx_REFCONF_VIRT_RIGHT_LIM_EN) ActualReply.Value.Int32|=BIT1;
        break;

      case 29:
        ActualReply.Value.Int32=(ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF) >> 8) & 0x03;
        break;

      case 33:
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF);
        if(Value & TMC43xx_REFCONF_INV_STOP_DIR)
          ActualReply.Value.Int32=1;
        else
          ActualReply.Value.Int32=0;
        break;

      case 34:
        Value=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_REFERENCE_CONF);
        if(Value & TMC43xx_REFCONF_SOFT_STOP_EN)
          ActualReply.Value.Int32=1;
        else
          ActualReply.Value.Int32=0;
        break;

      case 50:
        ActualReply.Value.Int32=GetTorqueModeCurrent(ActualCommand.Motor);
        break;

      case 108:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].GammaVMin;
        break;

      case 109:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].GammaVAdd;
        break;

      case 110:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].Gamma;
        break;

      case 111:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].Beta;
        break;

      case 112:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].Offset;
        break;

      case 113:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].CurrentScalerMinimum;
        break;

      case 114:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].CurrentScalerMaximum;
        break;

      case 115:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityP;
        break;

      case 116:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityI;
        break;

      case 117:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityIClip;
        break;

      case 118:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityDClk;
        break;

      case 119:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].CorrectionVelocityDClip;
        break;

      case 120:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].UpscaleDelay;
        break;

      case 121:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].DownscaleDelay;
        break;

      case 123:
        ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_SCALE_PARAM);
        break;

      case 124:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].PositionCorrectionP;
        break;

      case 125:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].PositionCorrectionTolerance;
        break;

      case 126:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].CurrentScalerStartUp;
        break;

      case 127:
        ActualReply.Value.Int32=RelativePositioningOptionCode[ActualCommand.Motor];
        break;

      case 128:
        ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_RAMPMODE);
        break;

      case 129:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].ClosedLoopMode;
        break;

      case 131:
        ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_VENC_MEAN);
        break;

      case 132:
        ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_VENC);
        break;

      case 133:
        ActualReply.Value.Int32=GetClosedLoopInitFlag(ActualCommand.Motor);
        break;

      case 134:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].PositionWindow;
        break;

      case 136:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].EncVMeanWait;
        break;

      case 137:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].EncVMeanFilter;
        break;

      case 138:
        ActualReply.Value.Int32=ClosedLoopConfig[ActualCommand.Motor].EncVMeanInt;
        break;

      case 140:
        ActualReply.Value.Int32=8-(ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_STEP_CONF) & 0x0f);
        break;

      case 200:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].BoostCurrent;
        break;

      case 202:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].MotorResolution;
        break;

      case 206:
        ActualReply.Value.Int32=StallLevel[ActualCommand.Motor];
        break;

      case 208:
        ActualReply.Value.Int32=DriverFlags[ActualCommand.Motor];
        break;

      case 209:
        ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_ENC_POS)-EncoderOffset[ActualCommand.Motor];
        break;

      case 210:
        //ActualReply.Value.Int32=ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_ENC_CONST_R);
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].EncoderResolution;
        break;

      case 212:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].MaxPositionDeviation;
        break;

      case 213:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].MaxVelocityDeviation;
        break;

      case 214:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].SettingDelay;
        break;

      case 251:
        ActualReply.Value.Int32=(ReadTMC43xxInt(ActualCommand.Motor, TMC43xx_GENERAL_CONF) & TMC43xx_GCONF_REV_DIR) ? 1:0;
        break;

      #if DEVICE==TMCM1111
      case 253:
        ActualReply.Value.Int32=GearRatio[ActualCommand.Motor];
        break;

      case 254:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].StepDirMode;
        break;
      #endif

      case 255:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].UnitMode;
        break;

      //TMC5160 specific parameters
      case 162:
        ActualReply.Value.Int32=GetTMC5160ChopperBlankTime(ActualCommand.Motor);
        break;
  
      case 163:
        ActualReply.Value.Int32=GetTMC5160ChopperConstantTOffMode(ActualCommand.Motor);
        break;
  
      case 164:
        ActualReply.Value.Int32=GetTMC5160ChopperDisableFastDecayComp(ActualCommand.Motor);
        break;
  
      case 165:
        if(GetTMC5160ChopperConstantTOffMode(ActualCommand.Motor)==0)
          ActualReply.Value.Int32=GetTMC5160ChopperHysteresisEnd(ActualCommand.Motor);
        else
          ActualReply.Value.Int32=GetTMC5160ChopperFastDecayTime(ActualCommand.Motor);
        break;
  
      case 166:
        if(GetTMC5160ChopperConstantTOffMode(ActualCommand.Motor)==0)
          ActualReply.Value.Int32=GetTMC5160ChopperHysteresisStart(ActualCommand.Motor);
        else
          ActualReply.Value.Int32=GetTMC5160ChopperSineWaveOffset(ActualCommand.Motor);
        break;
  
      case 167:
        ActualReply.Value.Int32=GetTMC5160ChopperTOff(ActualCommand.Motor);
        break;
  
      case 168:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyIMin(ActualCommand.Motor);
        break;
  
      case 169:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyDownStep(ActualCommand.Motor);
        break;
  
      case 170:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyStallLevelMax(ActualCommand.Motor);
        break;
  
      case 171:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyUpStep(ActualCommand.Motor);
        break;
  
      case 172:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyStallLevelMin(ActualCommand.Motor);
        break;
  
      case 173:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyFilter(ActualCommand.Motor);
        break;
  
      case 174:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyStallThreshold(ActualCommand.Motor);
        break;
  
      case 177:
        ActualReply.Value.Int32=GetTMC5160ChopperDisableShortToGround(ActualCommand.Motor);
        break;
  
      case 180:
        #if defined(DEVTYPE_TMC43xx)
        ActualReply.Value.Int32=SmartEnergy[ActualCommand.Motor];
        #else
        ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_DRVSTATUS) >> 16) & 0x1f;
        #endif
        break;
  
      case 181:
        ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](MotorConfig[ActualCommand.Motor].StallVMin);
        break;
  
      case 182:
        Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TCOOLTHRS);
        if(Value>0)
          ActualReply.Value.Int32=16000000/Value;
         else
          ActualReply.Value.Int32=16777215;
        if(MotorConfig[ActualCommand.Motor].UnitMode==UNIT_MODE_INTERNAL) ActualReply.Value.Int32=ConvertVelocityUserToInternal(ActualReply.Value.Int32);
        break;
  
      case 184:
        ActualReply.Value.Int32=GetTMC5160ChopperRandomTOff(ActualCommand.Motor);
        break;
  
      case 185:
        ActualReply.Value.Int32=GetTMC5160ChopperSync(ActualCommand.Motor);
        break;
  
      case 186:
        Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TPWMTHRS);
        if(Value>0)
          ActualReply.Value.Int32=16000000/Value;
         else
          ActualReply.Value.Int32=16777215;
        if(MotorConfig[ActualCommand.Motor].UnitMode==UNIT_MODE_INTERNAL) ActualReply.Value.Int32=ConvertVelocityUserToInternal(ActualReply.Value.Int32);
        break;
  
      case 187:
        ActualReply.Value.Int32=GetTMC5160PWMGrad(ActualCommand.Motor);
        break;
  
      case 188:
        ActualReply.Value.Int32=GetTMC5160PWMAmpl(ActualCommand.Motor);
        break;
  
      case 189:
        ActualReply.Value.Int32=ReadTMC5160Int(ActualCommand.Motor, TMC5160_PWMSCALE);
        break;
  
      case 190:
        if((ReadTMC5160Int(ActualCommand.Motor, TMC5160_GCONF) & BIT2) && (ReadTMC5160Int(ActualCommand.Motor, TMC5160_DRVSTATUS) & BIT14))
          ActualReply.Value.Int32=1;
        else
          ActualReply.Value.Int32=0;
        break;
  
      case 191:
        ActualReply.Value.Int32=GetTMC5160PWMFrequency(ActualCommand.Motor);
        break;
  
      case 192:
        ActualReply.Value.Int32=GetTMC5160PWMAutoscale(ActualCommand.Motor);
        break;
  
      case 204:
        ActualReply.Value.Int32=GetTMC5160PWMFreewheelMode(ActualCommand.Motor);
        break;

      default:
        ActualReply.Status=REPLY_WRONG_TYPE;
        break;
    }
  } else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn GetVersion(void)
  \brief Command 136 (get version)

  Get the version number (when type==0) or
  the version string (when type==1).
********************************************************************/
static void GetVersion(void)
{
  UCHAR i;

  if(ActualCommand.Type==0)
  {
    TMCLReplyFormat=RF_SPECIAL;
    SpecialReply[0]=ModuleConfig.SerialHostAddress;
    for(i=0; i<8; i++)
      SpecialReply[i+1]=VersionString[i];
  }
  else if(ActualCommand.Type==1)
  {
    ActualReply.Value.Byte[3]=SW_TYPE_HIGH;
    ActualReply.Value.Byte[2]=SW_TYPE_LOW;
    ActualReply.Value.Byte[1]=SW_VERSION_HIGH;
    ActualReply.Value.Byte[0]=SW_VERSION_LOW;
  }
}


/************************************//**
   \fn Boot(void)
   \brief Enter bootloader mode

   Special command for exiting TMCL
   and calling the boot loader.
 ***************************************/
static void Boot(void)
{
#ifdef BOOTLOADER
  UINT Delay;

  if(ActualCommand.Type==0x81 && ActualCommand.Motor==0x92 &&
     ActualCommand.Value.Byte[3]==0xa3 && ActualCommand.Value.Byte[2]==0xb4 &&
     ActualCommand.Value.Byte[1]==0xc5 && ActualCommand.Value.Byte[0]==0xd6)
  {
    DISABLE_DRIVERS();

#if defined(MK20DX128)
    DeInitUSB();
    Delay=GetSysTimer();
    while(abs(GetSysTimer()-Delay)<1000);

    DisableInterrupts();
    SYST_CSR=0;
    NVICICER0=0xFFFFFFFF;  //alle Interrupts sperren
    NVICICPR0=0xFFFFFFFF;
    NVICICER1=0xFFFFFFFF;
    NVICICPR1=0xFFFFFFFF;
    NVICICER2=0xFFFFFFFF;
    NVICICPR2=0xFFFFFFFF;
    NVICICER3=0xFFFFFFFF;
    NVICICPR3=0xFFFFFFFF;
    BLMagic=0x12345678;
    ResetCPU(TRUE);
#elif defined(GD32F425)
    DetachUSB();
    Delay=GetSysTimer();
    while(abs(GetSysTimer()-Delay)<1000);
    __disable_irq();
    NVIC_DeInit();
    SysTick->CTRL=0;

    BLMagic=0x12345678;
    NVIC_SystemReset();
#endif
  }
#else
  ActualReply.Status=REPLY_CMD_NOT_AVAILABLE;
#endif
}


/**************************//**
   \fn SoftwareReset(void)
   \brief TMCL software reset command

   Issue a TMCL software reset.
 *******************************/
static void SoftwareReset(void)
{
  if(ActualCommand.Value.Int32==1234) ResetRequested=TRUE;
}
