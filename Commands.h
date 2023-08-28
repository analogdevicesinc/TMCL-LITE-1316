/*******************************************************************************
  Project: stepRocker Mini TMCL (for stepRocker V2.2)

  Module:  Commands.h
           Definitions needed for the TMCL interpreter


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
  \file Commands.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief TMCL command definitions

  This file contains all necessary definitions for the
  TMCL command interpreter.
*/

//States of the command interpreter
#define TCS_IDLE  0            //!< TMCL interpreter is in idle mode (no command to process)
#define TCS_UART  1            //!< processing a command from RS485
#define TCS_UART_ERROR 2       //!< last command from RS485 had bad check sum
#define TCS_CAN7  3            //!< last command from CAN with 7 bytes
#define TCS_CAN8  4            //!< last command from CAN with 8 bytes
#define TCS_USB   5            //!< last command from USB
#define TCS_USB_ERROR 6        //!< last command from USB had bad check sum

//Supported TMCL commands
#define TMCL_ROR   1           //!< ROR command opcode
#define TMCL_ROL   2           //!< ROL command opcode
#define TMCL_MST   3           //!< MST command opcode
#define TMCL_MVP   4           //!< MVP command opcode
#define TMCL_SAP   5           //!< SAP command opcode
#define TMCL_GAP   6           //!< GAP command opcode
#define TMCL_GetVersion 136    //!< GetVersion command opcode
#define TMCL_Boot 0xf2         //!< Boot command opcode
#define TMCL_SoftwareReset 0xff  //!< software reset command opcode

//Type codes of the MVP command
#define MVP_ABS   0            //!< absolute movement (with MVP command)
#define MVP_REL   1            //!< relative movement (with MVP command)
#define MVP_COORD 2            //!< coordinate movement (with MVO command)

//Relative positioning options
#define RMO_TARGET 0    //!< last target position
#define RMO_ACTINT 1    //!< actual ramp generator position
#define RMO_ACTENC 2    //!< actual encoder position

//Unit conversion mode
#define UNIT_MODE_INTERNAL 0   //!< use internal units of the TMC4361
#define UNIT_MODE_PPS      1   //!< use PPS units

//TMCL status codes
#define REPLY_OK 100                //!< command successfully executed
#define REPLY_CHKERR 1              //!< checksum error
#define REPLY_INVALID_CMD 2         //!< command not supported
#define REPLY_WRONG_TYPE 3          //!< wrong type code
#define REPLY_INVALID_VALUE 4       //!< wrong value
#define REPLY_EEPROM_LOCKED 5       //!< EEPROM is locked
#define REPLY_CMD_NOT_AVAILABLE 6   //!< command not available due to current state
#define REPLY_CMD_LOAD_ERROR 7      //!< error when storing command to EEPROM
#define REPLY_WRITE_PROTECTED 8     //!< EEPROM is write protected
#define REPLY_MAX_EXCEEDED 9        //!< maximum number of commands in EEPROM exceeded

//Reply format
#define RF_STANDARD 0               //!< use standard TMCL reply
#define RF_SPECIAL 1                //!< use special reply

//Data structures needed by the TMCL interpreter
//! TMCL command
typedef struct
{
  UCHAR Opcode;      //!< command opcode
  UCHAR Type;        //!< type parameter
  UCHAR Motor;       //!< motor/bank parameter
  union
  {
    long Int32;      //!< value parameter as 32 bit integer
    UCHAR Byte[4];   //!< value parameter as 4 bytes
  } Value;           //!< value parameter
} TTMCLCommand;

//! TMCL reply
typedef struct
{
  UCHAR Status;      //!< status code
  UCHAR Opcode;      //!< opcode of executed command
  union
  {
    long Int32;      //!< reply value as 32 bit integer
    UCHAR Byte[4];   //!< reply value as 4 bytes
  } Value;           //!< value parameter
} TTMCLReply;


//Prototypes of exported functions
void InitTMCL(void);
void ProcessCommand(void);
