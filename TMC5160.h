/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker TMCM-1316)

  Module:  TMC5160.h
           TMC5160 library

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
  \file TMC5160.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 1.00

  \brief TMC5160 Motor driver functions

  This file provides all functions needed for easy
  access to the TMC5160 stepper motor driver IC.
*/


#ifndef __TMC5160_H
#define __TMC5160_H

//Registers
#define TMC5160_GCONF        0x00
#define TMC5160_GSTAT        0x01
#define TMC5160_IFCNT        0x02
#define TMC5160_SLAVECONF    0x03
#define TMC5160_IOIN         0x04
#define TMC5160_X_COMPARE    0x05


#define TMC5160_IHOLD_IRUN   0x10
#define TMC5160_TPOWERDOWN   0x11
#define TMC5160_TSTEP        0x12
#define TMC5160_TPWMTHRS     0x13
#define TMC5160_TCOOLTHRS    0x14
#define TMC5160_THIGH        0x15
#define TMC5160_RAMPMODE     0x20
#define TMC5160_XACTUAL      0x21
#define TMC5160_VACTUAL      0x22
#define TMC5160_VSTART       0x23
#define TMC5160_A1           0x24
#define TMC5160_V1           0x25
#define TMC5160_AMAX         0x26
#define TMC5160_VMAX         0x27
#define TMC5160_DMAX         0x28
#define TMC5160_D1           0x2A
#define TMC5160_VSTOP        0x2B
#define TMC5160_TZEROWAIT    0x2C
#define TMC5160_XTARGET      0x2D
#define TMC5160_VDCMIN       0x33
#define TMC5160_SWMODE       0x34
#define TMC5160_RAMPSTAT     0x35
#define TMC5160_XLATCH       0x36
#define TMC5160_ENCMODE      0x38
#define TMC5160_XENC         0x39
#define TMC5160_ENC_CONST    0x3A
#define TMC5160_ENC_STATUS   0x3B
#define TMC5160_ENC_LATCH    0x3C
#define TMC5160_MSLUT0       0x60
#define TMC5160_MSLUT1       0x61
#define TMC5160_MSLUT2       0x62
#define TMC5160_MSLUT3       0x63
#define TMC5160_MSLUT4       0x64
#define TMC5160_MSLUT5       0x65
#define TMC5160_MSLUT6       0x66
#define TMC5160_MSLUT7       0x67
#define TMC5160_MSLUTSEL     0x68
#define TMC5160_MSLUTSTART   0x69
#define TMC5160_MSCNT        0x6A
#define TMC5160_MSCURACT     0x6B
#define TMC5160_CHOPCONF     0x6C
#define TMC5160_COOLCONF     0x6D
#define TMC5160_DCCTRL       0x6E
#define TMC5160_DRVSTATUS    0x6F
#define TMC5160_PWMCONF      0x70
#define TMC5160_PWMSCALE     0x71
#define TMC5160_LOST_STEPS   0x73

#define TMC5160_FACTORY_CONF  0x08
#define TMC5160_SHORT_CONF    0x09
#define TMC5160_DRV_CONF      0x0A
#define TMC5160_GLOBAL_SCALER 0x0B
#define TMC5160_OFFSET_READ   0x0C
#define TMC5160_ENC_DEVIATION 0x3D
#define TMC5160_PWMAUTO       0x72


#define WHICH_5160(m)       (m)

//Write bit
#define TMC5160_WRITE        0x80

//Ramp modes (Register TMC5160_RAMPMODE)
#define TMC5160_MODE_POSITION   0
#define TMC5160_MODE_VELPOS     1
#define TMC5160_MODE_VELNEG     2
#define TMC5160_MODE_HOLD       3

//Configuration bits (Register TMC5160_GCONF)
#define TMC5160_GCONF_ISCALE_ANALOG     0x00001
#define TMC5160_GCONF_INT_RSENSE        0x00002
#define TMC5160_GCONF_ENC_COMMUTATION   0x00008

#define TMC5160_GCONF_EN_PWM_MODE       0x00004
#define TMC5160_GCONF_SHAFT             0x00010
#define TMC5160_GCONF_DIAG0_ERROR       0x00020
#define TMC5160_GCONF_DIAG0_OTPW        0x00040
#define TMC5160_GCONF_DIAG0_STALL_STEP  0x00080
#define TMC5160_GCONF_DIAG1_STALL_DIR   0x00100
#define TMC5160_DIAG1_INDEX             0x00200
#define TMC5160_DIAG1_ONSTATE           0x00400
#define TMC5160_DIAG1_STEPS_SKIPPED     0x00800
#define TMC5160_GCONF_DIAG0_PUSHPULL    0x01000
#define TMC5160_GCONF_DIAG1_PUSHPULL    0x02000
#define TMC5160_GCONF_SMALL_HYSTERESIS  0x04000
#define TMC5160_GCONF_STOP_ENABLE       0x08000
#define TMC5160_GCONF_DIRECT_MODE       0x10000
#define TMC5160_GCONF_TEST_MODE         0x20000

#define TMC5160_GCONF_RECALIBRATE       0x00001
#define TMC5160_GCONF_FASTSTANDSTILL    0x00002
#define TMC5160_GCONF_MULTISTEP_FILT    0x00008

//End switch mode bits (Register TMC5160_SWMODE)
#define TMC5160_SW_STOPL_ENABLE   0x0001
#define TMC5160_SW_STOPR_ENABLE   0x0002
#define TMC5160_SW_STOPL_POLARITY 0x0004
#define TMC5160_SW_STOPR_POLARITY 0x0008
#define TMC5160_SW_SWAP_LR        0x0010
#define TMC5160_SW_LATCH_L_ACT    0x0020
#define TMC5160_SW_LATCH_L_INACT  0x0040
#define TMC5160_SW_LATCH_R_ACT    0x0080
#define TMC5160_SW_LATCH_R_INACT  0x0100
#define TMC5160_SW_LATCH_ENC      0x0200
#define TMC5160_SW_SG_STOP        0x0400
#define TMC5160_SW_SOFTSTOP       0x0800

//Status bits
#define TMC5160_RS_STOPL          0x0001
#define TMC5160_RS_STOPR          0x0002
#define TMC5160_RS_LATCHL         0x0004
#define TMC5160_RS_LATCHR         0x0008
#define TMC5160_RS_EV_STOPL       0x0010
#define TMC5160_RS_EV_STOPR       0x0020
#define TMC5160_RS_EV_STOP_SG     0x0040
#define TMC5160_RS_EV_POSREACHED  0x0080
#define TMC5160_RS_VELREACHED     0x0100
#define TMC5160_RS_POSREACHED     0x0200
#define TMC5160_RS_VZERO          0x0400
#define TMC5160_RS_ZEROWAIT       0x0800
#define TMC5160_RS_SECONDMOVE     0x1000
#define TMC5160_RS_SG             0x2000

//Encoder mode bits (Register TMC5160_ENCMODE)
#define TMC5160_EM_DECIMAL        0x0400
#define TMC5160_EM_LATCH_XACT     0x0200
#define TMC5160_EM_CLR_XENC       0x0100
#define TMC5160_EM_NEG_EDGE       0x0080
#define TMC5160_EM_POS_EDGE       0x0040
#define TMC5160_EM_CLR_ONCE       0x0020
#define TMC5160_EM_CLR_CONT       0x0010
#define TMC5160_EM_IGNORE_AB      0x0008
#define TMC5160_EM_POL_N          0x0004
#define TMC5160_EM_POL_B          0x0002
#define TMC5160_EM_POL_A          0x0001

#define TPOWERDOWN_FACTOR (4.17792*100.0/255.0)

void WriteTMC5160Datagram(UCHAR Which562, UCHAR Address, UCHAR x1, UCHAR x2, UCHAR x3, UCHAR x4);
void WriteTMC5160Int(UCHAR Which562, UCHAR Address, int Value);
int ReadTMC5160Int(UCHAR Which562, UCHAR Address);
void SetTMC5160ChopperTOff(UCHAR Motor, UCHAR TOff);
void SetTMC5160ChopperHysteresisStart(UCHAR Motor, UCHAR HysteresisStart);
void SetTMC5160ChopperHysteresisEnd(UCHAR Motor, UCHAR HysteresisEnd);
void SetTMC5160ChopperBlankTime(UCHAR Motor, UCHAR BlankTime);
void SetTMC5160ChopperSync(UCHAR Motor, UCHAR Sync);
void SetTMC5160ChopperMStepRes(UCHAR Motor, UCHAR MRes);
void SetTMC5160ChopperDisableShortToGround(UCHAR Motor, UCHAR Disable);
void SetTMC5160ChopperVHighChm(UCHAR Motor, UCHAR VHighChm);
void SetTMC5160ChopperVHighFs(UCHAR Motor, UCHAR VHighFs);
void SetTMC5160ChopperConstantTOffMode(UCHAR Motor, UCHAR ConstantTOff);
void SetTMC5160ChopperRandomTOff(UCHAR Motor, UCHAR RandomTOff);
void SetTMC5160ChopperDisableFastDecayComp(UCHAR Motor, UCHAR Disable);
void SetTMC5160ChopperFastDecayTime(UCHAR Motor, UCHAR Time);
void SetTMC5160ChopperSineWaveOffset(UCHAR Motor, UCHAR Offset);
UCHAR GetTMC5160ChopperTOff(UCHAR Motor);
UCHAR GetTMC5160ChopperHysteresisStart(UCHAR Motor);
UCHAR GetTMC5160ChopperHysteresisEnd(UCHAR Motor);
UCHAR GetTMC5160ChopperBlankTime(UCHAR Motor);
UCHAR GetTMC5160ChopperSync(UCHAR Motor);
UCHAR GetTMC5160ChopperMStepRes(UCHAR Motor);
UCHAR GetTMC5160ChopperDisableShortToGround(UCHAR Motor);
UCHAR GetTMC5160ChopperVHighChm(UCHAR Motor);
UCHAR GetTMC5160ChopperVHighFs(UCHAR Motor);
UCHAR GetTMC5160ChopperConstantTOffMode(UCHAR Motor);
UCHAR GetTMC5160ChopperRandomTOff(UCHAR Motor);
UCHAR GetTMC5160ChopperDisableFastDecayComp(UCHAR Motor);
UCHAR GetTMC5160ChopperFastDecayTime(UCHAR Motor);
UCHAR GetTMC5160ChopperSineWaveOffset(UCHAR Motor);
void SetTMC5160SmartEnergyUpStep(UCHAR Motor, UCHAR UpStep);
void SetTMC5160SmartEnergyDownStep(UCHAR Motor, UCHAR DownStep);
void SetTMC5160SmartEnergyStallLevelMax(UCHAR Motor, UCHAR Max);
void SetTMC5160SmartEnergyStallLevelMin(UCHAR Motor, UCHAR Min);
void SetTMC5160SmartEnergyStallThreshold(UCHAR Motor, char Threshold);
void SetTMC5160SmartEnergyIMin(UCHAR Motor, UCHAR IMin);
void SetTMC5160SmartEnergyFilter(UCHAR Motor, UCHAR Filter);
UCHAR GetTMC5160SmartEnergyUpStep(UCHAR Motor);
UCHAR GetTMC5160SmartEnergyDownStep(UCHAR Motor);
UCHAR GetTMC5160SmartEnergyStallLevelMax(UCHAR Motor);
UCHAR GetTMC5160SmartEnergyStallLevelMin(UCHAR Motor);
int GetTMC5160SmartEnergyStallThreshold(UCHAR Motor);
UCHAR GetTMC5160SmartEnergyIMin(UCHAR Motor);
UCHAR GetTMC5160SmartEnergyFilter(UCHAR Motor);
void SetTMC5160PWMFreewheelMode(UCHAR Motor, UCHAR Mode);
void SetTMC5160PWMSymmetric(UCHAR Motor, UCHAR Symmetric);
void SetTMC5160PWMAutoscale(UCHAR Motor, UCHAR Autoscale);
void SetTMC5160PWMFrequency(UCHAR Motor, UCHAR Frequency);
void SetTMC5160PWMGrad(UCHAR Motor, UCHAR PWMGrad);
void SetTMC5160PWMAmpl(UCHAR Motor, UCHAR PWMAmpl);
UCHAR GetTMC5160PWMFreewheelMode(UCHAR Motor);
UCHAR GetTMC5160PWMSymmetric(UCHAR Motor);
UCHAR GetTMC5160PWMAutoscale(UCHAR Motor);
UCHAR GetTMC5160PWMFrequency(UCHAR Motor);
UCHAR GetTMC5160PWMGrad(UCHAR Motor);
UCHAR GetTMC5160PWMAmpl(UCHAR Motor);

void Read5160State(UCHAR Which5160, UINT *StallGuard, UCHAR *SmartEnergy, UCHAR *Flags);
void InitMotorDrivers(void);
void DisableTMC5160(UCHAR Motor);
void EnableTMC5160(UCHAR Motor);

#endif
