/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  SysControl.h
           

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
  \file SysControl.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Motor monitoring 

  This file contains the definitions of the functions from the SysControl.c
  module.
*/

void SetStepDirMode(UCHAR Axis, UCHAR Mode);

UCHAR GetClosedLoopInitFlag(UCHAR Axis);

void StartTorqueMode(UCHAR Axis, int Torque);
void StopTorqueMode(UCHAR Axis);
int GetTorqueModeCurrent(UCHAR Axis);

void SystemControl(void);
