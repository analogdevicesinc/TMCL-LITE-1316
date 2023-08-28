/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Modul:   Globals.h
           Global variables and data structures.

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
  \file Globals.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Global variables

  This file contains the definitions for importing the variables
  defined in Globals.c.
*/

extern TModuleConfig ModuleConfig;
extern TMotorConfig MotorConfig[N_O_MOTORS];
extern TClosedLoopConfig ClosedLoopConfig[N_O_MOTORS];

extern UCHAR SmartEnergy[N_O_MOTORS];
extern UCHAR StallFlag[N_O_MOTORS];
extern UINT StallLevel[N_O_MOTORS];
extern UCHAR DriverFlags[N_O_MOTORS];
extern UCHAR VMaxModified[N_O_MOTORS];
extern int ClosedLoopPositionOffset[N_O_MOTORS];
extern int EncoderOffset[N_O_MOTORS];
extern UINT GearRatio[N_O_MOTORS];
extern UCHAR DeviationFlag[N_O_MOTORS];

extern UCHAR ExitTMCLFlag;
