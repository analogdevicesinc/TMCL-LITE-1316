/*******************************************************************************
* Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
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
