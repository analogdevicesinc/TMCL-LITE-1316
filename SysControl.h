/*******************************************************************************
* Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices, Inc.),
*
* Copyright © 2023 Analog Devices, Inc.
*******************************************************************************/

/**
  This file contains the definitions of the functions from the SysControl.c
  module.
*/

void SetStepDirMode(UCHAR Axis, UCHAR Mode);

UCHAR GetClosedLoopInitFlag(UCHAR Axis);

void StartTorqueMode(UCHAR Axis, int Torque);
void StopTorqueMode(UCHAR Axis);
int GetTorqueModeCurrent(UCHAR Axis);

void SystemControl(void);
