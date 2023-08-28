/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  IO.h
           Definitions of I/O functions

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
  \file IO.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief I/O functions

  This file contains definitions for using the I/O functions.
*/

void InitIO(void);
void EnableInterrupts(void);
void DisableInterrupts(void);
void ResetCPU(UCHAR ResetPeripherals);
