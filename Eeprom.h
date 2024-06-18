/*******************************************************************************
* Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains the EEPROM access function defintions.
*/

void WriteEepromByte(UINT Address, UCHAR Value);
void WriteEepromBlock(UINT Address, UCHAR *Block, UINT Size);
UCHAR ReadEepromByte(UINT Address);
void ReadEepromBlock(UINT Address, UCHAR *Block, UINT Size);

