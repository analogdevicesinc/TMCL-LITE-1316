/*******************************************************************************
* Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices, Inc.),
*
* Copyright © 2023 Analog Devices, Inc.
*******************************************************************************/

/**
  This file contains the definitions of the SPI access functions.
*/

void InitSPI(void);
UCHAR ReadWriteSPI(USHORT DeviceNumber, UCHAR Data, UCHAR LastTransfer);



