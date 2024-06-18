/*******************************************************************************
* Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains EEPROM access functions.
*/

#include <stdlib.h>
#if defined(MK20DX128)
  #include "derivative.h"
#elif defined(GD32F425)
  #include "gd32f4xx.h"
#endif
#include "bits.h"
#include "stealthRocker.h"
#include "SPI.h"


/***************************************************************//**
   \fn WriteEepromByte(UINT Address, UCHAR Value)
   \brief Write a byte to the EEPROM
   \param Address  EEPROM location (0..16383)
   \param Value    Byte to be written

   This function writes a byte to the EEPROM at the specified
   EEPROM location.
********************************************************************/
void WriteEepromByte(UINT Address, UCHAR Value)
{
  //Schreiben erlauben
  ReadWriteSPI(SPI_DEV_EEPROM, 0x06, TRUE);  //Befehl "Write Enable"
  do
  {
    ReadWriteSPI(SPI_DEV_EEPROM, 0x05, FALSE);  //Befehl "Get Status"
  } while((ReadWriteSPI(SPI_DEV_EEPROM, 0x00, TRUE) & 0x02)==0x00);  //Warte bis "Write Enable"-Bit gesetzt

  //Eigentliches Schreiben
  ReadWriteSPI(SPI_DEV_EEPROM, 0x02, FALSE); //Befehl "Write"
  ReadWriteSPI(SPI_DEV_EEPROM, Address >> 8, FALSE);
  ReadWriteSPI(SPI_DEV_EEPROM, Address & 0xff, FALSE);
  ReadWriteSPI(SPI_DEV_EEPROM, Value, TRUE);

  //Warten bis Schreibvorgang beendet ist
  do
  {
    ReadWriteSPI(SPI_DEV_EEPROM, 0x05, FALSE);  //Befehl "Get Status"
  } while(ReadWriteSPI(SPI_DEV_EEPROM, 0x00, TRUE) & 0x01);
}


/***************************************************************//**
   \fn WriteEepromBlock(UINT Address, UCHAR *Block, UINT Size)
   \brief Copy memory block to EEPROM
   \param Address  EEPROM location (0..16383)
   \param Block  pointer at memory block to be copied to the EEPROM
   \param Size   size of block to be copied (bytes)

   This function copies a memory block to the EEPROM. It is capable
   of filling the entire EEPROM just whith one function call.
********************************************************************/
void WriteEepromBlock(UINT Address, UCHAR *Block, UINT Size)
{
  UINT i;

  //Schreiben erlauben
  ReadWriteSPI(SPI_DEV_EEPROM, 0x06, TRUE);  //Befehl "Write Enable"
  do
  {
    ReadWriteSPI(SPI_DEV_EEPROM, 0x05, FALSE);  //Befehl "Get Status"
  } while((ReadWriteSPI(SPI_DEV_EEPROM, 0x00, TRUE) & 0x02)==0x00);  //Warte bis "Write Enable"-Bit gesetzt

  //Schreibvorgang (Startadresse)
  ReadWriteSPI(SPI_DEV_EEPROM, 0x02, FALSE); //Befehl "Write"
  ReadWriteSPI(SPI_DEV_EEPROM, Address >> 8, FALSE);
  ReadWriteSPI(SPI_DEV_EEPROM, Address & 0xff, FALSE);

  //Eigentliches Schreiben der Daten
  for(i=0; i<Size; i++)
  {
    //Adresse mitzählen und bei Überlauf der untersten sechs Bits das EEPROM deselektieren
    //und neuen Write-Befehl senden (bzw. beim letzten Datenbyte einfach nur EEPROM
    //deselektieren).
    //Dies ist erforderlich, da beim Beschreiben im 25128 nur die untersten sechs Bits der
    //Adresse hochgezählt werden (anders als beim Lesen).
    Address++;
    ReadWriteSPI(SPI_DEV_EEPROM, *(Block+i), (Address & 0x0000003f)==0 || i==Size-1);
    if((Address & 0x0000003f)==0 && i<Size-1)  //Adressbits übergelaufen, aber noch Bytes zu schreiben?
    {
      //Warte bis Schreibvorgang beendet
      do
      {
        ReadWriteSPI(SPI_DEV_EEPROM, 0x05, FALSE);  //Befehl "Get Status"
      } while(ReadWriteSPI(SPI_DEV_EEPROM, 0x00, TRUE) & 0x01);

      //Neuer "Write Enable"-Befehl
      ReadWriteSPI(SPI_DEV_EEPROM, 0x06, TRUE);  //Befehl "Write Enable"
      do
      {
        ReadWriteSPI(SPI_DEV_EEPROM, 0x05, FALSE);  //Befehl "Get Status"
      } while((ReadWriteSPI(SPI_DEV_EEPROM, 0x00, TRUE) & 0x02)==0x00);  //Warte bis "Write Enable"-Bit gesetzt

      //Neuer "Write"-Befehl (mit der nächsten Adresse)
      ReadWriteSPI(SPI_DEV_EEPROM, 0x02, FALSE); //Befehl "Write"
      ReadWriteSPI(SPI_DEV_EEPROM, Address >> 8, FALSE);
      ReadWriteSPI(SPI_DEV_EEPROM, Address & 0xff, FALSE);
    }
  }

  //Warte bis Schreibvorgang beendet
  do
  {
    ReadWriteSPI(SPI_DEV_EEPROM, 0x05, FALSE);  //Befehl "Get Status"
  } while(ReadWriteSPI(SPI_DEV_EEPROM, 0x00, TRUE) & 0x01);
}


/***************************************************************//**
   \fn ReadEepromByte(UINT Address)
   \brief Read a byte from the EEPROM
   \param Address  EEPROM location (0..16383)
   \return byte read from EEPROM

   This function reads one byte from the EEPROM.
********************************************************************/
UCHAR ReadEepromByte(UINT Address)
{
  ReadWriteSPI(SPI_DEV_EEPROM, 0x03, FALSE);  //Befehl "Read"
  ReadWriteSPI(SPI_DEV_EEPROM, Address >> 8, FALSE);
  ReadWriteSPI(SPI_DEV_EEPROM, Address & 0xff, FALSE);

  return ReadWriteSPI(SPI_DEV_EEPROM, 0, TRUE);
}


/***************************************************************//**
   \fn ReadEepromBlock(UINT Address, UCHAR *Block, UINT Size)
   \brief Copy block from EEPROM to RAM
   \param   Address  EEPROM start address (0..16383)
   \param   Block    RAM start address
   \param   Size     Length of block (bytes)

   Read a memory block from the EEPROM. This can also be the entire
   EEPROM.
********************************************************************/
void ReadEepromBlock(UINT Address, UCHAR *Block, UINT Size)
{
  UINT i;

  ReadWriteSPI(SPI_DEV_EEPROM, 0x03, FALSE);  //Befehl "Read"
  ReadWriteSPI(SPI_DEV_EEPROM, Address >> 8, FALSE);
  ReadWriteSPI(SPI_DEV_EEPROM, Address & 0xff, FALSE);

  for(i=0; i<Size; i++)
    *(Block+i)=ReadWriteSPI(SPI_DEV_EEPROM, 0, i==Size-1);  //beim letzten Byte EEPROM deselektieren
}
