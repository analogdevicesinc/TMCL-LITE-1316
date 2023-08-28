/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  IO_gd.c
           I/O routines and some other useful stuff

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
  \file IO_gd.c
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief I/O functions

  This file provides functions for intializing and
  using the I/O ports and some other miscellaneous
  stuff.
*/


#include <stdlib.h>
#include "gd32f4xx.h"
#include "bits.h"
#include "stealthRocker.h"
#include "IO.h"

#ifdef BOOTLOADER
extern UINT BLMagic;                        //!< Magic code for bootloader
#endif

/*********************************************//**
  \fn InitIO(void)
  \brief Initialize I/O

  This function initalizes all I/O port pins of
  the CPU that are not initialized in somewhere
  else in other initialization functions.
*************************************************/
void InitIO(void)
{
#if defined(BOOTLOADER)
  UINT Count1;
  UINT Count2;
  UINT i;
  volatile UINT t;
#endif

  //===== Port A =====
  rcu_periph_clock_enable(RCU_GPIOA);

  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15|GPIO_PIN_4);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_15|GPIO_PIN_4);
  GPIO_BOP(GPIOA)=GPIO_PIN_15|GPIO_PIN_4;

  //Clock output (PA8)
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_8);
  gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_8);
  rcu_ckout0_config(RCU_CKOUT0SRC_HXTAL, RCU_CKOUT0_DIV1);

  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);

  //===== Port B =====
  rcu_periph_clock_enable(RCU_GPIOB);

  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_7);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_7);
  GPIO_BOP(GPIOB)=GPIO_PIN_15|GPIO_PIN_14;
  GPIO_BC(GPIOB)=GPIO_PIN_7;

  gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_13|GPIO_PIN_12|GPIO_PIN_6|GPIO_PIN_5);

  //===== Port C =====
  rcu_periph_clock_enable(RCU_GPIOC);

  gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_5|GPIO_PIN_4);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_5|GPIO_PIN_4);
  GPIO_BC(GPIOC)=GPIO_PIN_5|GPIO_PIN_4;

  //===== Port D =====
  rcu_periph_clock_enable(RCU_GPIOD);

  gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_11|GPIO_PIN_10);
  gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_11|GPIO_PIN_10);
  GPIO_BOP(GPIOD)=GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_11|GPIO_PIN_10;

  gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_13|GPIO_PIN_12|GPIO_PIN_9|GPIO_PIN_8);

  //===== Port E =====
  rcu_periph_clock_enable(RCU_GPIOE);

  gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_0);
  gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_0);
  GPIO_BOP(GPIOE)=GPIO_PIN_1|GPIO_PIN_0;
  GPIO_BC(GPIOE)=GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13;

  gpio_mode_set(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9|
                                                        GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_0);

  #ifdef BOOTLOADER
  //Check if SDIO and SDCLK are connected together and
  //return to bootloader if yes.
  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_14);

  Count1=Count2=0;
  for(i=0; i<10; i++)
  {
    GPIO_BC(GPIOA)=BIT13;
    for(t=0; t<4000; t++) asm volatile("nop\n");
    if(!(GPIO_ISTAT(GPIOA) & BIT14)) Count1++;

    GPIO_BOP(GPIOA)=BIT13;
    for(t=0; t<4000; t++) asm volatile("nop\n");
    if((GPIO_ISTAT(GPIOA) & BIT14)) Count2++;
  }
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14|GPIO_PIN_13);
  gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_14|GPIO_PIN_13);

  if(Count1==10 && Count2==10)
  {
    __disable_irq();
    NVIC_DeInit();
    SysTick->CTRL=0;

    BLMagic=0x12345678;
    NVIC_SystemReset();
  }
  #endif

  //Release TMC4361 reset
  GPIO_BOP(GPIOB)=GPIO_PIN_7;
}


/***************************************************************//**
   \fn EnableInterrupts(void)
   \brief Enable all interrupts

  This function globally enables all interrupts.
********************************************************************/
void EnableInterrupts(void)
{
  asm volatile("CPSIE I\n");
}


/***************************************************************//**
   \fn DisableInterrupts(void)
   \brief Disable all interrupts

  This function globally disables all interrupts.
********************************************************************/
void DisableInterrupts(void)
{
  asm volatile("CPSID I\n");
}


void NVIC_DeInit(void)
{
  uint32_t index;

  for(index=0; index<8; index++)
  {
    NVIC->ICER[index] = 0xFFFFFFFF;
    NVIC->ICPR[index] = 0xFFFFFFFF;
  }

  for(index = 0; index < 240; index++)
  {
     NVIC->IP[index] = 0x00000000;
  }
}
