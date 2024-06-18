/*******************************************************************************
* Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains basic type and macro definitions needed by all
  modules of this project.
*/

typedef unsigned char UCHAR;                   //!< 8 bits unsigned
typedef unsigned short USHORT;                 //!< 16 bits unsigned
typedef unsigned int UINT;                     //!< 32 bits unsigned

#define TRUE 1
#define FALSE 0

#define SW_TYPE_HIGH 0x05                      //!< module number (1316) high byte
#define SW_TYPE_LOW  0x24                      //!< module number (1316) low byte

#define SW_VERSION_HIGH 0x01                   //!< software version high byte
#define SW_VERSION_LOW  0x02                   //!< software version low byte

#define N_O_MOTORS 1                           //!< number of motors supported by this module
#define WHICH_262(a) (a)                       //!< extract TMC262 motor number (for future extensions)

#if defined(MK20DX128)
  #define DISABLE_DRIVERS() GPIOD_PSOR = BIT2    //!< turn off all motor drivers
  #define ENABLE_DRIVERS()  GPIOD_PCOR = BIT2    //!< turn on all motor drivers

  #define LED1_ON()       GPIOA_PSOR = BIT5      //!< turn on LED 1
  #define LED1_OFF()      GPIOA_PCOR = BIT5      //!< turn off LED 1
  #define LED1_TOGGLE()   GPIOA_PTOR = BIT5      //!< toggle LED 1

  #define LED2_ON()       GPIOE_PSOR = BIT5      //!< turn on LED 2
  #define LED2_OFF()      GPIOE_PCOR = BIT5      //!< turn off LED 2
  #define LED2_TOGGLE()   GPIOE_PTOR = BIT5      //!< toggle LED 2

  #define SPI_DEV_EEPROM    0x0001               //!< SPI device number of the EEPROM
  #define SPI_DEV_TMC43xx_0 0x0101               //!< SPI device number of TMC4361

#elif defined(GD32F425)
  #define ENABLE_DRIVERS()     GPIO_BC(GPIOE)=BIT1
  #define DISABLE_DRIVERS()    GPIO_BOP(GPIOE)=BIT1

  #define LED1_ON()            GPIO_BOP(GPIOC)=BIT5
  #define LED1_OFF()           GPIO_BC(GPIOC)=BIT5
  #define LED1_TOGGLE()        GPIO_TG(GPIOC)=BIT5
  #define LED2_ON()            GPIO_BOP(GPIOC)=BIT4
  #define LED2_OFF()           GPIO_BC(GPIOC)=BIT4
  #define LED2_TOGGLE()        GPIO_TG(GPIOC)=BIT4

  #define SPI_DEV_EEPROM              1
  #define SPI_DEV_TMC43xx_0           2

  #define SELECT_EEPROM()       GPIO_BC(GPIOA)=BIT15
  #define DESELECT_EEPROM()     GPIO_BOP(GPIOA)=BIT15
  #define SELECT_TMC43xx_0()    GPIO_BC(GPIOA)=BIT4
  #define DESELECT_TMC43xx_0()  GPIO_BOP(GPIOA)=BIT4
#endif

#define RAMP_TRAPEZ 0                          //!< trapezoidal ramp
#define RAMP_SSHAPE 1                          //!< s-shaped ramp


//! Global module settings
typedef struct
{
  UCHAR SerialBitrate;         //!< RS485 baud rate (0..7, 0=9600bps)
  UCHAR SerialModuleAddress;   //!< RS485 TMCL module address
  UCHAR SerialHostAddress;     //!< RS485 TMCL reply address
  UCHAR CANReceiveID;          //!< CAN receive ID
  UCHAR CANSendID;             //!< CAN send ID
  UCHAR CANBitrate;            //!< CAN bit rate
} TModuleConfig;

//! Motor configuration data
typedef struct
{
  UCHAR IRun;
  UCHAR IStandby;
  UCHAR MicrostepResolution;
  UCHAR SwitchMode;
  USHORT FreewheelingDelay;
  UINT StallVMin;
  USHORT SettingDelay;
  UCHAR BoostCurrent;
  UCHAR RampType;
  UCHAR UnitMode;
  UCHAR StepDirMode;
  int EncoderResolution;
  UINT MaxPositionDeviation;
  UINT MaxVelocityDeviation;
  UINT AMax;
  UINT DMax;
  UINT AStart;
  UINT DFinal;
  UINT DStop;
  USHORT MotorResolution;
} TMotorConfig;

typedef struct
{
  UCHAR ClosedLoopMode;
  UINT GammaVMin;
  UINT GammaVAdd;
  UCHAR Gamma;
  USHORT Beta;
  int Offset;
  UCHAR CurrentScalerMinimum;
  UCHAR CurrentScalerMaximum;
  UCHAR CurrentScalerStartUp;
  UINT UpscaleDelay;
  UINT DownscaleDelay;
  UINT CorrectionVelocityP;
  UINT CorrectionVelocityI;
  UINT CorrectionVelocityIClip;
  UINT CorrectionVelocityDClk;
  UINT CorrectionVelocityDClip;
  UINT PositionCorrectionP;
  UINT PositionCorrectionTolerance;
  UINT PositionWindow;
  UCHAR EncVMeanWait;
  UCHAR EncVMeanFilter;
  UINT EncVMeanInt;
  signed char EncoderCorrectionYOffset;
} TClosedLoopConfig;

//! coolStep configuration data
typedef struct
{
  UINT ThresholdSpeed;         //!< coolStep threshold speed
  UCHAR SlowRunCurrent;        //!< coolStep slow run current
  UCHAR HysteresisStart;       //!< coolStep hysteresis start
} TCoolStepConfig;
