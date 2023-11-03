/*******************************************************************************
* Copyright © 2018 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains all macro and function definitions of the TMC4361 library.
*/


#ifndef __TMC4361_H
#define __TMC4361_H

//Register address definitions
#define TMC43xx_GENERAL_CONF        0x00
#define TMC43xx_REFERENCE_CONF      0x01
#define TMC43xx_START_CONF          0x02
#define TMC43xx_INPUT_FILT_CONF     0x03
#define TMC43xx_SPI_OUT_CONF        0x04
#define TMC43xx_CURRENT_CONF        0x05
#define TMC43xx_SCALE_VALUES        0x06
#define TMC43xx_ENC_IN_CONF         0x07
#define TMC43xx_ENC_IN_DATA         0x08
#define TMC43xx_ENC_OUT_DATA        0x09
#define TMC43xx_STEP_CONF           0x0A
#define TMC43xx_SPI_STATUS_SEL      0x0B
#define TMC43xx_EVENT_CLR_CONF      0x0C
#define TMC43xx_INTR_CONF           0x0D
#define TMC43xx_EVENTS              0x0E
#define TMC43xx_STATUS              0x0F
#define TMC43xx_STEP_DIR_TIME       0x10
#define TMC43xx_START_OUT_ADD       0x11
#define TMC43xx_GEAR_RATIO          0x12
#define TMC43xx_START_DELAY         0x13
#define TMC43xx_CLK_GATING_DELAY    0x14
#define TMC43xx_STDBY_DELAY         0x15
#define TMC43xx_FREEWHEEL_DELAY     0x16
#define TMC43xx_VDRV_SCALE_LIM      0x17
#define TMC43xx_PWM_VMAX            0x17    //same as VDRV_SCALE_LIM
#define TMC43xx_UPSCALE_DELAY       0x18
#define TMC43xx_CL_UPSCALE_DELAY    0x18    //same as UPSCALE_DELAY
#define TMC43xx_HOLD_SCALE_DELAY    0x19
#define TMC43xx_CL_DOWNSCALE_DELAY  0x19    //same as HOLD_SCALE_DELAY
#define TMC43xx_DRV_SCALE_DELAY     0x1A
#define TMC43xx_BOOST_TIME          0x1B
#define TMC43xx_CL_BETA_GAMMA       0x1C
#define TMC43xx_DAC_ADDR_AB         0x1D
#define TMC43xx_SPI_SWITCH_VEL      0x1D    //same as DAC_ADDR_AB
#define TMC43xx_HOME_SAFETY_MARGIN  0x1E
#define TMC43xx_PWM_FREQ            0x1F
#define TMC43xx_CHOPSYNC_DIV        0x1F    //same as PWM_FREQ
#define TMC43xx_RAMPMODE            0x20
#define TMC43xx_XACTUAL             0x21
#define TMC43xx_VACTUAL             0x22
#define TMC43XX_AACTUAL             0x23
#define TMC43xx_VMAX                0x24
#define TMC43xx_VSTART              0x25
#define TMC43xx_VSTOP               0x26
#define TMC43xx_VBREAK              0x27
#define TMC43xx_AMAX                0x28
#define TMC43xx_DMAX                0x29
#define TMC43xx_ASTART              0x2A
#define TMC43xx_DFINAL              0x2B
#define TMC43xx_DSTOP               0x2C
#define TMC43xx_BOW1                0x2D
#define TMC43xx_BOW2                0x2E
#define TMC43xx_BOW3                0x2F
#define TMC43xx_BOW4                0x30
#define TMC43xx_CLOCK_FREQ          0x31
#define TMC43xx_POSCOMP             0x32
#define TMC43xx_VIRT_STOP_LEFT      0x33
#define TMC43xx_VIRT_STOP_RIGHT     0x34
#define TMC43xx_XHOME               0x35
#define TMC43xx_XLATCH_R            0x36   //read only
#define TMC43xx_REV_CNT_R           0x36   //read only, same as XLATCH_R
#define TMC43xx_XRANGE_W            0x36   //write only, same as XLATCH_R
#define TMC43xx_XTARGET             0x37
#define TMC43xx_XPIPE0              0x38
#define TMC43xx_XPIPE1              0x39
#define TMC43xx_XPIPE2              0x3A
#define TMC43xx_XPIPE3              0x3B
#define TMC43xx_XPIPE4              0x3C
#define TMC43xx_XPIPE5              0x3D
#define TMC43xx_XPIPE6              0x3E
#define TMC43xx_XPIPE7              0x3F
#define TMC43xx_SH_REG0             0x40
#define TMC43xx_SH_REG1             0x41
#define TMC43xx_SH_REG2             0x42
#define TMC43xx_SH_REG3             0x43
#define TMC43xx_SH_REG4             0x44
#define TMC43xx_SH_REG5             0x45
#define TMC43xx_SH_REG6             0x46
#define TMC43xx_SH_REG7             0x47
#define TMC43xx_SH_REG8             0x48
#define TMC43xx_SH_REG9             0x49
#define TMC43xx_SH_REG10            0x4A
#define TMC43xx_SH_REG11            0x4B
#define TMC43xx_SH_REG12            0x4C
#define TMC43xx_SH_REG13            0x4D
#define TMC43xx_IFREEZE_DFREEZE     0x4E
#define TMC43xx_CLK_GATING          0x4F
#define TMC43xx_ENC_POS             0x50
#define TMC43xx_ENC_LATCH_R         0x51   //read only
#define TMC43xx_ENC_RES_VAL_W       0x51   //write only, same as ENC_LATCH_R
#define TMC43xx_ENC_POS_DEV_R       0x52   //read only
#define TMC43xx_CL_TR_TOLERANCE_W   0x52   //write only, same as ENC_POS_DEV
#define TMC43xx_ENC_POS_DEV_TOL     0x53
#define TMC43xx_ENC_IN_RES_W        0x54   //write only
#define TMC43xx_ENC_CONST_R         0x54   //read only, same as ENC_IN_RES_W
#define TMC43xx_ENC_OUT_RES         0x55   //write only
#define TMC43xx_SER_CLK_IN          0x56   //write only
#define TMC43xx_SSI_IN_DELAY        0x57   //write only
#define TMC43xx_BISS_TIMEOUT        0x57   //write only, same as SSI_IN_DELAY
#define TMC43xx_SSI_IN_PTIME        0x58   //write only
#define TMC43xx_BISS_IN_BUSYR       0x58   //write only, same as SSI_IN_PTIME
#define TMC43xx_CL_OFFSET           0x59   //write only
#define TMC43xx_PID_P_W             0x5A   //write only
#define TMC43xx_CL_VMAX_P_W         0x5A   //write only, same as PID_P_W
#define TMC43xx_PID_VEL_R           0x5A   //read only, same as PID_P_W
#define TMC43xx_PID_I_W             0x5B   //write only
#define TMC43xx_CL_VMAX_CALC_I_W    0x5B   //write only, same as PID_I_W
#define TMC43xx_PID_ISUM_RD         0x5B   //read only, same as PID_I_W
#define TMC43xx_PID_D_W             0x5C   //write only
#define TMC43xx_CL_DELTA_P_W        0x5C   //write only
#define TMC43xx_PID_I_CLIP_D_CLK_W  0x5D   //write_only
#define TMC43xx_PID_E_R             0x5D   //read only, same as PID_I_CLIP_W
#define TMC43xx_PID_DV_CLIP         0x5E   //write only
#define TMC43xx_PID_TOLERANCE       0x5F   //write only
#define TMC43xx_CL_TOLERANCE        0x5F   //write only, same as PID_TOLERANCE
#define TMC43xx_FS_VEL              0x60   //write only
#define TMC43xx_DC_VEL              0x60   //write only, same as FS_VEL
#define TMC43xx_CL_VMIN_EMF         0x60   //write only, same as FS_VEL
#define TMC43xx_DC_TIME_SG          0x61   //write only
#define TMC43xx_CL_VADD_EMF         0x61   //write only, same as DC_TIME_SG
#define TMC43xx_DC_LSPTM            0x62   //write only
#define TMC43xx_ENC_VEL_ZERO        0x62   //write only, same as DC_LSPTM
#define TMC43xx_ENC_VMEAN_WAIT_FILT 0x63   //write only
#define TMC43xx_SNCHRO_SET          0x64   //write only
#define TMC43xx_VENC                0x65   //read only
#define TMC43xx_VENC_MEAN           0x66   //read only
#define TMC43xx_VSTALL_LIMIT        0x67   //write only
#define TMC43xx_CIRCULAR_DEC        0x7C   //write only
#define TMC43xx_COMP_OFFSET_AMPL    0x7D   //write only
#define TMC43xx_ADDR_TO_ENC         0x68   //write only
#define TMC43xx_DATA_TO_ENC         0x69   //write only
#define TMC43xx_ADDR_FROM_ENC       0x6A   //read only
#define TMC43xx_DATA_FROM_ENC       0x6B   //read only
#define TMC43xx_COVER_LOW           0x6C   //write only
#define TMC43xx_COVER_HIGH          0x6D   //write only
#define TMC43xx_POLLING_1_R         0x6C   //read only
#define TMC43xx_POLLING_2_R         0x6D   //read only
#define TMC43xx_COVER_DRV_LOW       0x6E   //read only
#define TMC43xx_COVER_DRV_HIGH      0x6F   //read only
#define TMC43xx_MSLUT0              0x70   //write only
#define TMC43xx_MSLUT1              0x71   //write only
#define TMC43xx_MSLUT2              0x72   //write only
#define TMC43xx_MSLUT3              0x73   //write only
#define TMC43xx_MSLUT4              0x74   //write only
#define TMC43xx_MSLUT5              0x75   //write only
#define TMC43xx_MSLUT6              0x76   //write only
#define TMC43xx_MSLUT7              0x77   //write only
#define TMC43xx_MSLUTSEL            0x78   //write only
#define TMC43xx_MSCNT               0x79   //read only
#define TMC43xx_CURRENT_AB          0x7A   //read only
#define TMC43xx_CURRENT_SPI         0x7B   //read only
#define TMC43xx_SCALE_PARAM         0x7C   //read only
#define TMC43xx_ENC_COMP_OFFSET     0x7D   //write only
#define TMC43xx_START_SIN_OFFSET    0x7E   //write only

#define TMC43xx_WRITE               0x80

//TMC43xx GENERAL_CONFIG bits
#define TMC43xx_GCONF_USE_AVSTART         0x00000001
#define TMC43xx_GCONF_DIRECT_ACC_EN       0x00000002
#define TMC43xx_GCONF_DIRECT_BOW_EN       0x00000004
#define TMC43xx_GCONF_STEP_INACT_POL      0x00000008
#define TMC43xx_GCONF_TOGGLE_STEP         0x00000010
#define TMC43xx_GCONF_POL_DIR_OUT         0x00000020
#define TMC43xx_GCONF_INT_SD              0x00000000
#define TMC43xx_GCONF_EXT_SD_HIGH         0x00000040
#define TMC43xx_GCONF_EXT_SD_LOW          0x00000080
#define TMC43xx_GCONF_EXT_SD_TOGGLE       0x000000C0
#define TMC43xx_GCONF_DIR_IN_POL          0x00000100
#define TMC43xx_GCONF_SD_INDIRECT         0x00000200
#define TMC43xx_GCONF_ENC_INC             0x00000000
#define TMC43xx_GCONF_ENC_SSI             0x00000400
#define TMC43xx_GCONF_ENC_BISS            0x00000800
#define TMC43xx_GCONF_ENC_SPI             0x00000C00
#define TMC43xx_GCONF_ENC_DIFF_DIS        0x00001000
#define TMC43xx_GCONF_STDBY_CLOCK_LOW     0x00000000
#define TMC43xx_GCONF_STDBY_CLOCK_HIGH    0x00002000
#define TMC43xx_GCONF_STDBY_CHOPSYNC      0x00004000
#define TMC43xx_GCONF_STDBY_CLOCK_INT     0x00006000
#define TMC43xx_GCONF_INTR_POL            0x00008000
#define TMC43xx_GCONF_TARGET_REACHED_POL  0x00010000
#define TMC43xx_GCONF_CLK_GATING_EN       0x00020000
#define TMC43xx_GCONF_CLK_GATING_STDBY_EN 0x00040000
#define TMC43xx_GCONF_FS_EN               0x00080000
#define TMC43xx_GCONF_FS_SDOUT            0x00100000
#define TMC43xx_GCONF_DCSTEP_OFF          0x00000000
#define TMC43xx_GCONF_DCSTEP_AUTO         0x00200000
#define TMC43xx_GCONF_DCSTEP_TMC21xx      0x00400000
#define TMC43xx_GCONF_DCSTEP_TMC26x       0x00600000
#define TMC43xx_GCONF_PWM_OUT_EN          0x00800000
#define TMC43xx_GCONF_SER_ENC_OUT_EN      0x01000000
#define TMC43xx_GCONF_SER_ENC_OUT_DIFF    0x02000000
#define TMC43xx_GCONF_AUTO_DIRECT_SD_OFF  0x04000000
#define TMC43xx_GCONF_CIRC_CNT_XLATCH     0x08000000
#define TMC43xx_GCONF_REV_DIR             0x10000000
#define TMC43xx_GCONF_INTR_TR_PU_PD_EN    0x20000000
#define TMC43xx_GCONF_INTR_WIRED_AND      0x40000000
#define TMC43xx_GCONF_TR_WIRED_AND        0x80000000

//TMC43xx_SPI_OUT_CONF bits
#define TMC43xx_SPIOUT_OFF                0
#define TMC43xx_SPIOUT_TMC23x             0x00000008
#define TMC43xx_SPIOUT_TMC24x             0x00000009
#define TMC43xx_SPIOUT_TMC26x_389         0x0000000A
#define TMC43xx_SPIOUT_TMC26x_389_SD      0x0000000B
#define TMC43xx_SPIOUT_TMC21xx_SD         0x0000000C
#define TMC43xx_SPIOUT_TMC21xx            0x0000000D
#define TMC43xx_SPIOUT_SCALE              0x00000004
#define TMC43xx_SPIOUT_SINLUT             0x00000005
#define TMC43xx_SPIOUT_DACADDR            0x00000006
#define TMC43xx_SPIOUT_DAC                0x00000002
#define TMC43xx_SPIOUT_DAC_INV            0x00000003
#define TMC43xx_SPIOUT_DAC_MAPPED         0x00000001
#define TMC43xx_SPIOUT_COVER_ONLY         0x0000000F
#define TMC43xx_SPIOUT_MD_OFF             0x00000000
#define TMC43xx_SPIOUT_MD_FALLING         0x00000010
#define TMC43xx_SPIOUT_MD_NO_STANDBY      0x00000020
#define TMC43xx_SPIOUT_MD_ALWAYS          0x00000030
#define TMC43xx_SPIOUT_STDBY_ON_STALL     0x00000040
#define TMC43xx_SPIOUT_STALL_FLAG         0x00000080
#define TMC43xx_STALL_LOAD_LIMIT(x)       ((x & 0x07) << 8)
#define TMC43xx_SPIOUT_PHASE_SHIFT        0x00000100
#define TMC43xx_SPIOUT_THREE_PHASE_EN     0x00000010
#define TMC43xx_SPIOUT_SCALE_VAL_TR_EN    0x00000020
#define TMC43xx_SPIOUT_DISABLE_POLLING    0x00000040
#define TMC43xx_SPIOUT_ENABLE_SHADOW_DATAGRAMS     0x00000080
#define TMC43xx_SPIOUT_POLL_BLOCK_MULTI(x)   (((x) & 0x0f) << 8)
#define TMC43xx_SPIOUT_COVER_DONE_NOT_FOR_CURRENT  0x00001000

//TMC43xx ENC_IN configuration bits
#define TMC43xx_ENC_IN_MODE_OL            0x00000000
#define TMC43xx_ENC_IN_MODE_CL            0x00400000
#define TMC43xx_ENC_IN_MODE_PID_0         0x00800000
#define TMC43xx_ENC_IN_MODE PID_V         0x00C00000
#define TMC43xx_ENC_IN_CL_CALIBRATION_EN  0x01000000
#define TMC43xx_ENC_IN_CL_EMF_EN          0x02000000
#define TMC43xx_ENC_IN_CL_VLIMIT_EN       0x08000000
#define TMC43xx_ENC_IN_CL_VELOCITY_EN     0x10000000
#define TMC43xx_ENC_IN_SER_VAR_LIMIT      0x80000000

//TMC43xx_CURRENT_CONF bits
#define TMC43xx_CURCONF_HOLD_EN           0x00000001
#define TMC43xx_CURCONF_DRIVE_EN          0x00000002
#define TMC43xx_CURCONF_BOOST_ACC_EN      0x00000004
#define TMC43xx_CURCONF_BOOST_DEC_EN      0x00000008
#define TMC43xx_CURCONF_BOOST_START_EN    0x00000010
#define TMC43xx_CURCONF_SEC_DRIVE_EN      0x00000020
#define TMC43xx_CURCONF_FREEWHEELING_EN   0x00000040
#define TMC43xx_CURCONF_CL_SCALE_EN       0x00000080
#define TMC43xx_CURCONF_PWM_SCALE_REF     0x00000100
#define TMC43xx_CURCONF_PWM_AMPL(x)       (x<<16)

//TMC43xx_SCALE_VALUES bits
#define TMC43xx_SCALEVAL_BOOST(x)         (x & 0xff)
#define TMC43xx_SCALEVAL_DRV1(x)          ((x & 0xff) << 8)
#define TMC43xx_SCALEVAL_DRV2(x)          ((x & 0xff) << 16)
#define TMC43xx_SCALEVAL_HOLD(x)          ((x & 0xff) << 24)

//TMC43xx_EVENTS register bits
#define TMC43xx_EV_TARGET_REACHED         0x00000001
#define TMC43xx_EV_POSCOMP_REACHED        0x00000002
#define TMC43xx_EV_VELOCITY_REACHED       0x00000004
#define TMC43xx_EV_VZERO                  0x00000008
#define TMC43xx_EV_VPOSITIVE              0x00000010
#define TMC43xx_EV_VNEGATIVE              0x00000020
#define TMC43xx_EV_AZERO                  0x00000040
#define TMC43xx_EV_APOSITIVE              0x00000080
#define TMC43xx_EV_ANEGATIVE              0x00000100
#define TMC43xx_EV_MAX_PHASE_TRAP         0x00000200
#define TMC43xx_EV_FROZEN                 0x00000400
#define TMC43xx_EV_STOP_LEFT              0x00000800
#define TMC43xx_EV_STOP_RIGHT             0x00001000
#define TMC43xx_EV_VIRT_STOP_LEFT         0x00002000
#define TMC43xx_EV_VIRT_STOP_RIGHT        0x00004000
#define TMC43xx_EV_HOME_ERROR             0x00008000
#define TMC43xx_EV_XLATCH_DONE            0x00010000
#define TMC43xx_EV_FS_ACTIVE              0x00020000
#define TMC43xx_EV_ENC_FAIL               0x00040000
#define TMC43xx_EV_N_ACTIVE               0x00080000
#define TMC43xx_EV_ENC_DONE               0x00100000
#define TMC43xx_EV_SER_ENC_DATA_FAIL      0x00200000
#define TMC43xx_EV_CRC_FAIL               0x00400000
#define TMC43xx_EV_SER_DATA_DONE          0x00800000
#define TMC43xx_EV_BISS_FLAG              0x01000000
#define TMC43xx_EV_COVER_DONE             0x02000000
#define TMC43xx_EV_ENC_VZERO              0x04000000
#define TMC43xx_EV_CL_MAX                 0x08000000
#define TMC43xx_EV_CL_FIT                 0x10000000
#define TMC43xx_EV_STOP_ON_STALL          0x20000000
#define TMC43xx_EV_MOTOR                  0x40000000
#define TMC43xx_EV_RST                    0x80000000

//TMC43xx_STATUS register bits
#define TMC43xx_ST_TARGET_REACHED         0x00000001
#define TMC43xx_ST_POSCOMP_REACHED        0x00000002
#define TMC43xx_ST_VEL_REACHED            0x00000004
#define TMC43xx_ST_VEL_POS                0x00000008
#define TMC43xx_ST_VEL_NEG                0x00000010
#define TMC43xx_ST_RAMP_ACC               0x00000020
#define TMC43xx_ST_RAMP_DEC               0x00000040
#define TMC43xx_ST_STOP_LEFT_ACTIVE       0x00000080
#define TMC43xx_ST_STOP_RIGHT_ACTIVE      0x00000100
#define TMC43xx_ST_VIRT_STOP_LEFT_ACTIVE  0x00000200
#define TMC43xx_ST_VIRT_STOP_RIGHT_ACTIVE 0x00000400
#define TMC43xx_ST_HOME_ERROR             0x00001000
#define TMC43xx_ST_ENC_FAIL               0x00004000

//TMC43xx ramp modes
#define TMC43xx_RAMPMODE_VEL_HOLD    0
#define TMC43xx_RAMPMODE_VEL_TRAPEZ  1
#define TMC43xx_RAMPMODE_VEL_SSHAPE  2
#define TMC43xx_RAMPMODE_POS_HOLD    4
#define TMC43xx_RAMPMODE_POS_TRAPEZ  5
#define TMC43xx_RAMPMODE_POS_SSHAPE  6

//TMC43xx reference switch configuration
#define TMC43xx_REFCONF_STOP_LEFT_EN      0x00000001
#define TMC43xx_REFCONF_STOP_RIGHT_EN     0x00000002
#define TMC43xx_REFCONF_POL_STOP_LEFT     0x00000004
#define TMC43xx_REFCONF_POL_STOP_RIGHT    0x00000008
#define TMC43xx_REFCONF_INV_STOP_DIR      0x00000010
#define TMC43xx_REFCONF_SOFT_STOP_EN      0x00000020
#define TMC43xx_REFCONF_VIRT_LEFT_LIM_EN  0x00000040
#define TMC43xx_REFCONF_VIRT_RIGHT_LIM_EN 0x00000080
#define TMC43xx_REFCONF_VIRT_STOP_HARD    0x00000100
#define TMC43xx_REFCONF_VIRT_STOP_LINEAR  0x00000200
#define TMC43xx_REFCONF_CIRCULAR          0x00400000
#define TMC43xx_REFCONF_STOP_ON_STALL     0x04000000
#define TMC43xx_REFCONF_DRV_AFTER_STALL   0x08000000
#define TMC43xx_REFCONF_CIRCULAR_ENC_EN   0x80000000


//TMC43xx access functions
void WriteTMC43xxBytes(UCHAR Axis, UCHAR Address, UCHAR x1, UCHAR x2, UCHAR x3, UCHAR x4);
void WriteTMC43xxInt(UCHAR Axis, UCHAR Address, int Value);
int ReadTMC43xxInt(UCHAR Axis, UCHAR Address);
void TMC43xxSetBits(UCHAR Axis, UCHAR Address, UINT BitMask);
void TMC43xxClearBits(UCHAR Axis, UCHAR Address, UINT BitMask);
void TMC43xxWriteBits(UCHAR Axis, UCHAR Address, UINT Value, UCHAR Start, UCHAR Size);
UINT PeekTMC43xxEvents(UCHAR Axis);
UINT ReadAndClearTMC43xxEvents(UCHAR Axis, UINT EventMask);
UCHAR GetHomeInput(UCHAR Motor);
void HardStop(UINT Axis);
void InitTMC43xx(void);
void ResetTMC43xx(void);

//Unit conversion functions
int ConvertVelocityUserToInternal(int UserVelocity);
int ConvertAccelerationUserToInternal(int UserAcceleration);
int ConvertVelocityInternalToUser(int InternalVelocity);
int ConvertAccelerationInternalToUser(int InternalAcceleration);
int ConvertInternalToInternal(int Internal);

#endif
