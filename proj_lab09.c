/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   solutions/instaspin_foc/src/proj_lab09.c
//! \brief Automatic field weakening 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup PROJ_LAB09 PROJ_LAB09
//@{

//! \defgroup PROJ_LAB09_OVERVIEW Project Overview
//!
//! Experimentation with Field Weakening
//!

// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main.h"
#include "F2802x_Device.h" /*  This library comes from C:\ti\controlSUITE\development_kits\C2000_LaunchPad\f2802x_common\  */
//#include "C:\ti\motorware\motorware_1_01_00_14\sw\drivers\i2c\src\32b\f28x\f2806x\i2c.h"

#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif

// Include header files used in the main function

// **************************************************************************
// the defines

#define LED_BLINK_FREQ_Hz   5
#define interval   16.667L
#define count 250

bool ARM_status = 1;
volatile Uint32 DutyOnTime1 = 0;
volatile Uint32 DutyOffTime1 = 0;
volatile Uint32 Period1 = 0;
_iq RPM_Max = 0;
volatile Uint16 Kv = 880;
volatile Uint16 Slave_Addr = 0x70;
volatile Uint16 SevenSegInit[] = {0x21, 0xA3, 0xEF, 0x81};
volatile Uint16 i2cIndex = 0;
volatile float RC_SigTimeOn = 0;
volatile float RC_SigTimeOff = 0;
_iq RCSpeedRef = 0;
volatile Uint32 eCapBuf = 0;
float_t Ls_d_ben;
float_t Ls_q_ben;
volatile float USER_MOTOR_Rs_M = 0.069;
//volatile float_t Rs_ben = 0.069;
#define LED_BLINK_FREQ_Hz   5
int XRDY_bit;
Uint16 i = 0;
Uint16 delay;
Uint16 CPU_Cycles = 0;
uint8_t RxByte = 0;
//***************************************************************************
//  Function prototypes
void InitECapture();
void InitI2C();
void I2C_Send(uint16_t byte_count);
void I2C_SendtoAddr(uint16_t Addr, uint16_t data);
void I2C_SendwStart(uint16_t data);
// **************************************************************************

// **************************************************************************
//	Unions and Structs
union ReceiverData_U
{
	volatile uint8_t RxData[2];
}Rec_Data;
// **************************************************************************
// the globals

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

CTRL_Handle ctrlHandle;

HAL_Handle halHandle;

USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);

CAP_Handle capHandle;//Ben

I2C_Handle i2cHandle;//Ben

PLL_Handle pllHandle;//Ben

GPIO_Handle gpioHandle;

#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else
CTRL_Obj ctrl;				//v1p7 format
#endif

uint16_t gLEDcnt = 0;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif
FW_Obj fw;
FW_Handle fwHandle;

_iq Iq_Max_pu;


#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;

// **************************************************************************
// the functions

void main(void)
{
	//Ben was here



  uint_least8_t estNumber = 0;

  /********************************************************************************/
  //  Bens
  	capHandle = CAP_init((void *)CAP1_BASE_ADDR, sizeof(CAP_Obj));
  	//i2cHandle = I2C_init((void *)I2CA_BASE_ADDR,sizeof(I2C_Obj));

  /********************************************************************************/

#ifdef FAST_ROM_V1p6
  uint_least8_t ctrlNumber = 0;
#endif

  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
  #endif

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));

  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);

  // store user parameter error in global variable
  gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);


  // do not allow code execution if there is a user parameter error
  if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVars.Flag_enableSys = false;
        }
    }


  // initialize the user parameters
  USER_setParams(&gUserParams);


  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);


  // initialize the controller
#ifdef FAST_ROM_V1p6
  ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);  		//v1p6 format (06xF and 06xM devices)
  controller_obj = (CTRL_Obj *)ctrlHandle;
#else
  ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl));	//v1p7 format default
#endif


  {
    CTRL_Version version;

    // get the version number
    CTRL_getVersion(ctrlHandle,&version);

    gMotorVars.CtrlVersion = version;
  }


  // set the default controller parameters
  CTRL_setParams(ctrlHandle,&gUserParams);


  // Initialize field weakening
  fwHandle = FW_init(&fw,sizeof(fw));


  // Disable field weakening
  FW_setFlag_enableFw(fwHandle, false);


  // Clear field weakening counter
  FW_clearCounter(fwHandle);


  // Set the number of ISR per field weakening ticks
  FW_setNumIsrTicksPerFwTick(fwHandle, FW_NUM_ISR_TICKS_PER_CTRL_TICK);


  // Set the deltas of field weakening
  FW_setDeltas(fwHandle, FW_INC_DELTA, FW_DEC_DELTA);


  // Set initial output of field weakening to zero
  FW_setOutput(fwHandle, _IQ(0.0));


  // Set the field weakening controller limits
  FW_setMinMax(fwHandle,_IQ(USER_MAX_NEGATIVE_ID_REF_CURRENT_A/USER_IQ_FULL_SCALE_CURRENT_A),_IQ(0.0));


  // setup faults
  HAL_setupFaults(halHandle);


  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle);


  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);


  // enable global interrupts
  HAL_enableGlobalInts(halHandle);


  // enable debug interrupts
  HAL_enableDebugInt(halHandle);


  // disable the PWM
  HAL_disablePwm(halHandle);

  // setup the capture for RC
  InitECapture();


#ifdef DRV8301_SPI
  // turn on the DRV8301 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8301 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301Vars);
#endif


  // enable DC bus compensation
  CTRL_setFlag_enableDcBusComp(ctrlHandle, true);


  // compute scaling factors for flux and torque calculations
  gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
  gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
  gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
  gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

  /*Ben
  * Get the Voltage off the bus to calculate RPM_Max
  */
   RPM_Max = (gMotorVars.VdcBus_kV/10000) * Kv;

   // setup regs for i2c Ben
   //InitI2C();

  // I2C_ResetTxFifo(i2cHandle);


  for(;;)
  {
/*	  I2C_Send(0x21);
	  for(delay=0;delay<count;delay++)
  		{
  			CPU_Cycles = delay*5+9;
  		}
  			CPU_Cycles = 0;
      I2C_Send(0xAE);
	  for(delay=0;delay<count;delay++)
	  	{
	  		CPU_Cycles = delay*5+9;
	  	}
	  		CPU_Cycles = 0;
	  I2C_Send(0xEF);
	  for(delay=0;delay<count;delay++)
	  	{
	  		CPU_Cycles = delay*5+9;
	  	}
	  		CPU_Cycles = 0;
	  I2C_Send(0x81);
	  for(delay=0;delay<count;delay++)
	  	{
	  		CPU_Cycles = delay*5+9;
	  	}
	  		CPU_Cycles = 0;
	  I2C_SendtoAddr(0x00, 0x3F);
	  for(delay=0;delay<count;delay++)
	  		{
	  			CPU_Cycles = delay*5+9;
	  		}
	  			CPU_Cycles = 0;
	  I2C_SendtoAddr(0x01, 0x00);
	  for(delay=0;delay<count;delay++)
	  		{
	  			CPU_Cycles = delay*5+9;
	  		}
	  			CPU_Cycles = 0;
	  I2C_SendtoAddr(0x02, 0x3F);
	  for(delay=0;delay<count;delay++)
	  		{
	  			CPU_Cycles = delay*5+9;
	  		}
	  			CPU_Cycles = 0;
	  I2C_SendtoAddr(0x03, 0x00);
	  for(delay=0;delay<count;delay++)
	  		{
	  			CPU_Cycles = delay*5+9;
	  		}
	  			CPU_Cycles = 0;
	  I2C_SendtoAddr(0x04, 0x3F);
	  for(delay=0;delay<count;delay++)
	  		{
	  			CPU_Cycles = delay*5+9;
	  		}
	  			CPU_Cycles = 0;
	  I2C_SendtoAddr(0x05, 0x00);
	  for(delay=0;delay<count;delay++)
   	  		{
   	  			CPU_Cycles = delay*5+9;
   	  		}
	   	  			CPU_Cycles = 0;
	  I2C_SendtoAddr(0x06, 0x3F);
	  for(delay=0;delay<count;delay++)
 	  		{
 	  			CPU_Cycles = delay*5+9;
 	  		}
 	  			CPU_Cycles = 0;
 	  I2C_SendtoAddr(0x07, 0x00);
 	  for(delay=0;delay<count;delay++)
   	  		{
   	  			CPU_Cycles = delay*5+9;
   	  		}
   	  			CPU_Cycles = 0;
*/
	// Waiting for enable system flag to be set
    while(!(gMotorVars.Flag_enableSys));

    Flag_Latch_softwareUpdate = true;

    // Enable the Library internal PI.  Iq is referenced by the speed PI now
    CTRL_setFlag_enableSpeedCtrl(ctrlHandle, true);


    // loop while the enable system flag is true
    while(gMotorVars.Flag_enableSys)
      {
        CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

        // increment counters
        gCounter_updateGlobals++;

        // enable/disable the use of motor parameters being loaded from user.h
        CTRL_setFlag_enableUserMotorParams(ctrlHandle,gMotorVars.Flag_enableUserParams);

        //Rs_ben = gMotorVars.Rs_Ohm;	//This works for updating user variables
        Ls_d_ben = gMotorVars.Lsd_H;
        Ls_q_ben = gMotorVars.Lsq_H;

        // enable/disable Rs recalibration during motor startup
        EST_setFlag_enableRsRecalc(obj->estHandle,gMotorVars.Flag_enableRsRecalc);

        // enable/disable automatic calculation of bias values
        CTRL_setFlag_enableOffset(ctrlHandle,gMotorVars.Flag_enableOffsetcalc);


        if(CTRL_isError(ctrlHandle))
          {
            // set the enable controller flag to false
            CTRL_setFlag_enableCtrl(ctrlHandle,false);

            // set the enable system flag to false
            gMotorVars.Flag_enableSys = false;

            // disable the PWM
            HAL_disablePwm(halHandle);
          }
        else
          {
            // update the controller state
            bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

            // enable or disable the control
            CTRL_setFlag_enableCtrl(ctrlHandle, gMotorVars.Flag_MotorIdentified);//Ben Changed from Flag_Run_Identify

            //CTRL_setFlag_enableCtrl(ctrlHandle, gMotorVars.Flag_Run_Identify);//Ben Changed from Flag_Run_Identify


        	if(flag_ctrlStateChanged)
             	{
            		CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);//CTRL_State_OffLine;//Ben changed from CTRL_getState(ctrlHandle);

           		if(ctrlState == CTRL_State_OffLine)
            		{
            			// enable the PWM
            			HAL_enablePwm(halHandle);
            		}
                else if(ctrlState == CTRL_State_OnLine)
                  {
                    if(gMotorVars.Flag_enableOffsetcalc == true)
                    {
                      // update the ADC bias values
                      HAL_updateAdcBias(halHandle);
                    }
                    else
                    {
                      // set the current bias
                      HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(I_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(I_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(I_C_offset));

                      // set the voltage bias
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset));
                    }

                    // Return the bias value for currents
                    gMotorVars.I_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Current,0);
                    gMotorVars.I_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Current,1);
                    gMotorVars.I_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Current,2);

                    // Return the bias value for voltages
                    gMotorVars.V_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Voltage,0);
                    gMotorVars.V_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Voltage,1);
                    gMotorVars.V_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Voltage,2);

                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_Idle)
                  {
                    // disable the PWM
                    HAL_disablePwm(halHandle);
                    gMotorVars.Flag_Run_Identify = false;
                  }

                if((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true) &&
                  (ctrlState > CTRL_State_Idle) &&
                  (gMotorVars.CtrlVersion.minor == 6))
                  {
                    // call this function to fix 1p6
                    USER_softwareUpdate1p6(ctrlHandle);
                  }

              }
          }


        if(EST_isMotorIdentified(obj->estHandle))
          {
            _iq Is_Max_squared_pu = _IQ((USER_MOTOR_MAX_CURRENT*USER_MOTOR_MAX_CURRENT)/  \
    	      			  (USER_IQ_FULL_SCALE_CURRENT_A*USER_IQ_FULL_SCALE_CURRENT_A));
            _iq Id_squared_pu = _IQmpy(CTRL_getId_ref_pu(ctrlHandle),CTRL_getId_ref_pu(ctrlHandle));

            // Take into consideration that Iq^2+Id^2 = Is^2
            Iq_Max_pu = _IQsqrt(Is_Max_squared_pu-Id_squared_pu);

            //Set new max trajectory
            CTRL_setSpdMax(ctrlHandle, Iq_Max_pu);

            // set the current ramp
            EST_setMaxCurrentSlope_pu(obj->estHandle,gMaxCurrentSlope);
            gMotorVars.Flag_MotorIdentified = true;

            // set the speed reference Ben
            gMotorVars.SpeedRef_krpm = RCSpeedRef * 20000;


            CTRL_setSpd_ref_krpm(ctrlHandle,gMotorVars.SpeedRef_krpm);

            // set the speed acceleration
            CTRL_setMaxAccel_pu(ctrlHandle,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));

            if(Flag_Latch_softwareUpdate)
            {
              Flag_Latch_softwareUpdate = false;

              USER_calcPIgains(ctrlHandle);


              // initialize the watch window kp and ki current values with pre-calculated values
              gMotorVars.Kp_Idq = CTRL_getKp(ctrlHandle,CTRL_Type_PID_Id);
              gMotorVars.Ki_Idq = CTRL_getKi(ctrlHandle,CTRL_Type_PID_Id);
            }

          }
        else
          {
            Flag_Latch_softwareUpdate = true;

            // initialize the watch window kp and ki values with pre-calculated values
            gMotorVars.Kp_spd = CTRL_getKp(ctrlHandle,CTRL_Type_PID_spd);
            gMotorVars.Ki_spd = CTRL_getKi(ctrlHandle,CTRL_Type_PID_spd);


            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
          }


        // when appropriate, update the global variables
        if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
          {
            // reset the counter
            gCounter_updateGlobals = 0;

            updateGlobalVariables_motor(ctrlHandle);
          }


        // update Kp and Ki gains
        updateKpKiGains(ctrlHandle);

        // set field weakening enable flag depending on user's input
        FW_setFlag_enableFw(fwHandle,gMotorVars.Flag_enableFieldWeakening);

        // enable/disable the forced angle
        EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars.Flag_enableForceAngle);

        // enable or disable power warp
        CTRL_setFlag_enablePowerWarp(ctrlHandle,gMotorVars.Flag_enablePowerWarp);

       //Rs_ben = gMotorVars.Rs_Ohm;

#ifdef DRV8301_SPI
        HAL_writeDrvData(halHandle,&gDrvSpi8301Vars);

        HAL_readDrvData(halHandle,&gDrvSpi8301Vars);
#endif

      } // end of while(gFlag_enableSys) loop


    // disable the PWM
    HAL_disablePwm(halHandle);

    // set the default controller parameters (Reset the control to re-identify the motor)
    CTRL_setParams(ctrlHandle,&gUserParams);
    gMotorVars.Flag_Run_Identify = false;
    gMotorVars.Flag_enableCalibrate = false;//ben

  } // end of for(;;) loop

} // end of main() function


interrupt void mainISR(void)
{
  // toggle status LED
  if(gLEDcnt++ > (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
  {
    HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
    gLEDcnt = 0;
  }

  // acknowledge the ADC interrupt
  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);


  // convert the ADC data
  HAL_readAdcData(halHandle,&gAdcData);


  // run the controller
  CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData);


  // write the PWM compare values
  HAL_writePwmData(halHandle,&gPwmData);


  if(FW_getFlag_enableFw(fwHandle) == true)
    {
      FW_incCounter(fwHandle);

      if(FW_getCounter(fwHandle) > FW_getNumIsrTicksPerFwTick(fwHandle))
        {
    	  _iq refValue;
    	  _iq fbackValue;
    	  _iq output;

    	  FW_clearCounter(fwHandle);

    	  refValue = gMotorVars.VsRef;

    	  fbackValue = gMotorVars.Vs;

    	  FW_run(fwHandle, refValue, fbackValue, &output);

    	  CTRL_setId_ref_pu(ctrlHandle, output);

    	  gMotorVars.IdRef_A = _IQmpy(CTRL_getId_ref_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
        }
    }
  else
    {
      CTRL_setId_ref_pu(ctrlHandle, _IQmpy(gMotorVars.IdRef_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));
    }


  // setup the controller
  CTRL_setup(ctrlHandle);


  return;
} // end of mainISR() function


void updateGlobalVariables_motor(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  // get the speed estimate
  gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

  // get the real time speed reference coming out of the speed trajectory generator
  gMotorVars.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(obj->estHandle));

  // get the torque estimate
  gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

  // get the magnetizing current
  gMotorVars.MagnCurr_A = EST_getIdRated(obj->estHandle);

  // get the rotor resistance
  gMotorVars.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

  // get the stator resistance
  gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

  // get the stator inductance in the direct coordinate direction
  gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);

  // get the stator inductance in the quadrature coordinate direction
  gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);

  // get the flux in V/Hz in floating point
  gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

  // get the flux in Wb in fixed point
  gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

  // get the controller state
  gMotorVars.CtrlState = CTRL_getState(handle);

  // get the estimator state
  gMotorVars.EstState = EST_getState(obj->estHandle);

  // read Vd and Vq vectors per units
  gMotorVars.Vd = CTRL_getVd_out_pu(ctrlHandle);
  gMotorVars.Vq = CTRL_getVq_out_pu(ctrlHandle);

  // calculate vector Vs in per units
  gMotorVars.Vs = _IQsqrt(_IQmpy(gMotorVars.Vd, gMotorVars.Vd) + _IQmpy(gMotorVars.Vq, gMotorVars.Vq));

  // read Id and Iq vectors in amps
  gMotorVars.Id_A = _IQmpy(CTRL_getId_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
  gMotorVars.Iq_A = _IQmpy(CTRL_getIq_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

  // calculate vector Is in amps
  gMotorVars.Is_A = _IQsqrt(_IQmpy(gMotorVars.Id_A, gMotorVars.Id_A) + _IQmpy(gMotorVars.Iq_A, gMotorVars.Iq_A));

  // Get the DC buss voltage
  gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

  return;
} // end of updateGlobalVariables_motor() function


void updateKpKiGains(CTRL_Handle handle)
{
  if((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdate == false))
    {
      // set the kp and ki speed values from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVars.Kp_spd);
      CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVars.Ki_spd);

      // set the kp and ki current values for Id and Iq from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVars.Ki_Idq);
      CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVars.Ki_Idq);
	}

  return;
} // end of updateKpKiGains() function


void InitECapture()
{

	HAL_enableCapInt(halHandle);

    CAP_disableInt(capHandle, CAP_Int_Type_All);    // Disable all capture interrupts
    CAP_clearInt(capHandle, CAP_Int_Type_All);      // Clear all CAP interrupt flags
    CAP_disableCaptureLoad(capHandle);              // Disable CAP1-CAP4 register loads
    CAP_disableTimestampCounter(capHandle);         // Make sure the counter is stopped

    // Configure peripheral registers
    CAP_setCapOneShot(capHandle);                   // One-shot
    CAP_setStopWrap(capHandle, CAP_Stop_Wrap_CEVT2);// Stop at 4 events
    CAP_setCapEvtPolarity(capHandle, CAP_Event_1, CAP_Polarity_Rising);    // Rising edge
    CAP_setCapEvtPolarity(capHandle, CAP_Event_2, CAP_Polarity_Falling);     // Falling edge
//    CAP_setCapEvtPolarity(capHandle, CAP_Event_3, CAP_Polarity_Rising);    // Rising edge
//    CAP_setCapEvtPolarity(capHandle, CAP_Event_4, CAP_Polarity_Falling);     // Falling edge

    CAP_setCapEvtReset(capHandle, CAP_Event_1, CAP_Reset_Enable);   // Difference operation
    CAP_setCapEvtReset(capHandle, CAP_Event_2, CAP_Reset_Enable);   // Difference operation
//    CAP_setCapEvtReset(capHandle, CAP_Event_3, CAP_Reset_Enable);   // Difference operation
//    CAP_setCapEvtReset(capHandle, CAP_Event_4, CAP_Reset_Enable);   // Difference operation

    CAP_enableSyncIn(capHandle);                    // Enable sync in
    CAP_setSyncOut(capHandle, CAP_SyncOut_SyncIn);  // Pass through

    //CAP_enableCaptureLoad(capHandle);

    CAP_enableTimestampCounter(capHandle);          // Start Counter
    CAP_rearm(capHandle);                           // arm one-shot
    CAP_enableCaptureLoad(capHandle);               // Enable CAP1-CAP4 register loads
    CAP_enableInt(capHandle, CAP_Int_Type_CEVT2);   // 4 events = interrupt

    return;
}


interrupt void ecap1_isr(void)
{

    // Fetch Time-Stamp captured at T2
    DutyOnTime1 = CAP_getCap2(capHandle);
    RC_SigTimeOn = DutyOnTime1*(interval/1000000);//interval is the CPU_RATE
    // Fetch Time-Stamp captured at T3
//    DutyOffTime1 = CAP_getCap3(capHandle);
    // Fetch TimeStamp captured at T4
//    DutyOnTime2 = CAP_getCap4(capHandle);
    // Fetch Time-Stamp captured at T1
    //DutyOffTime1 = CAP_getCap1(capHandle);
    //RC_SigTimeOff = DutyOffTime1*(interval/1000000);

    Period1 = DutyOnTime1 + DutyOffTime1;

    /*Mapping of RC signal to RPM range of motor according to Kv rating
    * [(RC_SigTimeOn - RC_Low_End_mS) / (RC_High_End_mS - RC_Low_End_mS)] * Kv * Battery_Voltage
    */
    if(RC_SigTimeOn >= 1.11)
    {
    	RCSpeedRef = ((RC_SigTimeOn-1.1)/0.92)*14080;
    }
    else
    {
    	RCSpeedRef = 0;
    }

    eCapBuf = CAP_getCap1(capHandle);

    CAP_clearInt(capHandle, CAP_Int_Type_CEVT2);
    CAP_clearInt(capHandle, CAP_Int_Type_Global);
    CAP_rearm(capHandle);

    // Acknowledge this interrupt to receive more interrupts from group 4
//    PIE_clearInt(myPie, PIE_GroupNumber_4);
    HAL_clearInt(halHandle);
}

void InitI2C()
{
	HAL_enableI2cInt(halHandle);
	I2C_setMode(i2cHandle, 0x0000);//Clear IRS bit so clock can be initialized
	I2C_setupClock(i2cHandle, 6, 10, 5);//Prescale set to 60MHz/6 = 7-12MHz, clkLow 10, clkHigh 5
	I2C_setMode(i2cHandle, 0x0020);//Set IRS bit in MDR
	I2C_setMasterSlaveAddr(i2cHandle, Slave_Addr);//Slave_Addr = 0x70
	I2C_setSlaveAddress(i2cHandle, 0x50);
	I2C_enableInt(i2cHandle, (I2C_IntEnable_e)(I2C_IntEn_Tx_Rdy | I2C_IntEn_Rx_Rdy | I2C_IntEn_Stop));//Enable ints for Rx ready and Reg ready

	return;
}

void I2C_Send(uint16_t data)
{
	I2C_setRptMode(i2cHandle);//set RM bit in MDR
	I2C_setMaster(i2cHandle);//set MST bit in MDR
	I2C_setTransmit(i2cHandle);//set TRX bit in MDR
	I2C_setFreeRun(i2cHandle);//set FREE bit in MDR
	I2C_setStart(i2cHandle);//set STT bit in MDR

	XRDY_bit = I2C_getXRDY(i2cHandle);//retval = i2c->I2CSTR & I2C_I2CSTR_XRDY_BITS
	while(XRDY_bit == 0){};//wait for TRX reg ready

	I2C_putData(i2cHandle, data);//Send Data
		for(delay=0;delay<count;delay++)
		{
			CPU_Cycles = delay*5+9;
		}
			CPU_Cycles = 0;

	XRDY_bit = I2C_getXRDY(i2cHandle);//retval = i2c->I2CSTR & I2C_I2CSTR_XRDY_BITS
	while(XRDY_bit == 0){};//wait for TRX reg ready

	I2C_setStop(i2cHandle);

		return;

//mike was here again
	I2C_setDataCount(i2cHandle, data);
	I2C_setTransmit(i2cHandle);//set TRX bit in MDR
	I2C_setMaster(i2cHandle);//set MST bit in MDR
	I2C_setFreeRun(i2cHandle);//set FREE bit in MDR
	I2C_setStop(i2cHandle);//set STP bit in MDR
	I2C_setStart(i2cHandle);//set STT bit in MDR
	I2C_enableFifo(i2cHandle);
	I2C_clearTxFifoInt(i2cHandle);
	I2C_putData(i2cHandle, 0x21);//Send Data
	I2C_clearTxFifoInt(i2cHandle);
	I2C_putData(i2cHandle, 0xA3);//Send Data
	I2C_clearTxFifoInt(i2cHandle);
	I2C_putData(i2cHandle, 0xEF);//Send Data
	I2C_clearTxFifoInt(i2cHandle);
	I2C_putData(i2cHandle, 0x81);//Send Data
	I2C_clearStopCondDetected(i2cHandle);
	I2C_setMaster(i2cHandle);//set MST bit in MDR
	I2C_ResetTxFifo(i2cHandle);
	I2C_clearTxFifoInt(i2cHandle);
	//I2C_setStart(i2cHandle);//set STT bit in MDR
	I2C_enableFifo(i2cHandle);
	I2C_clearTxFifoInt(i2cHandle);
	I2C_putData(i2cHandle, 0x00);//Send Data
	I2C_clearTxFifoInt(i2cHandle);
	I2C_putData(i2cHandle, 0x3F);//Send Data
	I2C_putData(i2cHandle, 0x01);//Send Data
	I2C_putData(i2cHandle, 0x00);//Send Data

	return;

}

void I2C_SendwStart(uint16_t byte_count)
{
	I2C_clearStopCondDetected(i2cHandle);
	I2C_setRptMode(i2cHandle);//set RM bit in MDR
	I2C_setMaster(i2cHandle);//set MST bit in MDR
	I2C_setTransmit(i2cHandle);//set TRX bit in MDR
	I2C_setFreeRun(i2cHandle);//set FREE bit in MDR
	I2C_setStart(i2cHandle);//set STT bit in MDR

	for(i=0;i<byte_count;i++)
	{
		XRDY_bit = I2C_getXRDY(i2cHandle);//retval = i2c->I2CSTR & I2C_I2CSTR_XRDY_BITS
		while(XRDY_bit == 0){};//wait for TRX reg ready

		I2C_putData(i2cHandle, SevenSegInit[i]);//Send Data
		for(delay=0;delay<count;delay++)
		{
			CPU_Cycles = delay*5+9;
		}
		CPU_Cycles = 0;
	}

	I2C_setStop(i2cHandle);

	return;
}

void I2C_SendtoAddr(uint16_t Addr, uint16_t data)
{
	I2C_setRptMode(i2cHandle);//set RM bit in MDR
	I2C_setMaster(i2cHandle);//set MST bit in MDR
	I2C_setTransmit(i2cHandle);//set TRX bit in MDR
	I2C_setFreeRun(i2cHandle);//set FREE bit in MDR
	I2C_setStart(i2cHandle);//set STT bit in MDR

	XRDY_bit = I2C_getXRDY(i2cHandle);//retval = i2c->I2CSTR & I2C_I2CSTR_XRDY_BITS
	while(XRDY_bit == 0){};//wait for TRX reg ready

	I2C_putData(i2cHandle, Addr);//Send Data
		for(delay=0;delay<count;delay++)
		{
			CPU_Cycles = delay*5+9;
		}
			CPU_Cycles = 0;

	XRDY_bit = I2C_getXRDY(i2cHandle);//retval = i2c->I2CSTR & I2C_I2CSTR_XRDY_BITS
	while(XRDY_bit == 0){};//wait for TRX reg ready

	I2C_putData(i2cHandle, data);//Send Data
		for(delay=0;delay<count;delay++)
		{
			CPU_Cycles = delay*5+9;
		}
			CPU_Cycles = 0;

	I2C_setStop(i2cHandle);

	return;
}

interrupt void i2c_isr(void)//ben
{
	Uint16	IntSource;

	IntSource = I2C_getIntSource(i2cHandle);//Get interrupt source for state machine

	switch(IntSource)
	{
		case I2C_IntSrc_None:// = 0
		break;

		case I2C_IntSrc_Arb_Lost:// = 1
		break;

		case I2C_IntSrc_NACK:// = 2
		break;

		case I2C_IntSrc_Reg_Rdy:// = 3
		break;

		case I2C_IntSrc_Rx_Rdy:// = 4
			I2C_setSlave(i2cHandle);
			if(RxByte < 1)
			{
				Rec_Data.RxData[RxByte] = I2C_getData(i2cHandle);
				RxByte++;
			}
			else if(RxByte == 1)
			{
				Rec_Data.RxData[RxByte] = I2C_getData(i2cHandle);
				RxByte = 0;
			}
	//Ben need to finish Rx handling
		break;

		case I2C_IntSrc_Tx_Rdy:// = 5

		break;

		case I2C_IntSrc_Stop:// = 6
			I2C_clearStopCondDetected(i2cHandle);
		break;

		case I2C_IntSrc_Slave_Addr:// = 7
		break;

		default:
		asm("ESTOP0");//Halt if state number is not valid

	}
}

//@} //defgroup
// end of file



