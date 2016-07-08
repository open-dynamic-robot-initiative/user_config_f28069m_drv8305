#ifndef _USER_MTR1_H_
#define _USER_MTR1_H_
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

//! \file   solutions/instaspin_foc/boards/boostxldrv8301_revB/f28x/f2806xF/src/user.h
//! \brief  Contains the public interface for user initialization data for the CTRL, HAL, and EST modules 
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "sw/modules/types/src/types.h"
#include "sw/modules/motor/src/32b/motor.h"
#include "sw/modules/est/src/32b/est.h"
#include "sw/modules/est/src/est_states.h"
#include "sw/modules/est/src/est_Flux_states.h"
#include "sw/modules/est/src/est_Ls_states.h"
#include "sw/modules/est/src/est_Rs_states.h"


// platforms
#include "sw/modules/fast/src/32b/userParams_eou.h"
#include "user_mtr_on_j1.h"

//!
//!
//! \defgroup USER USER
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines


//! \brief CURRENTS AND VOLTAGES
// **************************************************************************
//! \brief Defines the voltage scale factor for the system
//! \brief Compile time calculation for scale factor (ratio) used throughout the system
#define USER_VOLTAGE_SF               ((float_t)((USER_ADC_FULL_SCALE_VOLTAGE_V)/(USER_IQ_FULL_SCALE_VOLTAGE_V)))

//! \brief Defines the current scale factor for the system
//! \brief Compile time calculation for scale factor (ratio) used throughout the system
#define USER_CURRENT_SF               ((float_t)((USER_ADC_FULL_SCALE_CURRENT_A)/(USER_IQ_FULL_SCALE_CURRENT_A)))


//! \brief CLOCKS & TIMERS
// **************************************************************************
//! \brief Defines the system clock frequency, MHz
#define USER_SYSTEM_FREQ_MHz             (90.0)

//! \brief Defines the address of controller handle 0 and 1
//!
#define USER_CTRL_HANDLE_ADDRESS    (0x13C40)
#define USER_CTRL_HANDLE_ADDRESS_1  (0x13D36)

//! \brief Defines the address of estimator handle 0 and 1
//!
#define USER_EST_HANDLE_ADDRESS     (0x13840)
#define USER_EST_HANDLE_ADDRESS_1   (0x13A3E)

//! \brief Defines the direct voltage (Vd) scale factor
//!
#define USER_VD_SF                 (0.95)

//! \brief Defines the Pulse Width Modulation (PWM) period, usec
//! \brief Compile time calculation
#define USER_PWM_PERIOD_usec       (1000.0/USER_PWM_FREQ_kHz)

//! \brief Defines the Interrupt Service Routine (ISR) frequency, Hz
//!
#define USER_ISR_FREQ_Hz           ((float_t)USER_PWM_FREQ_kHz * 1000.0 / (float_t)USER_NUM_PWM_TICKS_PER_ISR_TICK)

//! \brief Defines the Interrupt Service Routine (ISR) period, usec
//!
#define USER_ISR_PERIOD_usec       (USER_PWM_PERIOD_usec * (float_t)USER_NUM_PWM_TICKS_PER_ISR_TICK)


//! \brief DECIMATION
// **************************************************************************
//! \brief Defines the controller frequency, Hz
//! \brief Compile time calculation
#define USER_CTRL_FREQ_Hz          (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)

//! \brief Defines the estimator frequency, Hz
//! \brief Compile time calculation
#define USER_EST_FREQ_Hz           (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_EST_TICK)

//! \brief Defines the trajectory frequency, Hz
//! \brief Compile time calculation
#define USER_TRAJ_FREQ_Hz          (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)

//! \brief Defines the controller execution period, usec
//! \brief Compile time calculation
#define USER_CTRL_PERIOD_usec      (USER_ISR_PERIOD_usec * USER_NUM_ISR_TICKS_PER_CTRL_TICK)

//! \brief Defines the controller execution period, sec
//! \brief Compile time calculation
#define USER_CTRL_PERIOD_sec       ((float_t)USER_CTRL_PERIOD_usec/(float_t)1000000.0)


//! \brief LIMITS
// **************************************************************************
//! \brief Defines the maximum current slope for Id trajectory during PowerWarp
//! \brief For Induction motors only, controls how fast Id input can change under PowerWarp control
#define USER_MAX_CURRENT_SLOPE_POWERWARP   (0.3*USER_MOTOR_RES_EST_CURRENT/USER_IQ_FULL_SCALE_CURRENT_A/USER_TRAJ_FREQ_Hz)  // 0.3*RES_EST_CURRENT / IQ_FULL_SCALE_CURRENT / TRAJ_FREQ Typical to produce 1-sec rampup/down

//! \brief Defines the starting maximum acceleration AND deceleration for the speed profiles, Hz/s
//! \brief Updated in run-time through user functions
//! \brief Inverter, motor, inertia, and load will limit actual acceleration capability
#define USER_MAX_ACCEL_Hzps                 (17.0)      // Given by excel spreadsheet. Default: 20.0

//! \brief Defines maximum acceleration for the estimation speed profiles, Hz/s
//! \brief Only used during Motor ID (commission)
#define USER_MAX_ACCEL_EST_Hzps           (5.0)         // 5.0 Default, don't change

//! \brief Defines the maximum current slope for Id trajectory during estimation
#define USER_MAX_CURRENT_SLOPE           (USER_MOTOR_RES_EST_CURRENT/USER_IQ_FULL_SCALE_CURRENT_A/USER_TRAJ_FREQ_Hz)      // USER_MOTOR_RES_EST_CURRENT/USER_IQ_FULL_SCALE_CURRENT_A/USER_TRAJ_FREQ_Hz Default, don't change

//! \brief Defines the fraction of IdRated to use during rated flux estimation
//!
#define USER_IDRATED_FRACTION_FOR_RATED_FLUX (1.0)      // 1.0 Default, don't change

//! \brief Defines the fraction of IdRated to use during inductance estimation
//!
#define USER_IDRATED_FRACTION_FOR_L_IDENT    (1.0)      // 1.0 Default, don't change

//! \brief Defines the IdRated delta to use during estimation
//!
#define USER_IDRATED_DELTA                  (0.00002)

//! \brief Defines the fraction of SpeedMax to use during inductance estimation
//!
#define USER_SPEEDMAX_FRACTION_FOR_L_IDENT  (1.0)      // 1.0 Default, don't change

//! \brief Defines flux fraction to use during inductance identification
//!
#define USER_FLUX_FRACTION           (1.0)            // 1.0 Default, don't change

//! \brief Defines the PowerWarp gain for computing Id reference
//! \brief Induction motors only
#define USER_POWERWARP_GAIN                   (1.0)         // 1.0 Default, don't change


//! \brief POLES
// **************************************************************************
//! \brief Defines the analog voltage filter pole location, rad/s
//! \brief Compile time calculation from Hz to rad/s
#define USER_VOLTAGE_FILTER_POLE_rps  (2.0 * MATH_PI * USER_VOLTAGE_FILTER_POLE_Hz)

//! \brief Defines the software pole location for the voltage and current offset estimation, rad/s
//! \brief Should not be changed from default of (20.0)
#define USER_OFFSET_POLE_rps            (20.0)   // 20.0 Default, do not change

//! \brief Defines the software pole location for the flux estimation, rad/s
//! \brief Should not be changed from default of (100.0)
#define USER_FLUX_POLE_rps              (100.0)   // 100.0 Default, do not change

//! \brief Defines the software pole location for the direction filter, rad/s
#define USER_DIRECTION_POLE_rps             (6.0)   // 6.0 Default, do not change

//! \brief Defines the software pole location for the speed control filter, rad/s
#define USER_SPEED_POLE_rps           (100.0)   // 100.0 Default, do not change

//! \brief Defines the software pole location for the DC bus filter, rad/s
#define USER_DCBUS_POLE_rps           (100.0)   // 100.0 Default, do not change

//! \brief Defines the convergence factor for the estimator
//! \brief Do not change from default for FAST
#define   USER_EST_KAPPAQ               (1.5)   // 1.5 Default, do not change

// **************************************************************************
// end the defines


//! \brief USER MOTOR & ID SETTINGS
// **************************************************************************
// this section defined in user_j1.h or user_j5.h

#ifndef USER_MOTOR
#error Motor is not defined in user.h
#endif

#ifndef USER_MOTOR_TYPE
#error The motor type is not defined in user.h
#endif

#ifndef USER_MOTOR_NUM_POLE_PAIRS
#error Number of motor pole pairs is not defined in user.h
#endif

#ifndef USER_MOTOR_Rr
#error The rotor resistance is not defined in user.h
#endif

#ifndef USER_MOTOR_Rs
#error The stator resistance is not defined in user.h
#endif

#ifndef USER_MOTOR_Ls_d
#error The direct stator inductance is not defined in user.h
#endif

#ifndef USER_MOTOR_Ls_q
#error The quadrature stator inductance is not defined in user.h
#endif

#ifndef USER_MOTOR_RATED_FLUX
#error The rated flux of motor is not defined in user.h
#endif

#ifndef USER_MOTOR_MAGNETIZING_CURRENT
#error The magnetizing current is not defined in user.h
#endif

#ifndef USER_MOTOR_RES_EST_CURRENT
#error The resistance estimation current is not defined in user.h
#endif

#ifndef USER_MOTOR_IND_EST_CURRENT
#error The inductance estimation current is not defined in user.h
#endif

#ifndef USER_MOTOR_MAX_CURRENT
#error The maximum current is not defined in user.h
#endif

#ifndef USER_MOTOR_FLUX_EST_FREQ_Hz
#error The flux estimation frequency is not defined in user.h
#endif


// **************************************************************************
// the functions


//! \brief      Sets the user parameter values
//! \param[in]  pUserParams  The pointer to the user param structure
extern void USER_setParamsMtr1(USER_Params *pUserParams);


//! \brief      Computes the scale factor needed to convert from torque created by Ld, Lq, Id and Iq, from per unit to Nm
//! \return     The scale factor to convert torque from (Ld - Lq) * Id * Iq from per unit to Nm, in IQ24 format
extern _iq USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(USER_Params *pUserParams);


//! \brief      Computes the scale factor needed to convert from torque created by flux and Iq, from per unit to Nm
//! \return     The scale factor to convert torque from Flux * Iq from per unit to Nm, in IQ24 format
extern _iq USER_computeTorque_Flux_Iq_pu_to_Nm_sf(USER_Params *pUserParams);


//! \brief      Computes the scale factor needed to convert from per unit to Wb
//! \param[in]  pUserParams  The pointer to the user param structure
//! \return     The scale factor to convert from flux per unit to flux in Wb, in IQ24 format
extern _iq USER_computeFlux_pu_to_Wb_sf(USER_Params *pUserParams);


//! \brief      Computes the scale factor needed to convert from per unit to V/Hz
//! \return     The scale factor to convert from flux per unit to flux in V/Hz, in IQ24 format
extern _iq USER_computeFlux_pu_to_VpHz_sf(USER_Params *pUserParams);


////! \brief      Computes Flux in Wb or V/Hz depending on the scale factor sent as parameter
////! \param[in]  handle       The controller (CTRL) handle
////! \param[in]  sf           The scale factor to convert flux from per unit to Wb or V/Hz
////! \return     The flux in Wb or V/Hz depending on the scale factor sent as parameter, in IQ24 format
//extern _iq USER_computeFlux(CTRL_Handle handle, const _iq sf);
//
//
////! \brief      Computes Torque in Nm
////! \param[in]  handle          The controller (CTRL) handle
////! \param[in]  torque_Flux_sf  The scale factor to convert torque from (Ld - Lq) * Id * Iq from per unit to Nm
////! \param[in]  torque_Ls_sf    The scale factor to convert torque from Flux * Iq from per unit to Nm
////! \return     The torque in Nm, in IQ24 format
//extern _iq USER_computeTorque_Nm(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf);
//
//
////! \brief      Computes Torque in lbin
////! \param[in]  handle          The controller (CTRL) handle
////! \param[in]  torque_Flux_sf  The scale factor to convert torque from (Ld - Lq) * Id * Iq from per unit to lbin
////! \param[in]  torque_Ls_sf    The scale factor to convert torque from Flux * Iq from per unit to lbin
////! \return     The torque in lbin, in IQ24 format
//extern _iq USER_computeTorque_lbin(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _USER_MTR1_H_ definition

