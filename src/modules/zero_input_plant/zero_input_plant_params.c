/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file zero_input_plant.c
 *
 * Parameters defined by the vehicle plant
 *
 * @author Wenxuan Gao <jyz_john@sjtu.edu.cn>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Airspeed mode
 *
 * For small wings or VTOL without airspeed sensor this parameter can be used to
 * enable flying without an airspeed reading
 *
 * @value 0 Normal (use airspeed if available)
 * @value 1 Airspeed disabled
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_ARSP_MODE, 0);

/**
 * Enable airspeed scaling
 *
 * This enables a logic that automatically adjusts the output of the rate controller to take
 * into account the real torque produced by an aerodynamic control surface given
 * the current deviation from the trim airspeed (FW_AIRSPD_TRIM).
 *
 * Enable when using aerodynamic control surfaces (e.g.: plane)
 * Disable when using rotor wings (e.g.: autogyro)
 *
 * @boolean
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_ARSP_SCALE_EN, 1);



/**
 * Mass of the vehicle.
 *
 * @unit kg
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_MASS, 20.0f);

/**
 * Wing surface of the vehicle.
 *
 * @unit m^2
 * @min 0.0
 * @max 10.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_S, 0.9f);

/**
 * Wing Span of the vehicle.
 *
 * @unit m
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_B, 3.0f);

/**
 * Mean Aerodynamic chord of the vehicle.
 *
 * @unit m
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_C, 0.3f);

/**
 * Aspect Ratio of the vehicle.
 *
 * @unit %
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_AR, 10.0f);

/**
 * Moment of inertia along X axis of the body frame.
 *
 * @unit kg*m^2
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_I_XX, 5.0f);

/**
 * Moment of inertia along Y axis of the body frame.
 *
 * @unit kg*m^2
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_I_YY, 5.0f);

/**
 * Moment of inertia along Z axis of the body frame.
 *
 * @unit kg*m^2
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_I_ZZ, 10.0f);

/**
 * Product of inertia w.r.t. X-Z plane of the body frame.
 *
 * @unit kg*m^2
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_I_XZ, 0.1f);

/**
 * Stall angle of attack.
 *
 * @unit rad
 * @min 0.0
 * @max 1.5708
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_ALPHA0, 0.2618f);

/**
 * Transition rate constant of the sigmoid function.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_M0, 30.0f);

/**
 * Zero AOA lift coefficient
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CL0, 0.1f);

/**
 * Zero AOA drag coefficient
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CD0, 0.1f);

/**
 * Zero AOA pitching moment coefficient
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CM0, 0.1f);

/**
 * Lift coefficient derivative w.r.t. AOA.
 *
 * @min -10.0
 * @max 10.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CLALPHA, 1.0f);

/**
 * Pitching moment coefficient derivative w.r.t. AOA.
 *
 * @min -10.0
 * @max 10.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CMALPHA, -1.0f);

/**
 * Lift coefficient derivative w.r.t. q.
 *
 * @min -50.0
 * @max 50.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CLQ, 10.0f);

/**
 * Drag coefficient derivative w.r.t. q.
 *
 * @min -50.0
 * @max 50.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CDQ, -0.1f);

/**
 * Pitching moment coefficient derivative w.r.t. q.
 *
 * @min -50.0
 * @max 50.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CMQ, -10.0f);

/**
 * Lift coefficient derivative w.r.t. elevator deflection angle.
 *
 * @min -10.0
 * @max 10.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CLELE, 0.1f);

/**
 * Drag coefficient derivative w.r.t. elevator deflection angle.
 *
 * @min -10.0
 * @max 10.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CDELE, 0.01f);

/**
 * Piching moment coefficient derivative w.r.t. elevator deflection angle.
 *
 * @min -10.0
 * @max 10.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CMELE, -1.0f);

/**
 * Zero AOA side force coefficient.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CY0, 0.0f);

/**
 * Zero AOA rolling moment coefficient.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CR0, 0.0f);

/**
 * Zero AOA yawing moment coefficient.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CN0, 0.0f);

/**
 * Side force coefficient derivative w.r.t. sideslip angle.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CYBETA, -0.1f);

/**
 * Rolling moment coefficient derivative w.r.t. sideslip angle.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CRBETA, -0.01f);

/**
 * Side force coefficient derivative w.r.t. sideslip angle.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CNBETA, 0.1f);

/**
 * Side force coefficient derivative w.r.t. p.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CYP, 0.0f);

/**
 * Rolling moment coefficient derivative w.r.t. p.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CRP, -0.1f);

/**
 * Yawing moment coefficient derivative w.r.t. p.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CNP, -0.1f);

/**
 * Side force coefficient derivative w.r.t. r.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CYR, 0.1f);

/**
 * Rolling moment coefficient derivative w.r.t. r.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CRR, 0.1f);

/**
 * Yawing moment coefficient derivative w.r.t. r.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CNR, -0.1f);

/**
 * Side force coefficient derivative w.r.t. aileron deflection angle.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CYAIL, -0.01f);

/**
 * Rolling moment coefficient derivative w.r.t. aileron deflection angle.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CRAIL, 0.1f);

/**
 * Yawing moment coefficient derivative w.r.t. aileron deflection angle.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CNAIL, 0.001f);

/**
 * Side force coefficient derivative w.r.t. rudder deflection angle.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CYRUD, 0.1f);

/**
 * Roling moment coefficient derivative w.r.t. rudder deflection angle.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CRRUD, -0.001f);

/**
 * Yawing moment coefficient derivative w.r.t. rudder deflection angle.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Zero Input Plant
 */
PARAM_DEFINE_FLOAT(VEHICLE_CNRUD, -0.1f);
