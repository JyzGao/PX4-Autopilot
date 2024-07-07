/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file AttitudeControl_EulerAngle.hpp
 *
 * A Euler angle based attitude controller.
 *
 * @author Wenxuan Gao	<jyz_john@sjtu.edu.cn>
 *
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

class AttitudeControlEulerAngle
{
public:
	AttitudeControlEulerAngle() = default;
	~AttitudeControlEulerAngle() = default;

	/**
	 * Set proportional attitude control gain
	 * @param proportional_gain 3D vector containing gains for roll, pitch, yaw
	 * @param yaw_weight A fraction [0,1] deprioritizing yaw compared to roll and pitch
	 */
	void setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight);

	/**
	 * Set hard limit for output rate setpoints
	 * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param ed desired vehicle attitude setpoint in Euler angles
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 */
	void setAttitudeSetpoint(const matrix::Eulerf &ed, const float yawspeed_setpoint)
	{
		_attitude_setpoint_euler = ed;
		// _attitude_setpoint_euler.normalize();
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param e_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Eulerf &e_delta)
	{
		_attitude_setpoint_euler = e_delta + _attitude_setpoint_euler;
		// _attitude_setpoint_euler.normalize();
	}

	/**
	 * Run one control loop cycle calculation
	 * @param e estimation of the current vehicle attitude in Euler angles
	 * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
	 */
	matrix::Vector3f update(const matrix::Eulerf &e) const;

private:
	matrix::Vector3f _proportional_gain;
	matrix::Vector3f _rate_limit;
	float _yaw_w{0.f}; ///< yaw weight [0,1] to deprioritize caompared to roll and pitch

	matrix::Eulerf _attitude_setpoint_euler; ///< latest known Euler angle setpoint e.g. from position control
	// matrix::Quatf _attitude_setpoint_q; ///< latest known attitude setpoint e.g. from position control
	float _yawspeed_setpoint{0.f}; ///< latest known yawspeed feed-forward setpoint
};
