/****************************************************************************
 *
 *   Copyright (c) 2013-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ecl_zero_input_plant.h
 * Definition of atrcraft's nonlinear dynamics with zero inputs.
 *
 * @author Wenxuan Gao <jyz_john@sjtu.edu.cn>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Wenxuan Gao, 2024.
 */

#ifndef ECL_ZERO_INPUT_PLANT_H
#define ECL_ZERO_INPUT_PLANT_H

#include <mathlib/mathlib.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>


struct ECL_ControlData {
	float roll;
	float pitch;
	float yaw;
	float body_x_rate;
	float body_y_rate;
	float body_z_rate;
	float roll_setpoint;
	float pitch_setpoint;
	float yaw_setpoint;
	float roll_rate_setpoint;
	float pitch_rate_setpoint;
	float yaw_rate_setpoint;
	float airspeed_min;
	float airspeed_max;
	float airspeed;
	float scaler;
	float groundspeed;
	float groundspeed_scaler;
	bool lock_integrator;
};


class ECL_ZeroInputPlant
{
public:
	ECL_ZeroInputPlant() = default;
	~ECL_ZeroInputPlant() = default;


	/* Additional Setters */
	void set_max_rate_pos(float max_rate_pos)
	{
		_max_rate = max_rate_pos;
	}

	void set_max_rate_neg(float max_rate_neg)
	{
		_max_rate_neg = max_rate_neg;
	}

	void set_bodyrate_setpoint(float rate)
	{
		_bodyrate_setpoint = math::constrain(rate, -_max_rate_neg, _max_rate);
	}

protected:
	float _max_rate_neg{0.0f};

private:

	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};	/**< vehicle attitude */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};		/**< local position subscription */
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */



	vehicle_attitude_s 			_att {};		/**< vehicle attitude */
	vehicle_local_position_s		_local_pos {};		/**< local position */
	vehicle_rates_setpoint_s		_rates_sp {};		/* attitude rates setpoint */
	vehicle_status_s			_vehicle_status {};	/**< vehicle status */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_ACRO_X_MAX>) _param_fw_acro_x_max,
		(ParamFloat<px4::params::FW_ACRO_Y_MAX>) _param_fw_acro_y_max,
		(ParamFloat<px4::params::FW_ACRO_Z_MAX>) _param_fw_acro_z_max,

		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_STALL>) _param_fw_airspd_stall,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamInt<px4::params::FW_ARSP_MODE>) _param_fw_arsp_mode,

		(ParamInt<px4::params::FW_ARSP_SCALE_EN>) _param_fw_arsp_scale_en,

		(ParamBool<px4::params::FW_BAT_SCALE_EN>) _param_fw_bat_scale_en,

		(ParamFloat<px4::params::FW_DTRIM_P_FLPS>) _param_fw_dtrim_p_flps,
		(ParamFloat<px4::params::FW_DTRIM_P_VMAX>) _param_fw_dtrim_p_vmax,
		(ParamFloat<px4::params::FW_DTRIM_P_VMIN>) _param_fw_dtrim_p_vmin,
		(ParamFloat<px4::params::FW_DTRIM_R_FLPS>) _param_fw_dtrim_r_flps,
		(ParamFloat<px4::params::FW_DTRIM_R_VMAX>) _param_fw_dtrim_r_vmax,
		(ParamFloat<px4::params::FW_DTRIM_R_VMIN>) _param_fw_dtrim_r_vmin,
		(ParamFloat<px4::params::FW_DTRIM_Y_VMAX>) _param_fw_dtrim_y_vmax,
		(ParamFloat<px4::params::FW_DTRIM_Y_VMIN>) _param_fw_dtrim_y_vmin,

		(ParamFloat<px4::params::FW_FLAPERON_SCL>) _param_fw_flaperon_scl,
		(ParamFloat<px4::params::FW_FLAPS_LND_SCL>) _param_fw_flaps_lnd_scl,
		(ParamFloat<px4::params::FW_FLAPS_SCL>) _param_fw_flaps_scl,
		(ParamFloat<px4::params::FW_FLAPS_TO_SCL>) _param_fw_flaps_to_scl,

		(ParamFloat<px4::params::FW_MAN_P_MAX>) _param_fw_man_p_max,
		(ParamFloat<px4::params::FW_MAN_P_SC>) _param_fw_man_p_sc,
		(ParamFloat<px4::params::FW_MAN_R_MAX>) _param_fw_man_r_max,
		(ParamFloat<px4::params::FW_MAN_R_SC>) _param_fw_man_r_sc,
		(ParamFloat<px4::params::FW_MAN_Y_SC>) _param_fw_man_y_sc,

		(ParamFloat<px4::params::FW_P_RMAX_NEG>) _param_fw_p_rmax_neg,
		(ParamFloat<px4::params::FW_P_RMAX_POS>) _param_fw_p_rmax_pos,
		(ParamFloat<px4::params::FW_P_TC>) _param_fw_p_tc,
		(ParamFloat<px4::params::FW_PR_FF>) _param_fw_pr_ff,
		(ParamFloat<px4::params::FW_PR_I>) _param_fw_pr_i,
		(ParamFloat<px4::params::FW_PR_IMAX>) _param_fw_pr_imax,
		(ParamFloat<px4::params::FW_PR_P>) _param_fw_pr_p,
		(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,

		(ParamFloat<px4::params::FW_R_RMAX>) _param_fw_r_rmax,
		(ParamFloat<px4::params::FW_R_TC>) _param_fw_r_tc,
		(ParamFloat<px4::params::FW_RLL_TO_YAW_FF>) _param_fw_rll_to_yaw_ff,
		(ParamFloat<px4::params::FW_RR_FF>) _param_fw_rr_ff,
		(ParamFloat<px4::params::FW_RR_I>) _param_fw_rr_i,
		(ParamFloat<px4::params::FW_RR_IMAX>) _param_fw_rr_imax,
		(ParamFloat<px4::params::FW_RR_P>) _param_fw_rr_p,

		(ParamBool<px4::params::FW_W_EN>) _param_fw_w_en,
		(ParamFloat<px4::params::FW_W_RMAX>) _param_fw_w_rmax,
		(ParamFloat<px4::params::FW_WR_FF>) _param_fw_wr_ff,
		(ParamFloat<px4::params::FW_WR_I>) _param_fw_wr_i,
		(ParamFloat<px4::params::FW_WR_IMAX>) _param_fw_wr_imax,
		(ParamFloat<px4::params::FW_WR_P>) _param_fw_wr_p,

		(ParamFloat<px4::params::FW_Y_RMAX>) _param_fw_y_rmax,
		(ParamFloat<px4::params::FW_YR_FF>) _param_fw_yr_ff,
		(ParamFloat<px4::params::FW_YR_I>) _param_fw_yr_i,
		(ParamFloat<px4::params::FW_YR_IMAX>) _param_fw_yr_imax,
		(ParamFloat<px4::params::FW_YR_P>) _param_fw_yr_p,

		(ParamFloat<px4::params::TRIM_PITCH>) _param_trim_pitch,
		(ParamFloat<px4::params::TRIM_ROLL>) _param_trim_roll,
		(ParamFloat<px4::params::TRIM_YAW>) _param_trim_yaw
	)

};

#endif // ECL_ZERO_INPUT_PLANT_H
