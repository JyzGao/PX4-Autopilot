/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file ZeroInputPlant.hpp
 * Definition of atrcraft's nonlinear dynamics with zero inputs.
 *
 * @author Wenxuan Gao <jyz_john@sjtu.edu.cn>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Wenxuan Gao, 2024.
 */


#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/zero_input_response.h>


using matrix::Eulerf;
using matrix::Quatf;
using uORB::SubscriptionData;
using namespace time_literals;

// static constexpr float CONSTANTS_ONE_G = 9.80665f;	// m/s^2
static constexpr hrt_abstime T_WIND_EST_TIMEOUT =
	10_s; // time after which the wind estimate is disabled if no longer updating

class ZeroInputPlant final : public ModuleBase<ZeroInputPlant>, public ModuleParams,
	public px4::WorkItem
{
public:
	ZeroInputPlant(bool vtol = false);
	~ZeroInputPlant() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	void publishZeroInputResponse(const hrt_abstime &timestamp_sample);

	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};	/**< vehicle attitude */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};		/**< vehicle local position subscription */
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};	/**< vehicle local position subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vtol_vehicle_status_pub{ORB_ID(vtol_vehicle_status)};	/**< vehicle status subscription */
	uORB::Subscription _wind_sub{ORB_ID(wind)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};

	uORB::SubscriptionData<airspeed_validated_s> _airspeed_validated_sub{ORB_ID(airspeed_validated)};

	uORB::Publication<zero_input_response_s>     _zero_input_response_pub{ORB_ID(zero_input_response)};

	vehicle_air_data_s 			_vehicle_air_data{};
	vehicle_attitude_s 			_att{};			/**< vehicle attitude */
	vehicle_angular_velocity_s 		_angular_velocity{};	/**< vehicle angular velocity */
	vehicle_local_position_s		_local_pos {};		/**< local position */
	vehicle_status_s			_vehicle_status {};	/**< vehicle status */
	vtol_vehicle_status_s 			_vtol_vehicle_status{}; /**< vtol vehicle status */
	zero_input_response_s 			_zero_input_response{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	hrt_abstime _last_run{0};

	/* wind estimates */
	matrix::Vector2f _wind_vel{0.0f, 0.0f}; 		///< wind velocity vector [m/s]
	bool _wind_valid{false}; 			///< flag if a valid wind estimate exists
	hrt_abstime _time_wind_last_received{0}; 	///< last time wind estimate was received in microseconds. Used to detect timeouts.

	float _airspeed_scaling{1.0f};
	bool _landed{true};
	float _battery_scale{1.0f};
	bool _is_tailsitter{false};

	float _vehicle_mass;
	float _vehicle_s;
	float _vehicle_b;
	float _vehicle_c;
	float _vehicle_ar;
	float _vehicle_ixx;
	float _vehicle_iyy;
	float _vehicle_izz;
	float _vehicle_ixz;

	float _vehicle_alpha0;
	float _vehicle_m0;

	float _vehicle_cl0;
	float _vehicle_cd0;
	float _vehicle_cm0;
	float _vehicle_clalpha;
	float _vehicle_cmalpha;
	float _vehicle_clq;
	float _vehicle_cdq;
	float _vehicle_cmq;
	float _vehicle_clele;
	float _vehicle_cdele;
	float _vehicle_cmele;

	float _vehicle_cy0;
	float _vehicle_cr0;
	float _vehicle_cn0;
	float _vehicle_cybeta;
	float _vehicle_crbeta;
	float _vehicle_cnbeta;
	float _vehicle_cyp;
	float _vehicle_crp;
	float _vehicle_cnp;
	float _vehicle_cyr;
	float _vehicle_crr;
	float _vehicle_cnr;
	float _vehicle_cyail;
	float _vehicle_crail;
	float _vehicle_cnail;
	float _vehicle_cyrud;
	float _vehicle_crrud;
	float _vehicle_cnrud;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_STALL>) _param_fw_airspd_stall,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamInt<px4::params::FW_ARSP_MODE>) _param_fw_arsp_mode,
		(ParamInt<px4::params::FW_ARSP_SCALE_EN>) _param_fw_arsp_scale_en,

		(ParamFloat<px4::params::VEHICLE_MASS>) _param_vehicle_mass,
		(ParamFloat<px4::params::VEHICLE_S>) _param_vehicle_s,
		(ParamFloat<px4::params::VEHICLE_B>) _param_vehicle_b,
		(ParamFloat<px4::params::VEHICLE_C>) _param_vehicle_c,
		(ParamFloat<px4::params::VEHICLE_AR>) _param_vehicle_ar,
		(ParamFloat<px4::params::VEHICLE_I_XX>) _param_vehicle_ixx,
		(ParamFloat<px4::params::VEHICLE_I_YY>) _param_vehicle_iyy,
		(ParamFloat<px4::params::VEHICLE_I_ZZ>) _param_vehicle_izz,
		(ParamFloat<px4::params::VEHICLE_I_XZ>) _param_vehicle_ixz,

		(ParamFloat<px4::params::VEHICLE_ALPHA0>) _param_vehicle_alpha0,
		(ParamFloat<px4::params::VEHICLE_M0>) _param_vehicle_m0,

		(ParamFloat<px4::params::VEHICLE_CL0>) _param_vehicle_cl0,
		(ParamFloat<px4::params::VEHICLE_CD0>) _param_vehicle_cd0,
		(ParamFloat<px4::params::VEHICLE_CM0>) _param_vehicle_cm0,
		(ParamFloat<px4::params::VEHICLE_CLALPHA>) _param_vehicle_clalpha,
		(ParamFloat<px4::params::VEHICLE_CMALPHA>) _param_vehicle_cmalpha,
		(ParamFloat<px4::params::VEHICLE_CLQ>) _param_vehicle_clq,
		(ParamFloat<px4::params::VEHICLE_CDQ>) _param_vehicle_cdq,
		(ParamFloat<px4::params::VEHICLE_CMQ>) _param_vehicle_cmq,
		(ParamFloat<px4::params::VEHICLE_CLELE>) _param_vehicle_clele,
		(ParamFloat<px4::params::VEHICLE_CDELE>) _param_vehicle_cdele,
		(ParamFloat<px4::params::VEHICLE_CMELE>) _param_vehicle_cmele,

		(ParamFloat<px4::params::VEHICLE_CY0>) _param_vehicle_cy0,
		(ParamFloat<px4::params::VEHICLE_CR0>) _param_vehicle_cr0,
		(ParamFloat<px4::params::VEHICLE_CN0>) _param_vehicle_cn0,
		(ParamFloat<px4::params::VEHICLE_CYBETA>) _param_vehicle_cybeta,
		(ParamFloat<px4::params::VEHICLE_CRBETA>) _param_vehicle_crbeta,
		(ParamFloat<px4::params::VEHICLE_CNBETA>) _param_vehicle_cnbeta,
		(ParamFloat<px4::params::VEHICLE_CYP>) _param_vehicle_cyp,
		(ParamFloat<px4::params::VEHICLE_CRP>) _param_vehicle_crp,
		(ParamFloat<px4::params::VEHICLE_CNP>) _param_vehicle_cnp,
		(ParamFloat<px4::params::VEHICLE_CYR>) _param_vehicle_cyr,
		(ParamFloat<px4::params::VEHICLE_CRR>) _param_vehicle_crr,
		(ParamFloat<px4::params::VEHICLE_CNR>) _param_vehicle_cnr,
		(ParamFloat<px4::params::VEHICLE_CYAIL>) _param_vehicle_cyail,
		(ParamFloat<px4::params::VEHICLE_CRAIL>) _param_vehicle_crail,
		(ParamFloat<px4::params::VEHICLE_CNAIL>) _param_vehicle_cnail,
		(ParamFloat<px4::params::VEHICLE_CYRUD>) _param_vehicle_cyrud,
		(ParamFloat<px4::params::VEHICLE_CRRUD>) _param_vehicle_crrud,
		(ParamFloat<px4::params::VEHICLE_CNRUD>) _param_vehicle_cnrud
	)

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();
	void        	wind_poll();
	void		vehicle_land_detected_poll();
	float 		get_airspeed_and_update_scaling();
};
