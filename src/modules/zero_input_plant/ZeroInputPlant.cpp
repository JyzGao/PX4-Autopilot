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

#include "ZeroInputPlant.hpp"

// #include <vtol_att_control/vtol_type.h>

using namespace time_literals;
using math::constrain;
using math::gradual;
using math::radians;
using matrix::wrap_pi;
using matrix::wrap_2pi;


ZeroInputPlant::ZeroInputPlant(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// check if VTOL first
	// if (vtol) {
	// 	int32_t vt_type = -1;

	// 	if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
	// 		_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
	// 	}
	// }

	/* fetch initial parameter values */
	parameters_update();
}

ZeroInputPlant::~ZeroInputPlant()
{
	perf_free(_loop_perf);
}

bool
ZeroInputPlant::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
ZeroInputPlant::parameters_update()
{
	/* physical parameters */
	_vehicle_mass = _param_vehicle_mass.get();
	_vehicle_s = _param_vehicle_s.get();
	_vehicle_b = _param_vehicle_b.get();
	_vehicle_c = _param_vehicle_c.get();
	_vehicle_ar = _param_vehicle_ar.get();
	_vehicle_ixx = _param_vehicle_ixx.get();
	_vehicle_iyy = _param_vehicle_iyy.get();
	_vehicle_izz = _param_vehicle_izz.get();
	_vehicle_ixz = _param_vehicle_ixz.get();

	/* airfoil aerodynamic parameters */
	_vehicle_alpha0 = _param_vehicle_alpha0.get();
	_vehicle_m0 = _param_vehicle_m0.get();

	/* longitudinal aerodynamic derivatives */
	_vehicle_cl0 = _param_vehicle_cl0.get();
	_vehicle_cd0 = _param_vehicle_cd0.get();
	_vehicle_cm0 = _param_vehicle_cm0.get();
	_vehicle_clalpha = _param_vehicle_clalpha.get();
	_vehicle_cmalpha = _param_vehicle_cmalpha.get();
	_vehicle_clq = _param_vehicle_clq.get();
	_vehicle_cdq = _param_vehicle_cdq.get();
	_vehicle_cmq = _param_vehicle_cmq.get();
	_vehicle_clele = _param_vehicle_clele.get();
	_vehicle_cdele = _param_vehicle_cdele.get();
	_vehicle_cmele = _param_vehicle_cmele.get();

	/* lateral aerodynamic derivatives */
	_vehicle_cy0 = _param_vehicle_cy0.get();
	_vehicle_cr0 = _param_vehicle_cr0.get();
	_vehicle_cn0 = _param_vehicle_cn0.get();
	_vehicle_cybeta = _param_vehicle_cybeta.get();
	_vehicle_crbeta = _param_vehicle_crbeta.get();
	_vehicle_cnbeta = _param_vehicle_cnbeta.get();
	_vehicle_cyp = _param_vehicle_cyp.get();
	_vehicle_crp = _param_vehicle_crp.get();
	_vehicle_cnp = _param_vehicle_cnp.get();
	_vehicle_cyr = _param_vehicle_cyr.get();
	_vehicle_crr = _param_vehicle_crr.get();
	_vehicle_cnr = _param_vehicle_cnr.get();
	_vehicle_cyail = _param_vehicle_cyail.get();
	_vehicle_crail = _param_vehicle_crail.get();
	_vehicle_cnail = _param_vehicle_cnail.get();
	_vehicle_cyrud = _param_vehicle_cyrud.get();
	_vehicle_crrud = _param_vehicle_crrud.get();
	_vehicle_cnrud = _param_vehicle_cnrud.get();

	return PX4_OK;
}


void
ZeroInputPlant::wind_poll()
{
	if (_wind_sub.updated()) {
		wind_s wind;
		_wind_sub.update(&wind);

		// assumes wind is valid if finite
		_wind_valid = PX4_ISFINITE(wind.windspeed_north)
			      && PX4_ISFINITE(wind.windspeed_east);

		_time_wind_last_received = hrt_absolute_time();

		_wind_vel(0) = wind.windspeed_north;
		_wind_vel(1) = wind.windspeed_east;

	} else {
		// invalidate wind estimate usage (and correspondingly NPFG, if enabled) after subscription timeout
		_wind_valid = _wind_valid && (hrt_absolute_time() - _time_wind_last_received) < T_WIND_EST_TIMEOUT;
	}
}


void
ZeroInputPlant::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float ZeroInputPlant::get_airspeed_and_update_scaling()
{
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().calibrated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if ((_param_fw_arsp_mode.get() == 0) && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the stall airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_stall.get();
		}
	}

	/*
	 * For scaling our actuators using anything less than the stall
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and it's the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	const float airspeed_constrained = constrain(constrain(airspeed, _param_fw_airspd_stall.get(),
					   _param_fw_airspd_max.get()), 0.1f, 1000.0f);

	_airspeed_scaling = (_param_fw_arsp_scale_en.get()) ? (_param_fw_airspd_trim.get() / airspeed_constrained) : 1.0f;

	return airspeed;
}

void FixedwingAttitudeControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// run controller if attitude, velocity, or angular rate changed
	bool is_att_changed = _att_sub.update(&_att);
	bool is_omega_changed = _vehicle_rates_sub.copy(&_angular_velocity);
	bool is_v_changed = _local_pos_sub.update(&_local_pos);

	if (is_att_changed || is_omega_changed || is_v_changed) {

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		const float dt = math::constrain((_att.timestamp - _last_run) * 1e-6f, 0.002f, 0.04f);
		_last_run = _att.timestamp;

		/* get current rotation matrix and euler angles from control state quaternions */
		matrix::Dcmf R = matrix::Quatf(_att.q);			// Dcmbe
		const matrix::Eulerf euler_angles(R);			// phi, theta, psi
		float rollspeed = _angular_velocity.xyz[0];		// p
		float pitchspeed = _angular_velocity.xyz[1];		// q
		float yawspeed = _angular_velocity.xyz[2];		// r
		const float airspeed = get_airspeed_and_update_scaling();		// Calibrated airspeed
		// const matrix::Vecor3f ve{_local_pos.vx,_local_pos.vy,_local_pos.vz};	// vehicle ground speed in the Earth frame

		_vehicle_air_data_sub.update(&_vehicle_air_data);
		wind_poll();
		const matrix::Vecor3f Va_e{_local_pos.vx-_wind_vel(0),_local_pos.vy-_wind_vel(1),_local_pos.vz};	// vehicle airspeed in the Earth frame
		const matrix::Vecor3f uvw = R*Va_e;			// vehicle airspeed in body frame
		const float u = uvw(0);					// u
		const float v = uvw(1);					// v
		const float w = uvw(2);					// w

		float alpha = wrap_pi(atan2f(w, u));			// AOA
		float beta = wrap_pi(atan2f(v, sqrtf(u*u+v*v+w*w)));	// side slip angle

		if (_is_tailsitter) {
			/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
			 *
			 * Since the VTOL airframe is initialized as a multicopter we need to
			 * modify the estimated attitude for the fixed wing operation.
			 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
			 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
			 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
			 * Additionally, in order to get the correct sign of the pitch, we need to multiply
			 * the new x axis of the rotation matrix with -1
			 *
			 * original:			modified:
			 *
			 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
			 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
			 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
			 * */
			matrix::Dcmf R_adapted = R;		//modified rotation matrix

			/* move z to x */
			R_adapted(0, 0) = R(0, 2);
			R_adapted(1, 0) = R(1, 2);
			R_adapted(2, 0) = R(2, 2);

			/* move x to z */
			R_adapted(0, 2) = R(0, 0);
			R_adapted(1, 2) = R(1, 0);
			R_adapted(2, 2) = R(2, 0);

			/* change direction of pitch (convert to right handed system) */
			R_adapted(0, 0) = -R_adapted(0, 0);
			R_adapted(1, 0) = -R_adapted(1, 0);
			R_adapted(2, 0) = -R_adapted(2, 0);

			/* fill in new attitude data */
			R = R_adapted;

			/* lastly, roll- and yawspeed have to be swaped */
			float helper = rollspeed;
			rollspeed = -yawspeed;
			yawspeed = helper;
		}

		// vehicle status must be updated before the vehicle_control_mode_poll(), otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);
		_vtol_vehicle_status_pub.update(&_vtol_vehicle_status);

		vehicle_land_detected_poll();

		// lock integrator if no rate control enabled, or in RW mode (but not transitioning VTOL or tailsitter), or for long intervals (> 20 ms)
		// bool lock_integrator = !_vcontrol_mode.flag_control_rates_enabled
		// 		       || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
		// 			   !_vehicle_status.in_transition_mode && !_is_tailsitter)
		// 		       || (dt > 0.02f);

		/* if we are in rotary wing mode, do nothing */
		// if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
		// 	perf_end(_loop_perf);
		// 	return;
		// }

		// calculate Ca
		matrix::Vector3f Ca{0.0f, 0.0f, 0.0f};
		float cl_sa = _vehicle_cl0 + _vehicle_clalpha*alpha;
		Ca(2) = cl_sa + _vehicle_c*_vehicle_clq*pitchspeed/2/airspeed;
		float cd_sa = _vehicle_cd0 + cl_sa*cl_sa/float(M_PI)/0.9/_vehicle_ar;
		Ca(0) = cd_sa + _vehicle_c*_vehicle_cdq*pitchspeed/2/airspeed;
		float cypr = (_vehicle_cyp*rollspeed+_vehicle_cyr*yawspeed)*_vehicle_b/2/airspeed;
		Ca(1) = _vehicle_cy0 + _vehicle_cybeta*beta + cypr;

		// calculate Cfp
		matrix::Vector3f Cfp{0.0f, 0.0f, 0.0f};
		Cfp(0) = 1-cosf(2*alpha);
		Cfp(1) = -2*sinf(beta);
		Cfp(2) = sinf(2*alpha);

		// calculate Cwind
		matrix::Vector3f Cwind{0.0f, 0.0f, 0.0f};
		float eta1 = expf(-_vehicle_m0*(alpha-_vehicle_alpha0));
		float eta2 = expf(_vehicle_m0*(alpha+_vehicle_alpha0));
		float eta = (1 + eta1 + eta2)/(1 + eta1)/(1 + eta2);
		Cwind = (1-eta)*Ca + eta*Cfp;
		Cwind(0) = -Cwind(0);
		Cwind(2) = -Cwind(2);

		// calculate Cbody
		matrix::Vector3f Cbody{0.0f, 0.0f, 0.0f};
		Cbody(0) = cosf(alpha)*cosf(beta)*Cwind(0)-cosf(alpha)*sinf(beta)*Cwind(1)-sinf(alpha)*Cwind(2);
		Cbody(1) = sinf(beta)*Cwind(0)+cosf(beta)*Cwind(1);
		Cbody(2) = sinf(alpha)*cosf(beta)*Cwind(0)-sinf(alpha)*sinf(beta)*Cwind(1)+cosf(alpha)*Cwind(2);

		// Calculate aerodynamic force
		float dp = 0.5*_vehicle_air_data.rho*airspeed*airspeed;
		matrix::Vector3f Force = _vehicle_s*dp*Cbody;

		// Calculate gravity
		matrix::Vector3f Gb = R*matrix::Vector3f{0.0f, 0.0f, _vehicle_mass*CONSTANTS_ONE_G};
		Force += Gb;

		// calculate Cma
		matrix::Vector3f Cma{0.0f, 0.0f, 0.0f};
		float crpr = (_vehicle_crp*rollspeed+_vehicle_crr*yawspeed)*_vehicle_b/2/airspeed;
		Cma(0) = _vechile_b*(_vehicle_cr0 + crpr + _vehicle_crbeta*beta);
		Cma(1) = _vechile_c*(_vehicle_cm0 + _vehicle_c*_vehicle_cmq*pitchspeed/2/airspeed + _vehicle_cmalpha*alpha);
		float cnpr = (_vehicle_cnp*rollspeed+_vehicle_cnr*yawspeed)*_vehicle_b/2/airspeed;
		Cma(2) = _vechile_b*(_vehicle_cn0 + cnpr + _vehicle_cnbeta*beta);

		// Calculate aerodynamic moment
		matrix::Vector3f Moment = (1-eta)*_vehicle_s*dp*Cma;

		// Calculate state derivatives
		// Calculate accelerations in body frame
		matrix::Vector3f aab{0.0f, 0.0f, 0.0f};
		aab(0) = Force(0)/_vehicle_mass+yawspeed*v-pitchspeed*w;
		aab(1) = Force(1)/_vehicle_mass+rollspeed*w-yawspeed*u;
		aab(2) = Force(2)/_vehicle_mass+pitchspeed*u-rollspeed*v;

		// Calculate angular acelerations in body frame
		matrix::Vector3f domega{0.0f, 0.0f, 0.0f};
		float den = _vehicle_ixx*_vehicle_izz - _vehicle_ixz*_vehicle_ixz;
		domega(0) = (Moment(0)*_vehicle_izz+_vehicle_ixz*Moment(2)+_vehicle_ixz*rollspeed*pitchspeed*(_vehicle_ixx-_vehicle_iyy+_vehicle_izz)+pitchspeed*yawspeed*(_vehicle_izz*(_vehicle_iyy-_vehicle_izz)-_vehicle_ixz*_vehicle_ixz))/den;
		domega(1) = (Moment(1)-rollspeed*yawspeed(_vehicle_ixx-_veicle_izz)-(rollspeed*rollspeed-yawspeed*yawspeed)*_vehicle_ixz)/_vehicle_iyy;
		domega(2) = (Moment(0)*_vehicle_ixz+_vehicle_ixx*Moment(2)+_vehicle_ixz*yawspeed*pitchspeed*(_vehicle_iyy-_vehicle_izz-_vehicle_ixx)+pitchspeed*rollspeed*(_vehicle_ixx*(_vehicle_ixx-_vehicle_iyy)+_vehicle_ixz*_vehicle_ixz))/den;

		/* Prepare data for attitude controllers */
		// ECL_ControlData control_input{};
		// control_input.roll = euler_angles.phi();
		// control_input.pitch = euler_angles.theta();
		// control_input.yaw = euler_angles.psi();
		// control_input.body_x_rate = rollspeed;
		// control_input.body_y_rate = pitchspeed;
		// control_input.body_z_rate = yawspeed;
		// control_input.roll_setpoint = _att_sp.roll_body;
		// control_input.pitch_setpoint = _att_sp.pitch_body;
		// control_input.yaw_setpoint = _att_sp.yaw_body;
		// control_input.airspeed_min = _param_fw_airspd_stall.get();
		// control_input.airspeed_max = _param_fw_airspd_max.get();
		// control_input.airspeed = airspeed;
		// control_input.scaler = _airspeed_scaling;
		// control_input.lock_integrator = lock_integrator;
		// float groundspeed = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);
		// float gspd_scaling_trim = (_param_fw_airspd_stall.get());
		// control_input.groundspeed = groundspeed;

		/* Only publish if any of the proper modes are enabled */
		// if (_vcontrol_mode.flag_control_rates_enabled ||
		//     _vcontrol_mode.flag_control_attitude_enabled ||
		//     _vcontrol_mode.flag_control_manual_enabled)
		// {
		// 	publishZeroInputResponse(angular_velocity.timestamp_sample);
		// }

		if(!_vehicle_status.is_vtol)
		{
			if(_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
			{
				_zero_input_response.dx[0] = _vehicle_mass*aab(0);
				_zero_input_response.dx[1] = _vehicle_mass*aab(1);
				_zero_input_response.dx[2] = _vehicle_mass*aab(2);
				_zero_input_response.dx[3] = _vehicle_ixx*domega(0)-_vehicle_ixz*domega(2);
				_zero_input_response.dx[4] = _vehicle_iyy*domega(1);
				_zero_input_response.dx[5] = _vehicle_izz*domega(2)-_vehicle_ixz*domega(0);
			}
			else
			{
				_zero_input_response.dx[0] = _vehicle_mass*(u*aab(0)+v*aab(1)+w*aab(2))/airspeed;
				_zero_input_response.dx[1] = _vehicle_mass*aab(1);
				_zero_input_response.dx[2] = _vehicle_mass*aab(2);
				_zero_input_response.dx[3] = _vehicle_ixx*domega(0)-_vehicle_ixz*domega(2);
				_zero_input_response.dx[4] = _vehicle_iyy*domega(1);
				_zero_input_response.dx[5] = _vehicle_izz*domega(2)-_vehicle_ixz*domega(0);
			}
		}
		else
		{
			if(!_vtol_vehicle_status.vtol_in_rw_mode ||
			(_vtol_vehicle_status.vtol_in_trans_mode && _vtol_vehicle_status.in_transition_to_fw))
			{
				_zero_input_response.dx[0] = _vehicle_mass*(u*aab(0)+v*aab(1)+w*aab(2))/airspeed;
				_zero_input_response.dx[1] = _vehicle_mass*aab(1);
				_zero_input_response.dx[2] = _vehicle_mass*aab(2);
				_zero_input_response.dx[3] = _vehicle_ixx*domega(0)-_vehicle_ixz*domega(2);
				_zero_input_response.dx[4] = _vehicle_iyy*domega(1);
				_zero_input_response.dx[5] = _vehicle_izz*domega(2)-_vehicle_ixz*domega(0);
			}
			else
			{
				_zero_input_response.dx[0] = _vehicle_mass*aab(0);
				_zero_input_response.dx[1] = _vehicle_mass*aab(1);
				_zero_input_response.dx[2] = _vehicle_mass*aab(2);
				_zero_input_response.dx[3] = _vehicle_ixx*domega(0)-_vehicle_ixz*domega(2);
				_zero_input_response.dx[4] = _vehicle_iyy*domega(1);
				_zero_input_response.dx[5] = _vehicle_izz*domega(2)-_vehicle_ixz*domega(0);
			}
		}

		publishZeroInputResponse(angular_velocity.timestamp_sample);
	}

	perf_end(_loop_perf);
}


void ZeroInputPlant::publishZeroInputResponse(const hrt_abstime &timestamp_sample)
{
	zero_input_response_s v_zero_input_response = {};
	v_zero_input_response.timestamp = hrt_absolute_time();
	v_zero_input_response.timestamp_sample = timestamp_sample;
	v_zero_input_response.dx[0] = _zero_input_response.dx[0];
	v_zero_input_response.dx[1] = _zero_input_response.dx[1];
	v_zero_input_response.dx[2] = _zero_input_response.dx[2];
	v_zero_input_response.dx[3] = _zero_input_response.dx[3];
	v_zero_input_response.dx[4] = _zero_input_response.dx[4];
	v_zero_input_response.dx[5] = _zero_input_response.dx[5];

	_zero_input_response_pub.publish(v_zero_input_response);
}


int ZeroInputPlant::task_spawn(int argc, char *argv[])
{
	// bool vtol = false;

	// if (argc > 1) {
	// 	if (strcmp(argv[1], "vtol") == 0) {
	// 		vtol = true;
	// 	}
	// }

	// ZeroInputPlant *instance = new ZeroInputPlant(vtol);
	ZeroInputPlant *instance = new ZeroInputPlant;

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ZeroInputPlant::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ZeroInputPlant::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
zero_input_plant is a module which calculates system derivatives without inputs.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("zero_input_plant", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
//	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int zero_input_plant_main(int argc, char *argv[])
{
	return ZeroInputPlant::main(argc, argv);
}
