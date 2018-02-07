/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <lib/mixer/mixer.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/controller_scope.h>
#include <uORB/topics/controller_rate_scope.h>
#include <uORB/uORB.h>

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f
#define ATTITUDE_TC_DEFAULT 0.2f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

#define MAX_GYRO_COUNT 3

class MulticopterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControl();

	/**
	 * Destructor, also kills the main task
	 */
	~MulticopterAttitudeControl();

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

	int		_v_att_sub;		/**< vehicle attitude subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_vehicle_status_sub;	/**< vehicle status subscription */
	int		_motor_limits_sub;		/**< motor limits subscription */
	int		_battery_status_sub;	/**< battery status subscription */
	int		_sensor_gyro_sub[MAX_GYRO_COUNT];	/**< gyro data subscription */
	int		_sensor_correction_sub;	/**< sensor thermal correction subscription */
	int		_sensor_bias_sub;	/**< sensor in-run bias correction subscription */

	unsigned _gyro_count;
	int _selected_gyro;

	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub;	/**< controller status publication */
    orb_advert_t    _controller_scope_pub;
    orb_advert_t    _controller_rate_scope_pub;

	orb_id_t _rates_sp_id;	/**< pointer to correct rates setpoint uORB metadata structure */
	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s		_v_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s		_actuators;		/**< actuator controls */
	struct vehicle_status_s			_vehicle_status;	/**< vehicle status */
	struct mc_att_ctrl_status_s 		_controller_status;	/**< controller status */
	struct battery_status_s			_battery_status;	/**< battery status */
	struct sensor_gyro_s			_sensor_gyro;		/**< gyro data before thermal correctons and ekf bias estimates are applied */
	struct sensor_correction_s		_sensor_correction;	/**< sensor thermal corrections */
	struct sensor_bias_s			_sensor_bias;		/**< sensor in-run bias corrections */
    struct controller_scope_s       _controller_scope;
    struct controller_rate_scope_s  _controller_rate_scope;

	MultirotorMixer::saturation_status _saturation_status{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;

	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */

	math::Matrix<3, 3>  _I;				/**< identity matrix */

	math::Matrix<3, 3>	_board_rotation = {};	/**< rotation matrix for the orientation that the board is mounted */

    math::Matrix<4, 4>  Tau_to_ww;
    math::Matrix<3, 3>  _J;
    math::Matrix<3, 3>  R;
    math::Matrix<3, 3>  R_sp;
    float scale_CT;   /**< 10000 * CT */
    math::Vector<2>    inf_s_pr;
    float              inf_s_yaw;

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_integ_lim;
		param_t roll_rate_d;
		param_t roll_rate_ff;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_integ_lim;
		param_t pitch_rate_d;
		param_t pitch_rate_ff;
		param_t tpa_breakpoint_p;
		param_t tpa_breakpoint_i;
		param_t tpa_breakpoint_d;
		param_t tpa_rate_p;
		param_t tpa_rate_i;
		param_t tpa_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_integ_lim;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;
		param_t yaw_ff;
		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;
		param_t yaw_auto_max;

		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
		param_t acro_expo;
		param_t acro_superexpo;
		param_t rattitude_thres;

		param_t roll_tc;
		param_t pitch_tc;

		param_t vtol_type;
		param_t vtol_opt_recovery_enabled;
		param_t vtol_wv_yaw_rate_scale;

		param_t bat_scale_en;

		param_t board_rotation;

		param_t board_offset[3];

	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_int_lim;			/**< integrator state limit for rate loop */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates */
		float yaw_ff;						/**< yaw control feed-forward */

		float tpa_breakpoint_p;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_i;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_d;				/**< Throttle PID Attenuation breakpoint */
		float tpa_rate_p;					/**< Throttle PID Attenuation slope */
		float tpa_rate_i;					/**< Throttle PID Attenuation slope */
		float tpa_rate_d;					/**< Throttle PID Attenuation slope */

		float roll_rate_max;
		float pitch_rate_max;
		float yaw_rate_max;
		float yaw_auto_max;
		math::Vector<3> mc_rate_max;		/**< attitude rate limits in stabilized modes */
		math::Vector<3> auto_rate_max;		/**< attitude rate limits in auto modes */
		matrix::Vector3f acro_rate_max;		/**< max attitude rates in acro mode */
		float acro_expo;					/**< function parameter for expo stick curve shape */
		float acro_superexpo;				/**< function parameter for superexpo stick curve shape */
		float rattitude_thres;

		int32_t vtol_type;					/**< 0 = Tailsitter, 1 = Tiltrotor, 2 = Standard airframe */
		bool vtol_opt_recovery_enabled;
		float vtol_wv_yaw_rate_scale;			/**< Scale value [0, 1] for yaw rate setpoint  */

		int32_t bat_scale_en;

		int32_t board_rotation;

		float board_offset[3];

	}		_params;

	TailsitterRecovery *_ts_opt_recovery{nullptr};	/**< Computes optimal rates for tailsitter recovery */

	/**
	 * Update our local parameter cache.
	 */
	void			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		battery_status_poll();
	void		parameter_update_poll();
	void		sensor_bias_poll();
	void		sensor_correction_poll();
	void		vehicle_attitude_poll();
	void		vehicle_attitude_setpoint_poll();
	void		vehicle_control_mode_poll();
	void		vehicle_manual_poll();
	void		vehicle_motor_limits_poll();
	void		vehicle_rates_setpoint_poll();
	void		vehicle_status_poll();

	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Throttle PID attenuation.
	 */
	math::Vector<3> pid_attenuations(float tpa_breakpoint, float tpa_rate);

	/**
	 * Get rate of vehicle.
	 */
	math::Vector<3> get_rate();

	/**
	 * Get u from tau.
	 */
    math::Vector<3> get_u_from_Tau(math::Vector<4> Tau);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace mc_att_control
{

MulticopterAttitudeControl	*g_control;
}

MulticopterAttitudeControl::MulticopterAttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),

	/* subscriptions */
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_vehicle_status_sub(-1),
	_motor_limits_sub(-1),
	_battery_status_sub(-1),
	_sensor_correction_sub(-1),
	_sensor_bias_sub(-1),

	/* gyro selection */
	_gyro_count(1),
	_selected_gyro(0),

	/* publications */
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_controller_status_pub(nullptr),
    _controller_scope_pub(nullptr),
    _controller_rate_scope_pub(nullptr),
	_rates_sp_id(nullptr),
	_actuators_id(nullptr),

	_actuators_0_circuit_breaker_enabled(false),

	_v_att{},
	_v_att_sp{},
	_v_rates_sp{},
	_manual_control_sp{},
	_v_control_mode{},
	_actuators{},
	_vehicle_status{},
	_controller_status{},
	_battery_status{},
	_sensor_gyro{},
	_sensor_correction{},
	_sensor_bias{},
    _controller_scope{},
    _controller_rate_scope{},
	_saturation_status{},
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	_vehicle_status.is_rotary_wing = true;

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_int_lim.zero();
	_params.rate_d.zero();
	_params.rate_ff.zero();
	_params.yaw_ff = 0.0f;
	_params.roll_rate_max = 0.0f;
	_params.pitch_rate_max = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.mc_rate_max.zero();
	_params.auto_rate_max.zero();
	_params.acro_rate_max.zero();
	_params.rattitude_thres = 1.0f;

	_params.bat_scale_en = 0;

	_params.board_rotation = 0;

	_params.board_offset[0] = 0.0f;
	_params.board_offset[1] = 0.0f;
	_params.board_offset[2] = 0.0f;

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_sp_prev.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	_I.identity();
	_board_rotation.identity();

    /* init Tau_to_ww */
    Tau_to_ww.zero();
    float k1 = 25427.18f;
    float k2 = 226019.35f;
    float k3 = 2032520.33f;

    Tau_to_ww(0, 0) = k1;
    Tau_to_ww(1, 0) = k1;
    Tau_to_ww(2, 0) = k1;
    Tau_to_ww(3, 0) = k1;

    Tau_to_ww(0, 1) = - k2;
    Tau_to_ww(1, 1) = k2;

    Tau_to_ww(2, 2) = k2;
    Tau_to_ww(3, 2) = - k2;

    Tau_to_ww(0, 3) = k3;
    Tau_to_ww(1, 3) = k3;
    Tau_to_ww(2, 3) = - k3;
    Tau_to_ww(3, 3) = - k3;

    _J.zero();
    _J(0, 0) = 0.01431f;
    _J(1, 1) = 0.01431f;
    _J(2, 2) = 0.02721f;

    R.identity();
    R_sp.identity();

    scale_CT = 0.09832f;
    inf_s_yaw = 0.0f;
    inf_s_pr.zero();

	_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_integ_lim	= 	param_find("MC_RR_INT_LIM");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.roll_rate_ff		= 	param_find("MC_ROLLRATE_FF");

	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p		= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i		= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_integ_lim	= 	param_find("MC_PR_INT_LIM");
	_params_handles.pitch_rate_d		= 	param_find("MC_PITCHRATE_D");
	_params_handles.pitch_rate_ff 		= 	param_find("MC_PITCHRATE_FF");

	_params_handles.tpa_breakpoint_p 	= 	param_find("MC_TPA_BREAK_P");
	_params_handles.tpa_breakpoint_i 	= 	param_find("MC_TPA_BREAK_I");
	_params_handles.tpa_breakpoint_d 	= 	param_find("MC_TPA_BREAK_D");
	_params_handles.tpa_rate_p	 	= 	param_find("MC_TPA_RATE_P");
	_params_handles.tpa_rate_i	 	= 	param_find("MC_TPA_RATE_I");
	_params_handles.tpa_rate_d	 	= 	param_find("MC_TPA_RATE_D");

	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_integ_lim	= 	param_find("MC_YR_INT_LIM");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	 	= 	param_find("MC_YAWRATE_FF");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");

	_params_handles.roll_rate_max		= 	param_find("MC_ROLLRATE_MAX");
	_params_handles.pitch_rate_max		= 	param_find("MC_PITCHRATE_MAX");
	_params_handles.yaw_rate_max		= 	param_find("MC_YAWRATE_MAX");
	_params_handles.yaw_auto_max		= 	param_find("MC_YAWRAUTO_MAX");

	_params_handles.acro_roll_max		= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max		= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max		= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.acro_expo		= 	param_find("MC_ACRO_EXPO");
	_params_handles.acro_superexpo		= 	param_find("MC_ACRO_SUPEXPO");

	_params_handles.rattitude_thres 	= 	param_find("MC_RATT_TH");

	_params_handles.roll_tc			= 	param_find("MC_ROLL_TC");
	_params_handles.pitch_tc		= 	param_find("MC_PITCH_TC");

	_params_handles.bat_scale_en		=	param_find("MC_BAT_SCALE_EN");

	/* rotations */
	_params_handles.board_rotation		=	param_find("SENS_BOARD_ROT");

	/* rotation offsets */
	_params_handles.board_offset[0]		=	param_find("SENS_BOARD_X_OFF");
	_params_handles.board_offset[1]		=	param_find("SENS_BOARD_Y_OFF");
	_params_handles.board_offset[2]		=	param_find("SENS_BOARD_Z_OFF");

	/* fetch initial parameter values */
	parameters_update();

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	delete _ts_opt_recovery;

	mc_att_control::g_control = nullptr;
}

void
MulticopterAttitudeControl::parameters_update()
{
	float v;

	float roll_tc, pitch_tc;

	param_get(_params_handles.roll_tc, &roll_tc);
	param_get(_params_handles.pitch_tc, &pitch_tc);

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	param_get(_params_handles.roll_rate_integ_lim, &v);
	_params.rate_int_lim(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_ff, &v);
	_params.rate_ff(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_integ_lim, &v);
	_params.rate_int_lim(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_ff, &v);
	_params.rate_ff(1) = v;

	param_get(_params_handles.tpa_breakpoint_p, &_params.tpa_breakpoint_p);
	param_get(_params_handles.tpa_breakpoint_i, &_params.tpa_breakpoint_i);
	param_get(_params_handles.tpa_breakpoint_d, &_params.tpa_breakpoint_d);
	param_get(_params_handles.tpa_rate_p, &_params.tpa_rate_p);
	param_get(_params_handles.tpa_rate_i, &_params.tpa_rate_i);
	param_get(_params_handles.tpa_rate_d, &_params.tpa_rate_d);

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_integ_lim, &v);
	_params.rate_int_lim(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;
	param_get(_params_handles.yaw_rate_ff, &v);
	_params.rate_ff(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);

	/* angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.mc_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.mc_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.mc_rate_max(2) = math::radians(_params.yaw_rate_max);

	/* auto angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.auto_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.auto_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_auto_max, &_params.yaw_auto_max);
	_params.auto_rate_max(2) = math::radians(_params.yaw_auto_max);

	/* manual rate control acro mode rate limits and expo */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);
	param_get(_params_handles.acro_expo, &_params.acro_expo);
	param_get(_params_handles.acro_superexpo, &_params.acro_superexpo);

	/* stick deflection needed in rattitude mode to control rates not angles */
	param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);

	if (_vehicle_status.is_vtol) {
		param_get(_params_handles.vtol_type, &_params.vtol_type);

		int32_t tmp;
		param_get(_params_handles.vtol_opt_recovery_enabled, &tmp);
		_params.vtol_opt_recovery_enabled = (tmp == 1);

		param_get(_params_handles.vtol_wv_yaw_rate_scale, &_params.vtol_wv_yaw_rate_scale);

	} else {
		_params.vtol_opt_recovery_enabled = false;
		_params.vtol_wv_yaw_rate_scale = 0.f;
	}

	param_get(_params_handles.bat_scale_en, &_params.bat_scale_en);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	/* rotation of the autopilot relative to the body */
	param_get(_params_handles.board_rotation, &(_params.board_rotation));

	/* fine adjustment of the rotation */
	param_get(_params_handles.board_offset[0], &(_params.board_offset[0]));
	param_get(_params_handles.board_offset[1], &(_params.board_offset[1]));
	param_get(_params_handles.board_offset[2], &(_params.board_offset[2]));

	/* get transformation matrix from sensor/board to body frame */
	get_rot_matrix((enum Rotation)_params.board_rotation, &_board_rotation);

	/* fine tune the rotation */
	math::Matrix<3, 3> board_rotation_offset;
	board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _params.board_offset[0],
					 M_DEG_TO_RAD_F * _params.board_offset[1],
					 M_DEG_TO_RAD_F * _params.board_offset[2]);
	_board_rotation = board_rotation_offset * _board_rotation;
}

void
MulticopterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_rates_sp_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

				_params_handles.vtol_type = param_find("VT_TYPE");
				_params_handles.vtol_opt_recovery_enabled = param_find("VT_OPT_RECOV_EN");
				_params_handles.vtol_wv_yaw_rate_scale = param_find("VT_WV_YAWR_SCL");

				parameters_update();

				if (_params.vtol_type == 0 && _params.vtol_opt_recovery_enabled) {
					// the vehicle is a tailsitter, use optimal recovery control strategy
					_ts_opt_recovery = new TailsitterRecovery();
				}

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		multirotor_motor_limits_s motor_limits = {};
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &motor_limits);

		_saturation_status.value = motor_limits.saturation_status;
	}
}

void
MulticopterAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_v_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	}
}

void
MulticopterAttitudeControl::sensor_correction_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_correction_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &_sensor_correction);
	}

	/* update the latest gyro selection */
	if (_sensor_correction.selected_gyro_instance < _gyro_count) {
		_selected_gyro = _sensor_correction.selected_gyro_instance;
	}
}

void
MulticopterAttitudeControl::sensor_bias_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_bias_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_bias), _sensor_bias_sub, &_sensor_bias);
	}

}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
MulticopterAttitudeControl::control_attitude(float dt)
{
	vehicle_attitude_setpoint_poll();

	_thrust_sp = _v_att_sp.thrust;

	/* construct attitude setpoint rotation matrix */
	math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
	R_sp = q_sp.to_dcm();

	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	R = q_att.to_dcm();

   /* get error of the attitude -by lu*/
    float eps = 0.001f;
    math::Matrix<3, 3> R_dTR = R_sp.transposed() * R;
    float tr_R_dTR = R_dTR(0, 0) + R_dTR(1, 1) + R_dTR(2, 2);
    if (tr_R_dTR < -1.0f) {
        tr_R_dTR = -1.0f;
    }

    math::Matrix<3, 3> e_hat = R_sp.transposed() * R - R.transposed() * R_sp;
    math::Vector<3> e_attitude(e_hat(2, 1), e_hat(0, 2), e_hat(1, 0));
    float temp_scale = 1.0f / (2.0f * (float)sqrt(1.0f + tr_R_dTR + eps));
    e_attitude = e_attitude * temp_scale;
    _controller_scope.e_att[0] = e_attitude(0);
    _controller_scope.e_att[1] = e_attitude(1);
    _controller_scope.e_att[2] = e_attitude(2);
    _controller_scope.temp_scale = temp_scale;

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	//math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	//math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation */
	//math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);
   
	/* calculate angle error */
	//float e_R_z_sin = e_R.length();
	//float e_R_z_cos = R_z * R_sp_z;

	/* calculate weight for yaw control */
	//float yaw_w = R_sp(2, 2) * R_sp(2, 2);

	/* calculate rotation matrix after roll/pitch only rotation */
	//math::Matrix<3, 3> R_rp;

	//if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		//float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		//math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

		//e_R = e_R_z_axis * e_R_z_angle;

		/* cross product matrix for e_R_axis */
		//math::Matrix<3, 3> e_R_cp;
		//e_R_cp.zero();
		//e_R_cp(0, 1) = -e_R_z_axis(2);
		//e_R_cp(0, 2) = e_R_z_axis(1);
		//e_R_cp(1, 0) = e_R_z_axis(2);
		//e_R_cp(1, 2) = -e_R_z_axis(0);
		//e_R_cp(2, 0) = -e_R_z_axis(1);
		//e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		//R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	//} else {
		/* zero roll/pitch rotation */
		//R_rp = R;
	//}

	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	//math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	//math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	//e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	//if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		//math::Quaternion q_error;
		//q_error.from_dcm(R.transposed() * R_sp);
		//math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag()  * 2.0f : -q_error.imag() * 2.0f;

		/* use fusion of Z axis based rotation and direct rotation */
		//float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		//e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	//}

    /* calculate augular rates setpoint -by lu */
    math::Matrix<3, 3> RTR_d = R.transposed() * R_sp;
    float trace_RTR_d = RTR_d(0, 0) + RTR_d(1, 1) + RTR_d(2, 2);
    if (trace_RTR_d < -1.0f) {
        trace_RTR_d = -1.0f;
    }

    //math::Matrix<3, 3> _I.identity();

    math::Matrix<3, 1> e_att;
    e_att(0, 0) = e_attitude(0);
    e_att(1, 0) = e_attitude(1);
    e_att(2, 0) = e_attitude(2);

    math::Matrix<3, 3> e_Re_RT = e_att * e_att.transposed();
    /*for (int _i_m = 0; _i_m < 3; _i_m++) {
        for (int _j_m = 0; _j_m < 3; _j_m++) {
            e_Re_RT(_i_m, _j_m) = e_attitude(_i_m)*e_attitude(_j_m);
        }
    }*/
    
    math::Matrix<3, 3> _E;
    _E = (_I * trace_RTR_d - R_dTR + e_Re_RT * 2.0f) * temp_scale;

	math::Vector<3> rates = get_rate();

    /* get the adj of _E -by lu*/
    math::Matrix<3, 3> _E_adj;
    _E_adj(0, 0) =   _E(1, 1) * _E(2, 2) - _E(1, 2) * _E(2, 1);
    _E_adj(0, 1) = - _E(1, 0) * _E(2, 2) + _E(1, 2) * _E(2, 0);
    _E_adj(0, 2) =   _E(1, 0) * _E(2, 1) - _E(1, 1) * _E(2, 0);

    _E_adj(1, 0) = - _E(0, 1) * _E(2, 2) + _E(0, 2) * _E(2, 1);
    _E_adj(1, 1) =   _E(0, 0) * _E(2, 2) - _E(0, 2) * _E(2, 0);
    _E_adj(1, 2) = - _E(0, 0) * _E(2, 1) + _E(0, 1) * _E(2, 0);

    _E_adj(2, 0) =   _E(0, 1) * _E(1, 2) - _E(0, 2) * _E(1, 1);
    _E_adj(2, 1) = - _E(0, 0) * _E(1, 2) + _E(0, 2) * _E(1, 0);
    _E_adj(2, 2) =   _E(0, 0) * _E(1, 1) - _E(1, 0) * _E(0, 1);

    float _E_norm = _E(0, 0) * _E(1, 1) * _E(2, 2) + _E(0, 1) * _E(1, 2) * _E(2, 0) + _E(1, 0) * _E(2, 1) * _E(0, 2);
    _E_norm = _E_norm - _E(0, 2) * _E(1, 1) * _E(2, 0) - _E(1, 2) * _E(2, 1) * _E(0, 0) - _E(0, 1) * _E(1, 0) * _E(2, 2);

    //_controller_scope.E_norm = _E_norm;

    if ( fabsf(_E_norm) < 0.01f ) {
        if ( _E_norm <= 0.0f ) {
            _E_norm = -0.01f;
        } else {
            _E_norm = 0.01f;
        }
    }

    _controller_scope.E_norm = _E_norm;

    float _E_norm_1 = 1.0f / _E_norm;
    math::Matrix<3, 3> _E_inv = _E_adj * _E_norm_1;

    float kp1 = 5.5f;
    float kp2 = 5.5f;
    float kp3 = 3.0f;

    math::Vector<3> _k_e_att(e_attitude(0)*kp1, e_attitude(1)*kp2, e_attitude(2)*kp3);

    math::Vector<3> _model_part;
    _model_part = R_dTR * rates;
    math::Vector<3> _control_part;
    _control_part = R_dTR * _E_inv * _k_e_att;

    _controller_scope.model_part[0] = _model_part(0);
    _controller_scope.model_part[1] = _model_part(1);
    _controller_scope.model_part[2] = _model_part(2);

    _controller_scope.control_part[0] = _control_part(0);
    _controller_scope.control_part[1] = _control_part(1);
    _controller_scope.control_part[2] = _control_part(2);

    //_rates_sp = -(R_dTR * rates + R_dTR * _E_inv * _k_e_att);
    //_rates_sp = - (_model_part + _control_part);
    _rates_sp = - _control_part;

    float yaw_w = 1.0f; //fake
	/* Feed forward the yaw setpoint rate. We need to transform the yaw from world into body frame.
	 * The following is a simplification of:
	 * R.transposed() * math::Vector<3>(0.f, 0.f, _v_att_sp.yaw_sp_move_rate * _params.yaw_ff)
	 */
	math::Vector<3> yaw_feedforward_rate(R(2, 0), R(2, 1), R(2, 2));
	yaw_feedforward_rate *= _v_att_sp.yaw_sp_move_rate * _params.yaw_ff;

	yaw_feedforward_rate(2) *= yaw_w;
	_rates_sp += yaw_feedforward_rate;
    _controller_scope.timestamp = hrt_absolute_time();
    if (_controller_scope_pub != nullptr) {
        orb_publish(ORB_ID(controller_scope), _controller_scope_pub, &_controller_scope);
    } else {
        _controller_scope_pub = orb_advertise(ORB_ID(controller_scope), &_controller_scope);
    }
	/* calculate angular rates setpoint */
	//_rates_sp = _params.att_p.emult(e_R);

	/* limit rates */
	for (int i = 0; i < 3; i++) {
		if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
		    !_v_control_mode.flag_control_manual_enabled) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.auto_rate_max(i), _params.auto_rate_max(i));

		} else {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
		}
	}

	/* VTOL weather-vane mode, dampen yaw rate */
	if (_vehicle_status.is_vtol && _v_att_sp.disable_mc_yaw_control) {
		if (_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) {

			const float wv_yaw_rate_max = _params.auto_rate_max(2) * _params.vtol_wv_yaw_rate_scale;
			_rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max, wv_yaw_rate_max);

			// prevent integrator winding up in weathervane mode
			_rates_int(2) = 0.0f;
		}
	}
}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
math::Vector<3>
MulticopterAttitudeControl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_v_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	math::Vector<3> pidAttenuationPerAxis;
	pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

	return pidAttenuationPerAxis;
}

/*
 * get Attitude rate 
 * Input: null
 * Output: rate
 */
math::Vector<3>
MulticopterAttitudeControl::get_rate()
{
	// get the raw gyro data and correct for thermal errors
	math::Vector<3> rates;

	if (_selected_gyro == 0) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

	} else if (_selected_gyro == 1) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

	} else if (_selected_gyro == 2) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

	} else {
		rates(0) = _sensor_gyro.x;
		rates(1) = _sensor_gyro.y;
		rates(2) = _sensor_gyro.z;
	}

	// rotate corrected measurements from sensor to body frame
	rates = _board_rotation * rates;

	// correct for in-run bias errors
	rates(0) -= _sensor_bias.gyro_x_bias;
	rates(1) -= _sensor_bias.gyro_y_bias;
	rates(2) -= _sensor_bias.gyro_z_bias;
	return rates;
}

/*
 * Get u from Tau
 * Input: 'Tau' vector
 * Output: 'u' vector
 */
math::Vector<3>
MulticopterAttitudeControl::get_u_from_Tau(math::Vector<4> Tau)
{

    math::Vector<4> ww = Tau_to_ww * Tau;
    math::Vector<4> w;
    math::Vector<4> alpha;

    for (int i = 0; i < 4; i++) {
        if (ww(i) < 1.0f) {
            ww(i) = 1.0f;
        }
        w(i) = (float)sqrt(ww(i));
        alpha(i) = ( w(i) - 125.98f ) / 625.0f;
    }

    math::Vector<3> u;
    u(0) = - alpha(0) + alpha(1);
    u(1) = alpha(2) - alpha(3);
    u(2) = alpha(0) + alpha(1) - alpha(2) - alpha(3);

    return u;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControl::control_attitude_rates(float dt)
{

    float eps = 0.2f;//0.01f;
    float k1_pr = 0.1f;//0.15f;//0.2f;
    float k2_pr = 0.01f;//0.05f;
    float k1_yaw = 0.015f;//0.02f;
    float k2_yaw = 0.0008f;//0.001f;

	/* reset integral if disarmed */
	if (!_v_control_mode.flag_armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
        inf_s_pr.zero();
        inf_s_yaw = 0.0f;
	}

	math::Vector<3> rates = get_rate();
/*
	math::Vector<3> rates_p_scaled = _params.rate_p.emult(pid_attenuations(_params.tpa_breakpoint_p, _params.tpa_rate_p));
	//math::Vector<3> rates_i_scaled = _params.rate_i.emult(pid_attenuations(_params.tpa_breakpoint_i, _params.tpa_rate_i));
	math::Vector<3> rates_d_scaled = _params.rate_d.emult(pid_attenuations(_params.tpa_breakpoint_d, _params.tpa_rate_d));
*/
	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;
    math::Vector<3> omega_err = rates - R.transposed() * R_sp * _rates_sp;

	/*_att_control = rates_p_scaled.emult(rates_err) +
		       _rates_int +
		       rates_d_scaled.emult(_rates_prev - rates) / dt +
		       _params.rate_ff.emult(_rates_sp);*/

    /* roll pitch sliding surface */
    float n_e1 = 0.4f;
    float s1_norm_pr = (float)sqrt( omega_err(0) * omega_err(0) + omega_err(1) * omega_err(1) );
    float inv_s_norm_pr = 1.0f / (s1_norm_pr + eps);
    float inv_s_pr_ne1 = 1.0f / ((float)pow(s1_norm_pr, n_e1) + eps);
    /* yaw sliding surface */
    float n_e2 = 0.4f;
    float s1_norm_yaw = fabsf(omega_err(2));
    float inv_s_norm_yaw = 1.0f / (s1_norm_yaw + eps);
    float inv_s_yaw_ne1 = 1.0f / ((float)pow(s1_norm_yaw, n_e2) + eps);

    math::Vector<3> _att_Tau;
    /* roll pitch controller */
    _att_Tau(0) = - k1_pr * omega_err(0) * inv_s_pr_ne1 - k2_pr * inf_s_pr(0);
    _att_Tau(1) = - k1_pr * omega_err(1) * inv_s_pr_ne1 - k2_pr * inf_s_pr(1);
    /* yaw controller */
    _att_Tau(2) = - k1_yaw * omega_err(2) * inv_s_yaw_ne1 - k2_yaw * inf_s_yaw;

    math::Matrix<3, 3> rates_hat;
    rates_hat.zero();
    rates_hat(0, 1) = - rates(2);
    rates_hat(0, 2) = rates(1);
    rates_hat(1, 2) = - rates(0);
    rates_hat(1, 0) = rates(2);
    rates_hat(2, 0) = - rates(1);
    rates_hat(2, 1) = rates(0);
    math::Vector<3> model_poly = rates_hat * _J * rates - _J * (rates_hat * R.transposed() * R_sp * _rates_sp);

    math::Vector<3> control_Tau = _att_Tau + model_poly;
    float temp_alpha = _thrust_sp * 6.25f + 1.2598f; /* mind the scale_CT = 10000 * CT */
    float thrust_Tau = temp_alpha * temp_alpha * scale_CT * 4.0f;
    math::Vector<4> all_Tau(thrust_Tau, control_Tau(0), control_Tau(1), control_Tau(2));
    _att_control = get_u_from_Tau(all_Tau);

    /* record the controller data */
    _controller_rate_scope.omega_err[0] = omega_err(0);
    _controller_rate_scope.omega_err[1] = omega_err(1);
    _controller_rate_scope.omega_err[2] = omega_err(2);
    _controller_rate_scope.s1_norm_pr = s1_norm_pr;
    _controller_rate_scope.s1_norm_yaw = s1_norm_yaw;
    _controller_rate_scope.att_Tau[0] = _att_Tau(0);
    _controller_rate_scope.att_Tau[1] = _att_Tau(1);
    _controller_rate_scope.att_Tau[2] = _att_Tau(2);
    _controller_rate_scope.model_poly[0] = model_poly(0);
    _controller_rate_scope.model_poly[1] = model_poly(1);
    _controller_rate_scope.model_poly[2] = model_poly(2);
    _controller_rate_scope.all_Tau[0] = all_Tau(0);
    _controller_rate_scope.all_Tau[1] = all_Tau(1);
    _controller_rate_scope.all_Tau[2] = all_Tau(2);
    _controller_rate_scope.all_Tau[3] = all_Tau(3);
    _controller_rate_scope.u[0] = _thrust_sp;
    _controller_rate_scope.u[1] = _att_control(0);
    _controller_rate_scope.u[2] = _att_control(1);
    _controller_rate_scope.u[3] = _att_control(2);
    _controller_rate_scope.inf_s_pr[0] = inf_s_pr(0);
    _controller_rate_scope.inf_s_pr[1] = inf_s_pr(1);
    _controller_rate_scope.inf_s_yaw = inf_s_yaw;
    _controller_rate_scope.timestamp = hrt_absolute_time();

    if (_controller_rate_scope_pub != nullptr) {
        orb_publish(ORB_ID(controller_rate_scope), _controller_rate_scope_pub, &_controller_rate_scope);
    } else {
        _controller_rate_scope_pub = orb_advertise(ORB_ID(controller_rate_scope), &_controller_rate_scope);
    }

	math::Vector<3> rates_d_scaled = _params.rate_d.emult(pid_attenuations(_params.tpa_breakpoint_d, _params.tpa_rate_d));
    _att_control = _att_control + rates_d_scaled.emult(_rates_prev - rates) / dt;

    if (_thrust_sp > MIN_TAKEOFF_THRUST) {
        inf_s_pr(0) = inf_s_pr(0) + omega_err(0) * inv_s_norm_pr * dt;
        inf_s_pr(1) = inf_s_pr(1) + omega_err(1) * inv_s_norm_pr * dt;
        inf_s_yaw = inf_s_yaw + omega_err(2) * inv_s_norm_yaw * dt;
    }

	_rates_sp_prev = _rates_sp;
	_rates_prev = rates;

	/* update integral only if motors are providing enough thrust to be effective */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
			// Check for positive control saturation
			bool positive_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

			// Check for negative control saturation
			bool negative_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

			// prevent further positive control saturation
			if (positive_saturation) {
				rates_err(i) = math::min(rates_err(i), 0.0f);

			}

			// prevent further negative control saturation
			if (negative_saturation) {
				rates_err(i) = math::max(rates_err(i), 0.0f);

			}

			// Perform the integration using a first order method and do not propaate the result if out of range or invalid
			float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;

			if (PX4_ISFINITE(rate_i) && rate_i > -_params.rate_int_lim(i) && rate_i < _params.rate_int_lim(i)) {
				_rates_int(i) = rate_i;

			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_params.rate_int_lim(i), _params.rate_int_lim(i));

	}
}

void
MulticopterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	mc_att_control::g_control->task_main();
}

void
MulticopterAttitudeControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	_gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);

	if (_gyro_count == 0) {
		_gyro_count = 1;
	}

	for (unsigned s = 0; s < _gyro_count; s++) {
		_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}

	_sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));
	_sensor_bias_sub = orb_subscribe(ORB_ID(sensor_bias));

	/* initialize parameters cache */
	parameters_update();

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	while (!_task_should_exit) {

		poll_fds.fd = _sensor_gyro_sub[_selected_gyro];

		/* wait for up to 100ms for data */
		int pret = px4_poll(&poll_fds, 1, 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("mc att ctrl: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on gyro changes */
		if (poll_fds.revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy gyro data */
			orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			vehicle_motor_limits_poll();
			battery_status_poll();
			vehicle_attitude_poll();
			sensor_correction_poll();
			sensor_bias_poll();

			/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			 * or roll (yaw can rotate 360 in normal att control).  If both are true don't
			 * even bother running the attitude controllers */
			if (_v_control_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual_control_sp.y) > _params.rattitude_thres ||
				    fabsf(_manual_control_sp.x) > _params.rattitude_thres) {
					_v_control_mode.flag_control_attitude_enabled = false;
				}
			}

			if (_v_control_mode.flag_control_attitude_enabled) {

				if (_ts_opt_recovery == nullptr) {
					// the  tailsitter recovery instance has not been created, thus, the vehicle
					// is not a tailsitter, do normal attitude control
					control_attitude(dt);

				} else {
					vehicle_attitude_setpoint_poll();
					_thrust_sp = _v_att_sp.thrust;
					math::Quaternion q(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
					math::Quaternion q_sp(&_v_att_sp.q_d[0]);
					_ts_opt_recovery->setAttGains(_params.att_p, _params.yaw_ff);
					_ts_opt_recovery->calcOptimalRates(q, q_sp, _v_att_sp.yaw_sp_move_rate, _rates_sp);

					/* limit rates */
					for (int i = 0; i < 3; i++) {
						_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
					}
				}

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					matrix::Vector3f man_rate_sp;
					man_rate_sp(0) = math::superexpo(_manual_control_sp.y, _params.acro_expo, _params.acro_superexpo);
					man_rate_sp(1) = math::superexpo(-_manual_control_sp.x, _params.acro_expo, _params.acro_superexpo);
					man_rate_sp(2) = math::superexpo(_manual_control_sp.r, _params.acro_expo, _params.acro_superexpo);
					man_rate_sp = man_rate_sp.emult(_params.acro_rate_max);
					_rates_sp = math::Vector<3>(man_rate_sp.data());
					_thrust_sp = _manual_control_sp.z;

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt);

				/* publish actuator controls */
				_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.control[7] = _v_att_sp.landing_gear;
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _sensor_gyro.timestamp;

				/* scale effort by battery status */
				if (_params.bat_scale_en && _battery_status.scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						_actuators.control[i] *= _battery_status.scale;
					}
				}

				_controller_status.roll_rate_integ = _rates_int(0);
				_controller_status.pitch_rate_integ = _rates_int(1);
				_controller_status.yaw_rate_integ = _rates_int(2);
				_controller_status.timestamp = hrt_absolute_time();

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {

						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}

				}

				/* publish controller status */
				if (_controller_status_pub != nullptr) {
					orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

				} else {
					_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
				}
			}

			if (_v_control_mode.flag_control_termination_enabled) {
				if (!_vehicle_status.is_vtol) {

					_rates_sp.zero();
					_rates_int.zero();
					_thrust_sp = 0.0f;
					_att_control.zero();

					/* publish actuator controls */
					_actuators.control[0] = 0.0f;
					_actuators.control[1] = 0.0f;
					_actuators.control[2] = 0.0f;
					_actuators.control[3] = 0.0f;
					_actuators.timestamp = hrt_absolute_time();
					_actuators.timestamp_sample = _sensor_gyro.timestamp;

					if (!_actuators_0_circuit_breaker_enabled) {
						if (_actuators_0_pub != nullptr) {

							orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
							perf_end(_controller_latency_perf);

						} else if (_actuators_id) {
							_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
						}
					}

					_controller_status.roll_rate_integ = _rates_int(0);
					_controller_status.pitch_rate_integ = _rates_int(1);
					_controller_status.yaw_rate_integ = _rates_int(2);
					_controller_status.timestamp = hrt_absolute_time();

					/* publish controller status */
					if (_controller_status_pub != nullptr) {
						orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

					} else {
						_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
					}

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}
				}
			}
		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
}

int
MulticopterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_ATTITUDE_CONTROL,
					   1700,
					   (px4_main_t)&MulticopterAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (mc_att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		mc_att_control::g_control = new MulticopterAttitudeControl;

		if (mc_att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != mc_att_control::g_control->start()) {
			delete mc_att_control::g_control;
			mc_att_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (mc_att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete mc_att_control::g_control;
		mc_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
