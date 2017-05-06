/****************************************************************************
 *
 *   Copyright (c) 2013 - 2015 PX4 Development Team. All rights reserved.
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
 * @file leader_esitimator.h
 *Leader Esitimator.
 *
 */

#pragma once

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <functional>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/topics/whycon_target.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_wt_message.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <matrix/Matrix.hpp>

using namespace matrix;
using namespace px4;
/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int whycon_observer_main(int argc, char *argv[]);
class WhyconObserver//MulticopterPositionControlMultiplatform
{
public:
	/**
	 * Constructor
	 */
	WhyconObserver();

	/**
	 * Destructor, also kills task.
	 */
	~WhyconObserver();

	/* Callbacks for topics */
	//void handle_vehicle_attitude(const px4_vehicle_attitude &msg);
	//void handle_parameter_update(const px4_parameter_update &msg);
	//void handle_position_setpoint_triplet(const px4_position_setpoint_triplet &msg);

	//void spin() { _n.spin(); }
	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();
	Vector<float, 3> _target_pos_body;
	Vector<float, 3> _target_pos_ground;
	Vector<float, 3> _delta_ob_frame; /*delta position between cam and fram*/
	enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, X_tz, n_x};

private:
	//const float alt_ctl_dz = 0.2f;
	bool	_task_should_exit;		/**< if true, task should exit */
	
	int		_control_ob_task;			/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */
	orb_advert_t	_target_pos_pub;
	int  	_whycon_target_sub;
	int		_control_state_sub;
	int		_local_pos_sub;			/**< vehicle local position */
	//orb_advert_t  	_uav_lde_pub;
	struct whycon_target_s  		_whycon_target;
	struct control_state_s			_ctrl_state;
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct vehicle_wt_message_s		_target_pos_message;
	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();
	void		status_reset();
	void		wo_frame_tf(float x,float y,float z,float local_x,float local_y,float local_z);
	
	static void	task_main_trampoline(int argc, char *argv[]);
	/**
	 * Main sensor collection task.
	 */
	void		task_main();

	
};
