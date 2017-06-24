/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <poll.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/ca_traject.h>
#include <mathlib/mathlib.h>
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
/**
 * daemon management function.
 */
__EXPORT int px4_daemon_app_main(int argc, char *argv[]);
/**
 * Mainloop of daemon.
 */
struct Trajectory{
    bool receive_flag[10];
    float t[11];
    float traject[10][7][4];
    int num_keyframe;
    int traject_order;
    bool all_done;
};
float pos_sp[4];
int px4_daemon_thread_main(int argc, char *argv[]);
void get_pos_from_traject( struct Trajectory _trajectory, float now_time);
/**
 * Print the correct usage.
 */
static void usage(const char *reason);
static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}
	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}
/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int px4_daemon_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}
	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}
		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("daemon",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 10000,
						 px4_daemon_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}
	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}
	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		return 0;
	}
	usage("unrecognized command");
	return 1;
}
void get_pos_from_traject( struct Trajectory _trajectory, float now_time)
{
    int order = _trajectory.traject_order;
    int index = 0;
    float _t_vector[order+1];
    float _traject_scale[order+1];
        pos_sp[0] = 0;
        pos_sp[1] = 0;
        pos_sp[2] = 0;
        pos_sp[3] = 0;
    if (_trajectory.all_done)
    {
        for (int i=1; i < _trajectory.num_keyframe+1 ; i++)
        {
            if(now_time <= _trajectory.t[i])
            {
                index = i - 1;
                break;
            }
        }
        for (int j=0; j < 4 ; j++)
        {
            for (int i=0; i < order+1 ; i++)
            {
                _t_vector[i] = (float)pow((double)now_time,order-i);
                _traject_scale[i] = _trajectory.traject[index][i][j];
                pos_sp[j] += _traject_scale[i] * _t_vector[i];
            }
        }
    }
    else
    {
        pos_sp[0] = 0;
        pos_sp[1] = 0;
        pos_sp[2] = 0;
        pos_sp[3] = 0;
    }
}
int px4_daemon_thread_main(int argc, char *argv[])
{
	warnx("[daemon] starting\n");
	thread_running = true;
	int ca_traject_sub = orb_subscribe(ORB_ID(ca_traject));
    /* traject struct */
    struct Trajectory _traject;
    memset(&_traject,0,sizeof(struct Trajectory));
    for (int i = 0; i <10; i++) {
        _traject.receive_flag[i] = false;
    }
    _traject.all_done = false;
    _traject.num_keyframe = 10;
	/* limit the update rate to 5 Hz */
	//orb_set_interval(sensor_sub_fd, 200);
	//orb_set_interval(ca_traject_sub_fd, 200);
	/* advertise attitude topic */
	//struct vehicle_attitude_s att;
	//memset(&att, 0, sizeof(att));
	//orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);
	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[1];
		fds[0].fd = ca_traject_sub;
        fds[0].events = POLLIN;
	int error_counter = 0;
	warnx("Start");
	while (!thread_should_exit) {
	    int poll_ret = px4_poll(&fds[0], 1, 1000);
		if (poll_ret == 0) {
			warnx("Got no data within a second");
		} else if (poll_ret < 0) { 
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				warnx("ERROR return value from poll(): %d", poll_ret);
			}
			error_counter++;
		} else { 
	    warnx("runing");
        bool updated = false;
        orb_check(ca_traject_sub, &updated);
			if (fds[0].revents & POLLIN) {
                struct ca_traject_s raw1;
                orb_copy(ORB_ID(ca_traject), ca_traject_sub, &raw1);
				PX4_INFO("Traject:\n  total: %d order+1: %d",
					 raw1.num_keyframe,
					 raw1.order_p_1);
				PX4_INFO("     phase %d :",raw1.index_keyframe);
                _traject.receive_flag[raw1.index_keyframe - 1] = true;
                _traject.num_keyframe = raw1.num_keyframe;
                _traject.traject_order = raw1.order_p_1 - 1;
				PX4_INFO(" PC_time : [%" PRIu64 "]",raw1.PC_time_usec);
				PX4_INFO(" onboard_PC_time : [%" PRIu64 "]",raw1.onboard_PC_time_usec);
				PX4_INFO("     start time %.2f finish time %.2f",
                     (double)raw1.t[0],
                     (double)raw1.t[1]);
                _traject.t[raw1.index_keyframe - 1] = raw1.t[0];
                _traject.t[raw1.index_keyframe] = raw1.t[1];
				PX4_INFO("        x: [%.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
                    (double)raw1.trajectory_coefficient_x[0],
                    (double)raw1.trajectory_coefficient_x[1],
                    (double)raw1.trajectory_coefficient_x[2],
                    (double)raw1.trajectory_coefficient_x[3],
                    (double)raw1.trajectory_coefficient_x[4],
                    (double)raw1.trajectory_coefficient_x[5],
                    (double)raw1.trajectory_coefficient_x[6]);
                for (int k=0 ; k<7 ;k++)
                {
                    _traject.traject[raw1.index_keyframe - 1][k][0] = raw1.trajectory_coefficient_x[k];
                }
				PX4_INFO("        y: [%.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
                    (double)raw1.trajectory_coefficient_y[0],
                    (double)raw1.trajectory_coefficient_y[1],
                    (double)raw1.trajectory_coefficient_y[2],
                    (double)raw1.trajectory_coefficient_y[3],
                    (double)raw1.trajectory_coefficient_y[4],
                    (double)raw1.trajectory_coefficient_y[5],
                    (double)raw1.trajectory_coefficient_y[6]);
                for (int k=0 ; k<7 ;k++)
                {
                    _traject.traject[raw1.index_keyframe - 1][k][1] = raw1.trajectory_coefficient_y[k];
                }
				PX4_INFO("        z: [%.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
                    (double)raw1.trajectory_coefficient_z[0],
                    (double)raw1.trajectory_coefficient_z[1],
                    (double)raw1.trajectory_coefficient_z[2],
                    (double)raw1.trajectory_coefficient_z[3],
                    (double)raw1.trajectory_coefficient_z[4],
                    (double)raw1.trajectory_coefficient_z[5],
                    (double)raw1.trajectory_coefficient_z[6]);
                for (int k=0 ; k<7 ;k++)
                {
                    _traject.traject[raw1.index_keyframe - 1][k][2] = raw1.trajectory_coefficient_z[k];
                }
				PX4_INFO("      yaw: [%.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
                    (double)raw1.trajectory_coefficient_r[0],
                    (double)raw1.trajectory_coefficient_r[1],
                    (double)raw1.trajectory_coefficient_r[2],
                    (double)raw1.trajectory_coefficient_r[3],
                    (double)raw1.trajectory_coefficient_r[4],
                    (double)raw1.trajectory_coefficient_r[5],
                    (double)raw1.trajectory_coefficient_r[6]);
                for (int k=0 ; k<7 ;k++)
                {
                    _traject.traject[raw1.index_keyframe - 1][k][3] = raw1.trajectory_coefficient_r[k];
                }
                /* check for receive_flagive traject done*/
                _traject.all_done = true;
                for (int j = 0; j<_traject.num_keyframe; j++)
                {
                    if(!_traject.receive_flag[j])
                    {
                        _traject.all_done = false;
                    }
                }
                if (_traject.all_done)
                    thread_should_exit = true;
			}
	    }
	}
    PX4_INFO(" t: %.2f ",(double)_traject.t[2]);
    get_pos_from_traject(_traject,_traject.t[0]);
    PX4_INFO(" pos_sp at %.2f s: %.2f %.2f %.2f %.2f",(double)_traject.t[0],(double)pos_sp[0],(double)pos_sp[1],(double)pos_sp[2],(double)pos_sp[3]);
    for (int j = 1; j<_traject.num_keyframe+1;j++)
    {
    get_pos_from_traject(_traject,_traject.t[j]);
    PX4_INFO(" pos_sp at %.2f s: %.2f %.2f %.2f %.2f",(double)_traject.t[j],(double)pos_sp[0],(double)pos_sp[1],(double)pos_sp[2],(double)pos_sp[3]);
    }
	warnx("[daemon] exiting.\n");
	thread_running = false;
	return 0;
}
