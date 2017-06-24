/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

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

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/ca_traject.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* subscribe to sensor_combined topic */
	//int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	int ca_traject_sub_fd = orb_subscribe(ORB_ID(ca_traject));

    /* traject struct */
    struct trajectory{
        bool receive_flag[10];
        float t[11];
        float traject[10][7][4];
        int num_keyframe;
        int traject_order;
        bool all_done;
    } _traject;

    memset(&_traject,0,sizeof(struct trajectory));

    for (int i = 0; i <10; i++) {
        _traject.receive_flag[i] = false;
    }
    _traject.all_done = false;
    _traject.num_keyframe = 10;
    
	/* limit the update rate to 5 Hz */
	//orb_set_interval(sensor_sub_fd, 200);
	orb_set_interval(ca_traject_sub_fd, 200);

	/* advertise attitude topic */
	//struct vehicle_attitude_s att;
	//memset(&att, 0, sizeof(att));
	//orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[1];
		fds[0].fd = ca_traject_sub_fd;
        fds[0].events = POLLIN;
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */

	int error_counter = 0;
	PX4_INFO("Start");

	for (int i = 0; i < 15 ; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
			PX4_INFO("test1");
		int poll_ret = px4_poll(&fds[0], 1, 100);
			PX4_INFO("test");

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");
			PX4_INFO("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				
                struct ca_traject_s raw1;
                orb_copy(ORB_ID(ca_traject), ca_traject_sub_fd, &raw1);
				PX4_INFO("Traject:\n  total: %d order+1: %d",
					 raw1.num_keyframe,
					 raw1.order_p_1);
				PX4_INFO("     phase %d :",raw1.index_keyframe);
                _traject.receive_flag[raw1.index_keyframe - 1] = true;
                _traject.num_keyframe = raw1.num_keyframe;
                _traject.traject_order = raw1.order_p_1 - 1;

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

                /* check for receive traject done*/
                _traject.all_done = true;
                for (int j = 0; j<_traject.num_keyframe; j++)
                {
                    if(!_traject.receive_flag[j])
                    {
                        _traject.all_done = false;
                    }
                }
				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
			}
		}
	}
    PX4_INFO("all traject receive done :\n phase: %d order: %d ",_traject.num_keyframe,_traject.traject_order);
    PX4_INFO("t: %.2f traject: %.2f",(double)_traject.t[0],(double)_traject.traject[0][1][3]);
    
	PX4_INFO("exiting");

	return 0;
}
