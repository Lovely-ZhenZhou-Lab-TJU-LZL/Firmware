/*************************************************************************
	> File Name: whycon_observer_main.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年03月22日 星期三 16时17分59秒
 ************************************************************************/

#include "whycon_observer.h"
#include <systemlib/mavlink_log.h>
namespace whycon_observer
{
    WhyconObserver     *g_control;
}

WhyconObserver::WhyconObserver() :
    _task_should_exit(false),
    _control_ob_task(-1),
    _mavlink_log_pub(nullptr),
    _target_pos_pub(nullptr),
    
    /*subscriptions*/
    _whycon_target_sub(-1),
    _control_state_sub(-1),
    _local_pos_sub(-1),
    _whycon_target{},
    _ctrl_state{},
    _local_pos{},
    _target_pos_message{}
{
    //TODO: init 
	_target_pos_body.zero();
	_target_pos_ground.zero();
	_delta_ob_frame(X_x) = 0.02;/* delta x*/
	_delta_ob_frame(X_y) = 0.000;/* delta y*/
	_delta_ob_frame(X_z) = 0.05;/* delta z*/
}

WhyconObserver::~WhyconObserver()
{
    if (_control_ob_task != -1){
        _task_should_exit = true;
        unsigned i = 0;
        do {
            usleep(20000);
                if(++i > 50){
                    px4_task_delete(_control_ob_task);
                    break;
                }
        } while (_control_ob_task != -1);
    }
    whycon_observer::g_control = nullptr;
}
void
WhyconObserver::status_reset()
{
//TODO: status reset
}
void
WhyconObserver::wo_frame_tf(float x,float y,float z,float local_x,float local_y,float local_z)
{
 /* get the current attitude */
math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
math::Matrix<3, 3> R = q_att.to_dcm();
math::Vector<3> wt_pos_b(-y+_delta_ob_frame(X_x),x+_delta_ob_frame(X_y),z+_delta_ob_frame(X_z));/*rotation and transfor*/
math::Vector<3> wt_pos_g = R*wt_pos_b; //q_att.conjugate(wt_pos_b);
_target_pos_ground(X_x) = wt_pos_g(X_x) + local_x;
_target_pos_ground(X_y) = wt_pos_g(X_y) + local_y;
_target_pos_ground(X_z) = wt_pos_g(X_z) + local_z;
}
void
WhyconObserver::poll_subscriptions()
{
    bool updated;
    //orb_check(xxxx_sub, &updated);
    //if (updated) {
    //orb_copy(ORB_ID(xxxx), _xxxx_sub, &_xxxxx);
    //}
    orb_check(_whycon_target_sub, &updated);
    if (updated){
    orb_copy(ORB_ID(whycon_target), _whycon_target_sub, &_whycon_target);
    }
    orb_check(_control_state_sub, &updated);
    if (updated){
    orb_copy(ORB_ID(control_state), _control_state_sub, &_ctrl_state);
    }
    orb_check(_local_pos_sub, &updated);
    if (updated){
    orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
    }
}

void
WhyconObserver::task_main_trampoline(int argc, char *argv[])
{
    whycon_observer::g_control->task_main();
}

void
WhyconObserver::task_main()
{
    //TODO subscriptions
    //_xxxx_sub = orb_subscribe(ORB_ID(xxxxx));
    _whycon_target_sub = orb_subscribe(ORB_ID(whycon_target));
    _control_state_sub = orb_subscribe(ORB_ID(control_state));
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    poll_subscriptions();
    px4_pollfd_struct_t fds[1];
    fds[0].fd = _whycon_target_sub;
    fds[0].events = POLLIN;
    hrt_abstime t_prev = 0;
    while (!_task_should_exit){
        int pret = px4_poll(&fds[0],(sizeof(fds) / sizeof(fds[0])), 500);
        hrt_abstime t = hrt_absolute_time();
        	 /* timed out - periodic check for _task_should_exit */
		 if (pret == 0) {
		 	// printf("poll nullptr\n");
		 	status_reset();
		 	continue;
			 }

		 /* this is undesirable but not much we can do */
		 if (pret < 0) {
		 	warn("poll error %d, %d", pret, errno);
		 	continue;
			 }
		//	 poll_subscriptions();
		float dt = t_prev != 0 ? (t - t_prev) * 0.001f : 0.0f; //ms
        poll_subscriptions();

        //PX4_INFO("dt     :  %.2f",(double)dt);
        //PX4_INFO("id     :  %d",_whycon_target.id);
        //PX4_INFO("pos_b  : [%.2f, %.2f, %.2f]",(double)_whycon_target.x,(double)_whycon_target.y,(double)_whycon_target.z);
        //PX4_INFO("q      : [%.2f, %.2f, %.2f, %.2f]",(double)_whycon_target.q[0],(double)_whycon_target.q[1],(double)_whycon_target.q[2],(double)_whycon_target.q[3]);
		_target_pos_body(X_x) = _whycon_target.x;
		_target_pos_body(X_y) = _whycon_target.y;
		_target_pos_body(X_z) = _whycon_target.z;
		 /* transfor the coordinate frame */
		wo_frame_tf(_whycon_target.x,_whycon_target.y,_whycon_target.z,_local_pos.x,_local_pos.y,_local_pos.z);
		//PX4_INFO("pos_g  : [%.2f, %.2f, %.2f]",(double)_target_pos_ground(X_x),(double)_target_pos_ground(X_y),(double)_target_pos_ground(X_z));
		_target_pos_message.timestamp = hrt_absolute_time();
		_target_pos_message.x = _target_pos_ground(X_x);
		_target_pos_message.y = _target_pos_ground(X_y);
		_target_pos_message.z = _target_pos_ground(X_z);
		_target_pos_message.id = _whycon_target.id;
		_target_pos_message.timestamp_received = _whycon_target.timestamp_received;
		_target_pos_message.dt = dt;
		_target_pos_message.q[0] = _whycon_target.q[0];
		_target_pos_message.q[1] = _whycon_target.q[1];
		_target_pos_message.q[2] = _whycon_target.q[2];
		_target_pos_message.q[3] = _whycon_target.q[3];		
		_target_pos_message.body_q[0] = _ctrl_state.q[0];
		_target_pos_message.body_q[1] = _ctrl_state.q[1];
		_target_pos_message.body_q[2] = _ctrl_state.q[2];
		_target_pos_message.body_q[3] = _ctrl_state.q[3];
		    if (_target_pos_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_wt_message), _target_pos_pub, &_target_pos_message);

			} else {
				_target_pos_pub = orb_advertise(ORB_ID(vehicle_wt_message), &_target_pos_message);
			}
        t_prev = t;
    }
    mavlink_log_info(&_mavlink_log_pub, "[wct] stopped");
    _control_ob_task = -1;
}

int
WhyconObserver::start()
{
	ASSERT(_control_ob_task == -1);

	/* start the task */
	_control_ob_task = px4_task_spawn_cmd("whycon_ob",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_DEFAULT,
					   10240,
					   (px4_main_t)&WhyconObserver::task_main_trampoline,
					   nullptr);

	if (_control_ob_task < 0) {
		warn("whycon_ob start failed");
		return -errno;
	}
	return OK;
}

int whycon_observer_main(int argc, char *argv[])
{
if (argc < 2) {
		warnx("usage: whycon_observer {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (whycon_observer::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		whycon_observer::g_control = new WhyconObserver;

		if (whycon_observer::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != whycon_observer::g_control->start()) {
			delete whycon_observer::g_control;
			whycon_observer::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (whycon_observer::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete whycon_observer::g_control;
		whycon_observer::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (whycon_observer::g_control) {
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
