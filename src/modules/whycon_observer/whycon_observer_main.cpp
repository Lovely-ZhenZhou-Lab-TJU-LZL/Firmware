/*************************************************************************
	> File Name: whycon_observer_main.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年03月22日 星期三 16时17分59秒
 ************************************************************************/

#include "whycon_observer.h"
namespace whycon_observer
{
    WhyconObserver     *g_control;
}

WhyconObserver::WhyconObserver() :
    _task_should_exit(false),
    _control_ob_task(-1)
{
    //TODO: init 
}

WhyconObserver::~WhyconObserver()
{
    if (_control_ob_task != -1){
        _task_should_exit = true;
        unsigned i = 0;
        do {
            usleep(20000)
                if(++i > 50){
                    px4_task_delete(_control_ob_task);
                    break;
                }
        } while (_control_ob_task != -1)
    }
    whycon_observer::g_control = nullptr;
}

void
WhyconObserver::poll_subscriptions()
{
    bool updated;
    //orb_check(xxxx_sub, &updated);
    //if (updated) {
    //orb_copy(ORB_ID(xxxx), _xxxx_sub, &_xxxxx);
    //}
}
void
WhyconObserver::task_main_trampoline(int argc, char *argv[])
{
    whycon_observer::g_control->task_main();
}
void
task_main()
{
    //TODO subscriptions
    //_xxxx_sub = orb_subscribe(ORB_ID(xxxxx));
}
