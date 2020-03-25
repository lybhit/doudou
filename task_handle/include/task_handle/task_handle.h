#ifndef TASK_HANDLE_ACTION_H_
#define TASK_HANDLE_ACTION_H_

#include <ros/ros.h>
#include <stack>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <boost/shared_ptr.hpp>

#include <std_msgs/String.h>
#include "goal_table.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;


class TaskHandle{
  public:
    TaskHandle();
    ~TaskHandle();

    //订阅机器人状态参数
    void statusCB(std_msgs::String::ConstPtr input);

    //订阅任务调度，点的数据可以从数据库中读取或者其他方式，通过id对应
    void taskCB(std_msgs::String::ConstPtr input);

    //监听线程
    void observeThread();

    //执行线程
    void runThread();


  private:
//     void activeCb();
//     void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    Client* ac_;

    ros::Subscriber status_sub_, task_sub_;

    bool detect_obstacle_;

    //点表
    boost::shared_ptr<GoalTable> gt_ptr_; 

    std::stack<move_base_msgs::MoveBaseGoal> goal_stack_;

    move_base_msgs::MoveBaseGoal cur_goal_;

    double wait_duration_;

    boost::recursive_mutex status_mutex_;
    boost::thread* observe_thread_;
    boost::thread* run_thread_;
    boost::condition_variable_any observe_cond_, run_cond_;

    bool enable_observe_;
    bool new_goal_;

    ros::Time mark_time_;
};

#endif
