#include <task_handle/task_handle.h>

#include <boost/thread.hpp>
#include <stack>

#include <geometry_msgs/Twist.h>

#include <std_msgs/String.h>
#include <tf/tf.h>

TaskHandle::TaskHandle():
  detect_obstacle_(false),
  ac_(NULL),
  observe_thread_(NULL),
  gt_ptr_(NULL),
  enable_observe_(false)
 {
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
  //ac_要绑定到move_base服务，这个初始化要注意啊！！！
  ac_ = new Client("move_base", true);

  private_nh.param("wait_duration", wait_duration_, 5.0);

  status_sub_ = nh.subscribe("detec_obs", 2, &TaskHandle::statusCB, this);
  task_sub_ = nh.subscribe("goal_id", 1, &TaskHandle::taskCB, this);

  observe_thread_ = new boost::thread(boost::bind(&TaskHandle::observeThread, this));
  run_thread_ = new boost::thread(boost::bind(&TaskHandle::runThread, this));
  
  gt_ptr_.reset(new GoalTable);

  geometry_msgs::Pose g_1;
  g_1.position.x = 1.3;
  g_1.position.y = 2.2;
  g_1.position.z = 0;

  g_1.orientation.x = 0.;
  g_1.orientation.y = 0.;
  g_1.orientation.z = 0.;
  g_1.orientation.w = 1.;

  geometry_msgs::Pose g_2;
  g_2.position.x = -0.88;
  g_2.position.y = -0.6;
  g_2.position.z = 0;

  g_2.orientation.x = 0.;
  g_2.orientation.y = 0.;
  g_2.orientation.z = 0.;
  g_2.orientation.w = 1.;

  gt_ptr_->addItem("T1", g_1);
  gt_ptr_->addItem("T2", g_2);

  std::cout << "construct obj TaskHandle" << std::endl;

}

TaskHandle::~TaskHandle()
{
  if(ac_ != NULL)
	delete ac_;
  
  observe_thread_->interrupt();
  observe_thread_->join();
  delete observe_thread_;

  run_thread_->interrupt();
  run_thread_->join();
  delete run_thread_;
}

void TaskHandle::statusCB(std_msgs::String::ConstPtr input)
{
    boost::unique_lock<boost::recursive_mutex> lock(status_mutex_);
    if(input->data == "true")
	{
        detect_obstacle_ = true;
        observe_cond_.notify_one();
	}else{
        detect_obstacle_ = false;
        run_cond_.notify_one();
	}
}

void TaskHandle::taskCB(std_msgs::String::ConstPtr input)
{
    ROS_INFO("recv taskCB msg");
    if(ac_->waitForServer())
    {
        ROS_INFO("ac_ now is ready");
    }
	move_base_msgs::MoveBaseGoal goal;

	//Use the map frame to define goal poses
	goal.target_pose.header.frame_id = "map";
	//Set the time stamp to "now"
	goal.target_pose.header.stamp = ros::Time::now();
	//Set the goal pose to the i-th waypoint
	goal.target_pose.pose = gt_ptr_->getPose(input->data);

	//Start the robot moving toward the goal
    // ac_->sendGoal(goal, Client::SimpleDoneCallback(), Client::SimpleActiveCallback(), Client::SimpleFeedbackCallback());
    if(ac_->isServerConnected())
    {
        ROS_INFO("it is ready to send goal.");
        ac_->sendGoal(goal, Client::SimpleDoneCallback(), Client::SimpleActiveCallback(), Client::SimpleFeedbackCallback());
    }

	boost::unique_lock<boost::recursive_mutex> lock(status_mutex_);
	new_goal_ = true;
	cur_goal_ = goal;
    lock.unlock();

}

void TaskHandle::observeThread()
{
	ros::NodeHandle n;
    ros::Rate r(20.0);
    boost::unique_lock<boost::recursive_mutex> lock(status_mutex_);
	while(n.ok())
    {
        ROS_INFO("observe thread is working.");
        while(!detect_obstacle_)
		{
            observe_cond_.wait(lock);
            ROS_INFO("observe thread is suspending.");
            mark_time_ = ros::Time::now();
		}

        lock.unlock();


        if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("goal reached.");
        }else if(ac_->getState() == actionlib::SimpleClientGoalState::ACTIVE){

            ac_->cancelGoal();
            goal_stack_.push(cur_goal_);
        }

        //lock for next iteration
        lock.lock();
        r.sleep();
	}

}

void TaskHandle::runThread()
{
    ros::NodeHandle n;
    ros::Rate r(20.0);
    boost::unique_lock<boost::recursive_mutex> lock(status_mutex_);
    while(n.ok())
    {
        ROS_INFO("run thread is working.");
        while(detect_obstacle_)
        {
            run_cond_.wait(lock);
            ROS_INFO("run thread is suspending.");
//            mark_time_ = ros::Time::now();
        }

        lock.unlock();

        if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("goal reached.");

        }else{
            if(ros::Time::now() > mark_time_ + ros::Duration(wait_duration_))
            {
                if(!goal_stack_.empty())
                {
                    goal_stack_.pop();
                }

            }else{
                if(!goal_stack_.empty())
                {
                    cur_goal_ = goal_stack_.top();
                    goal_stack_.pop();
                    ac_->sendGoal(cur_goal_, Client::SimpleDoneCallback(), Client::SimpleActiveCallback(), Client::SimpleFeedbackCallback());
                }

            }
        }

        lock.lock();
        r.sleep();
    }
}

//void TaskHandle::activeCb()
//{
//    ROS_INFO("Client is active.");
//}

//void TaskHandle::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
//{
//    ROS_INFO("feedback callback.");

//}
