#ifndef GOAL_TABLE_H_
#define GOAL_TABLE_H_

#include <iostream>
#include <string>
#include <map>
#include <geometry_msgs/Pose.h>

class GoalTable
{
public:
	GoalTable(){}
	
	~GoalTable(){}

	geometry_msgs::Pose getPose(const std::string& id);

	void addItem(std::string id, geometry_msgs::Pose val);

private:

	std::map<std::string, geometry_msgs::Pose> goal_map_;

};

geometry_msgs::Pose GoalTable::getPose(const std::string& id)
{
	return goal_map_[id];
}

void GoalTable::addItem(std::string id, geometry_msgs::Pose val)
{
	goal_map_[id] = val;
}

#endif