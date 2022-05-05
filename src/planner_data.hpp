#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>

#include <visualization_msgs/msg/marker.hpp>
#include "rclcpp/rclcpp.hpp"

#include <boost/graph/astar_search.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace test {
class VisualData
{
public:
  VisualData() { init(); }
  ~VisualData() {}

  void init();
  void setGaol(const geometry_msgs::msg::Point& data);
  void setStart(const geometry_msgs::msg::Point& data);
  geometry_msgs::msg::Point Vertex2Point(const ob::PlannerDataVertex& vertex);
  void updateData(const ob::PlannerData& data);
  visualization_msgs::msg::Marker goal;
  visualization_msgs::msg::Marker start;
  visualization_msgs::msg::Marker points;
  visualization_msgs::msg::Marker lineList;
};
class VisualMsgPublisher : public rclcpp::Node
{
public:
  VisualMsgPublisher(const VisualData& data) : Node("my_publisher"), count_(0.0), data_(data)
  {
      publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("topic", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&VisualMsgPublisher::timer_callback, this));
  }

private:
  void updateMarker();
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  float count_;
  std::vector<visualization_msgs::msg::Marker::SharedPtr> markers_;
  VisualData data_;

};

// void readPlannerData();
}





