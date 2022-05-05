#include <ompl/base/spaces/SE3StateSpace.h>
#include "planner_data.hpp"
namespace test {

void VisualData::updateData(const ob::PlannerData& data)
{
    setGaol(Vertex2Point(data.getGoalVertex(0)));
    setStart(Vertex2Point(data.getStartVertex(0)));
    for (unsigned int i = 0; i < data.numVertices(); i++)
    {
      points.points.push_back(Vertex2Point(data.getVertex(i)));
    }
}
geometry_msgs::msg::Point VisualData::Vertex2Point(const ob::PlannerDataVertex& vertex)
{
    auto se3state = vertex.getState()->as<ob::SE3StateSpace::StateType>();
    geometry_msgs::msg::Point point;
    point.x = se3state->getX();
    point.y = se3state->getY();
    point.z = se3state->getZ();
    return point;
}
void VisualData::init()
{
    goal.header.frame_id = "/0";
    goal.ns = "goal";
    goal.id = 0;
    goal.type = visualization_msgs::msg::Marker::SPHERE;
    goal.action = visualization_msgs::msg::Marker::ADD;
    goal.scale.x = 0.3;
    goal.scale.y = 0.3;
    goal.scale.z = 0.3;
    goal.color.r = 0.0f;
    goal.color.g = 1.0f;
    goal.color.b = 1.0f;
    goal.color.a = 1.0f;

    start.header.frame_id = "/0";
    start.ns = "start";
    start.id = 1;
    start.type = visualization_msgs::msg::Marker::SPHERE;
    start.action = visualization_msgs::msg::Marker::ADD;
    start.scale.x = 0.3;
    start.scale.y = 0.3;
    start.scale.z = 0.3;
    start.color.r = 1.0f;
    start.color.g = 0.0f;
    start.color.b = 0.0f;
    start.color.a = 1.0f;

    points.header.frame_id = "/0";
    points.ns = "points";
    points.id = 2;
    points.type = visualization_msgs::msg::Marker::POINTS;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.scale.z = 0.1;
    points.color.r = 1.0f;
    points.color.g = 1.0f;
    points.color.b = 1.0f;
    points.color.a = 1.0f;

    lineList.header.frame_id = "/0";
    lineList.ns = "lineList";
    lineList.id = 3;
    lineList.type = visualization_msgs::msg::Marker::LINE_LIST;
    lineList.action = visualization_msgs::msg::Marker::ADD;
    lineList.scale.x = 0.1;
    lineList.scale.y = 0.1;
    lineList.scale.z = 0.1;
    lineList.color.r = 1.0f;
    lineList.color.g = 0.0f;
    lineList.color.b = 1.0f;
    lineList.color.a = 1.0f;
}
void VisualData::setGaol(const geometry_msgs::msg::Point& data)
{
    goal.pose.position.x = data.x;
    goal.pose.position.y = data.y;
    goal.pose.position.z = data.z;
}
void VisualData::setStart(const geometry_msgs::msg::Point& data)
{
    start.pose.position.x = data.x;
    start.pose.position.y = data.y;
    start.pose.position.z = data.z;
}

void VisualMsgPublisher::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Publishing");
    updateMarker();
    for (auto marker : markers_)
    {
        publisher_->publish(*(marker.get()));
    }
}


void VisualMsgPublisher::updateMarker()
{
    markers_.clear();
    markers_.push_back(std::make_shared<visualization_msgs::msg::Marker>(data_.goal));
    markers_.push_back(std::make_shared<visualization_msgs::msg::Marker>(data_.start));
    markers_.push_back(std::make_shared<visualization_msgs::msg::Marker>(data_.points));
    count_ += 0.04;
}
}