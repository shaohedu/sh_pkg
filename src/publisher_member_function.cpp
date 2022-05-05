#include "rclcpp/rclcpp.hpp"
#include "planner_data.hpp"

int main(int argc, char** argv)
{
  auto space(std::make_shared<ob::SE3StateSpace>());
  auto si(std::make_shared<ob::SpaceInformation>(space));
  ob::PlannerDataStorage dataStorage;
  ob::PlannerData data(si);
  dataStorage.load("/home/dsh/dev_ws/myPlannerData", data);

  test::VisualData visualData;
  visualData.updateData(data);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<test::VisualMsgPublisher>(visualData));
  rclcpp::shutdown();
  return 0;
}
