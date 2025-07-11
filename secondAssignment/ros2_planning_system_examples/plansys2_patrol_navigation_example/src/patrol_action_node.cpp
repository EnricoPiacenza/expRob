#include <memory>
#include <algorithm>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using namespace std::chrono_literals;

class PatrolAction : public plansys2::ActionExecutorClient
{
public:
  PatrolAction()
  : plansys2::ActionExecutorClient("patrol", 100ms)
  {
    progress_ = 0.0;
    client_ = create_client<std_srvs::srv::SetBool>("patrol_action");
    service_ = this->create_service<std_srvs::srv::SetBool>(
      "response_patrol", std::bind(&PatrolAction::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    status_ = 0;
  }

private:

  void service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data) {
      response->success = true;
      status_ = 2;
    } 
  }

  void do_work()
  {
    if(status_ == 0){
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      auto result = client_->async_send_request(request);
      status_ = 1;
    }else if(status_ == 1){
      status_ = 1;
    }else{
      RCLCPP_INFO(this->get_logger(), "Patrolling completed");
      finish(true, 1.0, "Patrolling completed");
      status_ = 0;
    }
  }
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  float progress_;
  int status_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "patrol"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}