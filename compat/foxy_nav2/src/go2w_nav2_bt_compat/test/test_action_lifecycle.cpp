#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>
#include "gtest/gtest.h"
#include "nav2_msgs/action/wait.hpp"
#include "go2w_nav2_bt_compat/bt_action_node.hpp"

using namespace std::chrono_literals;
using Wait = nav2_msgs::action::Wait;
using ServerHandle = rclcpp_action::ServerGoalHandle<Wait>;

class TestAction : public go2w_foxy_bt::BtActionNode<Wait>
{
public:
  explicit TestAction(BT::NodeConfiguration config)
  : BtActionNode("WaitTest", "/acceptance/bt_wait", config) {}
  int request{2}, observed_result{0};
  void on_tick() override {goal_.time.sec = request;}
  BT::NodeStatus on_success() override
  {
    observed_result = result_.result->total_elapsed_time.sec;
    return BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus on_cancelled() override {return BT::NodeStatus::FAILURE;}
};

class ActionLifecycle : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_node = std::make_shared<rclcpp::Node>("acceptance_bt_wait_server");
    node = std::make_shared<rclcpp::Node>("bt_action_test");
    server = rclcpp_action::create_server<Wait>(server_node, "/acceptance/bt_wait",
      [this](const auto &, const std::shared_ptr<const Wait::Goal>) {
        std::this_thread::sleep_for(std::chrono::milliseconds(accept_delay_ms.load()));
        ++accepted;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<ServerHandle>) {
        ++canceled; return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<ServerHandle> handle) {
        std::lock_guard<std::mutex> lock(workers_mutex);
        workers.emplace_back([handle]() {
          // Goal 1 deliberately returns its canceled result while goal 2 is
          // still pending, exercising callback ownership across halt/restart.
          const auto until = std::chrono::steady_clock::now()+
            (handle->get_goal()->time.sec == 1 ? 150ms : 250ms);
          while (rclcpp::ok() && std::chrono::steady_clock::now()<until) {
            std::this_thread::sleep_for(2ms);
          }
          auto result = std::make_shared<Wait::Result>();
          result->total_elapsed_time.sec = handle->get_goal()->time.sec;
          if (handle->is_canceling()) {handle->canceled(result);} else {handle->succeed(result);}
        });
      });
    server_io = std::make_unique<go2w_foxy_bt::ActionIO>(server_node);
  }
  void TearDown() override
  {
    action.reset();
    // Finish any in-progress accept callback before freezing the worker list.
    server_io.reset();
    {std::lock_guard<std::mutex> lock(workers_mutex);
      for (auto & worker : workers) {if (worker.joinable()) {worker.join();}}}
    server.reset(); server_node.reset(); node.reset();
  }
  void create(int timeout_ms, const std::string & xml_timeout="")
  {
    BT::NodeConfiguration config;config.blackboard = BT::Blackboard::create();
    config.blackboard->set("node", node);
    config.blackboard->set("server_timeout", std::chrono::milliseconds(timeout_ms));
    config.blackboard->set("number_recoveries", 0);
    if (!xml_timeout.empty()) {config.input_ports["server_timeout"] = xml_timeout;}
    action = std::make_unique<TestAction>(config);
  }
  BT::NodeStatus run_until_terminal(int timeout_ms=1000)
  {
    const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
    auto status=BT::NodeStatus::RUNNING;
    while (std::chrono::steady_clock::now()<end) {
      status=action->executeTick();
      if (status!=BT::NodeStatus::RUNNING) {break;}
      std::this_thread::sleep_for(2ms);
    }
    return status;
  }
  void expect_cancel_without_more_ticks()
  {
    const auto end=std::chrono::steady_clock::now()+500ms;
    while (!canceled && std::chrono::steady_clock::now()<end) {std::this_thread::sleep_for(2ms);}
    EXPECT_EQ(accepted.load(),1); EXPECT_EQ(canceled.load(),1);
  }
  std::atomic<int> accept_delay_ms{0}, accepted{0}, canceled{0};
  std::mutex workers_mutex;
  std::vector<std::thread> workers;
  rclcpp::Node::SharedPtr node, server_node;
  rclcpp_action::Server<Wait>::SharedPtr server;
  std::unique_ptr<go2w_foxy_bt::ActionIO> server_io;
  std::unique_ptr<TestAction> action;
};

TEST_F(ActionLifecycle, timeout_cancels_a_late_acceptance_without_more_ticks)
{
  accept_delay_ms=80; create(20);
  EXPECT_EQ(run_until_terminal(),BT::NodeStatus::FAILURE);
  action->halt();
  expect_cancel_without_more_ticks();
}

TEST_F(ActionLifecycle, halt_before_acceptance_is_not_lost)
{
  accept_delay_ms=80; create(500);
  EXPECT_EQ(action->executeTick(),BT::NodeStatus::RUNNING);
  action->halt();
  expect_cancel_without_more_ticks();
}

TEST_F(ActionLifecycle, xml_port_overrides_native_foxy_blackboard_timeout)
{
  accept_delay_ms=80; create(10,"200");
  EXPECT_EQ(run_until_terminal(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(accepted.load(),1); EXPECT_EQ(canceled.load(),0);
}

TEST_F(ActionLifecycle, late_canceled_result_cannot_complete_the_next_goal)
{
  create(500);action->request=1;
  ASSERT_EQ(action->executeTick(),BT::NodeStatus::RUNNING);
  const auto end=std::chrono::steady_clock::now()+500ms;
  while (!accepted && std::chrono::steady_clock::now()<end) {std::this_thread::sleep_for(2ms);}
  ASSERT_EQ(accepted.load(),1);
  action->halt(); action->request=2;
  EXPECT_EQ(run_until_terminal(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action->observed_result,2);
  EXPECT_EQ(accepted.load(),2);
}

int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);::testing::InitGoogleTest(&argc,argv);
  const int result=RUN_ALL_TESTS();rclcpp::shutdown();return result;
}
