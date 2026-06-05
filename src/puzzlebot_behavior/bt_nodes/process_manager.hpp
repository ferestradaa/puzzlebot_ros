//process_manager.hpp
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "puzzlebot_interfaces/action/launch_node.hpp"
#include "puzzlebot_interfaces/action/kill_node.hpp"

namespace puzzlebot_bt {

template<typename ActionT>
class ProcessAction : public BT::StatefulActionNode {
public:
    using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

    ProcessAction(const std::string& name, const BT::NodeConfig& config,
                  rclcpp::Node::SharedPtr node, const std::string& action_name)
        : BT::StatefulActionNode(name, config), node_(node)
    {
        client_ = rclcpp_action::create_client<ActionT>(node_, action_name);
    }

    BT::NodeStatus onStart() override {
        done_ = false;
        succeeded_ = false;

        if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "process_manager not available");
            return BT::NodeStatus::FAILURE;
        }

        auto goal = buildGoal();
        auto opts = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
        opts.result_callback = [this](const typename GoalHandle::WrappedResult& result) {
            succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                          result.result->success);
            done_ = true;
        };

        client_->async_send_goal(goal, opts);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!done_) return BT::NodeStatus::RUNNING;
        return succeeded_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void onHalted() override {}

protected:
    rclcpp::Node::SharedPtr node_;
    std::atomic<bool> done_{false};
    std::atomic<bool> succeeded_{false};
    typename rclcpp_action::Client<ActionT>::SharedPtr client_;

    virtual typename ActionT::Goal buildGoal() = 0;
};


class LaunchNodeAction : public ProcessAction<puzzlebot_interfaces::action::LaunchNode> {
public:
    using Base = ProcessAction<puzzlebot_interfaces::action::LaunchNode>;

    LaunchNodeAction(const std::string& name, const BT::NodeConfig& config,
                     rclcpp::Node::SharedPtr node)
        : Base(name, config, node, "launch_node") {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("package"),
            BT::InputPort<std::string>("executable")
        };
    }

protected:
    puzzlebot_interfaces::action::LaunchNode::Goal buildGoal() override {
        puzzlebot_interfaces::action::LaunchNode::Goal goal;
        getInput("package", goal.package);
        getInput("executable", goal.executable);
        return goal;
    }
};


class KillNodeAction : public ProcessAction<puzzlebot_interfaces::action::KillNode> {
public:
    using Base = ProcessAction<puzzlebot_interfaces::action::KillNode>;

    KillNodeAction(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Node::SharedPtr node)
        : Base(name, config, node, "kill_node") {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("executable")
        };
    }

protected:
    puzzlebot_interfaces::action::KillNode::Goal buildGoal() override {
        puzzlebot_interfaces::action::KillNode::Goal goal;
        getInput("executable", goal.executable);
        return goal;
    }
};

} // namespace puzzlebot_bt