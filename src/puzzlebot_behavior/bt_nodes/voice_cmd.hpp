#pragma once
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <atomic>
#include <memory>
#include <string>

namespace puzzlebot_bt {

// Convencion: arranca=1, recoge=2, base=3, entrega=4

// Una sola suscripcion a /voice_command compartida por los nodos de voz.
// El nodo de voz publica el numero (Int32). consume() lee y limpia (0 = vacio).
class VoiceCommandListener {
public:
    explicit VoiceCommandListener(rclcpp::Node::SharedPtr node) : last_(0) {
        sub_ = node->create_subscription<std_msgs::msg::Int32>(
            "/voice_command", 10,
            [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
                last_ = msg->data;
            });
    }

    int consume() { return last_.exchange(0); }

private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    std::atomic<int> last_;
};

// Bloquea hasta recibir el comando esperado. Otro comando se ignora.
class WaitForVoiceCommand : public BT::StatefulActionNode {
public:
    WaitForVoiceCommand(const std::string& name, const BT::NodeConfig& config,
                        std::shared_ptr<VoiceCommandListener> voice)
        : BT::StatefulActionNode(name, config), voice_(voice) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("expected_cmd") };
    }

    BT::NodeStatus onStart() override {
        auto res = getInput<int>("expected_cmd");
        if (!res) return BT::NodeStatus::FAILURE;
        expected_ = res.value();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        int cmd = voice_->consume();
        if (cmd == expected_) return BT::NodeStatus::SUCCESS;
        return BT::NodeStatus::RUNNING;  // comando distinto o vacio: se ignora
    }

    void onHalted() override {}

private:
    std::shared_ptr<VoiceCommandListener> voice_;
    int expected_ = -1;
};

// Guard de interrupcion. recoge(2) -> inspect, base(3) -> center.
// Si apunta a otra zona: cambia search_zone y FAILURE (interrumpe la busqueda).
// Misma zona o sin comando: SUCCESS (sigue buscando).
class CheckSearchZoneInterrupt : public BT::ConditionNode {
public:
    CheckSearchZoneInterrupt(const std::string& name, const BT::NodeConfig& config,
                             std::shared_ptr<VoiceCommandListener> voice)
        : BT::ConditionNode(name, config), voice_(voice) {}

    static BT::PortsList providedPorts() {
        return { BT::BidirectionalPort<std::string>("search_zone") };
    }

    BT::NodeStatus tick() override {
        int cmd = voice_->consume();
        std::string wanted;
        if (cmd == 2) wanted = "inspect";
        else if (cmd == 3) wanted = "center";
        else return BT::NodeStatus::SUCCESS;  // sin comando de zona

        auto current = getInput<std::string>("search_zone");
        if (current && current.value() == wanted) {
            return BT::NodeStatus::SUCCESS;   // misma zona: ignora
        }
        setOutput("search_zone", wanted);
        return BT::NodeStatus::FAILURE;        // interrumpe la busqueda actual
    }

private:
    std::shared_ptr<VoiceCommandListener> voice_;
};

}  // namespace puzzlebot_bt