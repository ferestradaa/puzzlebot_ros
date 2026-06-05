// process_manager_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <signal.h>

#include <map>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <vector>

#include "puzzlebot_interfaces/action/launch_node.hpp"
#include "puzzlebot_interfaces/action/kill_node.hpp"

class ProcessManager : public rclcpp::Node {
public:
    using LaunchNode = puzzlebot_interfaces::action::LaunchNode;
    using KillNode = puzzlebot_interfaces::action::KillNode;

    ProcessManager() : Node("process_manager") {
        launch_server_ = rclcpp_action::create_server<LaunchNode>(
            this, "launch_node",
            [this](auto uuid, auto goal) { return handle_launch_goal(uuid, goal); },
            [this](auto handle) { return handle_launch_cancel(handle); },
            [this](auto handle) { handle_launch_accepted(handle); }
        );
        kill_server_ = rclcpp_action::create_server<KillNode>(
            this, "kill_node",
            [this](auto uuid, auto goal) { return handle_kill_goal(uuid, goal); },
            [this](auto handle) { return handle_kill_cancel(handle); },
            [this](auto handle) { handle_kill_accepted(handle); }
        );
        reap_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { reap_dead_processes(); }
        );
        RCLCPP_INFO(get_logger(), "ProcessManager ready");
    }

private:
    rclcpp_action::Server<LaunchNode>::SharedPtr launch_server_;
    rclcpp_action::Server<KillNode>::SharedPtr kill_server_;
    rclcpp::TimerBase::SharedPtr reap_timer_;
    std::map<std::string, pid_t> processes_;
    std::mutex processes_mutex_;

    void terminate_process(pid_t pid) {
        killpg(pid, SIGTERM);
        int waited = 0;
        while (waited < 2000) {
            if (waitpid(pid, nullptr, WNOHANG) == pid) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            waited += 50;
        }
        killpg(pid, SIGKILL);
        waitpid(pid, nullptr, 0);
    }

    void reap_dead_processes() {
        std::lock_guard<std::mutex> lock(processes_mutex_);
        for (auto it = processes_.begin(); it != processes_.end();) {
            int status;
            pid_t result = waitpid(it->second, &status, WNOHANG);
            if (result == it->second) {
                RCLCPP_WARN(get_logger(), "Process %s (pid %d) died on its own",
                            it->first.c_str(), it->second);
                it = processes_.erase(it);
            } else {
                ++it;
            }
        }
    }

    rclcpp_action::GoalResponse handle_launch_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const LaunchNode::Goal> goal)
    {
        std::lock_guard<std::mutex> lock(processes_mutex_);
        if (processes_.count(goal->executable)) {
            RCLCPP_WARN(get_logger(), "%s is running", goal->executable.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_launch_cancel(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<LaunchNode>>)
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    void handle_launch_accepted(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<LaunchNode>> handle)
    {
        std::thread([this, handle]() {
            const auto goal = handle->get_goal();
            auto result = std::make_shared<LaunchNode::Result>();

            pid_t pid = fork();
            if (pid == 0) {
                setpgid(0, 0);
                std::vector<const char*> argv;
                argv.push_back("ros2");
                argv.push_back("run");
                argv.push_back(goal->package.c_str());
                argv.push_back(goal->executable.c_str());
                for (const auto& arg : goal->args) {
                    argv.push_back(arg.c_str());
                }
                argv.push_back(nullptr);
                execvp("ros2", const_cast<char* const*>(argv.data()));
                _exit(1);  
            } else if (pid < 0) {
                result->success = false;
                result->message = "fork() failed";
                handle->abort(result);
                return;
            }

  
            setpgid(pid, pid);

            {
                std::lock_guard<std::mutex> lock(processes_mutex_);
                processes_[goal->executable] = pid;
            }
            RCLCPP_INFO(get_logger(), "Launched %s/%s pid=%d",
                        goal->package.c_str(), goal->executable.c_str(), pid);

            // si no se pidio esperar readiness, exito apenas arranca el proceso
            if (goal->ready_topic.empty()) {
                result->success = true;
                result->pid = pid;
                result->message = "launched";
                handle->succeed(result);
                return;
            }

            std::atomic<bool> node_ready{false};
            auto sub = create_subscription<std_msgs::msg::Bool>(
                goal->ready_topic, rclcpp::QoS(1).transient_local(),
                [&node_ready](std_msgs::msg::Bool::SharedPtr m){
                    if (m->data) node_ready = true;
                });

            auto fb = std::make_shared<LaunchNode::Feedback>();
            const double timeout = goal->ready_timeout_sec > 0.0f
                                       ? goal->ready_timeout_sec : 30.0;
            const auto start = now();

            while (rclcpp::ok() && !node_ready) {
                if ((now() - start).seconds() > timeout) {
                    RCLCPP_ERROR(get_logger(), "timeout waiting readiness de %s",
                                 goal->executable.c_str());
                    terminate_process(pid);
                    {
                        std::lock_guard<std::mutex> lock(processes_mutex_);
                        processes_.erase(goal->executable);
                    }
                    result->success = false;
                    result->pid = pid;
                    result->message = "timeout waiting readiness";
                    handle->abort(result);
                    return;
                }
                fb->status = "waiting readiness from nodo";
                handle->publish_feedback(fb);
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }

            RCLCPP_INFO(get_logger(), "%s rready", goal->executable.c_str());
            result->success = true;
            result->pid = pid;
            result->message = "ready";
            handle->succeed(result);
        }).detach();
    }

    // kill handlers
    rclcpp_action::GoalResponse handle_kill_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const KillNode::Goal> goal)
    {
        std::lock_guard<std::mutex> lock(processes_mutex_);
        if (!processes_.count(goal->executable)) {
            RCLCPP_WARN(get_logger(), "%s is not running", goal->executable.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_kill_cancel(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<KillNode>>)
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    void handle_kill_accepted(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<KillNode>> handle)
    {
        std::thread([this, handle]() {
            const auto goal = handle->get_goal();
            auto result = std::make_shared<KillNode::Result>();

            pid_t pid = -1;
            {
                std::lock_guard<std::mutex> lock(processes_mutex_);
                auto it = processes_.find(goal->executable);
                if (it != processes_.end()) {
                    pid = it->second;
                    processes_.erase(it);
                }
            }
            if (pid < 0) {
                result->success = false;
                result->message = "proceso not found";
                handle->abort(result);
                return;
            }

            terminate_process(pid);
            RCLCPP_INFO(get_logger(), "Killed %s (pid %d)", goal->executable.c_str(), pid);
            result->success = true;
            result->message = "killed";
            handle->succeed(result);
        }).detach();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProcessManager>());
    rclcpp::shutdown();
    return 0;
}