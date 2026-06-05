// process_manager_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <map>
#include <string>
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
                exit(1);
            } else if (pid < 0) {
                result->success = false;
                result->message = "fork() failed";
                handle->abort(result);
                return;
            }

            {
                std::lock_guard<std::mutex> lock(processes_mutex_);
                processes_[goal->executable] = pid;
            }

            RCLCPP_INFO(get_logger(), "Launched %s/%s pid=%d",
                        goal->package.c_str(), goal->executable.c_str(), pid);

            result->success = true;
            result->pid = pid;
            result->message = "launched";
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
                result->message = "proceso no encontrado";
                handle->abort(result);
                return;
            }

            kill(pid, SIGINT);
            waitpid(pid, nullptr, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

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