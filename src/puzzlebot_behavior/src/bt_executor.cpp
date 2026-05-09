#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "dummy_nodes.hpp"



class BTexecutor : public  rclcpp::Node{
    public: 
        BTexecutor() : Node("bt_executor"){

            factory_.registerNodeType<GetTargetPose>("GetTargetPose");
            factory_.registerNodeType<GoToTarget>("GoToTarget");

            std::string pkg_path = ament_index_cpp::get_package_share_directory("puzzlebot_behavior"); 
            YAML::Node config = YAML::LoadFile(pkg_path + "/config/config.yaml"); 

            std::string relative_tree_path = config["path_trees"]["main_tree"].as<std::string>();

            std::string xml_path = pkg_path + "/" + relative_tree_path;
            tree_ = factory_.createTreeFromFile(xml_path); 
            
            logger_ = std::make_shared<BT::StdCoutLogger>(tree_); 

            timer_ = this -> create_wall_timer(std::chrono::milliseconds(50),
                     std::bind(&BTexecutor::tickTree, this)); 
            
            RCLCPP_INFO(this -> get_logger(), "BT successfully initialized!"); 
        }


    private: 
        void tickTree(){
            auto status = tree_.tickOnce(); 
            if (status == BT::NodeStatus::SUCCESS){
                RCLCPP_INFO(this->get_logger(), "Tree finished with SUCCESS");
                timer_ -> cancel(); 
            } else if (status == BT::NodeStatus::FAILURE){
                RCLCPP_WARN(this->get_logger(), "Tree finished with FAILURE");
                timer_ -> cancel();
            }
        }

        BT::BehaviorTreeFactory factory_; 
        BT::Tree tree_; 
        std::shared_ptr<BT::StdCoutLogger> logger_; 
        rclcpp::TimerBase::SharedPtr timer_; 


};


int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BTexecutor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}