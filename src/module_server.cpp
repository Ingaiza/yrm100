#include <stdio.h>
#include <iostream>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_rfid_interfaces/action/module_worker.hpp"
#include "/home/ingaiza/yrm_module/src/yrm100/include/yrm100/uhf_module.hpp"
#include "/home/ingaiza/yrm_module/src/yrm100/include/yrm100/uhf_tag.hpp"
#include "/home/ingaiza/yrm_module/src/yrm100/include/yrm100/uhf_functions.hpp"

using ModuleWorker = my_rfid_interfaces::action::ModuleWorker;
using ModuleWorkerGoalHandle = rclcpp_action::ServerGoalHandle<ModuleWorker>;
using namespace std::placeholders;

const size_t EPC_SIZE = 12;
const size_t READ_WRITE_SIZE = 16;

class ModuleServerNode : public rclcpp::Node 
{

public:
    ModuleServerNode() : Node("module_server") 
    {   
        cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        module_worker_server_ = rclcpp_action::create_server<ModuleWorker>(
            this,
            "module_worker",
            std::bind(&ModuleServerNode::goal_callback, this , _1,_2),
            std::bind(&ModuleServerNode::cancel_callback, this, _1),
            std::bind(&ModuleServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group 
        );
        RCLCPP_INFO(this->get_logger(), "Action Server has Started");
    }

private:

    rclcpp_action::GoalResponse goal_callback (
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ModuleWorker::Goal> goal)
    {   
        (void)uuid;
        
        {
            std::lock_guard<std::mutex>lock(mutex_);
            if(goal_handle_)
            {
                if(goal_handle_->is_active())
                {   
                    RCLCPP_ERROR(this->get_logger(), "A goal is still active, rejecting new goal");
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
        }
        // Condition to REJECT GOAL
        if(goal->action != "read" && goal->action != "write" && goal->action != "single" && goal->action != "multi")
        {
            RCLCPP_INFO(this->get_logger(), "Rejecting Goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        RCLCPP_INFO(this->get_logger(), "Accepting Goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<ModuleWorkerGoalHandle> goal_handle)
    {
        (void)goal_handle;
        // you can check a condition before cancelling a goal
        RCLCPP_INFO(this->get_logger(), "Canceling the goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback
    (const std::shared_ptr<ModuleWorkerGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing Goal");
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<ModuleWorkerGoalHandle> goal_handle)
    {
        {
            std::lock_guard<std::mutex>lock(mutex_);
            this-> goal_handle_ = goal_handle;
        }
        
        std::string action = goal_handle->get_goal()->action;

        if(action == "read")
        {
            auto result = std::make_shared<ModuleWorker::Result>();
            auto feedback = std::make_shared<ModuleWorker::Feedback>();
            
            uint8_t data[16];
            uint8_t* epc;
            try
            {
                epc = read_select(data);
                std::copy(epc, epc + EPC_SIZE,result->read_epc.begin());
                std::copy(data, data + READ_WRITE_SIZE,result->read_data.begin());
                result->response = true;
                feedback->progress = "Read Operation Success";
                goal_handle->publish_feedback(feedback);
                goal_handle->succeed(result);


            }
            catch(const std::exception& e)
            {
                result->response = false;
                feedback->progress = "Read Operation Failed";
                goal_handle->publish_feedback(feedback);
                goal_handle->abort(result);
            }
            
          
        }
        else if(action == "write")
        {
            auto result = std::make_shared<ModuleWorker::Result>();
            auto feedback = std::make_shared<ModuleWorker::Feedback>();
            uint8_t* epc;
            uint8_t data[16];
            auto goal_data = goal_handle->get_goal()->write_data;
            std::copy(goal_data.begin(),goal_data.end(),data);
            try
            {
                epc = write_tag(data);
                std::copy(epc, epc + EPC_SIZE,result->write_epc.begin());
                result->response = true;
                feedback->progress = "Write Success";
                goal_handle->publish_feedback(feedback);
                goal_handle->succeed(result);
            }
            catch(const std::exception& e)
            {
                result->response = false;
                feedback->progress = "Write Failed";
                goal_handle->publish_feedback(feedback);
                goal_handle->abort(result);

            }

        }
        else if(action == "single")
        {
            auto result = std::make_shared<ModuleWorker::Result>();
            auto feedback = std::make_shared<ModuleWorker::Feedback>();
            uint8_t* epc;
            try
            {
                epc = single_poll();
                // assert(epc != nullptr); 
                if(epc)
                {
                    std::copy(epc, epc + EPC_SIZE,result->single_inventory_epc.begin());
                    result->response = true;
                    feedback->progress = "Single Inventory Success";
                    goal_handle->publish_feedback(feedback);
                    goal_handle->succeed(result);
                }
                else
                {
                    result->response = false;
                    feedback->progress = "Single Inventory Failed";
                    goal_handle->publish_feedback(feedback);
                    goal_handle->succeed(result);
                }
                
            }
            catch(const std::exception& e)
            {
                result->response = false;
                feedback->progress = "Single Poll Failed";
                goal_handle->publish_feedback(feedback);
                goal_handle->abort(result);

            }
        }
        else
        {
            auto result = std::make_shared<ModuleWorker::Result>();
            auto feedback = std::make_shared<ModuleWorker::Feedback>();
            std::vector<uint8_t> epc;
            // size_t* epc_size;
            try
            {
                auto option = multi_poll();
                try
                {
                    epc = option.value();
                    // assert(epc != nullptr);
                    // assert(epc_size != nullptr);  
                    std::cout<<"multi_poll completed successfully"<<std::endl;
                    std::cout<<"result size: "<<epc.size()<<std::endl;
                    result->multi_epc_size = static_cast<int>(epc.size());
                    std::copy(epc.begin(), epc.end(),result->multi_inventory_epc.begin());
                    result->response = true;
                    feedback->progress = "Multi Inventory Success";
                    goal_handle->publish_feedback(feedback);
                    goal_handle->succeed(result);
                }
                catch(const std::bad_optional_access& e)
                {
                    std::cerr <<"multi_poll failed with error: "<<e.what()<<std::endl;
                    throw;
                }
            }
            catch(const std::exception& e)
            {
                result->response = false;
                feedback->progress = "Multi Poll Failed";
                goal_handle->publish_feedback(feedback);
                goal_handle->abort(result);

            }
        }

    }

    rclcpp_action::Server<ModuleWorker>::SharedPtr module_worker_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group;
    std::shared_ptr<ModuleWorkerGoalHandle> goal_handle_;
    std::mutex mutex_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModuleServerNode>(); 
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node);
    // executor.spin();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
