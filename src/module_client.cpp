#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_rfid_interfaces/action/module_worker.hpp"
#include "my_rfid_interfaces/srv/command_service.hpp"

using ModuleWorker = my_rfid_interfaces::action::ModuleWorker;
using ModuleWorkerGoalHandle = rclcpp_action::ClientGoalHandle<ModuleWorker>;
using namespace std::placeholders; 

uint8_t WRITE_DATA[] = {0x99,0x88,0x77,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x55};


class ModuleClientNode : public rclcpp::Node
{
public:
    ModuleClientNode() : Node("module_client")
    {
        cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        module_client = rclcpp_action::create_client<ModuleWorker>(
            this, 
            "module_worker",
            cb_group,
            rcl_action_client_get_default_options());
        RCLCPP_INFO(this->get_logger(),"Module Client is Active");
        command_server_ = this->create_service<my_rfid_interfaces::srv::CommandService>(    
                                    "Command", 
                                    std::bind(&ModuleClientNode::command_server_callback,this,_1,_2));

    }

    void send_goal_read()
    {
        // Wait for Action Server
        while(!module_client->wait_for_action_server())
        {
            RCLCPP_INFO(this->get_logger(),"Waiting for Action Server...");
        }
        // Create goal
        auto goal = ModuleWorker::Goal();
        goal.action = "read";
        command = "read";
        // goal.epc = ???;
        // goal.tag_select = ???

        // Add Callbacks
        auto options = rclcpp_action::Client<ModuleWorker>::SendGoalOptions();
        options.result_callback = 
                std::bind(&ModuleClientNode::goal_result_callback, this, _1);
        options.goal_response_callback = 
                std::bind(&ModuleClientNode::goal_response_callback,this, _1);
        options.feedback_callback = 
                std::bind(&ModuleClientNode::goal_feedback_callback, this, _1, _2);


        // Send goal
        RCLCPP_INFO(this->get_logger(), "Sending Goal");
        module_client->async_send_goal(goal, options);

    }

    void send_goal_write(std::vector<uint8_t> write_data, size_t data_length)
    {
        // Wait for Action Server
        while(!module_client->wait_for_action_server())
        {
            RCLCPP_INFO(this->get_logger(),"Waiting for Action Server...");
        }

        // Create goal
        auto goal = ModuleWorker::Goal();
        std::copy_n(write_data.begin(),std::min(data_length,goal.write_data.size()),goal.write_data.begin());
        goal.action = "write";
        command = "write";
        
        // Add Callbacks
        auto options = rclcpp_action::Client<ModuleWorker>::SendGoalOptions();
        options.result_callback = 
                std::bind(&ModuleClientNode::goal_result_callback, this, _1);
        options.goal_response_callback = 
                std::bind(&ModuleClientNode::goal_response_callback,this, _1);
        options.feedback_callback = 
                std::bind(&ModuleClientNode::goal_feedback_callback, this, _1, _2);


        // Send goal
        RCLCPP_INFO(this->get_logger(), "Sending Goal");
        module_client->async_send_goal(goal, options);

    }
     void send_goal_single()
    {
        // Wait for Action Server
        while(!module_client->wait_for_action_server())
        {
            RCLCPP_INFO(this->get_logger(),"Waiting for Action Server...");
        }

        // Create goal
        auto goal = ModuleWorker::Goal();
        goal.action = "single";
        command = "single";

        // Add Callbacks
        auto options = rclcpp_action::Client<ModuleWorker>::SendGoalOptions();
        options.result_callback = 
                std::bind(&ModuleClientNode::goal_result_callback, this, _1);
        options.goal_response_callback = 
                std::bind(&ModuleClientNode::goal_response_callback,this, _1);
        options.feedback_callback = 
                std::bind(&ModuleClientNode::goal_feedback_callback, this, _1, _2);


        // Send goal
        RCLCPP_INFO(this->get_logger(), "Sending Goal");
        module_client->async_send_goal(goal, options);

    }

     void send_goal_multi()
    {
        // Wait for Action Server
        while(!module_client->wait_for_action_server())
        {
            RCLCPP_INFO(this->get_logger(),"Waiting for Action Server...");
        }

        // Create goal
        auto goal = ModuleWorker::Goal();
        goal.action = "multi";
        command = "multi";

        // Add Callbacks
        auto options = rclcpp_action::Client<ModuleWorker>::SendGoalOptions();
        options.result_callback = 
                std::bind(&ModuleClientNode::goal_result_callback, this, _1);
        options.goal_response_callback = 
                std::bind(&ModuleClientNode::goal_response_callback,this, _1);
        options.feedback_callback = 
                std::bind(&ModuleClientNode::goal_feedback_callback, this, _1, _2);


        // Send goal
        RCLCPP_INFO(this->get_logger(), "Sending Goal");
        module_client->async_send_goal(goal, options);

    }

private:

    // Callback to retrieve result once goal is done
    void goal_result_callback
    (const ModuleWorkerGoalHandle::WrappedResult &result)
    {
        auto status = result.code;

        if(status == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        }
        else if(status == rclcpp_action::ResultCode::ABORTED)
        {   
            RCLCPP_ERROR(this->get_logger(), "Goal Aborted");
        }
        else if(status == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(this->get_logger(), "Goal Canceled");
        }

        std::cout<<command<<std::endl;
        
        if(command == "read")
        {   
            // uint8_t read_data[16];
            size_t data_size = 16;
            size_t epc_size = 12;
            auto data = result.result->read_data;
            auto epc = result.result->read_epc;

            if(result.result->response)
            {
                try
                {   
                    read_data.resize(data_size);
                    std::copy_n(data.begin(),std::min(data.size(), read_data.size()),read_data.begin());
                    read_epc.resize(epc_size);
                    std::copy_n(epc.begin(),std::min(epc.size(), read_epc.size()),read_epc.begin());

                }
                catch(const std::exception& e)
                {
                    std::cout<<e.what()<<std::endl;
                }

                std::cout<<"Copy Success"<<std::endl;

                std::cout<<"READ TAG EPC: "<<std::endl;

                for(size_t i = 0; i < epc_size; i++) 
                {
                    std::cout << std::hex                    
                            << std::uppercase             
                            << std::setw(2)              
                            << std::setfill('0')          
                            << static_cast<int>(read_epc[i])  
                            << " ";                                                                                                                                                                                                   // Space between bytes
                }
                std::cout<<std::dec<<std::endl;

                std::cout<<"READ DATA: "<<std::endl;

                for(size_t i = 0; i < data_size; i++) 
                {
                    std::cout << std::hex                   
                            << std::uppercase              
                            << std::setw(2)               
                            << std::setfill('0')          
                            << static_cast<int>(read_data[i])  
                            << " ";                                                                                                                                                                                                   // Space between bytes
                }
                std::cout << std::dec << std::endl;

                goal_complete = true;
                goal_response = true;
            }

            
            
        }
        else if(command == "write")
        {
            size_t epc_size = 12;
            auto epc = result.result->write_epc;

            if(result.result->response)
            {
                try
                {
                    write_epc.resize(epc_size);
                    std::copy_n(epc.begin(),std::min(epc.size(), write_epc.size()),write_epc.begin());
                }
                catch(const std::exception& e)
                {
                    std::cout<< e.what() <<std::endl;
                }
                
                 std::cout<<"WRITE TAG EPC: "<<std::endl;

                for(size_t i = 0; i < epc_size; i++) 
                {
                    std::cout << std::hex                    
                            << std::uppercase             
                            << std::setw(2)              
                            << std::setfill('0')          
                            << static_cast<int>(write_epc[i])  
                            << " ";                                                                                                                                                                                                   // Space between bytes
                }
                std::cout<<std::dec<<std::endl;
                goal_complete = true;
                goal_response = true;
            }
        }
        else if(command == "single")
        {
            RCLCPP_INFO(this->get_logger(),"In SINGLE ELSE IF");
            size_t epc_size = 12;
            auto epc = result.result->single_inventory_epc;

            if(result.result->response)
            {
                RCLCPP_INFO(this->get_logger(),"result.result->response is True");
                try
                {
                   single_epc.resize(epc_size);
                   std::copy_n(epc.begin(),std::min(epc.size(),single_epc.size()),single_epc.begin());
                }
                catch(const std::exception& e)
                {
                    std::cout<< e.what() <<std::endl;
                }

                std::cout<<"Copy Success"<<std::endl;

                std::cout<<"SINGLE POLL EPC: "<<std::endl;
    
                for(size_t i = 0; i < epc_size; i++) 
                {
                    std::cout << std::hex                    // Set hex format
                            << std::uppercase              // Use uppercase letters
                            << std::setw(2)               // Width of 2 characters
                            << std::setfill('0')          // Fill with zeros
                            << static_cast<int>(single_epc[i])  // Cast to int for proper display
                            << " ";                                                                                                                                                                                                   // Space between bytes
                }
                std::cout << std::dec << std::endl;
                goal_complete = true;
                goal_response = true;  
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),"result.result->response is False");
            }

        }
        else
        {
            RCLCPP_INFO(this->get_logger(),"IN MULTI ELSE");
            size_t epc_size;
            auto epc = result.result->multi_inventory_epc;

            if(result.result->response)
            {
                RCLCPP_INFO(this->get_logger(),"result.result->response is True");
                try
                {
                    // std::copy_n(epc.begin(),std::min(epc.size(), sizeof(epc_data)),epc_data);
                    if(epc.size() == 0)
                    {
                        std::cout<<"epc.size() is 0"<<std::endl;
                    }
                    epc_size = result.result->multi_epc_size;                
                    multi_epc.resize(epc_size);
                    assert(epc.begin() != nullptr);
                    std::cout<<"epc.begin() assertion passed"<<std::endl;
                    std::copy_n(epc.begin(),std::min(epc.size(),multi_epc.size()),multi_epc.begin());
                }
                catch(const std::exception& e)
                {
                    std::cout<< e.what() <<std::endl;
                }

                std::cout<<"Copy Success"<<std::endl;

                std::cout<<"MULTI POLL EPC's: "<<std::endl;
    
                for(size_t i = 0; i < epc_size; i++) 
                {
                    std::cout << std::hex                         // Set hex format
                            << std::uppercase                    // Use uppercase letters
                            << std::setw(2)                     // Width of 2 characters
                            << std::setfill('0')               // Fill with zeros
                            << static_cast<int>(multi_epc[i]) // Cast to int for proper display
                            << " ";                                                                                                                                                                                                   // Space between bytes
                }
                std::cout << std::dec << std::endl;
                
                goal_complete = true;
                goal_response = true; 
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),"result.result->response is False");
            }
        }       
    }

    void goal_response_callback
    (const ModuleWorkerGoalHandle::SharedPtr &goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Goal got Rejected");
        }
        else
        {
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal has been Acceppted");
        }

    }

    void goal_feedback_callback
    (   const ModuleWorkerGoalHandle::SharedPtr &goal_handle,
        const std::shared_ptr<const ModuleWorker::Feedback> feedback)
    {
        (void)goal_handle;
        std::string response = feedback->progress;
        RCLCPP_WARN(this->get_logger(), "%s", response.c_str());
        
    }

    void command_server_callback
    (const my_rfid_interfaces::srv::CommandService::Request::SharedPtr request,
     const my_rfid_interfaces::srv::CommandService::Response::SharedPtr response)
    {
        if(request->command == "read")
        {
            send_goal_read();
            auto start = std::chrono::high_resolution_clock::now();
            bool timer = true;
            while(!goal_complete)
            {
               auto finish = std::chrono::high_resolution_clock::now();
               std::chrono::duration<double> time_elapsed = finish - start;
               if(time_elapsed.count() > 10.0) 
               {
                    timer = false;
                    break;
               }
            };
            if(timer)
            {
                response->response = goal_response;
                if(goal_response)
                {
                    std::copy(read_data.begin(),read_data.end(),response->data_read.begin());
                    std::copy(read_epc.begin(),read_epc.end(),response->epc_read.begin());
                }
            }
            else
            {
                response->response = false;
            }
            goal_complete = false;
            goal_response = false;
            
        }
        else if(request->command == "write")
        {
            size_t write_data_size = 16;
            std::vector<uint8_t> write_data;
            write_data.resize(write_data_size);

            auto request_data = request->data_write;
            std::copy(request_data.begin(),request_data.end(),write_data.begin());

            send_goal_write(write_data,write_data_size);
            auto start = std::chrono::high_resolution_clock::now();
            bool timer = true;
            while(!goal_complete)
            {
               auto finish = std::chrono::high_resolution_clock::now();
               std::chrono::duration<double> time_elapsed = finish - start;
               if(time_elapsed.count() > 10.0)
               {
                    timer = false;
                    break;
               } 
            };

            if(timer)
            {
                response->response = goal_response;

                if(goal_response)
                {
                    std::copy(write_epc.begin(),write_epc.end(),response->epc_write.begin());
                }
            }
            else
            {
                response->response = false;
            }
            goal_complete = false;
            goal_response = false;

        }
        else if(request->command == "single")
        {
            send_goal_single();
            auto start = std::chrono::high_resolution_clock::now();
            bool timer = true;
            while(!goal_complete)
            {
               auto finish = std::chrono::high_resolution_clock::now();
               std::chrono::duration<double> time_elapsed = finish - start;
               if(time_elapsed.count() > 10.0) 
               {
                    timer = false;
                    break;
               }
            };
            if(timer)
            {
                response->response = goal_response;
                if(goal_response)
                {
                    std::copy(single_epc.begin(),single_epc.end(),response->single_poll_epc.begin());
                }
            }
            else
            {
                response->response = false;
            }
            
            goal_complete = false;
            goal_response = false;
        }
        else
        {
            send_goal_multi();
            auto start = std::chrono::high_resolution_clock::now();
            bool timer = true;
            while(!goal_complete)
            {
               auto finish = std::chrono::high_resolution_clock::now();
               std::chrono::duration<double> time_elapsed = finish - start;
               if(time_elapsed.count() > 10.0)
               {
                    timer = false;
                    break;
                }
            };

            if(timer)
            {
                response->response = goal_response;

                if(goal_response)
                {
                    std::copy(multi_epc.begin(),multi_epc.end(),response->multi_poll_epc.begin());
                    response->multi_epc_size = static_cast<int>(multi_epc.size());
                }
            }
            else
            {
                response->response = false;
            }
            goal_complete = false;
            goal_response = false;
        }
    }

    rclcpp_action::Client<ModuleWorker>::SharedPtr module_client;
    rclcpp::Service<my_rfid_interfaces::srv::CommandService>::SharedPtr command_server_;
    rclcpp::TimerBase::SharedPtr timer_;
    ModuleWorkerGoalHandle::SharedPtr goal_handle_;
    rclcpp::CallbackGroup::SharedPtr cb_group;
    std::string command;
    bool goal_complete;
    std::vector<uint8_t> read_data;
    std::vector<uint8_t> read_epc;
    std::vector<uint8_t> write_epc;
    std::vector<uint8_t> single_epc;
    std::vector<uint8_t> multi_epc;
    bool goal_response;

};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModuleClientNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // node->send_goal_write(WRITE_DATA,write_data_size);
    // node->send_goal_read();
    // node->send_goal_multi();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
