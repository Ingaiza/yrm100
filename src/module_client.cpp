#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_rfid_interfaces/action/module_worker.hpp"
#include "my_rfid_interfaces/srv/command_service.hpp"
#include "/home/ingaiza/yrm_module/src/yrm100/include/yrm100/inventory.hpp"
#include "/home/ingaiza/yrm_module/src/yrm100/include/yrm100/hextostring.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>

using ModuleWorker = my_rfid_interfaces::action::ModuleWorker;
using ModuleWorkerGoalHandle = rclcpp_action::ClientGoalHandle<ModuleWorker>;
using namespace std::placeholders; 

std::string epc_file_name;

void create_epc_file()
{
    // epc_file_name = epc_name();
    std::time_t t = time(0);
    std::tm* now = std::localtime(&t);

    std::string basepath = "/home/ingaiza/aimbot_inventory/epc_inventory ";
    std::string t_stamp = std::to_string(now->tm_mday) + "-" + std::to_string(now->tm_mon+1)  + "-" + 
                            std::to_string(now->tm_year+1900)  + " " + std::to_string(now->tm_hour)  + "-" + 
                            std::to_string(now->tm_min)  + "-" + std::to_string(now->tm_sec);
    epc_file_name = basepath + t_stamp;
    std::ofstream epcfile(epc_file_name);
    
    if (epcfile.is_open()) 
    {
        epcfile << "AIMBOT PRODUCT EPC INVENTORY\n";
        epcfile << " \n";
        epcfile << "Inventory Timestamp: "<<now->tm_mday<<"/"<<now->tm_mon+1<<"/"<<now->tm_year+1900<<" "
                <<now->tm_hour<<":"<<now->tm_min<<":"<<now->tm_sec<<"\n";
        epcfile << " \n";
        epcfile.close();
        std::cout << "File created and written successfully.\n";
    } 
    else 
    {
        std::cout << "Error: Unable to create file.\n";
    }
}

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

        std::cout<<"WRITE DATA VALUES WHEN SENDING ACTION REQUEST: "<<"\n";
        for (size_t i = 0; i < write_data.size(); i++) {
            std::cout << std::hex 
                    << std::uppercase 
                    << std::setw(2) 
                    << std::setfill('0')
                    << static_cast<int>(write_data[i])
                    << " ";
        }
        std::cout<< std::dec<<'\n';
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

        std::cout<<command<<'\n';
        
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
                    std::cout<<e.what()<<'\n';
                }

                std::cout<<"Copy Success"<<'\n';

                std::cout<<"READ TAG EPC: "<<'\n';

                for(size_t i = 0; i < epc_size; i++) 
                {
                    std::cout << std::hex                    
                            << std::uppercase             
                            << std::setw(2)              
                            << std::setfill('0')          
                            << static_cast<int>(read_epc[i])  
                            << " ";                                                                                                                                                                                                   // Space between bytes
                }
                std::cout<<std::dec<<'\n';

                std::cout<<"READ DATA: "<<'\n';

                for(size_t i = 0; i < data_size; i++) 
                {
                    std::cout << std::hex                   
                            << std::uppercase              
                            << std::setw(2)               
                            << std::setfill('0')          
                            << static_cast<int>(read_data[i])  
                            << " ";                                                                                                                                                                                                   // Space between bytes
                }
                std::cout << std::dec << '\n';
                read_inventory();
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
                    std::cout<< e.what() <<'\n';
                }
                
                 std::cout<<"WRITE TAG EPC: "<<'\n';

                for(size_t i = 0; i < epc_size; i++) 
                {
                    std::cout << std::hex                    
                            << std::uppercase             
                            << std::setw(2)              
                            << std::setfill('0')          
                            << static_cast<int>(write_epc[i])  
                            << " ";                                                                                                                                                                                                   // Space between bytes
                }
                std::cout<<std::dec<<'\n';
                write_inventory();
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
                    std::cout<< e.what() <<'\n';
                }

                std::cout<<"Copy Success"<<'\n';

                std::cout<<"SINGLE POLL EPC: "<<'\n';
    
                for(size_t i = 0; i < epc_size; i++) 
                {
                    std::cout << std::hex                    // Set hex format
                            << std::uppercase              // Use uppercase letters
                            << std::setw(2)               // Width of 2 characters
                            << std::setfill('0')          // Fill with zeros
                            << static_cast<int>(single_epc[i])  // Cast to int for proper display
                            << " ";                                                                                                                                                                                                   // Space between bytes
                }
                std::cout << std::dec << '\n';
                epc_inventory(single_epc);
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
                        std::cout<<"epc.size() is 0"<<'\n';
                    }
                    epc_size = result.result->multi_epc_size;                
                    multi_epc.resize(epc_size);
                    assert(epc.begin() != nullptr);
                    std::cout<<"epc.begin() assertion passed"<<'\n';
                    std::copy_n(epc.begin(),std::min(epc.size(),multi_epc.size()),multi_epc.begin());
                }
                catch(const std::exception& e)
                {
                    std::cout<< e.what() <<'\n';
                }

                std::cout<<"Copy Success"<<'\n';

                std::cout<<"MULTI POLL EPC's: "<<'\n';
    
                for(size_t i = 0; i < epc_size; i++) 
                {
                    std::cout << std::hex                         // Set hex format
                            << std::uppercase                    // Use uppercase letters
                            << std::setw(2)                     // Width of 2 characters
                            << std::setfill('0')               // Fill with zeros
                            << static_cast<int>(multi_epc[i]) // Cast to int for proper display
                            << " ";                                                                                                                                                                                                   // Space between bytes
                }
                std::cout << std::dec << '\n';
                epc_inventory(multi_epc);
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
               if(time_elapsed.count() > 15.0) 
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
            write_user_data.resize(write_data_size);

            auto request_data = request->data_write;
            std::copy(request_data.begin(),request_data.end(),write_data.begin());
            std::copy(request_data.begin(),request_data.end(),write_user_data.begin());

            send_goal_write(write_data,write_data_size);
            auto start = std::chrono::high_resolution_clock::now();
            bool timer = true;
            while(!goal_complete)
            {
               auto finish = std::chrono::high_resolution_clock::now();
               std::chrono::duration<double> time_elapsed = finish - start;
               if(time_elapsed.count() > 15.0)
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
               if(time_elapsed.count() > 15.0) 
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
               if(time_elapsed.count() > 15.0)
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

    void read_inventory()
    {   
        std::time_t t = time(0);
        std::tm* now = std::localtime(&t);

        std::string basepath = "/home/ingaiza/aimbot_inventory/read_inventory ";
        std::string t_stamp = std::to_string(now->tm_mday) + "-" + std::to_string(now->tm_mon+1)  + "-" + 
                              std::to_string(now->tm_year+1900)  + " " + std::to_string(now->tm_hour)  + "-" + 
                              std::to_string(now->tm_min)  + "-" + std::to_string(now->tm_sec);
        std::string name = basepath + t_stamp;
 
        std::ofstream readfile(name); 
        RFIDDataMemory read;
        read.data = read_data;
        uint16_t location = read.getLocation();
        uint16_t product_category = read.getProductCategory();
        auto status = read.getStatus();
        uint8_t quantity = read.getQuantity();
        uint16_t price = read.getPrice();
        uint16_t supplier_id = read.getSupplierId();
        uint16_t batch_number = read.getBatchNumber();
        auto date = read.getDate();

        std::cout<<"Date: "<<date.tm_mday<<"/"<<date.tm_mon+1<<"/"<<date.tm_year+1900<<"\n";
        std::cout<<"Product Category: "<<product_category<<"\n";
        std::cout<<"Quantity(in Units): "<<static_cast<int>(quantity)<<"\n";
        std::cout<<"Price: "<<price<<"\n";
        std::cout<<"Location: "<<location<<"\n";
        std::cout<<"Supplier ID: "<<supplier_id<<"\n";
        std::cout<<"Batch Number: "<<batch_number<<"\n";
        std::cout<<"Status: "<<(status.inStock ? "In Stock" : (status.reserved ? "Reserved" : (status.damaged ? "Damaged" : (status.expired ? "Expired" : "INVALID STATUS"))))<<"\n";

        auto chunks = HexChunkProcessor::processHexChunksNonDestructive(read_epc);

        if(readfile.is_open())
        {
            readfile << "AIMBOT READ INVENTORY.\n";
            readfile << "  \n";
            for(size_t i = 0; i < chunks.size(); ++i)
            {   
                readfile <<"READ FROM TAG EPC: "<<chunks[i]<<"  | TIMESTAMP: "<<now->tm_mday<<"/"
                            <<now->tm_mon+1<<"/"<<now->tm_year+1900<<" "<<now->tm_hour<<":"
                            <<now->tm_min<<":"<<now->tm_sec<<"\n";
                readfile <<" \n";
            }
            readfile << "PRODUCT PARAMETERS\n";
            readfile << "  \n";
            readfile << "Date: "<<date.tm_mday<<"/"<<date.tm_mon+1<<"/"<<date.tm_year+1900<<"\n";
            readfile << "  \n";
            readfile << "Product Category: "<<product_category<<"\n";
            readfile << "  \n";
            readfile << "Quantity(in Units): "<<static_cast<int>(quantity)<<"\n";
            readfile << "  \n";
            readfile << "Price: "<<price<<"\n";
            readfile << "  \n";
            readfile << "Location: "<<location<<"\n";
            readfile << "  \n";
            readfile << "Supplier ID: "<<supplier_id<<"\n";
            readfile << "  \n";
            readfile << "Batch Number: "<<batch_number<<"\n";
            readfile << "  \n";
            readfile << "Status: "<<(status.inStock ? "In Stock" : (status.reserved ? "Reserved" : (status.damaged ? "Damaged" : (status.expired ? "Expired" : "INVALID STATUS"))))<<"\n";
            readfile.close();
        }
        else
        {
            std::cout<<"ERROR: Unable to create file\n";
        }

    }

    void write_inventory()
    {
        std::time_t t = time(0);
        std::tm* now = std::localtime(&t);

        std::string basepath = "/home/ingaiza/aimbot_inventory/write_inventory ";
        std::string t_stamp = std::to_string(now->tm_mday) + "-" + std::to_string(now->tm_mon+1)  + "-" + 
                              std::to_string(now->tm_year+1900)  + " " + std::to_string(now->tm_hour)  + "-" + 
                              std::to_string(now->tm_min)  + "-" + std::to_string(now->tm_sec);
        std::string name = basepath + t_stamp;

        std::ofstream writefile(name); 
        RFIDDataMemory write;
        write.data = write_user_data;
        uint16_t location = write.getLocation();
        uint16_t product_category = write.getProductCategory();
        auto status = write.getStatus();
        uint8_t quantity = write.getQuantity();
        uint16_t price = write.getPrice();
        uint16_t supplier_id = write.getSupplierId();
        uint16_t batch_number = write.getBatchNumber();
        auto date = write.getDate();

        std::cout<<"Date: "<<date.tm_mday<<"/"<<date.tm_mon+1<<"/"<<date.tm_year+1900<<"\n";
        std::cout<<"Product Category: "<<product_category<<"\n";
        std::cout<<"Quantity(in Units): "<<static_cast<int>(quantity)<<"\n";
        std::cout<<"Price: "<<price<<"\n";
        std::cout<<"Location: "<<location<<"\n";
        std::cout<<"Supplier ID: "<<supplier_id<<"\n";
        std::cout<<"Batch Number: "<<batch_number<<"\n";
        std::cout<<"Status: "<<(status.inStock ? "In Stock" : (status.reserved ? "Reserved" : (status.damaged ? "Damaged" : (status.expired ? "Expired" : "INVALID STATUS"))))<<"\n";


        auto chunks = HexChunkProcessor::processHexChunksNonDestructive(write_epc);
   
        if(writefile.is_open())
        {
            writefile << "AIMBOT WRITE INVENTORY.\n";
            writefile << "  \n";
            for(size_t i = 0; i < chunks.size(); ++i)
            {   
                writefile <<"WRITTEN TO TAG EPC: "<<chunks[i]<<"  | TIMESTAMP: "<<now->tm_mday<<"/"
                            <<now->tm_mon+1<<"/"<<now->tm_year+1900<<" "<<now->tm_hour<<":"
                            <<now->tm_min<<":"<<now->tm_sec;
            }
            writefile << "  \n";
            writefile << "Inventory Timestamp: "<<now->tm_mday<<"/"<<now->tm_mon+1<<"/"<<now->tm_year+1900<<" "
                      <<now->tm_hour<<":"<<now->tm_min<<":"<<now->tm_sec<<"\n";
            writefile << "  \n";
            writefile << "PRODUCT PARAMETERS\n";
            writefile << "  \n";
            writefile << "Date: "<<date.tm_mday<<"/"<<date.tm_mon+1<<"/"<<date.tm_year+1900<<"\n";
            writefile << "  \n";
            writefile << "Product Category: "<<product_category<<"\n";
            writefile << "  \n";
            writefile << "Quantity(in Units): "<<static_cast<int>(quantity)<<"\n";
            writefile << "  \n";
            writefile << "Price: "<<price<<"\n";
            writefile << "  \n";
            writefile << "Location: "<<location<<"\n";
            writefile << "  \n";
            writefile << "Supplier ID: "<<supplier_id<<"\n";
            writefile << "  \n";
            writefile << "Batch Number: "<<batch_number<<"\n";
            writefile << "  \n";
            writefile << "Status: "<<(status.inStock ? "In Stock" : (status.reserved ? "Reserved" : (status.damaged ? "Damaged" : (status.expired ? "Expired" : "INVALID STATUS"))))<<"\n";

            writefile.close();
        }
        else
        {
            std::cout<<"ERROR: Unable to create file\n";
        }
    }

    void epc_inventory(std::vector<uint8_t>epc_value)
    {
    
        auto chunks = HexChunkProcessor::processHexChunksNonDestructive(epc_value);
        std::ofstream appendfile(epc_file_name, std::ios::app);

        std::time_t t = time(0);
        std::tm* now = std::localtime(&t);

        if(appendfile.is_open())
        {
            for(size_t i = 0; i < chunks.size(); ++i)
            {   
                appendfile <<"EPC CODE: "<<chunks[i]<<"  | TIMESTAMP: "<<now->tm_mday<<"/"
                           <<now->tm_mon+1<<"/"<<now->tm_year+1900<<" "<<now->tm_hour<<":"
                           <<now->tm_min<<":"<<now->tm_sec<<"\n";
                appendfile <<" \n";
            }
        }
        appendfile.close();
        std::cout << "\nFile updated successfully.\n";    

    
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
    std::vector<uint8_t> write_user_data;
    bool goal_response;

};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModuleClientNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    create_epc_file();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

