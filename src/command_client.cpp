#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "rclcpp/rclcpp.hpp"
#include "my_rfid_interfaces/srv/command_service.hpp"

#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>


std::vector<uint8_t> WRITE_DATA = {0x99,0x88,0x77,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x55};

class CommandClient : public rclcpp::Node 
{
public:
    CommandClient() : Node("command_client") 
    {
        // command_client_callback(2,3);
        // thread_x = std::thread(std::bind(&CommandClient::command_client_callback, this, 2 , 3));
        //threads.push_back(std::thread(std::bind(&CommandClient::command_client_callback, this,"multi" , WRITE_DATA)));
        
    }

    void command_client_callback(std::string command , std::vector<uint8_t> write_data)
    {
        auto client = this->create_client<my_rfid_interfaces::srv::CommandService>("Command");
        
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Server . . .");
        }
        
        auto request = std::make_shared<my_rfid_interfaces::srv::CommandService::Request>();
        request->command = command;
        if(command == "write")
        {
            write_data.resize(16);
            std::copy(write_data.begin(),write_data.end(),request->data_write.begin());
        }

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            if(command == "read")
            {
                if(response->response)
                {
                    RCLCPP_WARN(this->get_logger(),"READ SUCCESS");
                    read_data.resize(response->data_read.size());
                    std::copy(response->data_read.begin(),response->data_read.end(),read_data.begin());
                    std::cout<<"READ DATA: "<<std::endl;
                    for(size_t i = 0; i < read_data.size(); i++) 
                    {
                        std::cout << std::hex                         // Set hex format
                                << std::uppercase                    // Use uppercase letters
                                << std::setw(2)                     // Width of 2 characters
                                << std::setfill('0')               // Fill with zeros
                                << static_cast<int>(read_data[i]) // Cast to int for proper display
                                << " ";                                                                                                                                                                                                   // Space between bytes
                    }
                    std::cout << std::dec << std::endl;

                    read_epc.resize(response->epc_read.size());
                    std::copy(response->epc_read.begin(),response->epc_read.end(),read_epc.begin());
                    std::cout<<"READ TAG EPC: "<<std::endl;
                    for(size_t i = 0; i < read_epc.size(); i++) 
                    {
                        std::cout << std::hex                         // Set hex format
                                << std::uppercase                    // Use uppercase letters
                                << std::setw(2)                     // Width of 2 characters
                                << std::setfill('0')               // Fill with zeros
                                << static_cast<int>(read_epc[i]) // Cast to int for proper display
                                << " ";                                                                                                                                                                                                   // Space between bytes
                    }
                    std::cout << std::dec << std::endl;


                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),"READ FAILED");
                }
            }
            else if(command == "write")
            {
                if(response->response)
                {
                    RCLCPP_WARN(this->get_logger(),"WRITE SUCCESS");

                    write_epc.resize(response->epc_write.size());
                    std::copy(response->epc_write.begin(),response->epc_write.end(),write_epc.begin());
                    std::cout<<"WRITE TAG EPC: "<<std::endl;
                    for(size_t i = 0; i < write_epc.size(); i++) 
                    {
                        std::cout << std::hex                         // Set hex format
                                << std::uppercase                    // Use uppercase letters
                                << std::setw(2)                     // Width of 2 characters
                                << std::setfill('0')               // Fill with zeros
                                << static_cast<int>(write_epc[i]) // Cast to int for proper display
                                << " ";                                                                                                                                                                                                   // Space between bytes
                    }
                    std::cout << std::dec << std::endl;


                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),"WRITE FAILED");
                }

            }
            else if(command == "single")
            {
                if(response->response)
                {
                    RCLCPP_WARN(this->get_logger(),"SINGLE POLL SUCCESS");

                    single_epc.resize(response->single_poll_epc.size());
                    std::copy(response->single_poll_epc.begin(),response->single_poll_epc.end(),single_epc.begin());
                    std::cout<<"SINGLE POLL TAG EPC: "<<std::endl;
                    for(size_t i = 0; i < single_epc.size(); i++) 
                    {
                        std::cout << std::hex                         // Set hex format
                                << std::uppercase                    // Use uppercase letters
                                << std::setw(2)                     // Width of 2 characters
                                << std::setfill('0')               // Fill with zeros
                                << static_cast<int>(single_epc[i]) // Cast to int for proper display
                                << " ";                                                                                                                                                                                                   // Space between bytes
                    }
                    std::cout << std::dec << std::endl;


                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),"SINGLE POLL FAILED");
                }
            }
            else
            {
                if(response->response)
                {
                    RCLCPP_WARN(this->get_logger(),"MULTI POLL SUCCESS");
                    std::cout<<response->multi_epc_size<<std::endl;
    
                    multi_epc.resize(response->multi_epc_size);
                    std::cout<<"Starting Copy"<<std::endl;
                    std::copy_n(response->multi_poll_epc.begin(),std::min(response->multi_poll_epc.size(),multi_epc.size()),multi_epc.begin());
                    std::cout<<"MULTI POLL TAG EPC: "<<std::endl;
                    for(size_t i = 0; i < multi_epc.size(); i++) 
                    {
                        std::cout << std::hex                         // Set hex format
                                << std::uppercase                    // Use uppercase letters
                                << std::setw(2)                     // Width of 2 characters
                                << std::setfill('0')               // Fill with zeros
                                << static_cast<int>(multi_epc[i]) // Cast to int for proper display
                                << " ";                                                                                                                                                                                                   // Space between bytes
                    }
                    std::cout << std::dec << std::endl;


                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),"MULTI POLL FAILED");
                }

            }

        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service Call Failed");
        }
    }

    void render_imgui_window()
    {
        ImGui::Begin("AIMBOT COMMAND CLIENT");
        /*

        #1->CREATE INTERFACES
        #2->FETCH PARAMETERS i.e EPC's &  DATA
        #3->UPDATE INTERFACES

        */
       ImGui::End();
    }

private:
    std::thread thread_x;
    std::vector<std::thread> threads;
    std::vector<uint8_t> read_data;
    std::vector<uint8_t> read_epc;
    std::vector<uint8_t> write_epc;
    std::vector<uint8_t> single_epc;
    std::vector<uint8_t> multi_epc;
};

static void glfw_error_callback(int error , const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

int main(int argc, char **argv)
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<CommandClient>();

    // Initialize GLFW and OpenGL 
    glfwSetErrorCallback(glfw_error_callback);
    if(!glfwInit())
        return 1;

    // OpenGL version setup
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // Create the User Interface Window
    GLFWwindow* window = glfwCreateWindow(1280, 720, "AIMBOT COMMAND CLIENT", nullptr, nullptr);
    if(window == nullptr)
        return 1;
    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Setting up ImGui Context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    while(!glfwWindowShouldClose(window))
    {
        rclcpp::spin_some(node);

        //ImGui Frame Initialization
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        node->render_imgui_window();

        // render the user interface window
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


