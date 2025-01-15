#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "rclcpp/rclcpp.hpp"
#include "my_rfid_interfaces/srv/command_service.hpp"

#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>


class CommandClient : public rclcpp::Node 
{
public:
    CommandClient() : Node("command_client") 
    {
        // command_client_callback(2,3);
        // thread_x = std::thread(std::bind(&CommandClient::command_client_callback, this, 2 , 3));
        // threads.push_back(std::thread(std::bind(&CommandClient::command_client_callback, this,"multi" , write_data)));
        
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
                    success = true;
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
                    success == false;
                    RCLCPP_ERROR(this->get_logger(),"READ FAILED");
                }
            }
            else if(command == "write")
            {
                if(response->response)
                {
                    success = true;
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
                    success = false;
                    RCLCPP_ERROR(this->get_logger(),"WRITE FAILED");
                }

            }
            else if(command == "single")
            {
                if(response->response)
                {
                    success = true;
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
                    success = false;
                    RCLCPP_ERROR(this->get_logger(),"SINGLE POLL FAILED");
                }
            }
            else
            {
                if(response->response)
                {
                    success = true;
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
                    success = false;
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
        ImGui::SetNextWindowSize(ImVec2(1000,550));
        ImGui::Begin("AIMBOT COMMAND CLIENT", &window_open, ImGuiWindowFlags_NoCollapse);


        // READ INTERFACE
        ImGui::SetCursorPosY(68.75F);
        ImGui::SetCursorPosX(120.0f);

        if(success)
        {
            for(size_t i = 0; i < read_epc.size(); i++)
            {
                if(i == 0)  ImGui::Text("READ TAG EPC:");
                if(i > 0) ImGui::SameLine();
                else ImGui::SetCursorPosX(120.0f);
                ImGui::Text(" %02X",read_epc[i]);
            }
        }
        else
        {
            ImGui::Text("READ TAG EPC:");
        }
        ImGui::SetCursorPosY(103.125F);
        ImGui::SetCursorPosX(120.0f);
        if(success)
        {
            for(size_t i = 0; i < read_data.size(); i++)
            {
                if(i == 0)  ImGui::Text("READ TAG DATA:");
                if(i > 0) ImGui::SameLine();
                else ImGui::SetCursorPosX(120.0f);
                ImGui::Text(" %02X",read_data[i]);
            }
        }
        else
        {
            ImGui::Text("READ TAG DATA: "); 
        }
        ImGui::SetCursorPosY(35.0f);
        if(ImGui::Button("READ", ImVec2(100,100)))
        {
            /*If Button is pressed send the read command here*/
            threads.push_back(std::thread(std::bind(&CommandClient::command_client_callback, this,"read",write_data)));
        }

        // WRITE INTERFACE
        ImGui::SetCursorPosY(68.75F);
        ImGui::SetCursorPosX(620.0f);
        ImGui::SetNextItemWidth(200.0f);
        write_data.resize(16);
        if(ImGui::InputText("WRITE DATA", buf , IM_ARRAYSIZE(buf), ImGuiInputTextFlags_CharsUppercase))
        {
            ConvertHexStringToBytes(buf, write_data);
        }
        ImGui::SetCursorPosY(35.0f);
        ImGui::SetCursorPosX(500.0f);
        if(ImGui::Button("WRITE", ImVec2(75,75)))
        {
            /*If Button is pressed send the write command here*/
            threads.push_back(std::thread(std::bind(&CommandClient::command_client_callback, this,"write",write_data)));
        }

        // SINGLE INVENTORY INTERFACE
        ImGui::SetCursorPosY(343.75F);
        ImGui::SetCursorPosX(120.0f);
        ImGui::BeginGroup();
        {
            if(success)
            {
                for(size_t i = 0; i < single_epc.size(); i++)
                {
                    if(i == 0)  ImGui::Text("SINGLE POLL EPC:");
                    if(i > 0) ImGui::SameLine();
                    else ImGui::SetCursorPosX(120.0f);
                    ImGui::Text(" %02X",single_epc[i]);
                }
            }
            else
            {
                ImGui::Text("SINGLE POLL EPC: ");
            }
        }
        ImGui::EndGroup();
        ImGui::SetCursorPosY(310.0f);
        if(ImGui::Button("SINGLE", ImVec2(100,100)))
        {
            /*If Button is pressed send the Single Inventory command here*/
            threads.push_back(std::thread(std::bind(&CommandClient::command_client_callback, this,"single",write_data)));
        }

        //MULTI INTERFACE
        float startY = 310.0f;
        float lineSpacing = 40.0f;  

        ImGui::SetCursorPosY(startY);
        ImGui::SetCursorPosX(620.0f);
        ImGui::BeginGroup();
        {
            ImGui::Text("EPC 1: ");
            ImGui::SetCursorPosY(startY + lineSpacing);
            ImGui::Text("EPC 2: ");
            ImGui::SetCursorPosY(startY + lineSpacing * 2);
            ImGui::Text("EPC 3: ");
            ImGui::SetCursorPosY(startY + lineSpacing * 3);
            ImGui::Text("EPC 4: ");
            ImGui::SetCursorPosY(startY + lineSpacing * 4);
            ImGui::Text("EPC 5: ");
        }
        ImGui::EndGroup();
        ImGui::SetCursorPosY(310.0f);
        ImGui::SetCursorPosX(500.0f);
        if(ImGui::Button("MULTI", ImVec2(100,100)))
        {
            /*If Button is pressed send the MULTI command here*/
            threads.push_back(std::thread(std::bind(&CommandClient::command_client_callback, this,"multi",write_data)));
        }

       ImGui::End();
    }
    
    void ConvertHexStringToBytes(const char* hexString, std::vector<uint8_t>& bytes) 
    {
        std::string str(hexString);
        std::stringstream ss(str);
        std::string token;
        size_t index = 0;
        
        bytes.clear();
        bytes.resize(16, 0);  
        
        try 
        {
            while (std::getline(ss, token, ',') && index < bytes.size()) 
            {
                // Remove spaces and "0x" prefix if present
                token.erase(remove_if(token.begin(), token.end(), isspace), token.end());
                if (token.substr(0, 2) == "0x") 
                {
                    token = token.substr(2);
                }
                
                // Only convert if we have valid data
                if (!token.empty()) 
                {
                    bytes[index++] = static_cast<uint8_t>(std::stoi(token, nullptr, 16));
                }
            }
        } 
        catch (const std::exception& e) 
        {
            std::cerr << "Conversion error: " << e.what() << std::endl;
        }
    }

private:
    std::thread thread_x;
    std::vector<std::thread> threads;
    std::vector<uint8_t> read_data;
    std::vector<uint8_t> read_epc;
    std::vector<uint8_t> write_epc;
    std::vector<uint8_t> single_epc;
    std::vector<uint8_t> multi_epc;
    std::vector<uint8_t> write_data;
    bool success;
    bool window_open = true;
    char buf[128];
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
    GLFWwindow* window = glfwCreateWindow(1280, 720, "AIMBOT", nullptr, nullptr);
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


