#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "hextostring.hpp"

#include "rclcpp/rclcpp.hpp"
#include "my_rfid_interfaces/srv/command_service.hpp"
#include "/home/ingaiza/yrm_module/src/yrm100/include/yrm100/inventory.hpp"

#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

RFIDDataMemory usermem;

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
            std::cout<<"WRITE DATA VALUES WHEN SENDING REQUEST: "<<"\n";
            for (size_t i = 0; i < write_data.size(); i++) {
                std::cout << std::hex 
                        << std::uppercase 
                        << std::setw(2) 
                        << std::setfill('0')
                        << static_cast<int>(write_data[i])
                        << " ";
            }
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

                    // auto chunks = HexChunkProcessor::processHexChunks(multi_epc);
                    // for (size_t i = 0; i < chunks.size(); ++i) {
                    //     std::cout << "Chunk " << (i + 1) << " (12 bytes): " << chunks[i] << "\n";
                        
                    //     // Print formatted bytes
                    //     std::cout << "Formatted bytes: ";
                    //     for (size_t j = 0; j < chunks[i].length(); j += 2) {
                    //         if (j > 0) std::cout << " ";
                    //         std::cout << chunks[i].substr(j, 2);
                    //     }
                    //     std::cout << "\n\n";
                    // }

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
        {   
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

            if (ImGui::InputText("WRITE DATA", buf, IM_ARRAYSIZE(buf),ImGuiInputTextFlags_CharsUppercase | ImGuiInputTextFlags_CharsHexadecimal)) 
            {
                ConvertHexStringToBytes8_t(buf, write_data);
                
                std::cout << "Converted bytes: ";
                for (size_t i = 0; i < write_data.size(); i++) {
                    std::cout << std::hex 
                            << std::uppercase 
                            << std::setw(2) 
                            << std::setfill('0')
                            << static_cast<int>(write_data[i])
                            << " ";
                }
                std::cout << std::endl;
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
                
                if (!multi_epc.empty()) {
                    auto chunks = HexChunkProcessor::processHexChunksNonDestructive(multi_epc);
                    for (size_t i = 0; i < chunks.size(); ++i) 
                    {                
                        // Set position for each new line
                        ImGui::SetCursorPosX(620.0f);
                        ImGui::SetCursorPosY(startY + (i * lineSpacing));
                        
                        // Print EPC number
                        ImGui::Text("EPC %zu:", i + 1);
                        
                        // Print each byte with spacing
                        for (size_t j = 0; j < chunks[i].length(); j += 2) 
                        {
                            ImGui::SameLine();
                            ImGui::Text("%s", chunks[i].substr(j, 2).c_str());
                        }
                    }
                } else {
                    ImGui::SetCursorPosX(620.0f);
                    ImGui::Text("No EPCs detected");
                }
            }
            ImGui::EndGroup();
            ImGui::SetCursorPosY(310.0f);
            ImGui::SetCursorPosX(500.0f);
            if(ImGui::Button("MULTI", ImVec2(100,100)))
            {
                /*If Button is pressed send the MULTI command here*/
                threads.push_back(std::thread(std::bind(&CommandClient::command_client_callback, this,"multi",write_data)));
            }
        }
        ImGui::End();
    }
    
    void render_new_window()
    {

        ImGui::SetNextWindowSize(ImVec2(500,500));
        ImGui::Begin("WRITE DATA CLIENT", &window_open, ImGuiWindowFlags_NoCollapse);
        {  
            std::time_t t = time(0);
            std::tm* now = std::localtime(&t);
            
            //SET DATE
            std::tm tagdate = {};
            tagdate.tm_year = now->tm_year;
            tagdate.tm_mon = now->tm_mon;
            tagdate.tm_mday = now->tm_mday;
            usermem.setDate(tagdate);

            ImGui::Text("DATE: %d/%d/%d",tagdate.tm_mday,tagdate.tm_mon + 1,tagdate.tm_year + 1900);
            ImGui::SetCursorPosY(60.0f); 
            if(ImGui::InputInt("PRODUCT CATEGORY", &category_buf, 100))
            {   
                if(category_buf > 65535) category_buf = 65535;
                product_category_id = hexStringToUint16(decimaltohex(category_buf));
                usermem.setProductCategory(product_category_id);
            }
            ImGui::SetCursorPosY(100.0f);
            if(ImGui::InputInt("LOCATION CODE", &location_buf, 100))
            {   
                if(location_buf > 65535) location_buf = 65535;
                location_code = hexStringToUint16(decimaltohex(location_buf));
                usermem.setLocation(location_code);
            }
            ImGui::SetCursorPosY(140.0f);
            if(ImGui::InputText("STATUS FLAG", status_buf, IM_ARRAYSIZE(status_buf), ImGuiInputTextFlags_CharsUppercase)) 
            {
                bool log = false;
                std::string status_str(status_buf);  
                
                // Remove whitespace
                status_str.erase(std::remove_if(status_str.begin(), status_str.end(), ::isspace), status_str.end());
                
                if(status_str == "INSTOCK") 
                {
                    usermem.setStatus(true, false, false, false);
                    log = true;
                }
                else if(status_str == "RESERVED") 
                {
                    usermem.setStatus(false, true, false, false);
                    log = true;
                }
                else if(status_str == "DAMAGED") 
                {
                    usermem.setStatus(false, false, true, false);
                    log = true;
                }
                else if(status_str == "EXPIRED") 
                {
                    usermem.setStatus(false, false, false, true);
                    log = true;
                }
                
                if(log) 
                {
                    std::cout << "STATUS FLAG: " << status_str << "\n";
                } else 
                {
                    std::cout << "Invalid status flag: " << status_str << "\n";
                }
            }
            ImGui::SetCursorPosY(180.0f);
            if(ImGui::InputInt("QUANTITY", &quantity_buf, 5))
            {
                if(quantity_buf > 255) quantity_buf = 255;
                quantity = hexStringToUint8(decimaltohex(quantity_buf));
                usermem.setQuantity(quantity);
            }
            ImGui::SetCursorPosY(220.0f);
            if(ImGui::InputInt("PRICE", &price_buf, 100))
            {
                if(price_buf > 65535) price_buf = 65535;
                price = hexStringToUint16(decimaltohex(price_buf));
                usermem.setPrice(price);
            }
            ImGui::SetCursorPosY(260.0f);
            if(ImGui::InputInt("SUPPLIER ID", &supplier_buf, 100))
            {
                if(supplier_buf > 65535) supplier_buf = 65535;
                supplier_id = hexStringToUint16(decimaltohex(supplier_buf));
                usermem.setSupplierId(supplier_id);
            }
            ImGui::SetCursorPosY(300.0f);
            if(ImGui::InputInt("BATCH NUMBER", &batch_buf, 100))
            {
                if(batch_buf > 65535) batch_buf = 65535;
                batch_number = hexStringToUint16(decimaltohex(batch_buf));
                usermem.setBatchNumber(batch_number);
            }
            ImGui::SetCursorPosY(340.0f);
            usermem.calculateChecksum();
            std::vector<uint8_t> data = usermem.getData();
            std::string data_string = HexChunkProcessor::bytesToHexString(data, 0, data.size());
            ImGui::Text("ENCODED WRITE DATA: ");
            ImGui::SameLine();
            ImGui::TextColored(GREEN_TEXT, "%s",data_string.c_str());

            ImGui::SetCursorPosY(380.0f);
            ImGui::TextColored(RED_TEXT, "NOTE:");
            ImGui::SetCursorPosY(400.0f);
            ImGui::TextColored(YELLOW_TEXT, "All Inputs except for ");
            ImGui::SameLine();
            {
                ImGui::TextColored(BLUE_TEXT, "STATUS FLAG");
                ImGui::SameLine();
                ImGui::TextColored(YELLOW_TEXT, "should be in decimal, Date is");
            }
            ImGui::TextColored(YELLOW_TEXT, "automatically set.");
            ImGui::TextColored(YELLOW_TEXT, "Unit for PRICE is USD and QUANTITY is in Storage Units.");
            ImGui::TextColored(BLUE_TEXT, "PRODUCT CATEGORY ID, LOCATION, PRICE, SUPPLIER ID");
            ImGui::SameLine();
            {
                ImGui::TextColored(YELLOW_TEXT, "and");
                ImGui::SameLine();
                ImGui::TextColored(BLUE_TEXT, "BATCH NUMBER");
            }
            ImGui::TextColored(YELLOW_TEXT, "can have any value from 0 to 65535.");
            ImGui::TextColored(BLUE_TEXT, "QUANTITY");
            ImGui::SameLine();
            {
                ImGui::TextColored(YELLOW_TEXT, "can only take values from 0 to 255 units.");
            }
            ImGui::TextColored(BLUE_TEXT, "STATUS FLAG");
            ImGui::SameLine();
            {
                ImGui::TextColored(YELLOW_TEXT, "takes a String Input and can either be:");
            }
            ImGui::TextColored(BLUE_TEXT, "INSTOCK , RESERVED , DAMAGED");
            ImGui::SameLine();
            {
                ImGui::TextColored(YELLOW_TEXT, "or");
                ImGui::SameLine();
                ImGui::TextColored(BLUE_TEXT, "EXPIRED");
            }
            ImVec2 text_size = ImGui::CalcTextSize("ASANTE");
            ImGui::SetCursorPosX(250.0f - (text_size.x/2));
            ImGui::TextColored(GREEN_TEXT, "ASANTE !");


           
        }
        ImGui::End();
    }
    void ConvertHexStringToBytes8_t(const char* hexString, std::vector<uint8_t>& bytes) 
    {
        std::string str(hexString);
        bytes.clear();
        bytes.resize(16, 0);  
        
        
        str.erase(remove_if(str.begin(), str.end(), 
            [](char c) { return !isxdigit(c); }), str.end());
        
        size_t index = 0;
        for (size_t i = 0; i < str.length() - 1 && index < bytes.size(); i += 2) 
        {
            try 
            {
                std::string byteStr = str.substr(i, 2);
                bytes[index++] = static_cast<uint8_t>(std::stoi(byteStr, nullptr, 16));
            }
            catch (const std::exception& e) 
            {
                std::cerr << "Conversion error at index " << i << ": " << e.what() << std::endl;
                break;
            }
        }
    }
    std::string decimaltohex(int data) 
    {
        if (data == 0) return "0";
        
        const char hex[] = "0123456789ABCDEF";
        std::string hex_string;
        
        bool negative = data < 0;
        if (negative) {
            data = -data;
        }
        
        while (data > 0) 
        {
            int remainder = data % 16;
            hex_string = hex[remainder] + hex_string;
            data /= 16;
        }
        
        if (negative) 
        {
            hex_string = "-" + hex_string;
        }
        
        return hex_string;
    }
    uint8_t hexStringToUint8(const std::string& hex_str) 
    {
        if (hex_str.empty()) return 0;
        
        uint8_t value = 0;
        for (char c : hex_str) {
            value *= 16;
            if (c >= '0' && c <= '9') 
            {
                value += c - '0';
            }
            else if (c >= 'A' && c <= 'F') 
            {
                value += c - 'A' + 10;
            }
            else if (c >= 'a' && c <= 'f') 
            {
                value += c - 'a' + 10;
            }
        }
        return value;
    }
    uint16_t hexStringToUint16(const std::string& hex_str) 
    {
        if (hex_str.empty()) return 0;
        
        uint16_t value = 0;
        for (char c : hex_str) {
            value *= 16;
            if (c >= '0' && c <= '9') 
            {
                value += c - '0';
            }
            else if (c >= 'A' && c <= 'F') 
            {
                value += c - 'A' + 10;
            }
            else if (c >= 'a' && c <= 'f') 
            {
                value += c - 'a' + 10;
            }
        }
        return value;
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
    uint16_t product_category_id;
    uint16_t location_code;
    uint8_t quantity;
    uint16_t price;
    uint16_t supplier_id;
    uint16_t batch_number;

    bool success;
    bool window_open = true;
    char buf[64] = "";
    int category_buf = 0;
    int location_buf = 0;
    int quantity_buf = 0;
    int price_buf = 0;
    int supplier_buf = 0;
    int batch_buf = 0;
    char status_buf[10] = "";

    const ImVec4 GREEN_TEXT = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
    const ImVec4 YELLOW_TEXT = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
    const ImVec4 RED_TEXT = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
    const ImVec4 BLUE_TEXT = ImVec4(0.0f, 0.0f, 1.0f, 1.0f);

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
        node->render_new_window();

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


