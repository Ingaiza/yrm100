cmake_minimum_required(VERSION 3.8)
project(yrm100)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set OpenGL preference early
set(OpenGL_GL_PREFERENCE GLVND)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(my_rfid_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# Create ImGui library with ALL source files
add_library(ImGui STATIC
 src/imgui.cpp
 src/imgui_draw.cpp
 src/imgui_tables.cpp
 src/imgui_widgets.cpp
 src/imgui_impl_opengl3.cpp
 src/imgui_impl_glfw.cpp
)

# Set include directories for ImGui
target_include_directories(ImGui PUBLIC
 ${CMAKE_CURRENT_SOURCE_DIR}/include/yrm100
 ${OPENGL_INCLUDE_DIRS}
 ${GLFW_INCLUDE_DIRS}
)

# Compile options for ImGui
target_compile_options(ImGui PRIVATE
 -fPIC
 -std=c++11
)

# Add main executables
add_executable(module_server
 src/module_server.cpp
 src/uhf_functions.cpp
 src/uhf_buffer.cpp
 src/uhf_module.cpp
 src/uhf_serial.cpp
 src/uhf_tag.cpp
)

add_executable(module_client src/module_client.cpp)

add_executable(command_client 
src/command_client.cpp 
src/hextostring.cpp
)

# Link command_client with all necessary libraries
target_link_libraries(command_client
 ImGui
 ${OPENGL_LIBRARIES}
 glfw
)

# Include directories for command_client
target_include_directories(command_client PRIVATE
 ${CMAKE_CURRENT_SOURCE_DIR}/include/yrm100
 ${OPENGL_INCLUDE_DIRS}
 ${GLFW_INCLUDE_DIRS}
)

# Link libraries and set include directories for module_server
target_link_libraries(module_server ${Boost_LIBRARIES})
target_include_directories(module_server PRIVATE ${Boost_INCLUDE_DIRS})

# Add ROS dependencies
ament_target_dependencies(module_server
 rclcpp
 rclcpp_action
 my_rfid_interfaces
)

ament_target_dependencies(module_client
 rclcpp
 rclcpp_action
 my_rfid_interfaces
)

ament_target_dependencies(command_client
 rclcpp
 my_rfid_interfaces
)

# Testing configuration
if(BUILD_TESTING)
 find_package(ament_lint_auto REQUIRED)
 set(ament_cmake_copyright_FOUND TRUE)
 set(ament_cmake_cpplint_FOUND TRUE)
 ament_lint_auto_find_test_dependencies()
endif()

# Install executables
install(TARGETS
 module_server
 module_client
 command_client
 DESTINATION lib/${PROJECT_NAME}
)

ament_package()