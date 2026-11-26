cmake_minimum_required(VERSION 3.8)
project(robo_bt_plugins)
# CAMINHO_ORIGINAL (11/11/2026): Patch 01 do refinamento da navegacao.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

# nucleo sem ROS em tempo de execucao (so Costmap2D): testavel offline
add_library(caminho_original_core STATIC src/caminho_original.cpp)
set_target_properties(caminho_original_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(caminho_original_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(caminho_original_core nav2_costmap_2d nav2_util)

# plugin de BT (carregado pelo bt_navigator via plugin_lib_names)
add_library(robo_caminho_original_seguro_bt_node SHARED src/caminho_original_seguro_condition.cpp)
target_link_libraries(robo_caminho_original_seguro_bt_node caminho_original_core)
target_compile_definitions(robo_caminho_original_seguro_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(robo_caminho_original_seguro_bt_node rclcpp behaviortree_cpp nav2_behavior_tree
  nav2_costmap_2d nav2_util nav2_msgs nav_msgs geometry_msgs tf2 tf2_ros std_msgs)

# GOAL_TF_ATUAL (11/11): caminho para o FollowPath com carimbo zero (biblioteca separada; inerte sem o patch)
add_library(robo_caminho_sem_carimbo_bt_node SHARED src/caminho_sem_carimbo_action.cpp)
target_compile_definitions(robo_caminho_sem_carimbo_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(robo_caminho_sem_carimbo_bt_node behaviortree_cpp nav2_behavior_tree nav_msgs)

# ferramenta offline: avalia casos exportados do bag (compara com o modelo Python)
add_executable(avaliar_casos_caminho src/avaliar_casos.cpp)
target_link_libraries(avaliar_casos_caminho caminho_original_core)

install(TARGETS robo_caminho_original_seguro_bt_node robo_caminho_sem_carimbo_bt_node caminho_original_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS avaliar_casos_caminho RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_caminho_original test/test_caminho_original.cpp)
  target_link_libraries(test_caminho_original caminho_original_core)
  ament_add_gtest(test_bt_caminho_original test/test_bt_caminho_original.cpp TIMEOUT 120)
  target_link_libraries(test_bt_caminho_original robo_caminho_original_seguro_bt_node caminho_original_core)
  ament_target_dependencies(test_bt_caminho_original rclcpp behaviortree_cpp nav2_behavior_tree nav2_costmap_2d
    nav2_msgs nav_msgs geometry_msgs tf2_ros std_msgs)
  ament_add_gtest(test_bt_caminho_sem_carimbo test/test_bt_caminho_sem_carimbo.cpp)
  add_dependencies(test_bt_caminho_sem_carimbo robo_caminho_sem_carimbo_bt_node)
  target_compile_definitions(test_bt_caminho_sem_carimbo PRIVATE
    LIB_SEM_CARIMBO="$<TARGET_FILE:robo_caminho_sem_carimbo_bt_node>")
  ament_target_dependencies(test_bt_caminho_sem_carimbo behaviortree_cpp nav_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(robo_caminho_original_seguro_bt_node robo_caminho_sem_carimbo_bt_node)
ament_package()
