cmake_minimum_required(VERSION 3.8)
project(r2_ctrl)

# ---- コンパイラ警告 ----
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 必須パッケージ ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 自作 msg を使っているので必須
find_package(robomas_package_2 REQUIRED)

# ---- 実行ファイル ----
add_executable(r2_ctrl_node src/r2_ctrl.cpp)

# ---- 依存関係 ----
ament_target_dependencies(r2_ctrl_node
  rclcpp
  sensor_msgs
  robomas_package_2
)

# ---- インストール ----
install(TARGETS
  r2_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- ament ----
ament_package()