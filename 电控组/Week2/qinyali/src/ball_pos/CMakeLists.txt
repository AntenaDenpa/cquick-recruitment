cmake_minimum_required(VERSION 3.8)
project(ball_pos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ① 把 msg/BallPos.msg 编译成消息类型
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# ② 把 scripts/ 下的 Python 脚本装成可执行程序，放进 lib/ball_pos
install(PROGRAMS
  scripts/ball_pos_publisher.py
  scripts/ball_pos_subscriber.py
  scripts/go_to_balls.py
  scripts/go_to_balls_openloop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
