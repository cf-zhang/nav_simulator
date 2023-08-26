# 虚拟时钟  
## 开发环境
以`nav_simulator`作为工作空间，在`nav_simulator`文件夹下创建`src`文件夹
```
git clone https://github.com/cf-zhang/nav_simulator.git
cd nav_simulator
git checkout noetic  ##切换到noetic分支下
mkdir src
cd src
catkin_init_workspace
```
创建名为`clock_node`的软件包,依赖为`roscpp`、`rospy`、`std_msgs`、`rosgraph_msgs`
```
cd nav_simulator/src
catkin_creat_pkg clock_node roscpp rospy std_msgs rosgraph_msgs
```
## 代码
clock_node.cpp:
```
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

class ClockPublisher
{
public:
  ClockPublisher()
  {
    publisher_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);
    timer_ = nh_.createTimer(ros::Duration(0.01), &ClockPublisher::timerCallback, this);
    message_.clock = ros::Time(0);
    ROS_INFO("Virtual Clock has been created");
  }

private:
  void timerCallback(const ros::TimerEvent&)
  {
    message_.clock += ros::Duration(0.01);
    publisher_.publish(message_);
  }

  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Publisher publisher_;
  rosgraph_msgs::Clock message_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clock_publisher");
  ClockPublisher clockPublisher;
  ros::spin();
  return 0;
}
```
CMakeLists.txt:
```
cmake_minimum_required(VERSION 3.0.2)
project(clock_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosgraph_msgs
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES clock_node
  CATKIN_DEPENDS roscpp rosgraph_msgs rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/clock_node.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
```
## 功能验证
```
cd nav_simulator
catkin_make
##重新打开一个终端启动主节点 roscore
rosrun clock_node clock_node
rostop echo /clock  ##打印/clock话题的消息
```
效果如下：
```
---
clock: 
  secs: 3
  nsecs: 990000000
---
clock: 
  secs: 4
  nsecs:         0
---
clock: 
  secs: 4
  nsecs:  10000000
---
clock: 
  secs: 4
  nsecs:  20000000
---
clock: 
  secs: 4
  nsecs:  30000000
```