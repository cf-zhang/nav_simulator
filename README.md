# 一起动手写ROS仿真器
![Simulator](source/images/simulator.png)

本仓库是《一起动手写ROS仿真器》系列教程的代码和示例集合。这个教程旨在帮助你从零开始，逐步添加功能，最终完整实现一个用于ROS平台**机器人导航规划控制功能**的ROS仿真器

## 简介
ROS（Robot Operating System）是一个广泛使用的机器人开发平台，提供了一系列工具和库，用于构建机器人应用程序。在本教程中，我们将使用ROS来开发一个仿真器，模拟机器人在虚拟环境中的运动和感知。

## 教程目录

[bilibili](https://space.bilibili.com/554016964/channel/collectiondetail?sid=1560370)

[在线文档](https://nav-simulator.readthedocs.io/en/latest/)

## 安装依赖
在开始教程之前，请确保教程所使用的环境：

操作系统：  Ubuntu 20.04

ROS:&nbsp;&nbsp;&nbsp;&nbsp; galactic


## 使用方法

### 路径规划器仿真
```
# 编译
git clone https://github.com/cf-zhang/nav_simulator
cd nav_simulator
colcon build
source install/setup.bash
# 启动仿真器
ros2 launch simulator planner_simulator.launch.py
# 另一个终端启动仿真器
ros2 launch planner planner.launch.py
```


### others
ToDo


## 贡献
`一个人走的快，众人走的远`

欢迎对本教程提出问题、反馈和改进建议。如果您发现任何错误或问题，请随时提交Issue或Pull Request。

贡献列表

| name      | email | ROS-Distribution |
|:-----------:|:----:|:------------:|
| [cf-zhang](https://github.com/cf-zhang)    | chuanfazhang1992@gmail.com   | ROS2 - Galactic  |
| [nicklgw](https://github.com/nicklgw)      | nicklgw@aliyun.com   | ROS2 - Humble    |
| [emoPointer](https://github.com/emoPointer)| 2486547986@qq.com   | ROS - Noetic    |


持续为其他ROS版本招募各路伙伴共创，如果你热爱机器人开发，有强烈的责任心，
请通过下面的微信二维码联系我，并参与进来。参与方式可以参考： [fork_and_pullrequest](./source/docs/fork_and_pr.md)

<img src="./source/images/vx.jpg" alt="微信" width="300">

或者通过邮箱联系到我：` chuanfazhang1992@gmail.com `

## 许可证
本教程的代码遵循MIT许可证，详情请参见 LICENSE 文件。

---

*本教程旨在帮助大家学习ROS仿真器的构建，如果有任何侵权行为，请及时联系作者删除。谢谢！*

