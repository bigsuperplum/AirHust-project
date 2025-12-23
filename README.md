# 说明

本工作空间内可能要用到某些头文件，限于篇幅请按照编译时的错误提示自行安装，再编译本工作空间

起飞：
```bash
roslaunch sim_task task.launch
```
打开另一个窗口，运行
```bash
roslaunch work template.launch
```
（本 package 实现起飞、避障、穿门、穿环功能，然后在原地降落）

视觉识别也已在本工作空间内实现：
```bash
roslaunch sim_task task.launch
```
打开另一个窗口，运行
```bash
roslaunch vision vision.launch
```
（本节点运行需要手动删除地图中所有的障碍物：柱子、门、环和几乎所有的墙，否则会翻车）

（本节点可以在原点起飞，基本实现要求的识别任务，并按照要求降落）

（本节点运行需要联网）



