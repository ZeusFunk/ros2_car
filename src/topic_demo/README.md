# Topic 测试demo

## 编译并运行

```bash
cd ~/ros2_car
colcon build --packages-select test_topic
source install/setup.bash
```

## 运行发布者​（新终端）：

```
 bash ros2 run test_topic publisher
```

## （另一新终端）：

```
bash ros2 run test_topic subscriber
```

## 或者使用Launch ：

```
ros2 launch topic_demo testtopic.launch.py
```