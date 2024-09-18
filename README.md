# 実行例
`ros2_ws/src`下にcloneする。
```
cd ros2_ws/src/
git clone https://github.com/hida-yat/ros2.git
```
ビルドして実行する
```
cd ros2_ws
colcon build --symlink-install --packages-select urdf_test
source install/setup.bash
ros2 launch urdf_test gazebo.launch.py
```
gazeboが起動してロボットモデルが確認できる。差動二輪のgazeboプラグインがロードされているので、/cmd_velトピックに指令を送ることで、ロボットモデルは動き出す。

`teleop_twist_keyboard`でロボットモデルを動かすとすると、別タブでターミナルを起動し、
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
