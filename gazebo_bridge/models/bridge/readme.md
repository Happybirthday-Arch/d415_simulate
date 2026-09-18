打开gazebo模型的终端命令为：

cd ~/visual_camera_simulate/gazebo_bridge
export GAZEBO_MODEL_PATH="$PWD/models"
gazebo --verbose worlds/bridge.world

第一条进入目录，第三条启动世界文件。
第二条告诉 Gazebo：“去哪个目录找模型。”

- $PWD：终端当前所在目录。执行第一条后，就是 /home/lv/visual_camera_simulate/gazebo_bridge。
- "$PWD/models"：对应这个目录下的 models 文件夹。
- GAZEBO_MODEL_PATH：Gazebo 用来查找模型的环境变量。
- export：让随后从这个终端启动的 Gazebo 能读取这个变量。

清理gazebo后台内存，在出现打不开模型的时候用：
  pkill -x gzclient

  pkill -x gzserver
