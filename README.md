# tb20e_ros2

Takeuchi TB20eの4軸（swing、boom、arm、bucket）をROS 2 Humbleから操作するための
パッケージ群です。実機向けレバー指令とUnity向け位置指令を、同じゲームパッドまたは
HTTP入力から生成できます。

## パッケージ

- [`tb20e_control`](tb20e_control/README.md): `ros2_control` hardware interface、
  trajectory controller、ゲームパッド入力、安全監視
- [`tb20e_bringup`](tb20e_bringup/README.md): 実機とUnityをまとめて起動し、
  gamepad／HTTP入力を選択するlaunch

HTTP入力には、同じワークスペースに`scratch_hci_bridge`パッケージが必要です。

## 主なtopic

| 用途 | topic | 型・単位 |
|---|---|---|
| 実機現在角 | `/current_<axis>_angle` | `std_msgs/msg/Float64`, degree |
| 実機レバー指令 | `/manipulated_<axis>_lever` | `std_msgs/msg/Float64`, -100～100 |
| Unity現在角 | `/sim/tb20e/current_<axis>_angle` | `std_msgs/msg/Float64`, degree |
| Unity位置指令 | `/tb20e/<axis>/cmd` | `std_msgs/msg/Float64`, rad |

`<axis>`は`swing`、`boom`、`arm`、`bucket`です。

## ビルド

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to tb20e_bringup
source install/setup.bash
```

## 実行

最初は実機出力を無効にしてtopicと方向を確認します。

```bash
ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=gamepad real_output_enabled:=false
```

確認後に実機出力を有効化します。

```bash
ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=gamepad real_output_enabled:=true
```

Docker環境では、上記の`ros2 launch`を次のように実行できます。

```bash
cd ~/ros2_docker
docker compose -f compose.yaml -f compose.gamepad.yaml exec ros2 \
  bash -ic 'ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
    input_source:=gamepad real_output_enabled:=false'
```

### HTTP制御のノード起動例

HTTPモードでは、次のlaunchで`ros2_control_node`、`robot_state_publisher`、
`joint_state_broadcaster`／`tb20e_controller`のspawner、およびHTTP受付ノード
`scratch_hci_bridge`をまとめて起動します。bridgeを別途起動する必要はありません。
ゲームパッド用launchを終了してから切り替えてください。

同じワークスペースに`scratch_hci_bridge`のソースを配置し、初回は次を実行します。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to tb20e_bringup scratch_hci_bridge
source install/setup.bash
```

**Unityが実機と同じ`/current_*_angle`をpublishする検証環境:**

UnityとROS-TCP-Endpointを先に起動し、4軸の角度feedbackが届く状態にします。
実機レバー出力を無効にし、Unityへの位置指令を有効にして起動します。

```bash
ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=http \
  real_output_enabled:=false \
  unity_position_output_enabled:=true \
  http_host:=0.0.0.0 http_port:=8899
```

HTTPモードでは`*_sim_state_topic`の指定は不要です。hardwareは既定の
`/current_*_angle`を購読します。Unityが`/sim/tb20e/current_*_angle`を使う場合は、
上のコマンドに次の4引数を追加します。

```text
swing_state_topic:=/sim/tb20e/current_swing_angle
boom_state_topic:=/sim/tb20e/current_boom_angle
arm_state_topic:=/sim/tb20e/current_arm_angle
bucket_state_topic:=/sim/tb20e/current_bucket_angle
```

**実機とUnityを同時にHTTP制御する場合:**

実機の4軸feedbackが届く状態で、実機レバー出力を有効にします。

```bash
ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=http \
  real_output_enabled:=true \
  unity_position_output_enabled:=true \
  http_host:=0.0.0.0 http_port:=8899
```

**DockerでのUnity検証用起動例:**

```bash
cd ~/ros2_docker
docker compose exec ros2 bash -ic \
  'source /home/ros/ros2_ws/install/setup.bash && \
   ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
     input_source:=http real_output_enabled:=false \
     unity_position_output_enabled:=true \
     http_host:=0.0.0.0 http_port:=8899'
```

HTTP制御だけなら`compose.gamepad.yaml`は不要です。現在の`compose.yaml`では
ホストの`localhost:8899`からHTTP受付に接続できます。
起動後は別ターミナルでノード、controller、Actionを確認できます。

```bash
ros2 node list
ros2 control list_controllers
ros2 action info /tb20e_controller/follow_joint_trajectory
```

`tb20e_controller`が`active`で、Action serverが1つあることを確認します。
Dockerではこれらも`docker compose exec ros2 bash -ic '<コマンド>'`で実行します。

詳細な起動モード、Unity単独運用、topic切替、トラブルシュートは
[`tb20e_bringup/README.md`](tb20e_bringup/README.md)を参照してください。

## License

Apache License 2.0。詳細は[LICENSE](LICENSE)を参照してください。
