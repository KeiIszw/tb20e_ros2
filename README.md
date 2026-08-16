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

HTTP入力を使う場合:

```bash
ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=http real_output_enabled:=true
```

Docker環境では、上記の`ros2 launch`を次のように実行できます。

```bash
cd ~/ros2_docker
docker compose -f compose.yaml -f compose.gamepad.yaml exec ros2 \
  bash -ic 'ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
    input_source:=gamepad real_output_enabled:=false'
```

詳細な起動モード、Unity単独運用、topic切替、トラブルシュートは
[`tb20e_bringup/README.md`](tb20e_bringup/README.md)を参照してください。

## License

Apache License 2.0。詳細は[LICENSE](LICENSE)を参照してください。
