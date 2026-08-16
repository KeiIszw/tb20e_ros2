# tb20e_bringup

実機とUnityへ同じ操作を送るための統合launchパッケージです。
`tb20e_real_unity.launch.py`が入力元を1つだけ選び、`tb20e_control`と
`scratch_hci_bridge`を必要な組み合わせで起動します。

## 制御経路

### ゲームパッド

```text
/joy
  ├─> /tb20e_gamepad_controller/commands
  │     -> ros2_control
  │     -> /manipulated_<axis>_lever（実機）
  └─> Unity feedbackを基準に位置目標へ積分
        -> /tb20e/<axis>/cmd（Unity）
```

### HTTP

```text
HTTP command
  -> scratch_hci_bridge
  -> /tb20e_controller/follow_joint_trajectory
       ├─> ros2_control PID
       │     -> /manipulated_<axis>_lever（実機）
       └─> goal受理後
             -> /tb20e/<axis>/cmd（Unity）
```

HTTPの4軸Unity指令は、FollowJointTrajectory goalが受理された後にpublishされます。
Action serverが存在しない場合やgoalが拒否された場合はUnityへも送信されません。

## Topic契約

| 用途 | 既定topic | 型 | 単位 |
|---|---|---|---|
| 実機feedback | `/current_<axis>_angle` | `std_msgs/msg/Float64` | degree |
| 実機レバー指令 | `/manipulated_<axis>_lever` | `std_msgs/msg/Float64` | -100～100 |
| Unity feedback | `/sim/tb20e/current_<axis>_angle` | `std_msgs/msg/Float64` | degree |
| Unity位置指令 | `/tb20e/<axis>/cmd` | `std_msgs/msg/Float64` | rad |

Unity位置指令はnode namespaceに依存しない絶対topic名です。prefixに先頭の`/`がない
設定を渡した場合も、`/tb20e/<axis>/cmd`の形式へ正規化されます。

Unityと実機ではswingの正方向が逆であるため、Unityのswing feedbackと位置指令だけを
既定で`-1.0`倍します。実機向けレバー指令の方向はこの変換の影響を受けません。

## ビルド

`scratch_hci_bridge`も同じワークスペースへ配置して実行します。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select tb20e_control scratch_hci_bridge tb20e_bringup
source install/setup.bash
```

## 起動モード

### ゲームパッド、実機出力無効

初回確認用です。`/manipulated_*_lever`には常に0がpublishされます。ただし
`ros2_control`の起動には4軸すべてのfeedbackが必要です。

```bash
ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=gamepad \
  real_output_enabled:=false
```

### ゲームパッド、実機とUnityを同時操作

実機の非常停止、符号、範囲、neutralを確認してから有効化してください。

```bash
ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=gamepad \
  real_output_enabled:=true
```

### HTTP、実機とUnityを同時操作

```bash
ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=http \
  real_output_enabled:=true \
  http_host:=0.0.0.0 \
  http_port:=8899
```

`input_source`は`gamepad`または`http`のどちらか一方です。2つの入力源から同時に
4軸を操作しない構成になっています。

## 実機未接続でUnityだけを操作する

`real_output_enabled:=false`でもhardware interfaceはfeedbackを必要とします。
Unityをfeedback源として使うため、実機用state topicもUnity側へ向けます。

Unityが新topic `/sim/tb20e/current_*_angle` をpublishする場合:

```bash
ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=gamepad \
  real_output_enabled:=false \
  swing_state_topic:=/sim/tb20e/current_swing_angle \
  boom_state_topic:=/sim/tb20e/current_boom_angle \
  arm_state_topic:=/sim/tb20e/current_arm_angle \
  bucket_state_topic:=/sim/tb20e/current_bucket_angle
```

Unityが旧topic `/current_*_angle` をpublishしている場合は、hardware側は既定値のままにし、
ゲームパッド位置積分側のtopicだけ合わせます。

```bash
ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=gamepad \
  real_output_enabled:=false \
  swing_sim_state_topic:=/current_swing_angle \
  boom_sim_state_topic:=/current_boom_angle \
  arm_sim_state_topic:=/current_arm_angle \
  bucket_sim_state_topic:=/current_bucket_angle
```

Unityを先に起動し、4本のfeedbackが連続してpublishされている状態でlaunchしてください。
HTTPモードのUnity単独運用でも、Action goalを受理するcontrollerを動作させるため、同様に
hardwareの4本のstate topicをUnity feedbackへ向ける必要があります。

## 主なlaunch引数

| 引数 | 既定値 | 説明 |
|---|---:|---|
| `input_source` | `gamepad` | `gamepad`または`http` |
| `real_output_enabled` | `false` | 実機レバー出力の安全ゲート |
| `unity_position_output_enabled` | `true` | Unity位置指令の有効化 |
| `joy_device_id` | `0` | `joy_node`のdevice ID |
| `deadman_button` | `-1` | 押下中だけ操作を許可するbutton番号。`-1`は無効 |
| `http_host` | `0.0.0.0` | HTTP serverのbind先 |
| `http_port` | `8899` | HTTP serverのport |
| `<axis>_state_topic` | `/current_<axis>_angle` | hardware／実機用feedback |
| `<axis>_sim_state_topic` | `/sim/tb20e/current_<axis>_angle` | ゲームパッド積分用Unity feedback |
| `sim_feedback_timeout_sec` | `0.25` | Unity feedbackのtimeout |
| `<axis>_unity_speed_deg_s` | `50.0` | stick 100%時のUnity目標変化速度 |
| `swing_unity_position_sign` | `-1.0` | Unity swing座標の符号 |
| その他の`<axis>_unity_position_sign` | `1.0` | Unity各軸座標の符号 |
| `use_sim_time` | `false` | ROS simulation timeの使用 |

符号parameterは`-1.0`または`1.0`を指定します。

## 起動前確認

```bash
ros2 topic list | grep -E 'current_(swing|boom|arm|bucket)_angle'
ros2 topic hz /current_arm_angle
ros2 topic hz /sim/tb20e/current_arm_angle
ros2 topic echo /joy
```

実機とUnityを同時接続する場合、実機feedbackとUnity feedbackのtopicを分離してください。
同名topicへ両方がpublishすると、異なる角度が混在して安全監視が停止する可能性があります。

## 停止する場合

次のログは、指定した4本のfeedbackが起動後2秒以内に揃っていないことを示します。

```text
Activation refused: all four angle topics must provide fresh feedback within 2.000 s
```

以下を確認します。

1. Unityまたは実機を先に起動しているか。
2. `ros2 topic list`に指定topicが存在するか。
3. 型が`std_msgs/msg/Float64`、単位がdegreeか。
4. 4軸すべてが継続的にpublishされているか。
5. `ROS_DOMAIN_ID`、ROS TCP Endpoint、Firewallが一致しているか。

ゲームパッドモードではUnity feedbackが`sim_feedback_timeout_sec`を超えると、Unity位置目標の
積分とpublishを停止します。Joyが0.25秒以上途絶えた場合はレバー入力を0へ戻します。

## 安全上の注意

- `real_output_enabled`の既定値は`false`です。
- 実機出力を有効にする前に独立した非常停止と油圧ロックを確認してください。
- ゲームパッドとHTTPを同時に4軸指令源として使用しないでください。
- Unityでは同じ関節にレバー制御と位置制御を同時適用しないでください。
- software timeoutやclampは実機の安全機構の代わりにはなりません。
