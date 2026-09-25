# tb20e_control

TB20eのswing、boom、arm、bucketを制御する`ros2_control`パッケージです。
実機またはシミュレータの現在角度をROS 2 state interfaceへ変換し、controllerの出力を
レバー操作率としてpublishします。USBゲームパッドからの直接レバー操作と、Unity向け
位置指令の生成にも対応します。

## 提供する機能

- `Tb20eLeverHardware`: degreeの角度feedbackをposition／velocity stateへ変換
- `tb20e_controller`: `JointTrajectoryController`による4軸PID制御
- `tb20e_gamepad_controller`: `ForwardCommandController`によるレバー直接制御
- `tb20e_gamepad_node`: `/joy`のaxis割当、deadzone、timeout、Unity位置目標積分
- `joint_state_broadcaster`: 4軸状態を`/joint_states`へpublish
- feedback timeout、角度範囲、角度飛び、関節端、非数値に対する安全監視

## Hardware interface

Controller上のjoint順序は固定です。

```text
[swing_joint, boom_joint, arm_joint, bucket_joint]
```

| interface | ROS 2上の単位・意味 |
|---|---|
| `position` state | rad |
| `velocity` state | rad/s |
| `effort` command | 物理トルクではなくレバー操作率 -100～100 |

入力topicの角度はdegreeです。hardware interfaceがradへ変換し、制御周期間の差から速度を
推定します。swingはcontinuous jointとして`[-pi, pi]`に正規化し、境界をまたぐ場合も
最短角度差を使用します。

## 実機用topicと関節範囲

| 軸 | feedback topic | 範囲 [degree] | レバーtopic | 既定レバー符号 |
|---|---|---:|---|---:|
| swing | `/current_swing_angle` | continuous | `/manipulated_swing_lever` | `1.0` |
| boom | `/current_boom_angle` | -83～48 | `/manipulated_boom_lever` | `-1.0` |
| arm | `/current_arm_angle` | 32～155 | `/manipulated_arm_lever` | `1.0` |
| bucket | `/current_bucket_angle` | -31～159 | `/manipulated_bucket_lever` | `1.0` |

すべて`std_msgs/msg/Float64`です。feedbackはdegree、レバーtopicは-100～100です。

## ビルド

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tb20e_control
source install/setup.bash
```

## Trajectory controllerを起動する

```bash
ros2 launch tb20e_control tb20e_control.launch.py
```

以下を起動します。

- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `tb20e_controller`

4軸すべてのfeedbackが起動後2秒以内に届かない場合、hardwareのactivateを拒否して
launch全体を停止します。

動作確認:

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states
```

### FollowJointTrajectoryの例

位置は`[swing, boom, arm, bucket]`順のradです。

```bash
ros2 action send_goal \
  /tb20e_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [swing_joint, boom_joint, arm_joint, bucket_joint], points: [{positions: [0.0, -0.35, 1.40, 0.52], time_from_start: {sec: 5}}]}}"
```

PIDゲインは`config/tb20e_controllers_0.yaml`（0号機）と
`config/tb20e_controllers_1.yaml`（1号機）にあります。現在の値は初期調整値であり、
実機で同定された値ではありません。

## USBゲームパッドを起動する

```bash
ros2 launch tb20e_control tb20e_gamepad.launch.py
```

既定の割当:

| 入力 | 軸 | Joy axis | scale |
|---|---|---:|---:|
| 左スティック左右 | swing | 0 | `100.0` |
| 左スティック上下 | arm | 1 | `-100.0` |
| 右スティック左右 | bucket | 2 | `100.0` |
| 右スティック上下 | boom | 3 | `100.0` |

armはゲームパッドの上方向と機体の正方向を合わせるため既定で反転しています。
入力値はdeadzone外を再スケーリングし、-100～100へ変換します。

| 引数 | 既定値 | 説明 |
|---|---:|---|
| `<axis>_axis` | swing=0, arm=1, bucket=2, boom=3 | Joy axis番号 |
| `<axis>_scale` | armのみ`-100.0`、他は`100.0` | 最大操作率と方向 |
| `deadzone` | `0.10` | stick中央の無効範囲 |
| `joy_timeout_sec` | `0.25` | Joy途絶時に0へ戻す時間 |
| `neutral_hold_sec` | `0.5` | 操作開始前に全軸を中央で維持する時間 |
| `joy_device_id` | `0` | `joy_node`のdevice ID |

DモードのF310では接続直後に軸0～3が一時的に最大入力として見える場合があります。
起動・再接続後は全スティックを中央に戻し、button 9（Start）を離した状態で0.5秒待ってから
Startを一度押してください。押した次のJoyメッセージから操作を受け付けます。
Startをもう一度押すとスティック位置に関係なく指令は0になります。再開時はStartを離し、
スティックを中央で0.5秒維持してから押してください。Joy入力が途絶える、または入力形式が
不正になる場合も停止します。button番号は`jstest --event /dev/input/js0`で確認できます。
Start（button 9）は固定の開始・停止ボタンです。

入力確認:

```bash
ros2 topic echo /joy
```

`tb20e_control.launch.py`と`tb20e_gamepad.launch.py`を同時起動しないでください。
同じ4本のeffort command interfaceを使用します。

## ゲームパッドからUnity位置指令を生成する

`tb20e_gamepad.launch.py`単独ではUnity位置出力は既定で無効です。有効にする場合:

```bash
ros2 launch tb20e_control tb20e_gamepad.launch.py \
  command_output_enabled:=false \
  unity_position_output_enabled:=true
```

このlaunchは`ros2_control`も起動するため、`command_output_enabled:=false`でもhardware用の
4軸feedbackが必要です。実機未接続時のstate topic指定は
[`../tb20e_bringup/README.md`](../tb20e_bringup/README.md)を参照してください。

Unity feedbackを初期値としてstick量を位置変化へ積分し、次の絶対topicへradでpublishします。

```text
/TB20e/swing/cmd
/TB20e/boom/cmd
/TB20e/arm/cmd
/TB20e/bucket/cmd
```

| 引数 | 既定値 | 説明 |
|---|---:|---|
| `<axis>_sim_state_topic` | `/sim/TB20e_0/current_<axis>_angle` | Unity現在角、degree |
| `sim_feedback_timeout_sec` | `0.25` | Unity feedback timeout |
| `<axis>_unity_speed_deg_s` | `50.0` | stick 100%時の目標変化速度 |
| `swing_unity_position_sign` | `-1.0` | Unity swing座標変換 |
| その他の`<axis>_unity_position_sign` | `1.0` | Unity各軸座標変換 |

Unityと実機でswingの正方向が逆なので、swingだけfeedbackを内部座標へ取り込む際と
位置指令を出力する際の両方で符号反転します。符号は`-1.0`または`1.0`です。

Unity feedbackが欠落または古くなった場合、位置積分とUnity向けpublishを停止します。
復旧時は最新feedbackから目標を再初期化するため、停止中のstick入力を後からまとめて
適用することはありません。swingはwrapし、他軸は設定された関節範囲でclampします。

## Hardware parameter

`tb20e_control.launch.py`の引数からxacro経由でhardware interfaceへ渡します。

### 全軸共通

| 引数 | 既定値 | 説明 |
|---|---:|---|
| `command_output_enabled` | `true` | `false`では全レバーtopicへ0をpublish |
| `state_timeout_sec` | `0.10` | 運転中feedback timeout [s] |
| `initial_feedback_wait_sec` | `2.0` | activate時に4軸feedbackを待つ時間 [s] |
| `feedback_limit_tolerance_deg` | `2.0` | 関節範囲外feedbackの許容差 [degree] |
| `max_feedback_velocity_deg_s` | `180.0` | 角度飛び判定の上限 [degree/s] |
| `feedback_velocity_jitter_tolerance_sec` | `0.03` | 速度判定で許容する受信時刻の揺らぎ [s]。`0`で厳密なサンプル間判定 |

### 軸ごと

- `<axis>_state_topic`
- `<axis>_command_topic`
- `<axis>_lever_sign`
- `<axis>_lever_min`
- `<axis>_lever_max`

レバー符号は`-1.0`または`1.0`、出力範囲は`min <= 0 <= max`かつ`min < max`を
満たす必要があります。

設定例:

```bash
ros2 launch tb20e_control tb20e_control.launch.py \
  command_output_enabled:=false \
  state_timeout_sec:=0.20 \
  initial_feedback_wait_sec:=3.0 \
  boom_state_topic:=/sim/TB20e_0/current_boom_angle \
  boom_lever_min:=-60.0 \
  boom_lever_max:=60.0
```

## 安全監視

以下を検出すると全4軸のレバー出力を0へラッチします。

- 1軸以上のfeedback欠落またはtimeout
- NaN／Infinity
- 許容差を超えた関節範囲外feedback
- `max_feedback_velocity_deg_s`を超える角度飛び
- controllerからの非数値command

有限関節では端点から0.5 degree以内で、さらに範囲外へ向かうcommandだけを抑止します。
一度ラッチしたfaultは通信復旧だけでは自動解除しません。

復旧例:

```bash
ros2 control set_controller_state tb20e_controller inactive
ros2 control set_hardware_component_state Tb20eLeverSystem inactive
ros2 control set_hardware_component_state Tb20eLeverSystem active
ros2 control set_controller_state tb20e_controller active
```

状態遷移に失敗する場合はlaunch一式を再起動し、新しいtrajectory goalを送ってください。

## テスト

```bash
cd ~/ros2_ws
colcon test --packages-select tb20e_control --event-handlers console_direct+
colcon test-result --verbose
```

## 実機使用時の注意

既定PIDゲインは実機で同定された値ではありません。実機では独立した非常停止、油圧ロック、
通信watchdog、速度・圧力・作業範囲制限、起動時のneutral確認を用意してください。
software timeout、clamp、`command_output_enabled`は独立安全機構の代わりにはなりません。

## License

Apache License 2.0。詳細は[../LICENSE](../LICENSE)を参照してください。

### Unityの角度フィードバックと受信揺らぎ

`Float64`には送信時刻がないため、速度判定にはsteady clockの受信間隔を使用します。
Unity / TCP経由でメッセージがまとめて届く場合に備え、各サンプルの絶対角度変化から
`上限速度 × 受信間隔`を引いた超過量を累積し、負になった場合は0に戻します。
超過量が`上限速度 × feedback_velocity_jitter_tolerance_sec`を超えると停止します。
既定値では180 deg/sに対して5.4度の受信揺らぎを許容します。
継続的な超過や大きなジャンプは検出しますが、短い速度超過の検出には遅延が加わります
（例: 240 deg/sの連続運動は約90 msの超過蓄積後、次の観測で検出）。
角度範囲外・非有限値・受信タイムアウトの停止条件はそのままです。
トピック名の変更は不要です。実機とUnityが同じトピックへ同時にpublishしない構成で検証してください。

### 実機レバーの不感帯補償

各軸の `lever_positive_min` / `lever_negative_min` は、符号変換後の
正方向・負方向の最低操作率（正の大きさ）です。既定値0はその方向の補償無効。
`lever_start`（既定2）以上のPID出力で補償を開始し、`lever_stop`（既定1）以下で
ゼロに戻します。単位は補償前のレバー%、角度ではありません。反転時は開始判定を
やり直します。最低操作率以上の出力はそのまま、ゼロ・安全停止・関節端の抑止は
ゼロのままです。最低操作率が出力上限を超える設定は起動時に拒否します。

PIDと同じ `config/tb20e_controllers_0.yaml` の
`/**/tb20e_lever_hardware.ros__parameters` を編集して再起動すれば反映されます
（再ビルド不要）。負方向も正の大きさ（例: -48なら48）で記入します。
バケット正方向だけ45を設定し、他軸・負方向は無効です。
launchは `controllers_file` で指定されたYAMLを読み込みます。補償セクションのない
ファイルは補償無効の既定値を使います。明示的な起動引数はYAMLより優先します。
例: `bin/run_scratch_0.sh bucket_lever_positive_min:=0.0`。
以前の `config/lever_compensation_0.sh` は廃止しました。

このhardware層は目標角度を受け取らないため、角度許容差による停止判定ではありません。
PIDゲインやI項を変更した場合は、残留出力と開始・停止しきい値の関係を再調整します。
100 msのフィードバック監視と故障ラッチは変更していません。
