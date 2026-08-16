# tb20e_ros2

Takeuchi TB20e の Unity シミュレータを ROS 2 の
`joint_trajectory_controller` から PID 制御するためのパッケージです。
ROS 2 Humble / Ubuntu 22.04 を対象にしています。

## 構成

`tb20e_control` は次のデータ経路を提供します。

1. Unity が各関節の現在角度を `std_msgs/msg/Float64`（degree）で配信する。
2. hardware interface が角度を radian に変換し、制御周期間の角度差から速度を推定する。
3. `joint_trajectory_controller` が位置・速度を使って PID を計算する。
4. PID 出力をレバー操作量として `[-100, 100]` に制限し、20 Hz（50 ms）で Unity へ返す。

`effort` command interface は PID 出力を hardware interface へ渡すために使用します。
ここでの値は物理的なトルクではなく、レバー操作率（percent）です。
Unity側ではレバー操作率100%を100 degree/sとして扱い、URDFの全関節の
速度上限も100 degree/sに合わせています。

スイング角は `[-pi, pi]` に正規化し、速度計算時にも `+pi` と `-pi` の境界を
最短角度差で処理します。hardware の activate 時は、4軸すべての新しい角度feedbackが
揃うまで待機します。運転中に1軸でもfeedbackの欠落・timeout・非数・許容範囲外・
不自然な角度飛びを検出した場合、4軸すべての出力を `0.0` にラッチします。
通信が復旧しても自動再開せず、hardwareを再activateして新しい軌道goalを送る
必要があります。

非continuous軸は、関節端から0.5 degree以内で外向きのレバー指令を抑止します。

## Topic

| 軸 | Unity から受信する現在角度 | 角度範囲 [degree] | Unity へ送るレバー操作量 | 既定の符号 |
|---|---|---:|---|---:|
| swing | `/current_swing_angle` | -180 ～ 180 | `/manipulated_swing_lever` | +1 |
| boom | `/current_boom_angle` | -83 ～ 48 | `/manipulated_boom_lever` | -1 |
| arm | `/current_arm_angle` | 32 ～ 155 | `/manipulated_arm_lever` | +1 |
| bucket | `/current_bucket_angle` | -31 ～ 159 | `/manipulated_bucket_lever` | +1 |

すべて `std_msgs/msg/Float64` です。現在角度は Unity 側で 5 ms 周期、レバー操作量は
ROS 側で 50 ms 周期を想定しています。

## ビルド

ROS 2 ワークスペースの `src` 以下へ clone して依存関係を導入します。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/KeiIszw/tb20e_ros2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --packages-select tb20e_control
source install/setup.bash
```

## 起動

先に Unity 側で ROS 接続と TB20e シーンを起動し、次を実行します。

```bash
ros2 launch tb20e_control tb20e_control.launch.py
```

launch は以下をまとめて起動します。

- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `tb20e_controller` (`JointTrajectoryController`)

接続確認には次のコマンドが使えます。

```bash
ros2 topic hz /current_boom_angle
ros2 topic echo /joint_states
ros2 control list_controllers
```

## FollowJointTrajectory の例

次の例は、5 秒後の目標を `[swing, boom, arm, bucket]` の順で radian 指定します。
必ず Unity の実角度が受信できていることを確認してから送信してください。

```bash
ros2 action send_goal \
  /tb20e_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [swing_joint, boom_joint, arm_joint, bucket_joint], points: [{positions: [0.0, -0.35, 1.40, 0.52], velocities: [0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 5}}]}}"
```

関節制限は URDF に次のように定義しています。

- `swing_joint`: continuous
- `boom_joint`: -83 ～ 48 degree
- `arm_joint`: 32 ～ 155 degree
- `bucket_joint`: -31 ～ 159 degree

全関節の速度上限は100 degree/sです。

## USBゲームパッドでの操作

Linuxでゲームパッドが `/dev/input/js0` として認識され、Unityから4軸の角度feedbackが
届いている状態で、軌道制御用launchの代わりに次を起動します。

```bash
ros2 launch tb20e_control tb20e_gamepad.launch.py
```

既定の割り当てはSDL GameControllerの標準axis配列に合わせています。

| 入力 | 操作軸 | Joy axis | 出力範囲 |
|---|---|---:|---:|
| 左スティック 左右 | swing | 0 | -100 ～ 100 |
| 左スティック 上下（反転） | arm | 1 | -100 ～ 100 |
| 右スティック 左右 | bucket | 2 | -100 ～ 100 |
| 右スティック 上下 | boom | 3 | -100 ～ 100 |

スティック中央には既定で `0.10` のデッドゾーンを設けています。Joy入力が0.25秒以上
途絶えた場合やaxis数が不足する場合は、4軸すべてを自動的にゼロへ戻します。

ゲームパッドによってaxis番号や向きが異なる場合は、まず入力を確認します。

```bash
ros2 topic echo /joy
```

起動引数でaxis番号、方向、最大操作率を変更できます。scaleの符号で方向を指定します。
実機に合わせてarmの既定値だけ `-100.0`、その他は `100.0` です。次の例はarmを
既定と逆向きに戻し、boomを反転します。

```bash
ros2 launch tb20e_control tb20e_gamepad.launch.py \
  swing_axis:=0 arm_axis:=1 bucket_axis:=2 boom_axis:=3 \
  swing_scale:=100.0 arm_scale:=100.0 \
  bucket_scale:=100.0 boom_scale:=-100.0
```

特定ボタンを押している間だけ操作を許可するには、Joy messageのbutton番号を指定します。
既定値 `-1` ではボタン操作を要求しません。

```bash
ros2 launch tb20e_control tb20e_gamepad.launch.py deadman_button:=4
```

`tb20e_control.launch.py` と `tb20e_gamepad.launch.py` は同時に起動しないでください。
どちらも同じ4本のeffort command interfaceを使用します。

## Topic・timeout・符号・出力制限の変更

すべての通信設定は
`tb20e_control/urdf/tb20e.urdf.xacro` の xacro 引数から
`ros2_control` hardware parameter へ渡されます。launch 引数でも上書きできます。

```bash
ros2 launch tb20e_control tb20e_control.launch.py \
  state_timeout_sec:=0.20 \
  initial_feedback_wait_sec:=3.0 \
  feedback_limit_tolerance_deg:=1.0 \
  max_feedback_velocity_deg_s:=120.0 \
  boom_state_topic:=/sim/tb20e/current_boom_angle \
  boom_command_topic:=/sim/tb20e/manipulated_boom_lever \
  boom_lever_sign:=-1.0 \
  boom_lever_min:=-60.0 \
  boom_lever_max:=60.0
```

軸名を先頭に付けた以下の hardware parameter が各軸にあります。

- `<axis>_state_topic`
- `<axis>_command_topic`
- `<axis>_lever_sign`
- `<axis>_lever_min`
- `<axis>_lever_max`

全軸共通の以下の parameter も設定できます。

- `state_timeout_sec`: 運転中のfeedback timeout。既定値 `0.10` 秒
- `initial_feedback_wait_sec`: activate時に4軸のfeedbackを待つ時間。既定値 `2.0` 秒
- `feedback_limit_tolerance_deg`: 仕様角度範囲に対する受信許容差。既定値 `2.0` degree
- `max_feedback_velocity_deg_s`: 角度飛びとみなす速度の閾値。既定値 `180.0` degree/s

出力制限は必ず `min <= 0 <= max` かつ `min < max` にしてください。
符号は `-1.0` または `1.0` を指定してください。

## Safety fault からの復旧

fault時は4軸のレバー出力がゼロへラッチされます。Unityから4軸すべての正常なfeedbackが
再び届いていることを確認し、controllerとhardwareを順に再activateしてください。

```bash
ros2 control set_controller_state tb20e_controller inactive
ros2 control set_hardware_component_state Tb20eLeverSystem inactive
ros2 control set_hardware_component_state Tb20eLeverSystem active
ros2 control set_controller_state tb20e_controller active
```

その後、古い軌道を再開させず、新しい `FollowJointTrajectory` goalを送信してください。
状態遷移が失敗する場合はlaunch一式を停止・再起動し、同じく新しいgoalを送ってください。

`joint_trajectory_controller` は範囲外の目標角を受理する場合があります。送信側でも
上記の関節範囲を検査してください。hardware側では端点の外向き指令を抑止しますが、
範囲外goal自体を有効なgoalへ補正はしません。角度範囲は
`tb20e_control/urdf/tb20e.urdf.xacro` の `*_position_*_deg` propertyを変更すると、
URDFのjoint limitとhardwareのfeedback判定へ同時に反映されます。

## PID ゲイン調整

初期ゲインは
`tb20e_control/config/tb20e_controllers.yaml` の `gains` にあります。
シミュレータ用の出発点であり、機体特性に合わせた同定済みの値ではありません。

1. 最初は `i: 0.0` のまま、小さい `p` から開始する。
2. 定常偏差を減らせる範囲まで `p` を上げる。
3. 振動を抑えるために少量の `d` を加える。
4. 必要な場合だけ小さい `i` を追加し、積分飽和を確認する。

hardware interface 側でも最終的にレバー範囲へ clamp しますが、clamp が常時働くほど
大きなゲインは追従性と安定性を悪化させます。現在角度は200 Hzで受信し、20 Hzの
制御周期ごとの最新位置差から速度を推定します。PIDとレバー配信も20 Hzです。
角度ノイズが多い場合は、特に `d` を小さくしてください。スイングだけは
`angle_wraparound: true` を維持してください。

## Unity と併用する際の注意

- Unity は 4 本すべての現在角度 topic を degree で publish してください。
- Unity は 4 本のレバー topic を subscribe し、値を `-100.0` ～ `100.0` として扱います。
- 同じ topic へ別のレバー指令ノードを同時に publish しないでください。
- ROS time を別途供給する構成でなければ `use_sim_time:=false` のまま使用してください。
- ROS TCP Endpoint や DDS の接続先、`ROS_DOMAIN_ID`、Firewall を先に確認してください。

## Safety

このパッケージの既定ゲインと簡易 URDF は Unity シミュレーション専用です。
実機へそのまま接続しないでください。実機適用には、独立した非常停止、通信 watchdog、
油圧ロック、速度・圧力・作業範囲制限、起動時の中立確認、機体固有の符号確認が必要です。
timeout や software clamp は、それらの安全機構の代わりにはなりません。

## License

Apache License 2.0。詳細は [LICENSE](LICENSE) を参照してください。
