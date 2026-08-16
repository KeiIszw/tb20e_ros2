# tb20e_bringup

実機の `/current_*_angle` feedbackを用いて `/manipulated_*_lever` を生成しながら、
同じゲームパッドまたはHTTP指令をUnityの `/tb20e/<axis>/cmd` へ位置目標として送る
bringupパッケージです。

実機出力は既定で無効です。実機の非常停止、deadman、符号、関節範囲を確認してから
`real_output_enabled:=true` を指定してください。

## topic構成

| 用途 | topic | 型・単位 |
|---|---|---|
| 実機feedback | `/current_<axis>_angle` | `std_msgs/msg/Float64`, degree |
| 実機レバー指令 | `/manipulated_<axis>_lever` | `std_msgs/msg/Float64`, -100～100 |
| Unity feedback | `/sim/tb20e/current_<axis>_angle` | `std_msgs/msg/Float64`, degree |
| Unity位置指令 | `/tb20e/<axis>/cmd` | `std_msgs/msg/Float64`, rad |

`<axis>`は`swing`, `boom`, `arm`, `bucket`です。ゲームパッド入力ではUnityの
feedbackを起点にレバー量を位置変化へ積分し、HTTP入力では受理された実機用trajectoryと
同一の目標角度をUnityへ送ります。ゲームパッドとHTTPは同時起動せず、`input_source`で
どちらか一方を選択します。

Unity向けtopicはnode namespaceに依存しない絶対名
`/tb20e/{swing,boom,arm,bucket}/cmd`として生成します。Unityと実機でswingの正方向が
逆であるため、Unityのswing位置指令とfeedbackだけを既定で符号反転します。必要な場合は
`swing_unity_position_sign`（既定`-1.0`）で変更できます。

## ビルド

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select tb20e_control scratch_hci_bridge tb20e_bringup
source install/setup.bash
```

## 起動

```bash
ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=gamepad real_output_enabled:=false

ros2 launch tb20e_bringup tb20e_real_unity.launch.py \
  input_source:=http real_output_enabled:=true
```

最初は`real_output_enabled:=false`でtopicと方向を確認し、実機へ出力するときだけ
`true`へ変更してください。`false`でも実機feedbackは制御ノードの起動と比較表示に必要です。
ゲームパッドモードではUnity feedbackが`sim_feedback_timeout_sec`（既定0.25秒）を超えると、
Unity位置指令の積分とpublishを停止します。各軸の最大変化速度は
`<axis>_unity_speed_deg_s`（既定50 degree/s）で調整できます。

Unity側の4軸feedbackは `/sim/tb20e/current_<axis>_angle`、degreeを想定しています。
Unity側のトピックおよびコンポーネント設定はこのパッケージでは変更しません。
