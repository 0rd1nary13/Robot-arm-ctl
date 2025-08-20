## Robot-arm-ctl

基于 Lebai 六轴机械臂、外接相机与 Dynamixel 伺服的控制与手眼标定示例。项目包含：

- 视觉检测并自动对准目标的示例（圆形按钮检测与位姿估计）
- 机械臂遥操作（读取 Dynamixel 角度映射到机械臂关节）
- 相机内参标定、手眼标定（相机-末端外参）
- 简易 HTTP 服务用于视频预览与一键采集

### 目录结构

- `teleop.py`: 使用 Dynamixel 读取角度，实时驱动 Lebai 机械臂（关节空间）。
- `camera_test.py`: 视觉检测圆形按钮，估算深度与目标位姿，驱动机械臂靠近，并提供一次性 JPEG 输出的 HTTP 接口（端口 2000）。
- `calibration/`
  - `take_photo.py`: 开启采集服务，GET `/` 预览，POST `/` 采集当前帧到 `handeye_images/`，并记录当前机械臂 `actual_tcp_pose`。
  - `post.py`: 命令行工具，按回车发送 POST `/` 触发采集。
  - `calibrate.py`: 用棋盘格图片计算相机内参，输出 `calibration_data.npz`。
  - `calibrate_handeye.py`: 用 `arm_data.npy` 与采集到的图片进行手眼标定，输出 `handeye_result.npz`。

### 依赖与环境

- Python 3.10+
- 依赖包：OpenCV、NumPy、Requests、Dynamixel SDK、Lebai SDK

安装示例：

```bash
python -m venv .venv
source .venv/bin/activate
pip install opencv-python numpy requests dynamixel-sdk
# Lebai SDK 请根据厂商文档安装（示例代码中使用 import lebai_sdk）
```

硬件与连接：

- Lebai 机械臂控制器（示例 IP `192.168.10.200`）
- 夹爪/外设控制服务（示例 IP `192.168.10.201`，部分示例通过端口 5180 发送简单指令）
- 工业/USB 相机（示例设备 `/dev/video0`，自动回退到 `/dev/video1`）
- Dynamixel 控制器（串口示例：`/dev/cu.usbmodem1101`）

请根据实际环境修改代码中的以下参数：

- `LEBAI_IP`（机械臂 IP）
- `DEVICENAME`（Dynamixel 串口）
- 相机设备节点（`/dev/video0` 或 `/dev/video1`）
- `REAL_BUTTON_DIAMETER`（目标实物直径，单位 mm）
- `ROBOT_IP` 与端口（采集服务所在主机与端口）

### 快速开始

#### A. 采集与标定（推荐先完成）

1) 启动采集服务（预览与拍照保存）：

```bash
python calibration/take_photo.py
```

- 服务启动后：
  - 浏览器访问 `http://<主机>:2001/` 可获取 MJPEG 预览（GET `/`）。
  - 发送 POST `/` 可立即采集并保存图片到 `handeye_images/`，并记录对应时刻的 `actual_tcp_pose` 到内存。

2) 另开一个终端，运行触发器工具，多次按回车采集不同位姿：

```bash
python calibration/post.py
```

3) 采集完成后，在采集服务终端按 `Ctrl+C` 退出，程序会将所有位姿保存为 `arm_data.npy`。

4) 计算相机内参（基于采集到的棋盘格图像，图片默认在 `handeye_images/`）：

```bash
python calibration/calibrate.py
```

生成 `calibration_data.npz`（包含 `camera_matrix` 与 `dist_coeffs`）。

5) 计算手眼标定（相机相对末端位姿）：

```bash
python calibration/calibrate_handeye.py
```

生成 `handeye_result.npz`（包含相机-夹爪外参）。

> 注：当前 `camera_test.py` 中已硬编码一组内参与畸变系数示例，你可将其替换为自己的标定结果以提升精度。

#### B. 遥操作（Dynamixel → 机械臂）

确保 `DEVICENAME` 串口正确连接后：

```bash
python teleop.py
```

脚本会周期性读取 6 个 Dynamixel 关节角度，并调用 `lebai.towardj(...)` 将其映射到机械臂关节角，实时控制机械臂。

#### C. 视觉抓取测试（圆形按钮示例）

1) 如使用外部夹爪服务，请先确保对应服务已在 `192.168.10.201:5180` 可达（`camera_test.py` 会发送简单初始化指令）。

2) 运行视觉检测与执行：

```bash
python camera_test.py
```

3) 浏览器访问 `http://<主机>:2000/`：

- 服务以 `multipart/x-mixed-replace` 响应返回一帧带有圆检测覆盖的 JPEG；
- 若检测到圆（默认半径范围 25–40 像素），通过已知实物直径与相机内参估算深度，回推相机坐标下 3D 点，再结合当前 TCP 位姿构造目标位姿并执行 `movej`；
- 动作完成后机械臂复位至安全位姿。

### 安全注意事项

- 确保机械臂周围无障碍物与人员，设置合适的速度/加速度/限位与急停；
- 标定参数与 `REAL_BUTTON_DIAMETER` 必须符合实际，否则会产生较大位置误差；
- 首次联调建议降低速度，在空中完成路径验证后再靠近目标。

### 常见问题排查

- 无法打开相机：检查 `/dev/videoX` 是否存在、权限是否足够；
- 无法连接机械臂：确认 `LEBAI_IP` 与网络连通；
- 串口权限问题：在 macOS/Linux 下检查串口名与权限（必要时为设备授予访问权限）；
- 棋盘角点未检测到：确保棋盘尺寸 `chessboard_size` 与实际一致、图像曝光充足；
- 视觉检测失败导致深度为 0：圆检测参数需根据相机分辨率与目标尺寸微调（`minRadius/maxRadius/param1/param2`）。

### 许可证

未附带许可证。如需开源发布，请自行添加。


