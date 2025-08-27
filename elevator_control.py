import cv2
import lebai_sdk
import numpy as np
import json
import os
import time
import socket
from datetime import datetime

lebai_sdk.init()
LEBAI_IP = "192.168.10.200"

class ElevatorController:
    def __init__(self):
        self.lebai = None
        self.button_positions = {}
        self.floor_names = []
        self.data_dir = "elevator_data"
        self.current_floor = None
        
        # 运动参数配置 - 根据乐白SDK文档优化
        self.motion_config = {
            'approach': {
                'acc': 0.2,    # 接近阶段加速度 (m/s²) - 降低加速度
                'vel': 0.05,   # 接近阶段速度 (m/s) - 降低速度
                'timeout': 30   # 运动超时时间 (秒)
            },
            'press': {
                'acc': 0.1,    # 按压阶段加速度 (m/s²) - 最保守
                'vel': 0.02,   # 按压阶段速度 (m/s) - 最保守
                'timeout': 20   # 运动超时时间 (秒)
            },
            'retreat': {
                'acc': 0.2,    # 后退阶段加速度 (m/s²) - 降低加速度
                'vel': 0.05,   # 后退阶段速度 (m/s) - 降低速度
                'timeout': 30   # 运动超时时间 (秒)
            },
            'safe': {
                'acc': 0.3,    # 安全移动加速度 (m/s²) - 降低加速度
                'vel': 0.08,   # 安全移动速度 (m/s) - 降低速度
                'timeout': 45   # 运动超时时间 (秒)
            }
        }

    def connect_robot(self):
        """连接机器人"""
        try:
            print("连接机器人...")
            self.lebai = lebai_sdk.connect(LEBAI_IP, False)
            print("✅ 机器人连接成功")
            
            # 启动系统后关闭夹爪
            try:
                print("启动机器人系统...")
                self.lebai.start_sys()
                time.sleep(2)
                # 设置夹爪开合度为100%（完全打开），力度为0（最小力度）
                self.lebai.set_claw(0, 0)
                time.sleep(1)  # 等待夹爪动作完成
            except Exception as e:
                print(f"⚠️ 夹爪控制失败: {e}，继续执行...")
            
            return True
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False

    def debug_position(self, target_pose, description="目标位置"):
        """调试当前位置与目标位置的对比"""
        try:
            current_pose = self.lebai.get_kin_data()["actual_tcp_pose"]
            
            print(f"\n🔍 {description}对比:")
            print(f"目标位置: x={target_pose['x']:.4f}, y={target_pose['y']:.4f}, z={target_pose['z']:.4f}")
            print(f"当前位置: x={current_pose['x']:.4f}, y={current_pose['y']:.4f}, z={current_pose['z']:.4f}")
            
            # 计算位置误差
            pos_error = np.sqrt(
                (current_pose['x'] - target_pose['x'])**2 +
                (current_pose['y'] - target_pose['y'])**2 +
                (current_pose['z'] - target_pose['z'])**2
            )
            
            print(f"位置误差: {pos_error:.4f}m")
            
            # 计算角度误差（如果有角度数据）
            if 'rx' in target_pose and 'rx' in current_pose:
                rx_error = abs(current_pose['rx'] - target_pose['rx'])
                ry_error = abs(current_pose['ry'] - target_pose['ry'])
                rz_error = abs(current_pose['rz'] - target_pose['rz'])
                print(f"角度误差: rx={rx_error:.4f}, ry={ry_error:.4f}, rz={rz_error:.4f}")
            
            return pos_error < 0.01  # 返回是否在误差范围内
            
        except Exception as e:
            print(f"❌ 位置调试失败: {e}")
            return False

    def safe_move(self, target_pose, motion_type='safe', description="移动"):
        """
        安全移动机器人到目标位置
        motion_type: 运动类型 ('approach', 'press', 'retreat', 'safe')
        """
        try:
            config = self.motion_config[motion_type]
            
            print(f"{description}... (速度: {config['vel']:.2f} m/s, 加速度: {config['acc']:.2f} m/s²)")
            print(f"目标位置: x={target_pose['x']:.4f}, y={target_pose['y']:.4f}, z={target_pose['z']:.4f}")
            print(f"目标角度: rx={target_pose.get('rx', 0):.4f}, ry={target_pose.get('ry', 0):.4f}, rz={target_pose.get('rz', 0):.4f}")
            
            # 显示移动前的位置对比
            print("移动前位置对比:")
            self.debug_position(target_pose, "移动前")
            
            # 使用笛卡尔坐标直线运动
            self.lebai.movel([
                target_pose['x'], target_pose['y'], target_pose['z'],
                target_pose['rx'], target_pose['ry'], target_pose['rz']
            ], config['acc'], config['vel'])
            
            # 等待运动完成
            print(f"等待运动完成...")
            self.lebai.wait_move()
            
            # 显示移动后的位置对比
            print("移动后位置对比:")
            in_range = self.debug_position(target_pose, "移动后")
            
            if in_range:
                print(f"✅ {description}完成")
                return True
            else:
                print(f"⚠️ {description}完成，但位置有偏差")
                return True
                
        except Exception as e:
            print(f"❌ {description}失败: {e}")
            return False

    def load_button_mapping(self, filename=None):
        """加载按钮映射数据"""
        try:
            if filename is None:
                # 查找最新的映射文件
                files = [f for f in os.listdir(self.data_dir) if f.startswith('elevator_buttons_') and f.endswith('.json')]
                if not files:
                    print("❌ 未找到按钮映射文件")
                    return False
                filename = max(files, key=lambda x: os.path.getctime(os.path.join(self.data_dir, x)))

            filepath = os.path.join(self.data_dir, filename)
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)

            self.floor_names = data.get('floors', [])
            self.button_positions = data.get('button_positions', {})

            print(f"✅ 已加载按钮映射: {filepath}")
            print(f"可用楼层: {self.floor_names}")
            return True

        except Exception as e:
            print(f"❌ 加载数据失败: {e}")
            return False

    def go_to_floor(self, floor_name):
        """移动到指定楼层按钮并按下"""
        try:
            # 尝试多种可能的按钮键格式
            possible_keys = [
                f"{floor_name}_floor",
                f"<{floor_name}>_floor",
                f"<{floor_name.lower()}>_floor",
                f"<{floor_name.upper()}>_floor"
            ]
            
            button_key = None
            for key in possible_keys:
                if key in self.button_positions:
                    button_key = key
                    break
            
            if button_key is None:
                print(f"❌ 未找到楼层 {floor_name} 的按钮位置")
                print(f"💡 尝试过的键: {possible_keys}")
                print(f"💡 可用的按钮键: {list(self.button_positions.keys())}")
                return False

            print(f"前往楼层 {floor_name}...")

            # 获取按钮位置
            pose = self.button_positions[button_key]['pose']
            print(f"目标位置: x={pose['x']:.3f}, y={pose['y']:.3f}, z={pose['z']:.3f}")

            # 1. 移动到安全位置（按钮前方8cm，稍微抬高）
            safe_pose = {
                'x': pose['x'] - 0.08,
                'y': pose['y'],
                'z': pose['z'] + 0.02,
                'rx': pose['rx'],
                'ry': pose['ry'],
                'rz': pose['rz']
            }
            
            if not self.safe_move(safe_pose, 'safe', "移动到安全位置"):
                return False

            # 2. 缓慢接近按钮（按钮前方3cm）
            approach_pose = {
                'x': pose['x'] - 0.03,
                'y': pose['y'],
                'z': pose['z'],
                'rx': pose['rx'],
                'ry': pose['ry'],
                'rz': pose['rz']
            }
            
            if not self.safe_move(approach_pose, 'approach', "接近按钮"):
                return False

            # 3. 按按钮（向前2cm）
            press_pose = {
                'x': pose['x'] + 0.02,
                'y': pose['y'],
                'z': pose['z'],
                'rx': pose['rx'],
                'ry': pose['ry'],
                'rz': pose['rz']
            }
            
            if not self.safe_move(press_pose, 'press', f"按下楼层 {floor_name} 按钮"):
                return False

            # 4. 按住按钮1秒
            print("按住按钮...")
            time.sleep(1.0)

            # 5. 后退到接近位置
            if not self.safe_move(approach_pose, 'retreat', "后退"):
                return False

            # 6. 返回安全位置
            if not self.safe_move(safe_pose, 'safe', "返回安全位置"):
                return False

            self.current_floor = floor_name
            print(f"✅ 已成功按下楼层 {floor_name} 按钮")
            return True

        except Exception as e:
            print(f"❌ 按按钮失败: {e}")
            return False

    def press_elevator_button(self, button_type):
        """按下电梯控制按钮（开门、关门、报警等）"""
        try:
            button_key = f"elevator_{button_type}"
            if button_key not in self.button_positions:
                print(f"❌ 未找到电梯{button_type}按钮位置")
                return False

            print(f"按下电梯{button_type}按钮...")

            pose = self.button_positions[button_key]['pose']
            print(f"目标位置: x={pose['x']:.3f}, y={pose['y']:.3f}, z={pose['z']:.3f}")

            # 1. 移动到按钮前方3cm
            approach_pose = {
                'x': pose['x'] - 0.03,
                'y': pose['y'],
                'z': pose['z'],
                'rx': pose['rx'],
                'ry': pose['ry'],
                'rz': pose['rz']
            }
            
            if not self.safe_move(approach_pose, 'approach', "接近按钮"):
                return False

            # 2. 按按钮（向前2cm）
            press_pose = {
                'x': pose['x'] + 0.02,
                'y': pose['y'],
                'z': pose['z'],
                'rx': pose['rx'],
                'ry': pose['ry'],
                'rz': pose['rz']
            }
            
            if not self.safe_move(press_pose, 'press', f"按下{button_type}按钮"):
                return False

            # 3. 按住0.5秒
            time.sleep(0.5)

            # 4. 后退
            if not self.safe_move(approach_pose, 'retreat', "后退"):
                return False

            print(f"✅ 已按下电梯{button_type}按钮")
            return True

        except Exception as e:
            print(f"❌ 按按钮失败: {e}")
            return False

    def list_available_floors(self):
        """列出可用的楼层"""
        print("可用楼层:")
        for key in self.button_positions.keys():
            if key.endswith('_floor'):
                # 提取楼层名称，去除 "_floor" 后缀
                floor_name = key.replace('_floor', '')
                print(f"  - {floor_name}")
        
        # 显示原始按钮键用于调试
        print("\n调试信息 - 原始按钮键:")
        for key in self.button_positions.keys():
            if key.endswith('_floor'):
                print(f"  {key}")
        
        return [key.replace('_floor', '') for key in self.button_positions.keys() if key.endswith('_floor')]

    def go_home_position(self):
        """回到安全位置"""
        try:
            print("回到安全位置...")
            home_pose = {
                'x': 0.3, 'y': 0.0, 'z': 0.2,
                'rx': 0.0, 'ry': 0.0, 'rz': 0.0
            }

            if not self.safe_move(home_pose, 'safe', "回到安全位置"):
                return False
                
            print("✅ 已回到安全位置")
            return True

        except Exception as e:
            print(f"❌ 返回安全位置失败: {e}")
            return False

    def show_motion_config(self):
        """显示当前运动配置"""
        print("\n=== 当前运动配置 ===")
        for motion_type, config in self.motion_config.items():
            print(f"{motion_type.upper()}:")
            print(f"  加速度: {config['acc']:.2f} m/s²")
            print(f"  速度: {config['vel']:.2f} m/s")
            print(f"  超时: {config['timeout']} 秒")

    def update_motion_config(self, motion_type, acc=None, vel=None, timeout=None):
        """更新运动配置"""
        if motion_type not in self.motion_config:
            print(f"❌ 未知的运动类型: {motion_type}")
            return False
            
        config = self.motion_config[motion_type]
        if acc is not None:
            config['acc'] = float(acc)
        if vel is not None:
            config['vel'] = float(vel)
        if timeout is not None:
            config['timeout'] = int(timeout)
            
        print(f"✅ 已更新 {motion_type} 运动配置")
        return True

    def interactive_control(self):
        """交互式电梯控制"""
        print("\n=== 电梯控制系统 ===")
        print("指令:")
        print("  <楼层名> - 前往指定楼层")
        print("  open - 按开门按钮")
        print("  close - 按关门按钮")
        print("  alarm - 按报警按钮")
        print("  list - 列出可用楼层")
        print("  home - 回到安全位置")
        print("  config - 显示运动配置")
        print("  setconfig <类型> <加速度> <速度> <超时> - 设置运动参数")
        print("  debug - 显示当前位置信息")
        print("  status - 检查机器人状态")
        print("  test - 测试移动功能")
        print("  simple - 简单关节移动测试")
        print("  q - 退出")
        print("  h - 显示帮助")

        while True:
            try:
                cmd = input("\n请输入指令: ").strip().lower()

                if cmd == 'q':
                    break
                elif cmd == 'h':
                    print("指令:")
                    print("  <楼层名> - 前往指定楼层")
                    print("  open - 按开门按钮")
                    print("  close - 按关门按钮")
                    print("  alarm - 按报警按钮")
                    print("  list - 列出可用楼层")
                    print("  home - 回到安全位置")
                    print("  config - 显示运动配置")
                    print("  setconfig <类型> <加速度> <速度> <超时> - 设置运动参数")
                    print("  debug - 显示当前位置信息")
                    print("  status - 检查机器人状态")
                    print("  test - 测试移动功能")
                    print("  simple - 简单关节移动测试")
                    print("  q - 退出")
                elif cmd == 'list':
                    self.list_available_floors()
                elif cmd == 'open':
                    self.press_elevator_button('open')
                elif cmd == 'close':
                    self.press_elevator_button('close')
                elif cmd == 'alarm':
                    self.press_elevator_button('alarm')
                elif cmd == 'home':
                    self.go_home_position()
                elif cmd == 'config':
                    self.show_motion_config()
                elif cmd == 'debug':
                    try:
                        current_pose = self.lebai.get_kin_data()["actual_tcp_pose"]
                        print(f"\n📍 当前位置信息:")
                        print(f"  X: {current_pose['x']:.4f} m")
                        print(f"  Y: {current_pose['y']:.4f} m")
                        print(f"  Z: {current_pose['z']:.4f} m")
                        print(f"  Rx: {current_pose.get('rx', 0):.4f}")
                        print(f"  Ry: {current_pose.get('ry', 0):.4f}")
                        print(f"  Rz: {current_pose.get('rz', 0):.4f}")
                    except Exception as e:
                        print(f"❌ 获取位置信息失败: {e}")
                elif cmd == 'status':
                    self.check_robot_status()
                elif cmd == 'test':
                    self.test_movement()
                elif cmd == 'simple':
                    self.test_simple_movement()
                elif cmd.startswith('setconfig '):
                    parts = cmd[10:].strip().split()
                    if len(parts) == 4:
                        try:
                            motion_type = parts[0]
                            acc = float(parts[1])
                            vel = float(parts[2])
                            timeout = int(parts[3])
                            self.update_motion_config(motion_type, acc, vel, timeout)
                        except ValueError:
                            print("❌ 参数格式错误，正确格式: setconfig <类型> <加速度> <速度> <超时>")
                            print("   示例: setconfig approach 0.2 0.05 25")
                    else:
                        print("❌ 参数数量错误，正确格式: setconfig <类型> <加速度> <速度> <超时>")
                elif cmd:
                    # 尝试多种方式匹配楼层
                    target_floor = None
                    
                    # 1. 直接匹配
                    if self.go_to_floor(cmd):
                        continue
                    
                    # 2. 尝试添加尖括号
                    if not cmd.startswith('<') and not cmd.endswith('>'):
                        if self.go_to_floor(f"<{cmd}>"):
                            continue
                    
                    # 3. 尝试去除尖括号
                    if cmd.startswith('<') and cmd.endswith('>'):
                        clean_cmd = cmd[1:-1]
                        if self.go_to_floor(clean_cmd):
                            continue
                    
                    # 4. 如果都失败了，显示帮助信息
                    print(f"❌ 未找到楼层: {cmd}")
                    print("💡 请使用 'list' 命令查看可用楼层")
                    print("💡 或者检查楼层名称是否正确")
                else:
                    print("❌ 请输入有效指令")

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ 指令执行失败: {e}")

    def start_elevator_service(self, port=5000):
        """启动电梯控制网络服务"""
        try:
            print(f"启动电梯控制服务 (端口: {port})...")

            def handle_client(client_socket):
                try:
                    data = client_socket.recv(1024).decode('utf-8').strip()
                    print(f"收到请求: {data}")

                    response = "ERROR"

                    if data.startswith("FLOOR_"):
                        floor = data[6:]
                        if self.go_to_floor(floor):
                            response = "OK"
                    elif data == "OPEN":
                        if self.press_elevator_button('open'):
                            response = "OK"
                    elif data == "CLOSE":
                        if self.press_elevator_button('close'):
                            response = "OK"
                    elif data == "ALARM":
                        if self.press_elevator_button('alarm'):
                            response = "OK"
                    elif data == "HOME":
                        if self.go_home_position():
                            response = "OK"
                    elif data == "LIST":
                        floors = self.list_available_floors()
                        response = ",".join(floors)
                    elif data == "STATUS":
                        response = f"CURRENT_FLOOR:{self.current_floor}"

                    client_socket.send(response.encode('utf-8'))

                except Exception as e:
                    print(f"处理客户端请求失败: {e}")
                    client_socket.send(b"ERROR")
                finally:
                    client_socket.close()

            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.bind(('0.0.0.0', port))
            server_socket.listen(5)

            print(f"✅ 电梯控制服务已启动，监听端口 {port}")
            print("可用命令:")
            print("  FLOOR_<楼层名> - 前往指定楼层")
            print("  OPEN - 开门")
            print("  CLOSE - 关门")
            print("  ALARM - 报警")
            print("  HOME - 回到安全位置")
            print("  LIST - 列出楼层")
            print("  STATUS - 获取当前状态")

            while True:
                client_socket, address = server_socket.accept()
                print(f"接受连接来自: {address}")
                handle_client(client_socket)

        except Exception as e:
            print(f"❌ 启动服务失败: {e}")

    def check_robot_status(self):
        """检查机器人状态"""
        try:
            status = self.lebai.get_robot_state()
            print(f"🤖 机器人状态: {status}")
            
            if status == "IDLE":
                print("✅ 机器人空闲，可以执行指令")
                return True
            elif status == "MOVING":
                print("⚠️ 机器人正在运动中")
                return False
            elif status == "ESTOP":
                print("🚨 机器人紧急停止状态")
                return False
            else:
                print(f"❓ 未知状态: {status}")
                return False
                
        except Exception as e:
            print(f"❌ 获取机器人状态失败: {e}")
            return False

    def test_simple_movement(self):
        """测试简单移动功能（使用关节运动避免笛卡尔运动错误）"""
        try:
            print("🧪 测试简单移动功能...")
            
            # 获取当前关节位置
            kin_data = self.lebai.get_kin_data()
            current_joints = kin_data['actual_joint_pose']
            print(f"当前关节位置: {[f'{j:.3f}' for j in current_joints]}")
            
            # 测试小幅度关节移动（第一个关节移动1度）
            test_joints = current_joints.copy()
            test_joints[0] += 0.017  # 1度 ≈ 0.017弧度
            
            print(f"测试移动到关节位置: {[f'{j:.3f}' for j in test_joints]}")
            
            # 使用非常保守的参数进行关节运动
            print("使用保守参数进行关节运动...")
            self.lebai.movej(test_joints, 0.1, 0.05)  # 低加速度，低速度
            
            print("等待关节运动完成...")
            self.lebai.wait_move()
            
            # 检查新关节位置
            new_kin_data = self.lebai.get_kin_data()
            new_joints = new_kin_data['actual_joint_pose']
            print(f"移动后关节位置: {[f'{j:.3f}' for j in new_joints]}")
            
            # 计算实际移动距离
            actual_move = new_joints[0] - current_joints[0]
            print(f"第一个关节实际移动: {actual_move:.4f} rad")
            
            if abs(actual_move - 0.017) < 0.01:  # 允许较大误差
                print("✅ 关节移动测试成功！")
                return True
            else:
                print(f"⚠️ 关节移动测试完成，但精度不够")
                return True
                
        except Exception as e:
            print(f"❌ 关节移动测试失败: {e}")
            return False

    def test_movement(self):
        """测试运动功能"""
        print("\n=== 运动测试 ===")
        print("请输入目标位置 (x, y, z, rx, ry, rz) 或 'home' 回到安全位置:")
        while True:
            try:
                input_str = input("请输入 (例如: 0.1,0.2,0.3,0.1,0.2,0.3 或 home): ").strip()
                
                if input_str.lower() == 'home':
                    self.go_home_position()
                else:
                    pose_str = input_str.replace(' ', '').split(',')
                    if len(pose_str) == 6:
                        try:
                            pose = {
                                'x': float(pose_str[0]),
                                'y': float(pose_str[1]),
                                'z': float(pose_str[2]),
                                'rx': float(pose_str[3]),
                                'ry': float(pose_str[4]),
                                'rz': float(pose_str[5])
                            }
                            print(f"测试移动到: x={pose['x']:.3f}, y={pose['y']:.3f}, z={pose['z']:.3f}")
                            print(f"目标角度: rx={pose['rx']:.3f}, ry={pose['ry']:.3f}, rz={pose['rz']:.3f}")
                            
                            # 选择运动类型
                            motion_type = input("请选择运动类型 (approach, press, retreat, safe, 默认safe): ").strip().lower()
                            if motion_type == '':
                                motion_type = 'safe'
                                
                            if not self.safe_move(pose, motion_type, "测试移动"):
                                print("测试移动失败，请检查机器人状态或运动参数。")
                            else:
                                print("测试移动成功！")
                        except ValueError:
                            print("❌ 输入格式错误，请输入 x,y,z,rx,ry,rz 或 home")
                    else:
                        print("❌ 输入格式错误，请输入 x,y,z,rx,ry,rz 或 home")
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ 运动测试失败: {e}")

def main():
    controller = ElevatorController()

    print("=== 电梯控制系统 ===")

    # 连接机器人（包括启动系统和关闭夹爪）
    if not controller.connect_robot():
        return

    try:
        # 系统已在connect_robot中启动，夹爪已关闭
        controller.lebai.disable_joint_limits()

        # 加载按钮映射
        if not controller.load_button_mapping():
            print("请先运行 elevator_teach.py 进行施教")
            return

        # 选择控制模式
        print("\n选择控制模式:")
        print("1. 交互式控制")
        print("2. 网络服务模式")
        print("3. 运动测试")

        choice = input("请选择 (1, 2 或 3): ").strip()

        if choice == "2":
            port = input("请输入服务端口 (默认5000): ").strip()
            port = int(port) if port.isdigit() else 5000
            controller.start_elevator_service(port)
        elif choice == "3":
            controller.test_movement()
        else:
            controller.interactive_control()

    finally:
        try:
            controller.lebai.stop_sys()
        except:
            pass

if __name__ == "__main__":
    main()
