import cv2
import lebai_sdk
import numpy as np
import json
import os
import time
import requests
from datetime import datetime

lebai_sdk.init()
LEBAI_IP = "192.168.10.200"

class ElevatorButtonTeacher:
    def __init__(self):
        self.lebai = None
        self.button_positions = {}  # 楼层 -> 按钮位置映射
        self.floor_names = []  # 楼层名称列表
        self.data_dir = "elevator_data"
        
        # 远程拍照配置
        self.remote_camera_ip = '192.168.10.201'
        self.remote_camera_port = 2001

        # 创建数据存储目录
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

    def connect_robot(self):
        """连接机器人"""
        try:
            print("连接机器人...")
            self.lebai = lebai_sdk.connect(LEBAI_IP, False)
            print("✅ 机器人连接成功")
            return True
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False

    def start_teach_mode(self):
        """进入施教模式"""
        try:
            print("启动机器人系统...")
            self.lebai.start_sys()
            time.sleep(2)

            print("关闭夹爪...")
            try:
                # 设置夹爪开合度为100%（完全打开），力度为0（最小力度）
                self.lebai.set_claw(0, 0)
                time.sleep(1)  # 等待夹爪动作完成
                
            except Exception as e:
                print(f"⚠️ 夹爪控制失败: {e}，继续执行...")

            print("进入施教模式...")
            self.lebai.teach_mode()
            print("✅ 已进入施教模式")
            print("现在你可以手动移动机器人到电梯按钮位置")
            return True
        except Exception as e:
            print(f"❌ 进入施教模式失败: {e}")
            return False

    def manual_record_button(self, floor_name, button_type="floor", x=None, y=None, z=None, rx=None, ry=None, rz=None):
        """
        手动记录按钮位置（当机器人无法连接时使用）
        位置参数需要从机器人界面或日志中获取
        """
        # 检查输入参数
        if x is None or y is None or z is None:
            print("❌ 请提供按钮位置坐标 (x, y, z)")
            print("💡 示例: manual_record_button('1F', x=0.123, y=0.456, z=0.789)")
            return False

        # 使用默认的旋转角度如果未提供
        if rx is None: rx = 0.0
        if ry is None: ry = 0.0
        if rz is None: rz = 0.0

        # 验证坐标范围（合理的机器人工作空间）
        if not (-2.0 <= x <= 2.0 and -2.0 <= y <= 2.0 and -0.5 <= z <= 2.0):
            print(f"⚠️ 坐标值可能超出正常范围: x={x}, y={y}, z={z}")
            choice = input("是否继续保存? (y/N): ").strip().lower()
            if choice != 'y':
                return False

        pose = {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'rx': float(rx),
            'ry': float(ry),
            'rz': float(rz)
        }

        button_key = f"{floor_name}_{button_type}"

        button_data = {
            'pose': pose,
            'image': None,  # 手动录入时没有图像
            'floor': floor_name,
            'type': button_type,
            'timestamp': datetime.now().strftime("%Y%m%d_%H%M%S"),
            'recorded_at': datetime.now().isoformat(),
            'method': 'manual_entry'
        }

        self.button_positions[button_key] = button_data

        # 记录楼层名称（如果还没记录）
        if floor_name not in self.floor_names:
            self.floor_names.append(floor_name)

        print(f"✅ 已手动记录楼层 {floor_name} {button_type} 按钮位置:")
        print(f"   位置: x={x}, y={y}, z={z}")
        print(f"   旋转: rx={rx}, ry={ry}, rz={rz}")

        # 自动保存临时数据
        self.auto_save_temp_data()

        return True

    def remote_trigger_capture(self):
        """远程触发拍照"""
        url = f'http://{self.remote_camera_ip}:{self.remote_camera_port}/'
        
        try:
            print(f"正在远程触发拍照... ({url})")
            response = requests.post(url, timeout=5)
            
            if response.status_code == 200:
                print("✅ 远程拍照成功!")
                return True
            else:
                print(f"⚠️ 远程拍照失败，状态码: {response.status_code}")
                print(f"响应: {response.text}")
                return False
                
        except requests.RequestException as e:
            print(f"❌ 远程拍照请求失败: {e}")
            return False
        except Exception as e:
            print(f"❌ 远程拍照异常: {e}")
            return False

    def capture_button_position(self, floor_name, button_type="floor", use_remote_camera=False):
        """
        拍照并记录按钮位置
        floor_name: 楼层名称，如 "1F", "2F", "B1" 等
        button_type: 按钮类型，如 "floor", "open", "close", "alarm" 等
        use_remote_camera: 是否使用远程相机
        """
        # 检查机器人连接状态
        if self.lebai is None:
            print(f"❌ 无法记录按钮位置: 机器人未连接")
            print(f"💡 请先确保机器人连接正常，然后重新运行程序")
            return False

        try:
            # 获取当前机器人位置
            print(f"正在获取机器人位置...")
            kin_data = self.lebai.get_kin_data()
            if not kin_data or "actual_tcp_pose" not in kin_data:
                print(f"❌ 无法获取机器人位置数据")
                return False

            current_pose = kin_data["actual_tcp_pose"]
            print(f"记录楼层 {floor_name} 按钮位置...")
            print(f"当前位置: x={current_pose['x']:.4f}, y={current_pose['y']:.4f}, z={current_pose['z']:.4f}")
            print(f"旋转角度: rx={current_pose.get('rx', 0):.4f}, ry={current_pose.get('ry', 0):.4f}, rz={current_pose.get('rz', 0):.4f}")

            # 拍照
            print("正在拍照...")
            image_filename = None
            
            if use_remote_camera:
                # 使用远程相机
                if self.remote_trigger_capture():
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    image_filename = f"{self.data_dir}/button_{floor_name}_{button_type}_{timestamp}_remote.jpg"
                    print(f"✅ 远程拍照完成，图像文件: {image_filename}")
                else:
                    print("⚠️ 远程拍照失败，但位置信息已记录")
            else:
                # 使用本地相机
                cap = cv2.VideoCapture("/dev/video0")
                if cap is None or not cap.isOpened():
                    print("尝试使用 /dev/video1...")
                    cap = cv2.VideoCapture("/dev/video1")

                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        image_filename = f"{self.data_dir}/button_{floor_name}_{button_type}_{timestamp}.jpg"
                        success = cv2.imwrite(image_filename, frame)
                        if success:
                            print(f"✅ 按钮照片已保存: {image_filename}")
                        else:
                            print(f"❌ 照片保存失败")
                            image_filename = None
                    else:
                        print("❌ 拍照失败")
                    cap.release()
                else:
                    print("⚠️ 无法拍照，但位置信息已记录")

            # 保存按钮信息
            button_key = f"{floor_name}_{button_type}"
            button_data = {
                'pose': current_pose,
                'image': image_filename,
                'floor': floor_name,
                'type': button_type,
                'timestamp': datetime.now().strftime("%Y%m%d_%H%M%S"),
                'recorded_at': datetime.now().isoformat(),
                'camera_type': 'remote' if use_remote_camera else 'local'
            }

            self.button_positions[button_key] = button_data

            # 记录楼层名称（如果还没记录）
            if floor_name not in self.floor_names:
                self.floor_names.append(floor_name)

            print(f"✅ 按钮位置数据已记录 ({len(self.button_positions)} 个按钮)")

            # 自动保存临时数据
            self.auto_save_temp_data()

            return True

        except AttributeError as e:
            print(f"❌ 机器人连接错误: {e}")
            print("💡 请检查机器人连接状态")
            return False
        except KeyError as e:
            print(f"❌ 数据格式错误: {e}")
            print("💡 机器人返回的数据格式可能不正确")
            return False
        except Exception as e:
            print(f"❌ 记录按钮位置失败: {e}")
            print(f"错误类型: {type(e).__name__}")
            return False

    def auto_save_temp_data(self):
        """自动保存临时数据，避免数据丢失"""
        try:
            temp_data = {
                'floors': self.floor_names,
                'button_positions': self.button_positions,
                'temp_save_time': datetime.now().isoformat(),
                'robot_ip': LEBAI_IP,
                'button_count': len(self.button_positions)
            }

            temp_filename = f"{self.data_dir}/temp_elevator_buttons.json"
            with open(temp_filename, 'w', encoding='utf-8') as f:
                json.dump(temp_data, f, indent=2, ensure_ascii=False)

            print(f"💾 临时数据已自动保存 ({len(self.button_positions)} 个按钮)")
            return True

        except Exception as e:
            print(f"⚠️ 临时数据保存失败: {e}")
            return False

    def load_temp_data(self):
        """加载临时数据"""
        try:
            temp_filename = f"{self.data_dir}/temp_elevator_buttons.json"
            if not os.path.exists(temp_filename):
                print("没有找到临时数据文件")
                return False

            with open(temp_filename, 'r', encoding='utf-8') as f:
                temp_data = json.load(f)

            self.floor_names = temp_data.get('floors', [])
            self.button_positions = temp_data.get('button_positions', {})

            button_count = len(self.button_positions)
            save_time = temp_data.get('temp_save_time', '未知时间')

            print(f"✅ 已恢复临时数据: {button_count} 个按钮 (保存时间: {save_time})")
            return True

        except Exception as e:
            print(f"❌ 加载临时数据失败: {e}")
            return False

    def clear_temp_data(self):
        """清除临时数据文件"""
        try:
            temp_filename = f"{self.data_dir}/temp_elevator_buttons.json"
            if os.path.exists(temp_filename):
                os.remove(temp_filename)
                print("✅ 临时数据文件已清除")
            return True
        except Exception as e:
            print(f"⚠️ 清除临时数据失败: {e}")
            return False

    def save_button_mapping(self):
        """保存按钮映射数据到文件"""
        if not self.button_positions:
            print("❌ 没有按钮数据可保存")
            return None

        try:
            data = {
                'floors': self.floor_names,
                'button_positions': self.button_positions,
                'created_at': datetime.now().isoformat(),
                'robot_ip': LEBAI_IP,
                'total_buttons': len(self.button_positions),
                'floor_count': len(self.floor_names)
            }

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"{self.data_dir}/elevator_buttons_{timestamp}.json"

            # 确保目录存在
            os.makedirs(self.data_dir, exist_ok=True)

            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)

            print(f"✅ 按钮映射已保存到: {filename}")
            print(f"📊 共保存 {len(self.button_positions)} 个按钮，{len(self.floor_names)} 个楼层")

            # 保存成功后清除临时数据
            self.clear_temp_data()

            return filename

        except Exception as e:
            print(f"❌ 保存数据失败: {e}")
            print(f"错误类型: {type(e).__name__}")
            return None

    def load_button_mapping(self, filename=None):
        """加载按钮映射数据"""
        try:
            if filename is None:
                # 查找最新的映射文件
                if not os.path.exists(self.data_dir):
                    print(f"❌ 数据目录不存在: {self.data_dir}")
                    return False

                files = [f for f in os.listdir(self.data_dir) if f.startswith('elevator_buttons_') and f.endswith('.json')]
                if not files:
                    print("❌ 未找到按钮映射文件")
                    return False
                filename = max(files, key=lambda x: os.path.getctime(os.path.join(self.data_dir, x)))

            # 处理完整路径或相对路径
            if os.path.isabs(filename):
                filepath = filename
            else:
                filepath = os.path.join(self.data_dir, filename)

            if not os.path.exists(filepath):
                print(f"❌ 文件不存在: {filepath}")
                return False

            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)

            self.floor_names = data.get('floors', [])
            self.button_positions = data.get('button_positions', {})

            button_count = len(self.button_positions)
            print(f"✅ 已加载按钮映射: {filepath}")
            print(f"📊 共加载 {button_count} 个按钮，{len(self.floor_names)} 个楼层")
            print(f"🏢 楼层列表: {self.floor_names}")
            return True

        except json.JSONDecodeError as e:
            print(f"❌ JSON文件格式错误: {e}")
            return False
        except FileNotFoundError as e:
            print(f"❌ 文件未找到: {e}")
            return False
        except Exception as e:
            print(f"❌ 加载数据失败: {e}")
            print(f"错误类型: {type(e).__name__}")
            return False

    def show_current_buttons(self):
        """显示当前记录的所有按钮"""
        if not self.button_positions:
            print("当前没有记录任何按钮位置")
            return

        print("当前记录的按钮:")
        print("=" * 60)
        print(f"总共记录了 {len(self.button_positions)} 个按钮，{len(self.floor_names)} 个楼层")
        print()

        for button_key, button_data in self.button_positions.items():
            pose = button_data['pose']
            floor = button_data['floor']
            button_type = button_data['type']
            timestamp = button_data.get('timestamp', '未知时间')

            print(f"🏢 楼层: {floor} | 🔘 类型: {button_type}")
            print(f"📍 位置: x={pose['x']:.4f}, y={pose['y']:.4f}, z={pose['z']:.4f}")
            print(f"🔄 旋转: rx={pose.get('rx', 0):.4f}, ry={pose.get('ry', 0):.4f}, rz={pose.get('rz', 0):.4f}")
            print(f"🕒 记录时间: {timestamp}")
            if button_data.get('image'):
                print(f"📸 照片: {button_data['image']}")
            print("-" * 40)

    def list_saved_mappings(self):
        """列出已保存的映射文件"""
        try:
            files = [f for f in os.listdir(self.data_dir) if f.startswith('elevator_buttons_') and f.endswith('.json')]
            if not files:
                print("没有找到保存的按钮映射文件")
                return []

            print("已保存的按钮映射文件:")
            for i, filename in enumerate(sorted(files, reverse=True)):
                filepath = os.path.join(self.data_dir, filename)
                created_time = datetime.fromtimestamp(os.path.getctime(filepath)).strftime('%Y-%m-%d %H:%M:%S')
                print(f"{i+1}. {filename} (创建时间: {created_time})")

            return files

        except Exception as e:
            print(f"❌ 列出文件失败: {e}")
            return []

    def go_to_button(self, floor_name, button_type="floor"):
        """移动到指定楼层的按钮位置"""
        try:
            button_key = f"{floor_name}_{button_type}"
            if button_key not in self.button_positions:
                print(f"❌ 未找到楼层 {floor_name} 的按钮位置")
                return False

            pose = self.button_positions[button_key]['pose']
            print(f"移动到楼层 {floor_name} 按钮位置...")

            # 移动到按钮位置（稍微后退一点，避免碰到按钮）
            target_pose = pose.copy()
            target_pose['x'] -= 0.05  # 后退5cm

            self.lebai.movej([
                target_pose['x'], target_pose['y'], target_pose['z'],
                target_pose['rx'], target_pose['ry'], target_pose['rz']
            ], 1.0, 0.5)

            self.lebai.wait_move()
            print(f"✅ 已到达楼层 {floor_name} 按钮位置")
            return True

        except Exception as e:
            print(f"❌ 移动失败: {e}")
            return False

    def press_button(self, floor_name, button_type="floor"):
        """按下指定楼层的按钮"""
        try:
            if not self.go_to_button(floor_name, button_type):
                return False

            print(f"按下楼层 {floor_name} 按钮...")

            # 向前移动按按钮
            current_pose = self.lebai.get_kin_data()["actual_tcp_pose"]
            press_pose = current_pose.copy()
            press_pose['x'] += 0.03  # 向前3cm按按钮

            self.lebai.movej([
                press_pose['x'], press_pose['y'], press_pose['z'],
                press_pose['rx'], press_pose['ry'], press_pose['rz']
            ], 0.5, 0.3)

            self.lebai.wait_move()
            time.sleep(0.5)  # 按住0.5秒

            # 后退
            self.lebai.movej([
                current_pose['x'], current_pose['y'], current_pose['z'],
                current_pose['rx'], current_pose['ry'], current_pose['rz']
            ], 0.5, 0.3)

            self.lebai.wait_move()
            print(f"✅ 已按下楼层 {floor_name} 按钮")
            return True

        except Exception as e:
            print(f"❌ 按按钮失败: {e}")
            return False

    def set_remote_camera_config(self, ip, port):
        """设置远程相机配置"""
        self.remote_camera_ip = ip
        self.remote_camera_port = port
        print(f"✅ 远程相机配置已更新: {ip}:{port}")

    def show_remote_camera_config(self):
        """显示远程相机配置"""
        print(f"📷 远程相机配置:")
        print(f"   IP地址: {self.remote_camera_ip}")
        print(f"   端口: {self.remote_camera_port}")

    def interactive_teach_mode(self):
        """交互式施教模式"""
        print("\n=== 电梯按钮施教系统 ===")
        print("指令:")
        print("  t <楼层名> - 记录楼层按钮位置")
        print("  tr <楼层名> - 远程拍照记录楼层按钮位置")
        print("  o - 记录开门按钮")
        print("  or - 远程拍照记录开门按钮")
        print("  c - 记录关门按钮")
        print("  cr - 远程拍照记录关门按钮")
        print("  a - 记录报警按钮")
        print("  ar - 远程拍照记录报警按钮")
        print("  s - 保存映射数据")
        print("  l - 列出保存的文件")
        print("  r - 恢复临时数据")
        print("  show - 显示当前记录的按钮")
        print("  clear - 清除所有数据")
        print("  config - 显示远程相机配置")
        print("  setcam <ip> <port> - 设置远程相机配置")
        print("  q - 退出施教模式")
        print("  h - 显示帮助")

        while True:
            try:
                cmd = input("\n请输入指令: ").strip().lower()

                if cmd == 'q':
                    break
                elif cmd == 'h':
                    print("指令:")
                    print("  t <楼层名> - 记录楼层按钮位置")
                    print("  tr <楼层名> - 远程拍照记录楼层按钮位置")
                    print("  o - 记录开门按钮")
                    print("  or - 远程拍照记录开门按钮")
                    print("  c - 记录关门按钮")
                    print("  cr - 远程拍照记录关门按钮")
                    print("  a - 记录报警按钮")
                    print("  ar - 远程拍照记录报警按钮")
                    print("  s - 保存映射数据")
                    print("  l - 列出保存的文件")
                    print("  r - 恢复临时数据")
                    print("  show - 显示当前记录的按钮")
                    print("  clear - 清除所有数据")
                    print("  config - 显示远程相机配置")
                    print("  setcam <ip> <port> - 设置远程相机配置")
                    print("  q - 退出施教模式")
                elif cmd.startswith('t '):
                    floor_name = cmd[2:].strip()
                    if floor_name:
                        success = self.capture_button_position(floor_name, "floor", use_remote_camera=False)
                        if success:
                            print(f"✅ 楼层 {floor_name} 按钮位置已记录")
                        else:
                            print(f"❌ 记录楼层 {floor_name} 按钮位置失败")
                    else:
                        print("❌ 请输入楼层名称，如: t 1F")
                elif cmd.startswith('tr '):
                    floor_name = cmd[3:].strip()
                    if floor_name:
                        success = self.capture_button_position(floor_name, "floor", use_remote_camera=True)
                        if success:
                            print(f"✅ 楼层 {floor_name} 按钮位置已记录（远程拍照）")
                        else:
                            print(f"❌ 记录楼层 {floor_name} 按钮位置失败")
                    else:
                        print("❌ 请输入楼层名称，如: tr 1F")
                elif cmd == 'o':
                    success = self.capture_button_position("elevator", "open", use_remote_camera=False)
                    if success:
                        print("✅ 开门按钮位置已记录")
                    else:
                        print("❌ 记录开门按钮位置失败")
                elif cmd == 'or':
                    success = self.capture_button_position("elevator", "open", use_remote_camera=True)
                    if success:
                        print("✅ 开门按钮位置已记录（远程拍照）")
                    else:
                        print("❌ 记录开门按钮位置失败")
                elif cmd == 'c':
                    success = self.capture_button_position("elevator", "close", use_remote_camera=False)
                    if success:
                        print("✅ 关门按钮位置已记录")
                    else:
                        print("❌ 记录关门按钮位置失败")
                elif cmd == 'cr':
                    success = self.capture_button_position("elevator", "close", use_remote_camera=True)
                    if success:
                        print("✅ 关门按钮位置已记录（远程拍照）")
                    else:
                        print("❌ 记录关门按钮位置失败")
                elif cmd == 'a':
                    success = self.capture_button_position("elevator", "alarm", use_remote_camera=False)
                    if success:
                        print("✅ 报警按钮位置已记录")
                    else:
                        print("❌ 记录报警按钮位置失败")
                elif cmd == 'ar':
                    success = self.capture_button_position("elevator", "alarm", use_remote_camera=True)
                    if success:
                        print("✅ 报警按钮位置已记录（远程拍照）")
                    else:
                        print("❌ 记录报警按钮位置失败")
                elif cmd == 's':
                    if self.button_positions:
                        filename = self.save_button_mapping()
                        if filename:
                            print(f"✅ 数据已保存，可以使用 elevator_control.py 控制机器人")
                    else:
                        print("❌ 没有数据可保存，请先记录一些按钮位置")
                elif cmd == 'l':
                    self.list_saved_mappings()
                elif cmd == 'r':
                    success = self.load_temp_data()
                    if not success:
                        print("💡 没有临时数据可恢复")
                elif cmd == 'show':
                    self.show_current_buttons()
                elif cmd == 'clear':
                    confirm = input("确定要清除所有已记录的数据吗? (y/N): ").strip().lower()
                    if confirm == 'y':
                        self.button_positions.clear()
                        self.floor_names.clear()
                        self.clear_temp_data()
                        print("✅ 所有数据已清除")
                    else:
                        print("已取消清除操作")
                elif cmd == 'config':
                    self.show_remote_camera_config()
                elif cmd.startswith('setcam '):
                    parts = cmd[7:].strip().split()
                    if len(parts) == 2:
                        try:
                            ip = parts[0]
                            port = int(parts[1])
                            self.set_remote_camera_config(ip, port)
                        except ValueError:
                            print("❌ 端口号必须是数字，如: setcam 192.168.10.201 2001")
                    else:
                        print("❌ 格式错误，正确格式: setcam <ip> <port>")
                        print("   示例: setcam 192.168.10.201 2001")
                else:
                    print("❌ 未知指令，请输入 'h' 查看帮助")

            except KeyboardInterrupt:
                print("\n⚠️ 收到中断信号，正在保存数据...")
                if self.button_positions:
                    self.save_button_mapping()
                break
            except Exception as e:
                print(f"❌ 指令执行失败: {e}")
                print(f"错误类型: {type(e).__name__}")

    def end_teach_mode(self):
        """结束施教模式"""
        try:
            print("结束施教模式...")
            self.lebai.end_teach_mode()
            print("✅ 已结束施教模式")
            return True
        except Exception as e:
            print(f"❌ 结束施教模式失败: {e}")
            return False

def main():
    teacher = ElevatorButtonTeacher()

    print("=== 电梯按钮施教系统 ===")

    # 尝试恢复临时数据
    print("正在检查是否有未保存的临时数据...")
    has_temp_data = teacher.load_temp_data()

    if has_temp_data:
        print("发现临时数据，是否要继续使用这些数据？(y/N): ", end="")
        choice = input().strip().lower()
        if choice != 'y':
            print("正在清除临时数据...")
            teacher.clear_temp_data()
            teacher.button_positions.clear()
            teacher.floor_names.clear()

    # 连接机器人
    if not teacher.connect_robot():
        print("❌ 无法连接机器人，请检查：")
        print("   1. 机器人是否开机")
        print("   2. 网络连接是否正常")
        print("   3. IP地址是否正确 (当前: 192.168.10.200)")
        return

    # 进入施教模式
    if not teacher.start_teach_mode():
        print("❌ 无法进入施教模式")
        return

    try:
        # 显示当前状态
        if teacher.button_positions:
            print(f"\n📊 当前已记录 {len(teacher.button_positions)} 个按钮位置")
            teacher.show_current_buttons()
        else:
            print("\n📝 当前没有记录任何按钮位置，请开始录入")

        # 交互式施教
        teacher.interactive_teach_mode()

        # 保存数据
        if teacher.button_positions:
            print("\n正在保存数据...")
            filename = teacher.save_button_mapping()
            if filename:
                print("\n🎉 施教完成！")
                print(f"📁 数据文件: {filename}")
                print(f"📊 保存了 {len(teacher.button_positions)} 个按钮，{len(teacher.floor_names)} 个楼层")
                print("\n💡 现在可以使用以下命令控制机器人:")
                print("   python elevator_control.py")
        else:
            print("\n⚠️ 没有记录任何按钮位置")

    except Exception as e:
        print(f"\n❌ 施教过程中发生错误: {e}")
        print("正在尝试保存已记录的数据...")
        if teacher.button_positions:
            teacher.save_button_mapping()
    finally:
        # 结束施教模式
        print("\n正在结束施教模式...")
        teacher.end_teach_mode()

        try:
            if teacher.lebai:
                teacher.lebai.stop_sys()
        except:
            pass

        print("施教系统已关闭")

if __name__ == "__main__":
    main()
