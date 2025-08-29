#!/usr/bin/env python3
"""
测试基于关节电压的压感检测
用于验证 get_phy_data() 方法和电压下降检测的准确性

使用方法:
1. 连接机器人
2. 建立基线电压
3. 手动对机器人施加阻力
4. 观察电压变化和检测结果

作者: AI Assistant
"""

import time
import lebai_sdk
from pressure_detector import LebaiPressureDetector, create_voltage_thresholds


def test_voltage_detection():
    """测试电压检测功能"""
    print("🔋 测试基于关节电压的压感检测")
    print("=" * 50)
    
    # 连接机器人
    robot_ip = input("请输入机器人IP地址 (默认: 192.168.10.200): ").strip()
    if not robot_ip:
        robot_ip = "192.168.10.200"
    
    try:
        print("🔗 连接机器人...")
        lebai_sdk.init()
        lebai = lebai_sdk.connect(robot_ip, False)
        print("✅ 机器人连接成功")
        
        # 创建检测器
        thresholds = create_voltage_thresholds("normal")  # 使用正常敏感度
        detector = LebaiPressureDetector(lebai, thresholds)
        
        # 开始监控
        detector.start_monitoring()
        
        print("\n📊 开始实时监控...")
        print("💡 提示: 手动对机器人施加阻力来测试检测")
        print("🛑 按 Ctrl+C 停止测试")
        
        test_count = 0
        detection_count = 0
        
        while True:
            try:
                test_count += 1
                
                # 检查压感
                if detector.is_pressure_detected():
                    detection_count += 1
                    event = detector.get_last_pressure_event()
                    
                    print(f"\n🚨 检测到压感! (#{detection_count})")
                    print(f"   🔍 检测方法: {event.detection_method.value}")
                    print(f"   🎯 置信度: {event.confidence:.3f}")
                    print(f"   🔧 受影响关节: {event.affected_joints}")
                    
                    if event.voltage_drops:
                        max_drop = max(event.voltage_drops)
                        max_joint = event.voltage_drops.index(max_drop)
                        print(f"   🔋 最大电压下降: 关节{max_joint} = {max_drop:.2f}V")
                    
                    # 重置检测状态
                    detector.reset_pressure_state()
                    time.sleep(1)  # 避免重复检测
                
                # 每10次循环显示一次状态
                if test_count % 10 == 0:
                    status = detector.get_current_joint_status()
                    if status:
                        voltages = status['joint_voltages'][:6]  # 显示前6个关节
                        print(f"\r📊 [{test_count:4d}] 关节电压: {[f'{v:.1f}V' for v in voltages]} ", end='', flush=True)
                
                time.sleep(0.1)  # 10Hz监控频率
                
            except KeyboardInterrupt:
                print(f"\n\n🛑 测试结束")
                print(f"📈 测试统计:")
                print(f"   ⏱️  总测试次数: {test_count}")
                print(f"   🚨 检测次数: {detection_count}")
                print(f"   📊 检测率: {(detection_count/test_count*100):.2f}%" if test_count > 0 else "0%")
                break
                
            except Exception as e:
                print(f"⚠️ 测试错误: {e}")
                time.sleep(0.5)
        
        # 停止监控
        detector.stop_monitoring()
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
    
    finally:
        print("👋 测试结束！")


def show_raw_phy_data():
    """显示原始物理数据 (用于调试)"""
    print("🔍 显示原始 get_phy_data() 数据")
    print("=" * 40)
    
    robot_ip = input("请输入机器人IP地址 (默认: 192.168.10.200): ").strip()
    if not robot_ip:
        robot_ip = "192.168.10.200"
    
    try:
        print("🔗 连接机器人...")
        lebai_sdk.init()
        lebai = lebai_sdk.connect(robot_ip, False)
        print("✅ 机器人连接成功")
        
        print("\n📊 实时物理数据 (按 Ctrl+C 停止):")
        
        count = 0
        while True:
            try:
                count += 1
                phy_data = lebai.get_phy_data()
                
                if phy_data:
                    print(f"\n--- 数据 #{count} ---")
                    
                    # 显示关节电压
                    if 'joint_voltage' in phy_data:
                        voltages = phy_data['joint_voltage']
                        print(f"🔋 关节电压: {[f'{v:.2f}V' for v in voltages[:6]]}")
                    
                    # 显示关节电流
                    if 'joint_current' in phy_data:
                        currents = phy_data['joint_current']
                        print(f"⚡ 关节电流: {[f'{c:.3f}A' for c in currents[:6]]}")
                    
                    # 显示其他可用数据键
                    other_keys = [k for k in phy_data.keys() if k not in ['joint_voltage', 'joint_current']]
                    if other_keys:
                        print(f"📝 其他数据键: {other_keys}")
                
                else:
                    print(f"❌ 数据 #{count}: 无数据返回")
                
                time.sleep(1)  # 1秒间隔
                
            except KeyboardInterrupt:
                print(f"\n🛑 停止数据显示")
                break
                
            except Exception as e:
                print(f"⚠️ 获取数据错误: {e}")
                time.sleep(0.5)
        
    except Exception as e:
        print(f"❌ 连接失败: {e}")


if __name__ == "__main__":
    print("🔋 关节电压检测测试工具")
    print("1. 测试压感检测")
    print("2. 显示原始物理数据")
    
    choice = input("请选择 (1/2): ").strip()
    
    if choice == "2":
        show_raw_phy_data()
    else:
        test_voltage_detection()
