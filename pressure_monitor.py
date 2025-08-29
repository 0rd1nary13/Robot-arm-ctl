#!/usr/bin/env python3
"""
压感检测监控器 - 后台运行版本
在机器人操作过程中后台监控压感，记录碰撞事件
配合teleop.py使用，操作结束后生成时间-碰撞事件图表

使用方法:
1. 启动此脚本进行后台监控
2. 同时运行teleop.py操作机器人
3. 操作结束后查看生成的图表

作者: AI Assistant
"""

import time
import threading
import lebai_sdk
import json
import matplotlib.pyplot as plt
from datetime import datetime
from pressure_detector import LebaiPressureDetector, PressureThresholds
from typing import List, Dict, Optional
import signal
import sys


class PressureMonitor:
    """压感监控器 - 简化版本"""
    
    def __init__(self, robot_ip: str = "192.168.10.200"):
        """
        初始化监控器
        
        Args:
            robot_ip: 机器人IP地址
        """
        self.robot_ip = robot_ip
        self.lebai = None
        self.pressure_detector = None
        self.is_monitoring = False
        self.monitor_thread = None
        
        # 数据记录
        self.collision_events = []  # 碰撞事件记录
        self.start_time = None
        self.session_data = {
            'start_time': None,
            'end_time': None,
            'total_duration': 0,
            'collision_count': 0,
            'collision_events': []
        }
        
        # 设置信号处理器用于优雅退出
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        print("🤖 压感监控器 - 后台运行版本")
        print(f"🌐 机器人IP: {robot_ip}")
    
    def _signal_handler(self, signum, frame):
        """信号处理器 - 优雅退出"""
        print(f"\n🛑 接收到信号 {signum}，正在停止监控...")
        self.stop_monitoring()
        self._generate_report()
        sys.exit(0)
    
    def connect_robot(self) -> bool:
        """连接机器人"""
        try:
            print("🔗 连接机器人...")
            lebai_sdk.init()
            self.lebai = lebai_sdk.connect(self.robot_ip, False)
            
            # 不需要启动系统，因为teleop.py会处理
            print("✅ 机器人连接成功")
            return True
            
        except Exception as e:
            print(f"❌ 机器人连接失败: {e}")
            return False
    
    def setup_pressure_detector(self, sensitivity: str = "normal") -> None:
        """
        设置压感检测器
        
        Args:
            sensitivity: 敏感度 ("high", "normal", "low")
        """
        # 针对teleop操作优化的敏感度设置
        if sensitivity == "high":
            thresholds = PressureThresholds(
                position_deviation_threshold=0.015,  # 15mm - teleop操作更敏感
                velocity_threshold=0.008,            # 8mm/s
                contact_confidence_threshold=0.75,   # 75% 置信度
                detection_frequency=20.0             # 20Hz
            )
        elif sensitivity == "low":
            thresholds = PressureThresholds(
                position_deviation_threshold=0.06,   # 60mm - 大动作操作
                velocity_threshold=0.025,            # 25mm/s
                contact_confidence_threshold=0.9,    # 90% 高置信度
                detection_frequency=10.0             # 10Hz
            )
        else:  # normal - 适合teleop操作
            thresholds = PressureThresholds(
                position_deviation_threshold=0.03,   # 30mm
                velocity_threshold=0.015,            # 15mm/s
                contact_confidence_threshold=0.8,    # 80% 置信度
                detection_frequency=15.0             # 15Hz
            )
        
        self.pressure_detector = LebaiPressureDetector(self.lebai, thresholds)
        print(f"🔍 压感检测器设置完成 (敏感度: {sensitivity})")
    
    def start_monitoring(self) -> None:
        """开始后台监控"""
        if not self.pressure_detector:
            print("❌ 压感检测器未初始化")
            return
        
        try:
            # 开始压感监控 (运动状态分析模式)
            self.pressure_detector.start_monitoring(None)
            
            # 记录开始时间
            self.start_time = time.time()
            self.session_data['start_time'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # 启动监控线程
            self.is_monitoring = True
            self.monitor_thread = threading.Thread(target=self._monitoring_loop)
            self.monitor_thread.daemon = True
            self.monitor_thread.start()
            
            print("📡 压感监控已启动 (后台运行)")
            print("🎮 现在可以运行 teleop.py 开始操作机器人")
            print("🛑 按 Ctrl+C 停止监控并生成报告")
            
        except Exception as e:
            print(f"❌ 启动监控失败: {e}")
    
    def _monitoring_loop(self) -> None:
        """监控循环"""
        last_event = None
        
        while self.is_monitoring:
            try:
                # 检查压感检测
                if self.pressure_detector and self.pressure_detector.is_pressure_detected():
                    event = self.pressure_detector.get_last_pressure_event()
                    if event != last_event:
                        self._record_collision_event(event)
                        last_event = event
                        
                        # 重置检测状态，继续监控
                        time.sleep(0.5)  # 短暂延迟避免重复检测
                        self.pressure_detector.reset_pressure_state()
                
                time.sleep(0.067)  # 约15Hz
                
            except Exception as e:
                print(f"⚠️ 监控循环错误: {e}")
                time.sleep(0.1)
    
    def _record_collision_event(self, event) -> None:
        """记录碰撞事件"""
        if not self.start_time:
            return
        
        # 计算相对时间
        relative_time = time.time() - self.start_time
        
        # 记录事件
        collision_data = {
            'time': relative_time,
            'timestamp': datetime.fromtimestamp(event.timestamp).strftime('%H:%M:%S.%f')[:-3],
            'detection_method': event.detection_method.value,
            'confidence': event.confidence,
            'tcp_position': event.tcp_position,
            'tcp_velocity': event.tcp_velocity
        }
        
        self.collision_events.append(collision_data)
        self.session_data['collision_count'] += 1
        
        # 实时显示碰撞事件
        print(f"\n🚨 [{collision_data['timestamp']}] 检测到碰撞!")
        print(f"   ⏱️  时间: {relative_time:.2f}s")
        print(f"   🔍 方法: {event.detection_method.value}")
        print(f"   🎯 置信度: {event.confidence:.2f}")
        print(f"   📍 位置: x={event.tcp_position.get('x', 0):.3f}, "
              f"y={event.tcp_position.get('y', 0):.3f}, z={event.tcp_position.get('z', 0):.3f}")
        print("   继续监控中...")
    
    def stop_monitoring(self) -> None:
        """停止监控"""
        self.is_monitoring = False
        
        if self.pressure_detector:
            self.pressure_detector.stop_monitoring()
        
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        
        # 记录结束时间
        if self.start_time:
            self.session_data['end_time'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            self.session_data['total_duration'] = time.time() - self.start_time
            self.session_data['collision_events'] = self.collision_events
        
        print("🛑 压感监控已停止")
    
    def _generate_report(self) -> None:
        """生成监控报告和图表"""
        if not self.collision_events and not self.start_time:
            print("📊 无监控数据，跳过报告生成")
            return
        
        print(f"\n📊 生成监控报告...")
        
        # 生成文本报告
        self._save_text_report()
        
        # 生成图表
        self._generate_collision_chart()
        
        print("✅ 报告生成完成!")
    
    def _save_text_report(self) -> None:
        """保存文本报告"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_file = f"pressure_monitor_report_{timestamp}.json"
        
        try:
            with open(report_file, 'w', encoding='utf-8') as f:
                json.dump(self.session_data, f, indent=2, ensure_ascii=False)
            
            print(f"📄 文本报告已保存: {report_file}")
            
            # 打印摘要
            print(f"\n📋 监控摘要:")
            print(f"   ⏰ 开始时间: {self.session_data['start_time']}")
            print(f"   ⏰ 结束时间: {self.session_data['end_time']}")
            print(f"   ⏱️  总时长: {self.session_data['total_duration']:.1f}秒")
            print(f"   🚨 碰撞次数: {self.session_data['collision_count']}")
        
        except Exception as e:
            print(f"❌ 保存报告失败: {e}")
    
    def _generate_collision_chart(self) -> None:
        """生成时间-碰撞事件图表"""
        if not self.collision_events:
            print("📈 无碰撞事件，跳过图表生成")
            return
        
        try:
            # 准备数据
            collision_times = [event['time'] for event in self.collision_events]
            collision_confidences = [event['confidence'] for event in self.collision_events]
            total_time = self.session_data['total_duration']
            
            # 创建图表
            plt.style.use('dark_background')
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
            
            # 子图1: 时间线上的碰撞事件
            ax1.scatter(collision_times, [1]*len(collision_times), 
                       c=collision_confidences, cmap='Reds', s=100, alpha=0.8)
            ax1.set_xlim(0, max(total_time, max(collision_times) if collision_times else 1))
            ax1.set_ylim(0.5, 1.5)
            ax1.set_xlabel('Time (seconds)', color='white')
            ax1.set_title('Collision Events Timeline', color='cyan', fontsize=14)
            ax1.grid(True, alpha=0.3)
            ax1.set_yticks([])
            
            # 添加碰撞事件标注
            for i, (t, conf) in enumerate(zip(collision_times, collision_confidences)):
                ax1.annotate(f'#{i+1}\n{conf:.2f}', 
                           xy=(t, 1), xytext=(t, 1.3),
                           ha='center', va='bottom', color='yellow',
                           arrowprops=dict(arrowstyle='->', color='yellow', alpha=0.7))
            
            # 子图2: 碰撞置信度柱状图
            bars = ax2.bar(range(1, len(collision_times)+1), collision_confidences, 
                          color='orange', alpha=0.7)
            ax2.set_xlabel('Collision Event #', color='white')
            ax2.set_ylabel('Confidence', color='white')
            ax2.set_title('Collision Detection Confidence', color='cyan', fontsize=14)
            ax2.grid(True, alpha=0.3)
            ax2.set_ylim(0, 1.0)
            
            # 添加数值标注
            for i, (bar, conf) in enumerate(zip(bars, collision_confidences)):
                ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02,
                        f'{conf:.2f}', ha='center', va='bottom', color='white')
            
            # 设置整体标题
            fig.suptitle(f'Pressure Detection Report - {len(collision_times)} Collisions in {total_time:.1f}s', 
                        color='white', fontsize=16)
            
            plt.tight_layout()
            
            # 保存图表
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            chart_file = f"collision_events_{timestamp}.png"
            plt.savefig(chart_file, dpi=300, bbox_inches='tight', 
                       facecolor='black', edgecolor='none')
            
            print(f"📈 碰撞事件图表已保存: {chart_file}")
            
            # 显示图表
            plt.show()
            
        except Exception as e:
            print(f"❌ 生成图表失败: {e}")
    
    def run(self) -> None:
        """运行监控器"""
        print("\n🚀 启动压感监控器...")
        
        # 连接机器人
        if not self.connect_robot():
            return
        
        # 设置压感检测器
        sensitivity = input("选择敏感度 (high/normal/low) [默认: normal]: ").strip().lower()
        if sensitivity not in ['high', 'normal', 'low']:
            sensitivity = 'normal'
        
        self.setup_pressure_detector(sensitivity)
        
        # 开始监控
        self.start_monitoring()
        
        try:
            # 主线程等待用户中断
            while self.is_monitoring:
                time.sleep(1)
        
        except KeyboardInterrupt:
            print(f"\n🛑 用户中断监控...")
        
        finally:
            # 停止监控并生成报告
            self.stop_monitoring()
            self._generate_report()


def main():
    """主函数"""
    print("🤖 压感监控器 - 后台运行版本")
    print("=" * 50)
    print("📋 使用说明:")
    print("1. 启动此程序开始后台监控")
    print("2. 在另一个终端运行 teleop.py 操作机器人")
    print("3. 操作结束后按 Ctrl+C 停止并查看报告")
    print("=" * 50)
    
    # 获取机器人IP
    robot_ip = input("请输入机器人IP地址 (默认: 192.168.10.200): ").strip()
    if not robot_ip:
        robot_ip = "192.168.10.200"
    
    # 创建监控器
    monitor = PressureMonitor(robot_ip)
    
    try:
        # 运行监控
        monitor.run()
    
    except Exception as e:
        print(f"❌ 监控器错误: {e}")
    
    finally:
        print("👋 监控结束！")


if __name__ == '__main__':
    main()
