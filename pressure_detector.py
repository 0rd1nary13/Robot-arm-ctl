#!/usr/bin/env python3
"""
基于关节电压的压感检测器
使用 get_phy_data() 监控各个关节的电压变化来检测碰撞和阻力
当电机遇到阻力时，电压会下降，这是一个更直接和准确的检测方法

作者: AI Assistant (基于同事建议)
"""

from enum import Enum
from typing import Dict, Optional, Any, List
from collections import namedtuple, deque
import time
import numpy as np


class PressureDetectionMode(Enum):
    """压感检测模式"""
    VOLTAGE_DROP = "voltage_drop"              # 关节电压下降检测
    CURRENT_SPIKE = "current_spike"            # 电流峰值检测  
    POWER_ANOMALY = "power_anomaly"            # 功率异常检测
    MULTI_JOINT_ANALYSIS = "multi_joint"       # 多关节综合分析


PressureThresholds = namedtuple('PressureThresholds', [
    'voltage_drop_threshold',      # 电压下降阈值 (V)
    'current_spike_threshold',     # 电流峰值阈值 (A)
    'power_change_threshold',      # 功率变化阈值 (W)
    'detection_frequency',         # 检测频率 (Hz)
    'confidence_threshold',        # 置信度阈值
    'joint_count'                  # 关节数量
])


PressureEvent = namedtuple('PressureEvent', [
    'timestamp',                   # 时间戳
    'detection_method',            # 检测方法
    'confidence',                  # 置信度
    'joint_voltages',              # 关节电压
    'joint_currents',              # 关节电流
    'affected_joints',             # 受影响的关节
    'voltage_drops',               # 电压下降值
    'details'                      # 详细信息
])


class LebaiPressureDetector:
    """基于关节电压的Lebai机器人压感检测器"""
    
    def __init__(self, lebai, thresholds: PressureThresholds):
        """
        初始化压感检测器
        
        Args:
            lebai: Lebai机器人实例
            thresholds: 检测阈值配置
        """
        self.lebai = lebai
        self.thresholds = thresholds
        self.is_monitoring = False
        
        # 电压历史数据 (用于基线计算)
        self.voltage_history = deque(maxlen=50)  # 保存最近50次读数
        self.current_history = deque(maxlen=50)
        self.baseline_voltages = None
        self.baseline_currents = None
        
        # 检测状态
        self.last_pressure_event = None
        self.pressure_detected = False
        
        # 统计信息
        self.detection_stats = {
            'total_detections': 0,
            'voltage_detections': 0,
            'current_detections': 0,
            'false_positives': 0
        }
        
        print("🔋 基于关节电压的压感检测器已初始化")
        print(f"   📊 电压下降阈值: {thresholds.voltage_drop_threshold:.2f}V")
        print(f"   ⚡ 电流峰值阈值: {thresholds.current_spike_threshold:.2f}A")
        print(f"   🔍 检测频率: {thresholds.detection_frequency}Hz")
    
    def start_monitoring(self, target_position=None):
        """
        开始监控压感
        
        Args:
            target_position: 目标位置 (此方法中不使用，保持接口兼容)
        """
        self.is_monitoring = True
        self.pressure_detected = False
        self.last_pressure_event = None
        
        # 建立基线电压和电流
        self._establish_baseline()
        
        print("🔋 开始基于电压的压感监控")
        print("   📈 正在建立基线电压...")
    
    def stop_monitoring(self):
        """停止监控"""
        self.is_monitoring = False
        print("🔋 电压压感监控已停止")
        
        # 打印统计信息
        if self.detection_stats['total_detections'] > 0:
            print(f"📊 检测统计:")
            print(f"   🎯 总检测次数: {self.detection_stats['total_detections']}")
            print(f"   🔋 电压检测: {self.detection_stats['voltage_detections']}")
            print(f"   ⚡ 电流检测: {self.detection_stats['current_detections']}")
    
    def _establish_baseline(self, samples: int = 10):
        """
        建立基线电压和电流值
        
        Args:
            samples: 采样次数
        """
        print("📊 建立电压基线...")
        voltage_samples = []
        current_samples = []
        
        for i in range(samples):
            try:
                phy_data = self.lebai.get_phy_data()
                if phy_data and 'joint_voltage' in phy_data and 'joint_current' in phy_data:
                    voltages = phy_data['joint_voltage']
                    currents = phy_data['joint_current']
                    
                    if voltages and currents:
                        voltage_samples.append(voltages)
                        current_samples.append(currents)
                        print(f"   📈 采样 {i+1}/{samples}: V={[f'{v:.1f}' for v in voltages[:3]]}...")
                
                time.sleep(0.1)  # 100ms间隔
                
            except Exception as e:
                print(f"⚠️ 基线采样失败 {i+1}: {e}")
                continue
        
        if voltage_samples and current_samples:
            # 计算平均基线值
            self.baseline_voltages = np.mean(voltage_samples, axis=0)
            self.baseline_currents = np.mean(current_samples, axis=0)
            
            print(f"✅ 基线建立完成:")
            print(f"   🔋 基线电压: {[f'{v:.1f}V' for v in self.baseline_voltages[:6]]}")
            print(f"   ⚡ 基线电流: {[f'{c:.2f}A' for c in self.baseline_currents[:6]]}")
        else:
            print("❌ 无法建立基线，使用默认值")
            # 使用默认基线值 (6个关节)
            self.baseline_voltages = np.array([24.0] * 6)  # 24V默认
            self.baseline_currents = np.array([0.5] * 6)   # 0.5A默认
    
    def is_pressure_detected(self) -> bool:
        """
        检查是否检测到压感
        
        Returns:
            bool: 是否检测到压感
        """
        if not self.is_monitoring:
            return False
        
        try:
            # 获取物理数据
            phy_data = self.lebai.get_phy_data()
            if not phy_data:
                return False
            
            # 分析电压和电流数据
            detection_result = self._analyze_voltage_current(phy_data)
            
            if detection_result:
                self.pressure_detected = True
                self.last_pressure_event = detection_result
                self.detection_stats['total_detections'] += 1
                return True
            
            return False
            
        except Exception as e:
            print(f"⚠️ 压感检测错误: {e}")
            return False
    
    def _analyze_voltage_current(self, phy_data: Dict) -> Optional[PressureEvent]:
        """
        分析电压和电流数据检测压感
        
        Args:
            phy_data: 物理数据字典
            
        Returns:
            PressureEvent: 检测到的压感事件，如果没有则返回None
        """
        if 'joint_voltage' not in phy_data or 'joint_current' not in phy_data:
            return None
        
        voltages = np.array(phy_data['joint_voltage'])
        currents = np.array(phy_data['joint_current'])
        
        if len(voltages) == 0 or len(currents) == 0:
            return None
        
        # 确保基线存在
        if self.baseline_voltages is None or self.baseline_currents is None:
            return None
        
        # 调整数组长度匹配
        min_joints = min(len(voltages), len(self.baseline_voltages))
        voltages = voltages[:min_joints]
        currents = currents[:min_joints]
        baseline_v = self.baseline_voltages[:min_joints]
        baseline_c = self.baseline_currents[:min_joints]
        
        # 计算电压下降
        voltage_drops = baseline_v - voltages
        current_increases = currents - baseline_c
        
        # 检测显著的电压下降
        significant_voltage_drops = voltage_drops > self.thresholds.voltage_drop_threshold
        significant_current_spikes = current_increases > self.thresholds.current_spike_threshold
        
        # 找出受影响的关节
        affected_joints = []
        detection_methods = []
        total_confidence = 0.0
        
        # 电压下降检测
        if np.any(significant_voltage_drops):
            voltage_affected = np.where(significant_voltage_drops)[0].tolist()
            affected_joints.extend(voltage_affected)
            detection_methods.append("voltage_drop")
            
            # 计算电压下降置信度
            max_drop = np.max(voltage_drops[significant_voltage_drops])
            voltage_confidence = min(0.9, max_drop / (self.thresholds.voltage_drop_threshold * 2))
            total_confidence += voltage_confidence
            
            self.detection_stats['voltage_detections'] += 1
        
        # 电流峰值检测
        if np.any(significant_current_spikes):
            current_affected = np.where(significant_current_spikes)[0].tolist()
            affected_joints.extend(current_affected)
            detection_methods.append("current_spike")
            
            # 计算电流峰值置信度
            max_spike = np.max(current_increases[significant_current_spikes])
            current_confidence = min(0.9, max_spike / (self.thresholds.current_spike_threshold * 2))
            total_confidence += current_confidence
            
            self.detection_stats['current_detections'] += 1
        
        # 如果有检测到异常
        if affected_joints:
            # 去除重复关节
            affected_joints = list(set(affected_joints))
            
            # 计算综合置信度
            final_confidence = min(0.95, total_confidence / len(detection_methods) if detection_methods else 0)
            
            # 检查是否超过置信度阈值
            if final_confidence >= self.thresholds.confidence_threshold:
                # 创建压感事件
                event = PressureEvent(
                    timestamp=time.time(),
                    detection_method=PressureDetectionMode.VOLTAGE_DROP if "voltage_drop" in detection_methods else PressureDetectionMode.CURRENT_SPIKE,
                    confidence=final_confidence,
                    joint_voltages=voltages.tolist(),
                    joint_currents=currents.tolist(),
                    affected_joints=affected_joints,
                    voltage_drops=voltage_drops.tolist(),
                    details={
                        'detection_methods': detection_methods,
                        'baseline_voltages': baseline_v.tolist(),
                        'baseline_currents': baseline_c.tolist(),
                        'max_voltage_drop': np.max(voltage_drops).item() if len(voltage_drops) > 0 else 0,
                        'max_current_spike': np.max(current_increases).item() if len(current_increases) > 0 else 0
                    }
                )
                
                return event
        
        return None
    
    def get_last_pressure_event(self) -> Optional[PressureEvent]:
        """
        获取最后一次检测到的压感事件
        
        Returns:
            PressureEvent: 最后的压感事件
        """
        return self.last_pressure_event
    
    def reset_pressure_state(self):
        """重置压感检测状态"""
        self.pressure_detected = False
        self.last_pressure_event = None
        
        # 可选：重新建立基线 (如果机器人状态发生了显著变化)
        # self._establish_baseline(samples=5)
    
    def get_current_joint_status(self) -> Optional[Dict]:
        """
        获取当前关节状态信息 (用于调试)
        
        Returns:
            Dict: 关节状态信息
        """
        try:
            phy_data = self.lebai.get_phy_data()
            if not phy_data:
                return None
            
            voltages = phy_data.get('joint_voltage', [])
            currents = phy_data.get('joint_current', [])
            
            status = {
                'timestamp': time.time(),
                'joint_voltages': voltages,
                'joint_currents': currents,
                'baseline_voltages': self.baseline_voltages.tolist() if self.baseline_voltages is not None else None,
                'baseline_currents': self.baseline_currents.tolist() if self.baseline_currents is not None else None,
            }
            
            # 计算当前偏差
            if self.baseline_voltages is not None and voltages:
                min_joints = min(len(voltages), len(self.baseline_voltages))
                voltage_diffs = (self.baseline_voltages[:min_joints] - np.array(voltages[:min_joints])).tolist()
                status['voltage_differences'] = voltage_diffs
            
            if self.baseline_currents is not None and currents:
                min_joints = min(len(currents), len(self.baseline_currents))
                current_diffs = (np.array(currents[:min_joints]) - self.baseline_currents[:min_joints]).tolist()
                status['current_differences'] = current_diffs
            
            return status
            
        except Exception as e:
            print(f"⚠️ 获取关节状态失败: {e}")
            return None


# 预设的检测配置
def create_voltage_thresholds(sensitivity: str = "normal") -> PressureThresholds:
    """
    创建基于电压的检测阈值配置
    
    Args:
        sensitivity: 敏感度 ("high", "normal", "low")
        
    Returns:
        PressureThresholds: 阈值配置
    """
    if sensitivity == "high":
        return PressureThresholds(
            voltage_drop_threshold=1.0,    # 1.0V 下降即触发 - 高敏感度
            current_spike_threshold=0.3,   # 0.3A 增加即触发
            power_change_threshold=5.0,    # 5W 功率变化
            detection_frequency=20.0,      # 20Hz
            confidence_threshold=0.6,      # 60% 置信度
            joint_count=6                  # 6个关节
        )
    elif sensitivity == "low":
        return PressureThresholds(
            voltage_drop_threshold=3.0,    # 3.0V 下降才触发 - 低敏感度
            current_spike_threshold=1.0,   # 1.0A 增加才触发
            power_change_threshold=15.0,   # 15W 功率变化
            detection_frequency=10.0,      # 10Hz
            confidence_threshold=0.85,     # 85% 置信度
            joint_count=6
        )
    else:  # normal
        return PressureThresholds(
            voltage_drop_threshold=2.0,    # 2.0V 下降触发 - 正常敏感度
            current_spike_threshold=0.6,   # 0.6A 增加触发
            power_change_threshold=10.0,   # 10W 功率变化
            detection_frequency=15.0,      # 15Hz
            confidence_threshold=0.75,     # 75% 置信度
            joint_count=6
        )


if __name__ == "__main__":
    """测试模块"""
    print("🔋 基于关节电压的压感检测器")
    print("此模块需要与lebai_sdk配合使用")
    print("请参考 pressure_monitor.py 或 teach_mode_pressure_test.py 的使用示例")
