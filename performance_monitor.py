"""
CARLA性能监控工具 - 实时监控UI录制对FPS的影响
用于验证优化效果和诊断性能问题
"""

import time
import threading
from collections import deque
from typing import Dict, Optional


class PerformanceMonitor:
    """性能监控器 - 监控CARLA主循环性能"""
    
    def __init__(self, window_size=100):
        """
        初始化性能监控器
        
        Args:
            window_size: 滑动窗口大小 (帧数)
        """
        self.window_size = window_size
        
        # 性能数据缓冲区
        self.frame_times = deque(maxlen=window_size)
        self.render_times = deque(maxlen=window_size)
        self.ui_capture_times = deque(maxlen=window_size)
        
        # 时间戳记录
        self.frame_start_time = 0
        self.render_start_time = 0
        self.ui_capture_start_time = 0
        
        # 统计数据
        self.total_frames = 0
        self.start_time = time.time()
        
        # UI录制状态
        self.ui_recording = False
        self.ui_queue_size = 0
        

    
    def start_frame(self):
        """标记帧开始"""
        self.frame_start_time = time.time()
    
    def end_frame(self):
        """标记帧结束"""
        if self.frame_start_time > 0:
            frame_time = time.time() - self.frame_start_time
            self.frame_times.append(frame_time * 1000)  # 转换为毫秒
            self.total_frames += 1
    
    def start_render(self):
        """标记渲染开始"""
        self.render_start_time = time.time()
    
    def end_render(self):
        """标记渲染结束"""
        if self.render_start_time > 0:
            render_time = time.time() - self.render_start_time
            self.render_times.append(render_time * 1000)  # 转换为毫秒
    
    def start_ui_capture(self):
        """标记UI捕获开始"""
        self.ui_capture_start_time = time.time()
    
    def end_ui_capture(self):
        """标记UI捕获结束"""
        if self.ui_capture_start_time > 0:
            ui_time = time.time() - self.ui_capture_start_time
            self.ui_capture_times.append(ui_time * 1000)  # 转换为毫秒
    
    def update_ui_status(self, recording: bool, queue_size: int):
        """更新UI录制状态"""
        self.ui_recording = recording
        self.ui_queue_size = queue_size
    
    def get_stats(self) -> Dict:
        """获取性能统计信息"""
        current_time = time.time()
        runtime = current_time - self.start_time
        
        stats = {
            'runtime_seconds': runtime,
            'total_frames': self.total_frames,
            'avg_fps': self.total_frames / runtime if runtime > 0 else 0,
            'ui_recording': self.ui_recording,
            'ui_queue_size': self.ui_queue_size
        }
        
        # 计算近期FPS (基于frame_times)
        if len(self.frame_times) > 10:
            recent_avg_frame_time = sum(list(self.frame_times)[-30:]) / min(30, len(self.frame_times))
            stats['recent_fps'] = 1000 / recent_avg_frame_time if recent_avg_frame_time > 0 else 0
            stats['avg_frame_time_ms'] = recent_avg_frame_time
        else:
            stats['recent_fps'] = 0
            stats['avg_frame_time_ms'] = 0
        
        # 渲染时间统计
        if len(self.render_times) > 0:
            stats['avg_render_time_ms'] = sum(self.render_times) / len(self.render_times)
            stats['max_render_time_ms'] = max(self.render_times)
        else:
            stats['avg_render_time_ms'] = 0
            stats['max_render_time_ms'] = 0
        
        # UI捕获时间统计
        if len(self.ui_capture_times) > 0:
            stats['avg_ui_capture_ms'] = sum(self.ui_capture_times) / len(self.ui_capture_times)
            stats['max_ui_capture_ms'] = max(self.ui_capture_times)
            stats['ui_capture_fps'] = len(self.ui_capture_times) / runtime if runtime > 0 else 0
        else:
            stats['avg_ui_capture_ms'] = 0
            stats['max_ui_capture_ms'] = 0
            stats['ui_capture_fps'] = 0
        
        return stats
    
    def print_stats(self):
        """打印性能统计信息"""
        stats = self.get_stats()
        
        print(f"\n📊 性能监控报告:")
        print(f"   运行时间: {stats['runtime_seconds']:.1f}秒")
        print(f"   总帧数: {stats['total_frames']}")
        print(f"   平均FPS: {stats['avg_fps']:.1f}")
        print(f"   近期FPS: {stats['recent_fps']:.1f}")
        print(f"   平均帧时间: {stats['avg_frame_time_ms']:.1f}ms")
        
        print(f"\n🎨 渲染性能:")
        print(f"   平均渲染时间: {stats['avg_render_time_ms']:.1f}ms")
        print(f"   最大渲染时间: {stats['max_render_time_ms']:.1f}ms")
        
        if stats['ui_recording']:
            print(f"\n🎬 UI录制性能:")
            print(f"   录制状态: 开启")
            print(f"   队列大小: {stats['ui_queue_size']}")
            print(f"   捕获频率: {stats['ui_capture_fps']:.1f}Hz")
            print(f"   平均捕获时间: {stats['avg_ui_capture_ms']:.1f}ms")
            print(f"   最大捕获时间: {stats['max_ui_capture_ms']:.1f}ms")
        else:
            print(f"\n🎬 UI录制: 关闭")
    
    def get_performance_impact(self) -> Dict:
        """分析UI录制的性能影响"""
        if not self.ui_recording or len(self.ui_capture_times) == 0:
            return {'impact': 0, 'description': 'UI录制未开启'}
        
        stats = self.get_stats()
        
        # 计算UI捕获时间占总帧时间的百分比
        if stats['avg_frame_time_ms'] > 0:
            ui_impact_percent = (stats['avg_ui_capture_ms'] / stats['avg_frame_time_ms']) * 100
        else:
            ui_impact_percent = 0
        
        # 估算FPS影响
        baseline_fps = 35  # 目标FPS
        current_fps = stats['recent_fps']
        fps_loss = baseline_fps - current_fps
        fps_loss_percent = (fps_loss / baseline_fps) * 100 if baseline_fps > 0 else 0
        
        return {
            'ui_time_percent': ui_impact_percent,
            'fps_loss': fps_loss,
            'fps_loss_percent': fps_loss_percent,
            'current_fps': current_fps,
            'target_fps': baseline_fps,
            'description': f'UI录制占用{ui_impact_percent:.1f}%帧时间，FPS损失{fps_loss:.1f}'
        }


# 全局性能监控实例
_global_monitor: Optional[PerformanceMonitor] = None


def get_performance_monitor() -> PerformanceMonitor:
    """获取全局性能监控器实例"""
    global _global_monitor
    if _global_monitor is None:
        _global_monitor = PerformanceMonitor()
    return _global_monitor


def start_performance_reporting(interval_seconds=10):
    """启动性能报告线程"""
    monitor = get_performance_monitor()
    
    def report_loop():
        while True:
            time.sleep(interval_seconds)
            monitor.print_stats()
            
            # 性能影响分析
            impact = monitor.get_performance_impact()
            if impact['fps_loss_percent'] > 10:  # FPS损失超过10%
                print(f"⚠️ 性能警告: {impact['description']}")
    
    thread = threading.Thread(target=report_loop, daemon=True)
    thread.start()
    print(f"📈 性能报告线程已启动 (每{interval_seconds}秒报告一次)")


if __name__ == "__main__":
    # 测试性能监控器
    monitor = PerformanceMonitor()
    
    # 模拟一些性能数据
    for i in range(100):
        monitor.start_frame()
        time.sleep(0.02)  # 模拟20ms帧时间
        
        monitor.start_render()
        time.sleep(0.015)  # 模拟15ms渲染时间
        monitor.end_render()
        
        if i % 3 == 0:  # 模拟15Hz UI捕获
            monitor.start_ui_capture()
            time.sleep(0.002)  # 模拟2ms捕获时间
            monitor.end_ui_capture()
            monitor.update_ui_status(True, 5)
        
        monitor.end_frame()
    
    # 显示统计
    monitor.print_stats()
    impact = monitor.get_performance_impact()
    print(f"\n性能影响: {impact}")