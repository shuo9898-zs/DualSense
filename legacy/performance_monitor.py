"""
CARLA performance monitoring: real-time FPS impact of UI recording
Validate optimizations and diagnose performance issues.
"""

import time
import threading
from collections import deque
from typing import Dict, Optional


class PerformanceMonitor:
    """Monitor CARLA main-loop performance."""
    
    def __init__(self, window_size=100):
        """
        Initialize the performance monitor.
        
        Args:
            window_size: Sliding-window length in frames
        """
        self.window_size = window_size
        
        # Performance-data buffers
        self.frame_times = deque(maxlen=window_size)
        self.render_times = deque(maxlen=window_size)
        self.ui_capture_times = deque(maxlen=window_size)
        
        # Timestamp records
        self.frame_start_time = 0
        self.render_start_time = 0
        self.ui_capture_start_time = 0
        
        # Statistics
        self.total_frames = 0
        self.start_time = time.time()
        
        # UI recording status
        self.ui_recording = False
        self.ui_queue_size = 0
        

    
    def start_frame(self):
        """Mark the start of a frame."""
        self.frame_start_time = time.time()
    
    def end_frame(self):
        """Mark the end of a frame."""
        if self.frame_start_time > 0:
            frame_time = time.time() - self.frame_start_time
            self.frame_times.append(frame_time * 1000)  # Convert to milliseconds.
            self.total_frames += 1
    
    def start_render(self):
        """Mark the start of rendering."""
        self.render_start_time = time.time()
    
    def end_render(self):
        """Mark the end of rendering."""
        if self.render_start_time > 0:
            render_time = time.time() - self.render_start_time
            self.render_times.append(render_time * 1000)  # Convert to milliseconds.
    
    def start_ui_capture(self):
        """Mark the start of UI capture."""
        self.ui_capture_start_time = time.time()
    
    def end_ui_capture(self):
        """Mark the end of UI capture."""
        if self.ui_capture_start_time > 0:
            ui_time = time.time() - self.ui_capture_start_time
            self.ui_capture_times.append(ui_time * 1000)  # Convert to milliseconds.
    
    def update_ui_status(self, recording: bool, queue_size: int):
        """Update UI recording status."""
        self.ui_recording = recording
        self.ui_queue_size = queue_size
    
    def get_stats(self) -> Dict:
        """Get performance statistics."""
        current_time = time.time()
        runtime = current_time - self.start_time
        
        stats = {
            'runtime_seconds': runtime,
            'total_frames': self.total_frames,
            'avg_fps': self.total_frames / runtime if runtime > 0 else 0,
            'ui_recording': self.ui_recording,
            'ui_queue_size': self.ui_queue_size
        }
        
        # Calculate recent FPS from frame_times.
        if len(self.frame_times) > 10:
            recent_avg_frame_time = sum(list(self.frame_times)[-30:]) / min(30, len(self.frame_times))
            stats['recent_fps'] = 1000 / recent_avg_frame_time if recent_avg_frame_time > 0 else 0
            stats['avg_frame_time_ms'] = recent_avg_frame_time
        else:
            stats['recent_fps'] = 0
            stats['avg_frame_time_ms'] = 0
        
        # Rendering-time statistics
        if len(self.render_times) > 0:
            stats['avg_render_time_ms'] = sum(self.render_times) / len(self.render_times)
            stats['max_render_time_ms'] = max(self.render_times)
        else:
            stats['avg_render_time_ms'] = 0
            stats['max_render_time_ms'] = 0
        
        # UI capture-time statistics
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
        """Print performance statistics."""
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
        """Analyze the performance impact of UI recording."""
        if not self.ui_recording or len(self.ui_capture_times) == 0:
            return {'impact': 0, 'description': 'UI录制未开启'}
        
        stats = self.get_stats()
        
        # Calculate UI capture time as a percentage of total frame time.
        if stats['avg_frame_time_ms'] > 0:
            ui_impact_percent = (stats['avg_ui_capture_ms'] / stats['avg_frame_time_ms']) * 100
        else:
            ui_impact_percent = 0
        
        # Estimate the FPS impact.
        baseline_fps = 35  # Target FPS
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


# Global performance-monitor instance
_global_monitor: Optional[PerformanceMonitor] = None


def get_performance_monitor() -> PerformanceMonitor:
    """Get the global performance monitor."""
    global _global_monitor
    if _global_monitor is None:
        _global_monitor = PerformanceMonitor()
    return _global_monitor


def start_performance_reporting(interval_seconds=10):
    """Start the performance-reporting thread."""
    monitor = get_performance_monitor()
    
    def report_loop():
        while True:
            time.sleep(interval_seconds)
            monitor.print_stats()
            
            # Analyze performance impact.
            impact = monitor.get_performance_impact()
            if impact['fps_loss_percent'] > 10:  # FPS loss exceeds 10%.
                print(f"⚠️ 性能警告: {impact['description']}")
    
    thread = threading.Thread(target=report_loop, daemon=True)
    thread.start()
    print(f"📈 性能报告线程已启动 (每{interval_seconds}秒报告一次)")


if __name__ == "__main__":
    # Test the performance monitor.
    monitor = PerformanceMonitor()
    
    # Simulate performance data.
    for i in range(100):
        monitor.start_frame()
        time.sleep(0.02)  # Simulate a 20 ms frame time.
        
        monitor.start_render()
        time.sleep(0.015)  # Simulate a 15 ms rendering time.
        monitor.end_render()
        
        if i % 3 == 0:  # Simulate UI capture at 15 Hz.
            monitor.start_ui_capture()
            time.sleep(0.002)  # Simulate a 2 ms capture time.
            monitor.end_ui_capture()
            monitor.update_ui_status(True, 5)
        
        monitor.end_frame()
    
    # Display statistics.
    monitor.print_stats()
    impact = monitor.get_performance_impact()
    print(f"\n性能影响: {impact}")