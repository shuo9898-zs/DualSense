"""
Driving UI Recorder - 驾驶界面录制器
保存pygame渲染的驾驶UI为图像序列，30Hz频率，多线程异步处理
Author: GitHub Copilot & User
"""

import os
import time
import queue
import threading
import pygame
import numpy as np
import datetime


class UIRecorder:
    """驾驶UI录制器 - 多线程异步保存"""
    
    def __init__(self, data_manager=None, target_fps=10, quality_scale=0.2):
        """
        初始化录制器 - 超高性能版
        
        Args:
            data_manager: CARLA数据管理器
            target_fps: 目标保存频率 (Hz) - 10Hz足够记录驾驶界面变化
            quality_scale: 图像缩放比例 (0.2 = 1/5分辨率)
        """
        self.data_manager = data_manager
        self.target_fps = target_fps
        self.quality_scale = quality_scale
        self.frame_interval = 1.0 / target_fps  # 每帧间隔(秒)
        
        # 状态控制
        self.recording = False
        self.frame_count = 0
        self.last_save_time = 0
        
        # 多线程队列 - 6秒缓冲 (避免队列满)
        self.frame_queue = queue.Queue(maxsize=180)
        self.save_thread = None
        
        # 保存路径
        self.save_path = None
        self._setup_save_path()
        
        print(f"🎬 UI录制器: {target_fps}Hz, {quality_scale}x分辨率")
    
    def _setup_save_path(self):
        """设置保存路径 - 与CARLA数据同一文件夹"""
        if self.data_manager and hasattr(self.data_manager, 'save_path'):
            # 使用与CARLA数据相同的文件夹
            base_path = self.data_manager.save_path
        else:
            # 默认路径
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            base_path = f"./data_collected/carla_data_{timestamp}"
        
        self.save_path = os.path.join(base_path, "driving_ui")
        os.makedirs(self.save_path, exist_ok=True)
    
    def start_recording(self):
        """开始录制"""
        if self.recording:
            print("⚠️ 录制已在进行中")
            return False
        
        self.recording = True
        self.frame_count = 0
        self.last_save_time = time.time()
        
        # 启动保存线程
        self.save_thread = threading.Thread(target=self._save_worker, daemon=True)
        self.save_thread.start()
        
        print(f"✅ UI录制已启动 (10Hz, 384×216) ⚡超低影响")
        return True
    
    def stop_recording(self):
        """停止录制"""
        if not self.recording:
            return
        
        self.recording = False
        
        # 等待队列清空
        if self.frame_queue.qsize() > 0:
            print(f"⏳ 等待保存剩余 {self.frame_queue.qsize()} 帧")
        self.frame_queue.join()
        
        print(f"✅ UI录制完成: {self.frame_count} 帧")
    
    def capture_frame(self, display_surface):
        """
        捕获当前帧 (在主渲染循环中调用) - 高性能版本
        
        Args:
            display_surface: pygame显示surface
        """
        if not self.recording:
            return
        
        current_time = time.time()
        
        # 检查是否到了保存时间 (10Hz超低频率，最小化主线程负载)
        if current_time - self.last_save_time < self.frame_interval:
            return
        
        # 如果队列接近满，跳过这一帧避免主线程阻塞
        if self.frame_queue.qsize() > self.frame_queue.maxsize * 0.8:
            return
        
        try:
            # 使用更快的surface复制方法 - 直接复制surface而不是转换为numpy
            # 这避免了昂贵的array3d转换操作
            frame_surface = display_surface.copy()
            
            # 验证surface有效性
            if frame_surface.get_size()[0] == 0 or frame_surface.get_size()[1] == 0:
                print("⚠️ 无效的surface尺寸，跳过")
                return
            
            # 记录时间戳 (与gaze数据格式一致 - 毫秒)
            timestamp_ms = int(current_time * 1000)
            
            # 放入队列 (非阻塞) - 传递surface副本到后台处理
            self.frame_queue.put_nowait({
                'timestamp_ms': timestamp_ms,
                'frame_surface': frame_surface,  # surface副本，避免numpy转换开销
                'frame_id': self.frame_count
            })
            
            self.frame_count += 1
            self.last_save_time = current_time
            
            # 状态信息 - 每200帧打印一次（减少输出）
            if self.frame_count % 200 == 0:
                print(f"📷 UI录制: {self.frame_count} 帧")
            
        except queue.Full:
            pass  # 静默跳过，避免输出洪水
        except Exception as e:
            if self.frame_count % 100 == 0:  # 只在特定时候输出错误
                print(f"⚠️ UI捕获失败: {e}")
    
    def _save_worker(self):
        """后台保存线程 - 异步处理图像压缩和保存"""

        
        while self.recording or not self.frame_queue.empty():
            try:
                # 获取帧数据 (1秒超时)
                frame_data = self.frame_queue.get(timeout=1.0)
                
                # 保存帧
                self._save_frame(frame_data)
                
                # 标记任务完成
                self.frame_queue.task_done()
                
            except queue.Empty:
                # 队列空，继续循环
                continue
            except Exception as e:
                print(f"❌ 保存帧失败: {e}")
                self.frame_queue.task_done()
        

    
    def _save_frame(self, frame_data):
        """
        保存单个帧 - 高性能版本，直接使用pygame缩放
        
        Args:
            frame_data: 包含timestamp_ms, frame_surface, frame_id的字典
        """
        timestamp_ms = frame_data['timestamp_ms']
        frame_surface = frame_data['frame_surface']
        frame_id = frame_data['frame_id']
        
        try:
            # 获取原始尺寸
            original_width, original_height = frame_surface.get_size()
            
            # 计算新尺寸 (保持宽高比)
            new_width = int(original_width * self.quality_scale)
            new_height = int(original_height * self.quality_scale)
            
            # 使用pygame内置缩放 - 比numpy转换更快
            if self.quality_scale != 1.0:
                scaled_surface = pygame.transform.scale(frame_surface, (new_width, new_height))
            else:
                scaled_surface = frame_surface
            
            # 文件名格式: frame_XXXXXX_timestamp.jpg (使用JPEG减小文件体积)
            filename = f"frame_{frame_id:06d}_{timestamp_ms}.jpg"
            filepath = os.path.join(self.save_path, filename)
            
            # 使用JPEG格式保存 - 文件更小，保存更快
            pygame.image.save(scaled_surface, filepath)
            
            # 进度信息 - 每200帧打印一次（减少输出）
            if frame_id % 200 == 0:
                print(f"💾 已保存 {frame_id} 帧")
            
        except Exception as e:
            print(f"❌ 保存帧 {frame_id} 失败: {e}")
            import traceback
            traceback.print_exc()
    
    def get_stats(self):
        """获取录制统计信息"""
        return {
            'recording': self.recording,
            'frame_count': self.frame_count,
            'queue_size': self.frame_queue.qsize(),
            'save_path': self.save_path,
            'fps': self.target_fps,
            'quality_scale': self.quality_scale
        }


class UIRecorderManager:
    """UI录制管理器 - 与main.py集成"""
    
    def __init__(self, data_manager=None):
        self.recorder = UIRecorder(
            data_manager=data_manager,
            target_fps=10,          # 10Hz超低频率 - 最小化性能影响
            quality_scale=0.2       # 0.2x分辨率 (384x216)
        )
        self.enabled = True
    
    def toggle_recording(self):
        """切换录制状态"""
        if self.recorder.recording:
            self.recorder.stop_recording()
            return False
        else:
            return self.recorder.start_recording()
    
    def capture_if_recording(self, display_surface):
        """自动捕获帧 - 录制器已自动启动"""
        if self.enabled and self.recorder.recording:
            self.recorder.capture_frame(display_surface)
    
    def get_stats(self):
        """获取统计信息"""
        return self.recorder.get_stats()
    
    def cleanup(self):
        """清理资源"""
        if self.recorder.recording:
            self.recorder.stop_recording()