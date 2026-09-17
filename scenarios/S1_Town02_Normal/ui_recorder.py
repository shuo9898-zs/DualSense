"""
Driving UI Recorder
Save pygame driving UI frames at 30 Hz using asynchronous worker threads.
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
    """Driving UI recorder with multithreaded asynchronous saving"""
    
    def __init__(self, data_manager=None, target_fps=10, quality_scale=0.2):
        """
        Initialize the high-performance recorder.
        
        Args:
            data_manager: CARLA data manager
            target_fps: Saving rate in Hz; 10 Hz captures driving UI changes
            quality_scale: Image scale factor; 0.2 gives one-fifth resolution
        """
        self.data_manager = data_manager
        self.target_fps = target_fps
        self.quality_scale = quality_scale
        self.frame_interval = 1.0 / target_fps  # Frame interval in seconds
        
        # State control
        self.recording = False
        self.frame_count = 0
        self.last_save_time = 0
        
        # Worker queue with a six-second buffer to avoid filling up
        self.frame_queue = queue.Queue(maxsize=180)
        self.save_thread = None
        
        # Output path
        self.save_path = None
        self._setup_save_path()
        
        print(f"🎬 UI录制器: {target_fps}Hz, {quality_scale}x分辨率")
    
    def _setup_save_path(self):
        """Set the output path within the CARLA data folder."""
        if self.data_manager and hasattr(self.data_manager, 'save_path'):
            # Use the CARLA data folder.
            base_path = self.data_manager.save_path
        else:
            # Default path
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            base_path = f"./data_collected/carla_data_{timestamp}"
        
        self.save_path = os.path.join(base_path, "driving_ui")
        os.makedirs(self.save_path, exist_ok=True)
    
    def start_recording(self):
        """Start recording."""
        if self.recording:
            print("⚠️ 录制已在进行中")
            return False
        
        self.recording = True
        self.frame_count = 0
        self.last_save_time = time.time()
        
        # Start the saving thread.
        self.save_thread = threading.Thread(target=self._save_worker, daemon=True)
        self.save_thread.start()
        
        print(f"✅ UI录制已启动 (10Hz, 384×216) ⚡超低影响")
        return True
    
    def stop_recording(self):
        """Stop recording."""
        if not self.recording:
            return
        
        self.recording = False
        
        # Wait for the queue to drain.
        if self.frame_queue.qsize() > 0:
            print(f"⏳ 等待保存剩余 {self.frame_queue.qsize()} 帧")
        self.frame_queue.join()
        
        print(f"✅ UI录制完成: {self.frame_count} 帧")
    
    def capture_frame(self, display_surface):
        """
        Capture the current frame from the main rendering loop efficiently.
        
        Args:
            display_surface: pygame display surface
        """
        if not self.recording:
            return
        
        current_time = time.time()
        
        # Check the 10 Hz capture schedule to minimize main-thread load.
        if current_time - self.last_save_time < self.frame_interval:
            return
        
        # Skip the frame when the queue is nearly full to avoid blocking.
        if self.frame_queue.qsize() > self.frame_queue.maxsize * 0.8:
            return
        
        try:
            # Copy the surface directly rather than converting to numpy.
            # Avoid the expensive array3d conversion.
            frame_surface = display_surface.copy()
            
            # Validate the surface.
            if frame_surface.get_size()[0] == 0 or frame_surface.get_size()[1] == 0:
                print("⚠️ 无效的surface尺寸，跳过")
                return
            
            # Record timestamps in milliseconds, matching gaze data.
            timestamp_ms = int(current_time * 1000)
            
            # Enqueue the surface copy without blocking for background processing.
            self.frame_queue.put_nowait({
                'timestamp_ms': timestamp_ms,
                'frame_surface': frame_surface,  # Surface copy avoids numpy conversion overhead.
                'frame_id': self.frame_count
            })
            
            self.frame_count += 1
            self.last_save_time = current_time
            
            # Report status every 200 frames to reduce output.
            if self.frame_count % 200 == 0:
                print(f"📷 UI录制: {self.frame_count} 帧")
            
        except queue.Full:
            pass  # Skip silently to avoid flooding the console.
        except Exception as e:
            if self.frame_count % 100 == 0:  # Report errors only at selected intervals.
                print(f"⚠️ UI捕获失败: {e}")
    
    def _save_worker(self):
        """Background worker for asynchronous image compression and saving."""

        
        while self.recording or not self.frame_queue.empty():
            try:
                # Get frame data with a one-second timeout.
                frame_data = self.frame_queue.get(timeout=1.0)
                
                # Save the frame.
                self._save_frame(frame_data)
                
                # Mark the task complete.
                self.frame_queue.task_done()
                
            except queue.Empty:
                # Queue empty; continue looping.
                continue
            except Exception as e:
                print(f"❌ 保存帧失败: {e}")
                self.frame_queue.task_done()
        

    
    def _save_frame(self, frame_data):
        """
        Save one frame efficiently using pygame scaling.
        
        Args:
            frame_data: Dictionary containing timestamp_ms, frame_surface, and frame_id
        """
        timestamp_ms = frame_data['timestamp_ms']
        frame_surface = frame_data['frame_surface']
        frame_id = frame_data['frame_id']
        
        try:
            # Get original dimensions.
            original_width, original_height = frame_surface.get_size()
            
            # Calculate new dimensions while preserving aspect ratio.
            new_width = int(original_width * self.quality_scale)
            new_height = int(original_height * self.quality_scale)
            
            # Use pygame scaling to avoid numpy conversion overhead.
            if self.quality_scale != 1.0:
                scaled_surface = pygame.transform.scale(frame_surface, (new_width, new_height))
            else:
                scaled_surface = frame_surface
            
            # Filename: frame_XXXXXX_timestamp.jpg; JPEG reduces file size.
            filename = f"frame_{frame_id:06d}_{timestamp_ms}.jpg"
            filepath = os.path.join(self.save_path, filename)
            
            # Save as JPEG for smaller files and faster writes.
            pygame.image.save(scaled_surface, filepath)
            
            # Report progress every 200 frames to reduce output.
            if frame_id % 200 == 0:
                print(f"💾 已保存 {frame_id} 帧")
            
        except Exception as e:
            print(f"❌ 保存帧 {frame_id} 失败: {e}")
            import traceback
            traceback.print_exc()
    
    def get_stats(self):
        """Get recording statistics."""
        return {
            'recording': self.recording,
            'frame_count': self.frame_count,
            'queue_size': self.frame_queue.qsize(),
            'save_path': self.save_path,
            'fps': self.target_fps,
            'quality_scale': self.quality_scale
        }


class UIRecorderManager:
    """UI recording manager integrated with main.py"""
    
    def __init__(self, data_manager=None):
        self.recorder = UIRecorder(
            data_manager=data_manager,
            target_fps=10,          # 10 Hz recording to minimize performance impact
            quality_scale=0.2       # 0.2x resolution (384x216)
        )
        self.enabled = True
    
    def toggle_recording(self):
        """Toggle recording status."""
        if self.recorder.recording:
            self.recorder.stop_recording()
            return False
        else:
            return self.recorder.start_recording()
    
    def capture_if_recording(self, display_surface):
        """Automatically capture frames; the recorder starts automatically."""
        if self.enabled and self.recorder.recording:
            self.recorder.capture_frame(display_surface)
    
    def get_stats(self):
        """Get statistics."""
        return self.recorder.get_stats()
    
    def cleanup(self):
        """Clean up resources."""
        if self.recorder.recording:
            self.recorder.stop_recording()
