"""
Merge gaze data with driving UI screenshots.
Match the nearest screenshot by timestamp and overlay the gaze point.

Author: Anonymous contributors
Date: October 29, 2025
"""

import os
import csv
import cv2
import numpy as np
from pathlib import Path
from typing import List, Tuple, Optional


class GazeUImerger:
    """Merge gaze data and UI screenshots."""
    
    def __init__(self, gaze_csv_path: str, ui_folder_path: str, output_folder: str):
        self.gaze_csv_path = gaze_csv_path
        self.ui_folder_path = ui_folder_path
        self.output_folder = output_folder
        
        self.gaze_data = []
        self.ui_images = []
        
    def load_gaze_data(self) -> bool:
        """Read gaze CSV data."""
        print("\n📊 读取眼动数据...")
        
        try:
            with open(self.gaze_csv_path, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                self.gaze_data = list(reader)
            
            print(f"   ✅ 读取 {len(self.gaze_data)} 条眼动数据")
            
            if not self.gaze_data:
                print("   ❌ 没有数据")
                return False
            
            # Print CSV column names for debugging.
            first_row = self.gaze_data[0]
            print(f"   📋 CSV列名: {list(first_row.keys())}")
            
            # Detect the timestamp column and handle its format automatically.
            timestamp_col = None
            if 'pc_timestamp_str' in first_row:
                timestamp_col = 'pc_timestamp_str'
                # Convert string timestamps to numeric timestamps.
                import datetime
                for row in self.gaze_data:
                    dt_str = row['pc_timestamp_str']
                    dt = datetime.datetime.strptime(dt_str, '%Y-%m-%d %H:%M:%S.%f')
                    row['pc_timestamp'] = dt.timestamp()
            elif 'pc_timestamp' in first_row:
                timestamp_col = 'pc_timestamp'
            elif 'beam_timestamp' in first_row:
                timestamp_col = 'beam_timestamp'
            else:
                print("   ❌ 未找到时间戳列")
                return False
            
            print(f"   🕐 使用时间戳列: '{timestamp_col}'")
            
            if self.gaze_data:
                first = self.gaze_data[0]
                last = self.gaze_data[-1]
                first_ts = float(first['pc_timestamp'])
                last_ts = float(last['pc_timestamp'])
                print(f"   时间范围: {first_ts:.2f} ~ {last_ts:.2f} (秒)")
                print(f"   持续时长: {last_ts - first_ts:.1f} 秒")
            
            return True
            
        except Exception as e:
            print(f"   ❌ 读取失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def load_ui_images(self) -> bool:
        """Read the list of UI screenshots."""
        print("\n🖼️  读取驾驶UI截图...")
        
        try:
            ui_files = []
            ui_files.extend(Path(self.ui_folder_path).glob("*.jpg"))
            ui_files.extend(Path(self.ui_folder_path).glob("*.jpeg"))
            ui_files = sorted(ui_files)
            
            print(f"   📁 找到 {len(ui_files)} 个图像文件")
            
            for img_path in ui_files:
                filename = img_path.stem
                
                # Parse a timestamp from frame_000000_1761771885410.
                # Take the number after the final underscore.
                parts = filename.split("_")
                timestamp_str = parts[-1]
                
                try:
                    timestamp_ms = float(timestamp_str)
                    
                    # Detect units: values above 1e10 indicate milliseconds.
                    if timestamp_ms > 1e10:
                        timestamp = timestamp_ms / 1000.0  # Convert to seconds.
                    else:
                        timestamp = timestamp_ms
                    
                    self.ui_images.append({
                        'path': str(img_path),
                        'timestamp': timestamp,
                        'original_ts': timestamp_str
                    })
                except ValueError:
                    print(f"   ⚠️ 无法解析时间戳: {filename}")
                    continue
            
            self.ui_images.sort(key=lambda x: x['timestamp'])
            
            print(f"   ✅ 成功解析 {len(self.ui_images)} 张截图")
            
            if self.ui_images:
                first = self.ui_images[0]
                last = self.ui_images[-1]
                print(f"   时间范围: {first['timestamp']:.2f} ~ {last['timestamp']:.2f} (秒)")
                print(f"   持续时长: {last['timestamp'] - first['timestamp']:.1f} 秒")
                print(f"   示例文件名: {os.path.basename(first['path'])}")
            
            return len(self.ui_images) > 0
            
        except Exception as e:
            print(f"   ❌ 读取失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def find_nearest_ui(self, gaze_timestamp: float) -> Optional[dict]:
        """
        Find the temporally nearest UI screenshot using binary search.
        """
        if not self.ui_images:
            return None
        
        # Use binary search.
        left, right = 0, len(self.ui_images) - 1
        min_diff = float('inf')
        nearest = None
        
        while left <= right:
            mid = (left + right) // 2
            ui_img = self.ui_images[mid]
            diff = abs(ui_img['timestamp'] - gaze_timestamp)
            
            if diff < min_diff:
                min_diff = diff
                nearest = ui_img
            
            if ui_img['timestamp'] < gaze_timestamp:
                left = mid + 1
            else:
                right = mid - 1
        
        # Check adjacent positions.
        if nearest:
            idx = self.ui_images.index(nearest)
            
            # Check the preceding item.
            if idx > 0:
                prev = self.ui_images[idx - 1]
                prev_diff = abs(prev['timestamp'] - gaze_timestamp)
                if prev_diff < min_diff:
                    min_diff = prev_diff
                    nearest = prev
            
            # Check the following item.
            if idx < len(self.ui_images) - 1:
                next_img = self.ui_images[idx + 1]
                next_diff = abs(next_img['timestamp'] - gaze_timestamp)
                if next_diff < min_diff:
                    min_diff = next_diff
                    nearest = next_img
        
        return nearest
    
    def draw_gaze_point(self, img: np.ndarray, gaze_x: float, gaze_y: float, 
                       confidence: str, timestamp: float) -> np.ndarray:
        """Draw a gaze point on the image."""
        img = img.copy()
        h, w = img.shape[:2]
        
        # Scale coordinates.
        scale_x = w / 1920.0
        scale_y = h / 1080.0
        
        x = int(gaze_x * scale_x)
        y = int(gaze_y * scale_y)
        
        # Check bounds.
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        
        # Choose color and size based on confidence.
        if confidence == "high":
            color = (0, 255, 0)
            radius = 15
            thickness = 2
        elif confidence == "medium":
            color = (0, 255, 255)
            radius = 12
            thickness = 2
        elif confidence == "low":
            color = (0, 165, 255)
            radius = 10
            thickness = 2
        else:
            color = (0, 0, 255)
            radius = 8
            thickness = 1
        
        # Draw a crosshair.
        line_len = radius + 5
        cv2.line(img, (x - line_len, y), (x - radius, y), color, thickness)
        cv2.line(img, (x + radius, y), (x + line_len, y), color, thickness)
        cv2.line(img, (x, y - line_len), (x, y - radius), color, thickness)
        cv2.line(img, (x, y + radius), (x, y + line_len), color, thickness)
        
        # Draw a circle.
        cv2.circle(img, (x, y), radius, color, thickness)
        cv2.circle(img, (x, y), 2, color, -1)
        
        # Text annotation
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_pos = (x + 20, y - 20)
        cv2.putText(img, confidence, text_pos, font, 0.5, color, 1)
        
        return img
    
    def merge(self, max_time_diff: float = 5.0):
        """
        Perform the merge.
        
        Args:
            max_time_diff: Maximum time difference in seconds; skip larger offsets
        """
        print("\n🔄 开始合并...")
        print(f"   最大时间差: {max_time_diff} 秒")
        print(f"   策略: 为每条 gaze 数据找最近的 UI 截图\n")
        
        os.makedirs(self.output_folder, exist_ok=True)
        
        # Create the log file.
        log_path = os.path.join(self.output_folder, "merge_log.txt")
        log_file = open(log_path, 'w', encoding='utf-8')
        log_file.write("gaze_timestamp,gaze_x,gaze_y,confidence,ui_image,ui_timestamp,time_diff\n")
        
        matched = 0
        skipped = 0
        time_diffs = []
        
        total = len(self.gaze_data)
        
        for idx, gaze in enumerate(self.gaze_data):
            try:
                gaze_ts = float(gaze['pc_timestamp'])
                gaze_x = float(gaze['gaze_x'])
                gaze_y = float(gaze['gaze_y'])
                confidence = gaze['confidence']
                
                # Find the nearest UI screenshot.
                nearest_ui = self.find_nearest_ui(gaze_ts)
                
                if nearest_ui is None:
                    skipped += 1
                    continue
                
                time_diff = abs(nearest_ui['timestamp'] - gaze_ts)
                
                # Skip if the time difference is too large.
                if time_diff > max_time_diff:
                    skipped += 1
                    continue
                
                # Read the UI image.
                img = cv2.imread(nearest_ui['path'])
                if img is None:
                    skipped += 1
                    continue
                
                # Draw the gaze point.
                img_with_gaze = self.draw_gaze_point(img, gaze_x, gaze_y, confidence, gaze_ts)
                
                # Save.
                output_filename = f"merged_{gaze_ts:.6f}.jpg"
                output_path = os.path.join(self.output_folder, output_filename)
                cv2.imwrite(output_path, img_with_gaze, [cv2.IMWRITE_JPEG_QUALITY, 95])
                
                # Write a log entry.
                log_file.write(f"{gaze_ts:.6f},{gaze_x:.1f},{gaze_y:.1f},{confidence},"
                              f"{os.path.basename(nearest_ui['path'])},"
                              f"{nearest_ui['timestamp']:.6f},{time_diff:.4f}\n")
                
                time_diffs.append(time_diff)
                matched += 1
                
                # Display progress.
                if (idx + 1) % 100 == 0 or (idx + 1) == total:
                    progress = (idx + 1) / total * 100
                    print(f"   进度: {idx + 1}/{total} ({progress:.1f}%) - "
                          f"匹配: {matched}, 跳过: {skipped}")
                
            except Exception as e:
                print(f"   ⚠️ 处理第 {idx} 条数据时出错: {e}")
                skipped += 1
                continue
        
        log_file.close()
        
        # Summarize time differences.
        if time_diffs:
            avg_diff = sum(time_diffs) / len(time_diffs)
            max_diff = max(time_diffs)
            min_diff = min(time_diffs)
        else:
            avg_diff = max_diff = min_diff = 0
        
        print("\n" + "="*60)
        print("✅ 合并完成")
        print("="*60)
        print(f"总 Gaze 数据: {total} 条")
        print(f"成功匹配: {matched} 张")
        print(f"跳过: {skipped} 条")
        print(f"匹配率: {matched/total*100:.1f}%")
        print(f"\n时间差统计:")
        print(f"  平均: {avg_diff:.4f} 秒")
        print(f"  最小: {min_diff:.4f} 秒")
        print(f"  最大: {max_diff:.4f} 秒")
        print(f"\n输出文件夹: {self.output_folder}")
        print(f"日志文件: {log_path}")
        print("="*60)


def main():
    """Main entry point."""
    print("="*60)
    print("🎯 Gaze + Driving UI 合并工具")
    print("="*60)
    
    # Path configuration
    gaze_csv = r"C:\Users\USER\Desktop\CARLA_package\py code\35fps_Stable_Version\gaze_data\gaze_data_2025-12-04_11-07-34\gaze_data.csv"
    ui_folder = r"C:\Users\USER\Desktop\Experiments\S6_Town10HD_2_Trucks\data_collected\carla_data_2025-12-04_11-07-34\driving_ui"
    output_folder = r"C:\Users\USER\Desktop\Experiments\S6_Town10HD_2_Trucks\merged_ui"
    
    print(f"\n📂 输入路径:")
    print(f"   Gaze CSV: {gaze_csv}")
    print(f"   UI 文件夹: {ui_folder}")
    print(f"   输出文件夹: {output_folder}")
    
    if not os.path.exists(gaze_csv):
        print(f"\n❌ Gaze CSV 不存在")
        return
    
    if not os.path.exists(ui_folder):
        print(f"\n❌ UI 文件夹不存在")
        return
    
    merger = GazeUImerger(gaze_csv, ui_folder, output_folder)
    
    if not merger.load_gaze_data():
        return
    
    if not merger.load_ui_images():
        return
    
    # Merge with a maximum time difference of 5 seconds.
    merger.merge(max_time_diff=1.0)


if __name__ == "__main__":
    main()
