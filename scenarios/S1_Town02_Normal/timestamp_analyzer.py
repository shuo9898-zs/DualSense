"""
CARLA 数据时间戳同步分析器
分析gaze数据和UI图像的时间戳对齐情况
确保数据时间完全一致并提供同步工具
"""

import os
import csv
import re
from typing import Dict, List, Optional
import glob


class DataTimestampAnalyzer:
    """数据时间戳分析器 - 确保gaze和UI时间完全对齐"""
    
    def __init__(self, data_session_path: str):
        """
        初始化分析器
        
        Args:
            data_session_path: CARLA数据会话路径 (如 carla_data_2025-10-17_22-19-45)
        """
        self.session_path = data_session_path
        self.gaze_data = None
        self.vehicle_data = None
        self.ui_frames = None
        
        print(f"🔍 分析数据会话: {os.path.basename(data_session_path)}")
        
    def load_all_data(self):
        """加载所有时间戳相关数据"""
        self._load_gaze_data()
        self._load_vehicle_data()
        self._load_ui_frames()
        
    def _load_gaze_data(self):
        """加载gaze数据"""
        gaze_file = os.path.join(self.session_path, "gaze_data.csv")
        if not os.path.exists(gaze_file):
            print("❌ 未找到gaze_data.csv")
            return
        
        try:
            self.gaze_data = []
            with open(gaze_file, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    row['timestamp_ms'] = int(row['timestamp_ms'])
                    self.gaze_data.append(row)
            
            if self.gaze_data:
                timestamps = [row['timestamp_ms'] for row in self.gaze_data]
                print(f"✅ Gaze数据: {len(self.gaze_data)} 条记录")
                print(f"   时间范围: {min(timestamps)} - {max(timestamps)}")
                print(f"   持续时间: {(max(timestamps) - min(timestamps)) / 1000:.1f}秒")
        except Exception as e:
            print(f"❌ 加载gaze数据失败: {e}")
            
    def _load_vehicle_data(self):
        """加载车辆数据"""
        vehicle_file = os.path.join(self.session_path, "vehicle_data.csv")
        if not os.path.exists(vehicle_file):
            print("❌ 未找到vehicle_data.csv")
            return
            
        try:
            self.vehicle_data = []
            with open(vehicle_file, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    row['timestamp_ms'] = int(row['timestamp_ms'])
                    self.vehicle_data.append(row)
            
            if self.vehicle_data:
                timestamps = [row['timestamp_ms'] for row in self.vehicle_data]
                print(f"✅ 车辆数据: {len(self.vehicle_data)} 条记录")
                print(f"   时间范围: {min(timestamps)} - {max(timestamps)}")
                print(f"   持续时间: {(max(timestamps) - min(timestamps)) / 1000:.1f}秒")
        except Exception as e:
            print(f"❌ 加载车辆数据失败: {e}")
            
    def _load_ui_frames(self):
        """加载UI图像时间戳"""
        ui_dir = os.path.join(self.session_path, "driving_ui")
        if not os.path.exists(ui_dir):
            print("❌ 未找到driving_ui文件夹")
            return
            
        # 从文件名提取时间戳: frame_XXXXXX_timestamp.png
        frame_files = glob.glob(os.path.join(ui_dir, "frame_*.png"))
        
        ui_timestamps = []
        for file_path in frame_files:
            filename = os.path.basename(file_path)
            # 匹配格式: frame_000001_1760753986032.png
            match = re.match(r'frame_(\d+)_(\d+)\.png', filename)
            if match:
                frame_id = int(match.group(1))
                timestamp_ms = int(match.group(2))
                ui_timestamps.append({
                    'frame_id': frame_id,
                    'timestamp_ms': timestamp_ms,
                    'filename': filename
                })
        
        self.ui_frames = sorted(ui_timestamps, key=lambda x: x['timestamp_ms'])
        print(f"✅ UI图像: {len(self.ui_frames)} 张图片")
        if len(self.ui_frames) > 0:
            timestamps = [frame['timestamp_ms'] for frame in self.ui_frames]
            print(f"   时间范围: {min(timestamps)} - {max(timestamps)}")
            print(f"   持续时间: {(max(timestamps) - min(timestamps)) / 1000:.1f}秒")
    
    def analyze_timestamp_alignment(self):
        """分析时间戳对齐情况"""
        if not self.gaze_data or not self.ui_frames:
            print("❌ 数据未完全加载，无法分析对齐")
            return
        
        print("\n📊 时间戳对齐分析:")
        print("=" * 50)
        
        # 基本时间范围对比
        gaze_timestamps = [row['timestamp_ms'] for row in self.gaze_data]
        ui_timestamps = [frame['timestamp_ms'] for frame in self.ui_frames]
        
        gaze_start = min(gaze_timestamps)
        gaze_end = max(gaze_timestamps)
        ui_start = min(ui_timestamps)
        ui_end = max(ui_timestamps)
        
        print(f"Gaze时间范围: {gaze_start} - {gaze_end}")
        print(f"UI图像范围:   {ui_start} - {ui_end}")
        
        # 计算时间偏移
        start_diff = ui_start - gaze_start
        end_diff = ui_end - gaze_end
        
        print(f"\n⏰ 时间偏移分析:")
        print(f"开始时间偏移: {start_diff}ms ({start_diff/1000:.2f}秒)")
        print(f"结束时间偏移: {end_diff}ms ({end_diff/1000:.2f}秒)")
        
        # 重叠时间范围
        overlap_start = max(gaze_start, ui_start)
        overlap_end = min(gaze_end, ui_end)
        overlap_duration = (overlap_end - overlap_start) / 1000
        
        print(f"\n📏 数据重叠:")
        print(f"重叠时间范围: {overlap_start} - {overlap_end}")
        print(f"重叠持续时间: {overlap_duration:.2f}秒")
        
        if overlap_duration <= 0:
            print("❌ 警告: gaze数据和UI图像没有时间重叠!")
            return
            
        # 在重叠时间内分析数据密度
        gaze_in_overlap = [row for row in self.gaze_data 
                          if overlap_start <= row['timestamp_ms'] <= overlap_end]
        ui_in_overlap = [frame for frame in self.ui_frames 
                        if overlap_start <= frame['timestamp_ms'] <= overlap_end]
        
        print(f"\n📈 重叠时间内数据量:")
        print(f"Gaze数据点: {len(gaze_in_overlap)} 条")
        print(f"UI图像: {len(ui_in_overlap)} 张")
        print(f"Gaze频率: {len(gaze_in_overlap) / overlap_duration:.1f} Hz")
        print(f"UI频率: {len(ui_in_overlap) / overlap_duration:.1f} Hz")
        
        # 检查理论频率匹配
        expected_gaze_fps = 30  # GazeTracker设定30Hz
        expected_ui_fps = 30    # UIRecorder设定30Hz
        
        actual_gaze_fps = len(gaze_in_overlap) / overlap_duration
        actual_ui_fps = len(ui_in_overlap) / overlap_duration
        
        print(f"\n🎯 频率匹配检查:")
        print(f"Gaze - 预期: {expected_gaze_fps}Hz, 实际: {actual_gaze_fps:.1f}Hz")
        print(f"UI   - 预期: {expected_ui_fps}Hz, 实际: {actual_ui_fps:.1f}Hz")
        
        # 时间戳精度分析
        if len(gaze_in_overlap) > 1:
            gaze_times = sorted([row['timestamp_ms'] for row in gaze_in_overlap])
            gaze_intervals = [gaze_times[i+1] - gaze_times[i] for i in range(len(gaze_times)-1)]
            gaze_avg = sum(gaze_intervals) / len(gaze_intervals)
        else:
            gaze_avg = 0
            
        if len(ui_in_overlap) > 1:
            ui_times = sorted([frame['timestamp_ms'] for frame in ui_in_overlap])
            ui_intervals = [ui_times[i+1] - ui_times[i] for i in range(len(ui_times)-1)]
            ui_avg = sum(ui_intervals) / len(ui_intervals)
        else:
            ui_avg = 0
        
        print(f"\n⏱️ 采样间隔分析:")
        print(f"Gaze间隔: 平均 {gaze_avg:.1f}ms")
        print(f"UI间隔:   平均 {ui_avg:.1f}ms")
        
        return {
            'gaze_count': len(gaze_in_overlap),
            'ui_count': len(ui_in_overlap),
            'overlap_duration': overlap_duration,
            'gaze_fps': actual_gaze_fps,
            'ui_fps': actual_ui_fps,
            'start_offset_ms': start_diff,
            'end_offset_ms': end_diff
        }
    
    def find_temporal_matches(self, max_time_diff_ms=50):
        """
        找到gaze数据和UI图像的时间匹配对
        
        Args:
            max_time_diff_ms: 最大时间差阈值(毫秒)
        """
        if self.gaze_data is None or self.ui_frames is None:
            print("❌ 数据未完全加载")
            return []
        
        print(f"\n🔗 寻找时间匹配对 (阈值: ±{max_time_diff_ms}ms):")
        
        matches = []
        
        for ui_frame in self.ui_frames:
            ui_timestamp = ui_frame['timestamp_ms']
            
            # 找到时间最接近的gaze数据点
            min_diff = float('inf')
            closest_gaze = None
            
            for gaze_row in self.gaze_data:
                diff = abs(gaze_row['timestamp_ms'] - ui_timestamp)
                if diff < min_diff:
                    min_diff = diff
                    closest_gaze = gaze_row
            
            if min_diff <= max_time_diff_ms and closest_gaze:
                matches.append({
                    'ui_frame_id': ui_frame['frame_id'],
                    'ui_timestamp': ui_timestamp,
                    'ui_filename': ui_frame['filename'],
                    'gaze_timestamp': closest_gaze['timestamp_ms'],
                    'gaze_x': closest_gaze['gaze_x'],
                    'gaze_y': closest_gaze['gaze_y'],
                    'gaze_confidence': closest_gaze['confidence'],
                    'time_diff_ms': int(min_diff)
                })
        
        print(f"✅ 找到 {len(matches)} 个匹配对")
        
        if len(matches) > 0:
            time_diffs = [m['time_diff_ms'] for m in matches]
            avg_diff = sum(time_diffs) / len(time_diffs)
            max_diff = max(time_diffs)
            print(f"   时间差分布: 平均 {avg_diff:.1f}ms, 最大 {max_diff}ms")
        
        return matches
    
    def create_synchronized_dataset(self, output_file: str = None, max_time_diff_ms=50):
        """
        创建同步数据集 - 将gaze和UI数据按时间戳对齐
        
        Args:
            output_file: 输出CSV文件路径 (默认在会话文件夹内)
            max_time_diff_ms: 时间匹配阈值
        """
        matches = self.find_temporal_matches(max_time_diff_ms)
        
        if len(matches) == 0:
            print("❌ 没有找到时间匹配的数据对")
            return None
            
        if output_file is None:
            output_file = os.path.join(self.session_path, "synchronized_gaze_ui_data.csv")
        
        # 保存同步数据
        if matches:
            with open(output_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=matches[0].keys())
                writer.writeheader()
                writer.writerows(matches)
            print(f"💾 同步数据已保存: {output_file}")
        
        return matches


def analyze_data_session(session_path: str):
    """分析单个数据会话的时间戳对齐"""
    analyzer = DataTimestampAnalyzer(session_path)
    analyzer.load_all_data()
    
    # 基础分析
    stats = analyzer.analyze_timestamp_alignment()
    
    # 创建同步数据集
    if stats:
        sync_data = analyzer.create_synchronized_dataset()
        return analyzer, sync_data
    else:
        return analyzer, None


def analyze_all_sessions(data_root: str = "./data_collected"):
    """分析所有数据会话"""
    print("🔍 分析所有CARLA数据会话...")
    
    session_dirs = glob.glob(os.path.join(data_root, "carla_data_*"))
    
    results = []
    for session_dir in session_dirs:
        print(f"\n{'='*60}")
        print(f"分析会话: {os.path.basename(session_dir)}")
        print('='*60)
        
        try:
            analyzer, sync_data = analyze_data_session(session_dir)
            results.append({
                'session': os.path.basename(session_dir),
                'analyzer': analyzer,
                'sync_data': sync_data,
                'success': sync_data is not None
            })
        except Exception as e:
            print(f"❌ 分析失败: {e}")
            results.append({
                'session': os.path.basename(session_dir),
                'analyzer': None,
                'sync_data': None,
                'success': False
            })
    
    # 汇总报告
    print(f"\n{'='*60}")
    print("📊 汇总报告")
    print('='*60)
    
    successful = [r for r in results if r['success']]
    print(f"总会话数: {len(results)}")
    print(f"成功同步: {len(successful)}")
    print(f"成功率: {len(successful)/len(results)*100:.1f}%")
    
    return results


if __name__ == "__main__":
    # 分析最新的数据会话
    import sys
    
    if len(sys.argv) > 1:
        session_path = sys.argv[1]
        print(f"分析指定会话: {session_path}")
        analyzer, sync_data = analyze_data_session(session_path)
    else:
        # 分析所有会话
        results = analyze_all_sessions()
        
        # 显示同步数据样本
        for result in results:
            if result['success'] and result['sync_data']:
                print(f"\n📋 {result['session']} 同步数据样本 (前5条):")
                for i, match in enumerate(result['sync_data'][:5]):
                    print(f"  {i+1}: UI帧{match['ui_frame_id']} <-> Gaze({match['gaze_x']:.0f},{match['gaze_y']:.0f}) 时间差{match['time_diff_ms']}ms")
                break