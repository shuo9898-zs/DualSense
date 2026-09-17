"""
Gaze-UI对齐分析工具
处理30Hz gaze数据与10Hz UI图像的时间同步和坐标对齐
手动指定data_collected下的文件夹名称进行分析
"""

import os
import glob
import time
import csv
from typing import List, Dict, Tuple, Optional
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image
import numpy as np


class GazeUIAligner:
    """Gaze数据与UI图像对齐分析器"""
    
    def __init__(self, data_path: str, window_ms: int = 50):
        """
        初始化对齐器
        
        Args:
            data_path: 数据文件夹的完整路径
                      例如: C:\\Users\\USER\\Desktop\\CARLA_package\\py code\\35fps_Stable_Version\\data_collected\\carla_data_2025-10-17_23-03-08
            window_ms: 时间窗口大小(毫秒)，±window_ms匹配UI帧
        """
        # 使用完整路径
        self.data_folder = os.path.abspath(data_path)
        self.window_ms = window_ms
        
        # 检查文件夹是否存在
        if not os.path.exists(self.data_folder):
            raise FileNotFoundError(f"❌ 数据文件夹不存在: {self.data_folder}")
        
        # 文件路径
        self.gaze_file = os.path.join(self.data_folder, 'gaze_data.csv')
        self.ui_folder = os.path.join(self.data_folder, 'driving_ui')
        self.output_folder = os.path.join(self.data_folder, 'gaze_ui_analysis')
        
        # 数据容器
        self.gaze_data = []
        self.ui_frames = []
        self.aligned_data = []
        
        # 创建输出文件夹
        os.makedirs(self.output_folder, exist_ok=True)
        
        print(f"🎯 初始化Gaze-UI对齐分析器")
        print(f"   数据源: {os.path.basename(self.data_folder)}")
        print(f"   时间窗口: ±{window_ms}ms")
        print(f"   输出文件夹: {self.output_folder}")
    
    def load_data(self) -> bool:
        """加载gaze数据和UI图像信息"""
        
        # 1. 加载gaze数据
        if not os.path.exists(self.gaze_file):
            print(f"❌ 未找到gaze数据文件: {self.gaze_file}")
            return False
        
        print(f"📊 加载gaze数据...")
        with open(self.gaze_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    self.gaze_data.append({
                        'timestamp_ms': int(row['timestamp_ms']),
                        'gaze_x': float(row['gaze_x']),
                        'gaze_y': float(row['gaze_y']),
                        'confidence': row['confidence'],
                        'screen_width': int(row['screen_width']),
                        'screen_height': int(row['screen_height'])
                    })
                except (ValueError, KeyError) as e:
                    print(f"⚠️ 跳过无效gaze数据行: {e}")
        
        print(f"   ✅ 加载 {len(self.gaze_data)} 条gaze数据")
        
        # 2. 加载UI图像信息
        if not os.path.exists(self.ui_folder):
            print(f"❌ 未找到UI图像文件夹: {self.ui_folder}")
            return False
        
        print(f"🖼️ 扫描UI图像...")
        ui_files = glob.glob(os.path.join(self.ui_folder, 'frame_*.jpg'))
        ui_files.extend(glob.glob(os.path.join(self.ui_folder, 'frame_*.png')))
        
        for file_path in ui_files:
            filename = os.path.basename(file_path)
            # 从文件名提取时间戳: frame_000001_1760753777408.jpg
            try:
                parts = filename.split('_')
                frame_id = int(parts[1])
                timestamp = int(parts[2].split('.')[0])
                
                self.ui_frames.append({
                    'filename': filename,
                    'filepath': file_path,
                    'frame_id': frame_id,
                    'timestamp_ms': timestamp
                })
            except (IndexError, ValueError):
                print(f"⚠️ 无法解析文件名: {filename}")
        
        # 按时间戳排序
        self.ui_frames.sort(key=lambda x: x['timestamp_ms'])
        print(f"   ✅ 找到 {len(self.ui_frames)} 张UI图像")
        
        if len(self.gaze_data) == 0 or len(self.ui_frames) == 0:
            print("❌ 数据不足，无法进行对齐分析")
            return False
        
        return True
    
    def align_data(self):
        """执行时间窗口对齐匹配"""
        print(f"🔄 开始时间窗口对齐匹配...")
        
        self.aligned_data = []
        
        for ui_frame in self.ui_frames:
            ui_timestamp = ui_frame['timestamp_ms']
            
            # 定义时间窗口
            window_start = ui_timestamp - self.window_ms
            window_end = ui_timestamp + self.window_ms
            
            # 找到时间窗口内的所有gaze点
            gaze_in_window = []
            for gaze_point in self.gaze_data:
                if window_start <= gaze_point['timestamp_ms'] <= window_end:
                    gaze_in_window.append(gaze_point)
            
            if len(gaze_in_window) > 0:
                # 计算窗口内gaze点的统计信息
                gaze_x_values = [g['gaze_x'] for g in gaze_in_window]
                gaze_y_values = [g['gaze_y'] for g in gaze_in_window]
                
                # 获取原始屏幕尺寸 (从第一个gaze点)
                original_width = gaze_in_window[0]['screen_width']
                original_height = gaze_in_window[0]['screen_height']
                
                # 计算图像缩放比例
                try:
                    with Image.open(ui_frame['filepath']) as img:
                        img_width, img_height = img.size
                except:
                    # 默认假设0.2倍缩放
                    img_width, img_height = 384, 216
                
                scale_x = img_width / original_width
                scale_y = img_height / original_height
                
                # 映射gaze坐标到图像坐标
                mapped_x = [x * scale_x for x in gaze_x_values]
                mapped_y = [y * scale_y for y in gaze_y_values]
                
                # 计算置信度分布
                confidence_counts = {}
                for g in gaze_in_window:
                    conf = g['confidence']
                    confidence_counts[conf] = confidence_counts.get(conf, 0) + 1
                
                aligned_entry = {
                    'ui_frame': ui_frame,
                    'gaze_points': gaze_in_window,
                    'gaze_count': len(gaze_in_window),
                    'avg_gaze_x': np.mean(gaze_x_values),
                    'avg_gaze_y': np.mean(gaze_y_values),
                    'std_gaze_x': np.std(gaze_x_values) if len(gaze_x_values) > 1 else 0,
                    'std_gaze_y': np.std(gaze_y_values) if len(gaze_y_values) > 1 else 0,
                    'mapped_x': mapped_x,
                    'mapped_y': mapped_y,
                    'avg_mapped_x': np.mean(mapped_x),
                    'avg_mapped_y': np.mean(mapped_y),
                    'confidence_dist': confidence_counts,
                    'scale_x': scale_x,
                    'scale_y': scale_y,
                    'time_window': (window_start, window_end)
                }
                
                self.aligned_data.append(aligned_entry)
        
        print(f"   ✅ 成功对齐 {len(self.aligned_data)} 个UI帧")
        if self.aligned_data:
            avg_gaze_per_frame = np.mean([d['gaze_count'] for d in self.aligned_data])
            print(f"   平均每帧匹配 {avg_gaze_per_frame:.1f} 个gaze点")
    
    def generate_heatmap_overlay(self, sample_frames: int = 5) -> str:
        """生成热力图叠加可视化"""
        print(f"🎨 生成热力图叠加图...")
        
        if len(self.aligned_data) == 0:
            return "❌ 无对齐数据"
        
        # 选择几个代表性帧
        frame_count = min(sample_frames, len(self.aligned_data))
        frame_indices = np.linspace(0, len(self.aligned_data)-1, frame_count, dtype=int)
        
        fig, axes = plt.subplots(1, frame_count, figsize=(4*frame_count, 4))
        if frame_count == 1:
            axes = [axes]
        
        for i, frame_idx in enumerate(frame_indices):
            aligned_frame = self.aligned_data[frame_idx]
            
            # 加载UI图像
            try:
                ui_image = Image.open(aligned_frame['ui_frame']['filepath'])
                axes[i].imshow(ui_image)
                
                # 叠加gaze点
                mapped_x = aligned_frame['mapped_x']
                mapped_y = aligned_frame['mapped_y']
                
                # 绘制individual gaze点
                axes[i].scatter(mapped_x, mapped_y, 
                              c='red', s=30, alpha=0.6, marker='o')
                
                # 绘制平均注视点
                axes[i].scatter(aligned_frame['avg_mapped_x'], 
                              aligned_frame['avg_mapped_y'],
                              c='yellow', s=100, marker='x', linewidths=3)
                
                # 绘制标准差圆圈 (如果有足够的数据点)
                if aligned_frame['std_gaze_x'] > 0:
                    circle = patches.Circle((aligned_frame['avg_mapped_x'], 
                                           aligned_frame['avg_mapped_y']),
                                          aligned_frame['std_gaze_x'] * aligned_frame['scale_x'],
                                          fill=False, color='yellow', linewidth=2, alpha=0.7)
                    axes[i].add_patch(circle)
                
                axes[i].set_title(f'Frame {aligned_frame["ui_frame"]["frame_id"]}\n'
                                f'{aligned_frame["gaze_count"]} gaze points')
                axes[i].axis('off')
                
            except Exception as e:
                axes[i].text(0.5, 0.5, f'Error loading\n{str(e)}', 
                           ha='center', va='center', transform=axes[i].transAxes)
                axes[i].set_title(f'Frame {frame_idx} (Error)')
        
        plt.tight_layout()
        output_path = os.path.join(self.output_folder, 'heatmap_overlay.png')
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"   ✅ 热力图保存到: {output_path}")
        return output_path
    
    def generate_alignment_report(self) -> str:
        """生成对齐质量报告"""
        print(f"📋 生成对齐质量报告...")
        
        report_path = os.path.join(self.output_folder, 'alignment_report.txt')
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("🎯 Gaze-UI对齐分析报告\n")
            f.write("=" * 50 + "\n\n")
            
            # 基础统计
            f.write("📊 数据概况:\n")
            f.write(f"   Gaze数据点: {len(self.gaze_data)} 个\n")
            f.write(f"   UI图像: {len(self.ui_frames)} 张\n")
            f.write(f"   成功对齐: {len(self.aligned_data)} 帧\n")
            if len(self.ui_frames) > 0:
                f.write(f"   对齐成功率: {len(self.aligned_data)/len(self.ui_frames)*100:.1f}%\n\n")
            
            # 时间范围
            if self.gaze_data and self.ui_frames:
                gaze_start = min(g['timestamp_ms'] for g in self.gaze_data)
                gaze_end = max(g['timestamp_ms'] for g in self.gaze_data)
                ui_start = min(f['timestamp_ms'] for f in self.ui_frames)
                ui_end = max(f['timestamp_ms'] for f in self.ui_frames)
                
                f.write("⏱️ 时间范围:\n")
                f.write(f"   Gaze数据: {gaze_start} - {gaze_end} ({(gaze_end-gaze_start)/1000:.1f}秒)\n")
                f.write(f"   UI图像: {ui_start} - {ui_end} ({(ui_end-ui_start)/1000:.1f}秒)\n")
                f.write(f"   重叠区间: {max(gaze_start, ui_start)} - {min(gaze_end, ui_end)}\n\n")
            
            # 匹配质量统计
            if self.aligned_data:
                gaze_counts = [d['gaze_count'] for d in self.aligned_data]
                
                f.write("🎯 匹配质量:\n")
                f.write(f"   平均每帧gaze点数: {np.mean(gaze_counts):.1f}\n")
                f.write(f"   gaze点数范围: {min(gaze_counts)} - {max(gaze_counts)}\n")
                f.write(f"   标准差: {np.std(gaze_counts):.1f}\n\n")
                
                # 置信度分布
                all_confidences = {}
                for aligned in self.aligned_data:
                    for conf, count in aligned['confidence_dist'].items():
                        all_confidences[conf] = all_confidences.get(conf, 0) + count
                
                f.write("🔍 置信度分布:\n")
                total_conf = sum(all_confidences.values())
                for conf, count in all_confidences.items():
                    percentage = count / total_conf * 100 if total_conf > 0 else 0
                    f.write(f"   {conf}: {count} ({percentage:.1f}%)\n")
                f.write("\n")
                
                # 坐标映射信息
                if self.aligned_data:
                    first_aligned = self.aligned_data[0]
                    f.write("📐 坐标映射:\n")
                    f.write(f"   原始屏幕: {self.gaze_data[0]['screen_width']}x{self.gaze_data[0]['screen_height']}\n")
                    f.write(f"   缩放比例: {first_aligned['scale_x']:.3f}x, {first_aligned['scale_y']:.3f}x\n")
                    f.write(f"   时间窗口: ±{self.window_ms}ms\n")
        
        print(f"   ✅ 报告保存到: {report_path}")
        return report_path
    
    def save_aligned_data(self) -> str:
        """保存对齐后的数据到CSV"""
        print(f"💾 保存对齐数据...")
        
        csv_path = os.path.join(self.output_folder, 'aligned_gaze_ui_data.csv')
        
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            
            # 写入标题行
            writer.writerow([
                'ui_frame_id', 'ui_timestamp_ms', 'ui_filename',
                'gaze_count', 'avg_gaze_x', 'avg_gaze_y', 
                'std_gaze_x', 'std_gaze_y',
                'avg_mapped_x', 'avg_mapped_y',
                'scale_x', 'scale_y',
                'high_conf_count', 'medium_conf_count', 'low_conf_count',
                'window_start', 'window_end'
            ])
            
            # 写入数据行
            for aligned in self.aligned_data:
                ui_frame = aligned['ui_frame']
                conf_dist = aligned['confidence_dist']
                
                writer.writerow([
                    ui_frame['frame_id'],
                    ui_frame['timestamp_ms'], 
                    ui_frame['filename'],
                    aligned['gaze_count'],
                    aligned['avg_gaze_x'],
                    aligned['avg_gaze_y'],
                    aligned['std_gaze_x'],
                    aligned['std_gaze_y'],
                    aligned['avg_mapped_x'],
                    aligned['avg_mapped_y'],
                    aligned['scale_x'],
                    aligned['scale_y'],
                    conf_dist.get('HIGH_CONFIDENCE', 0),
                    conf_dist.get('MEDIUM_CONFIDENCE', 0),
                    conf_dist.get('LOW_CONFIDENCE', 0),
                    aligned['time_window'][0],
                    aligned['time_window'][1]
                ])
        
        print(f"   ✅ 对齐数据保存到: {csv_path}")
        return csv_path
    
    def run_full_analysis(self) -> Dict[str, str]:
        """运行完整的对齐分析"""
        print(f"🚀 开始完整Gaze-UI对齐分析...")
        start_time = time.time()
        
        # 1. 加载数据
        if not self.load_data():
            return {"error": "数据加载失败"}
        
        # 2. 对齐数据
        self.align_data()
        
        if len(self.aligned_data) == 0:
            return {"error": "无法对齐任何数据"}
        
        # 3. 生成可视化
        heatmap_path = self.generate_heatmap_overlay()
        
        # 4. 生成报告
        report_path = self.generate_alignment_report()
        
        # 5. 保存对齐数据
        csv_path = self.save_aligned_data()
        
        duration = time.time() - start_time
        print(f"✅ 分析完成! 耗时 {duration:.1f}秒")
        
        return {
            "heatmap": heatmap_path,
            "report": report_path,
            "csv": csv_path,
            "duration": f"{duration:.1f}s",
            "output_folder": self.output_folder
        }
    
    def generate_gaze_video(self, output_name='gaze_points.mp4'):
        """生成每帧gaze点叠加在UI图像上的视频"""
        print(f"🎬 生成gaze点叠加视频...")
        import matplotlib.animation as animation
        fig, ax = plt.subplots(figsize=(6, 4))
        def update(frame_idx):
            ax.clear()
            aligned = self.aligned_data[frame_idx]
            try:
                img = Image.open(aligned['ui_frame']['filepath'])
                ax.imshow(img)
                ax.scatter(aligned['mapped_x'], aligned['mapped_y'], c='red', s=30, alpha=0.6)
                ax.scatter(aligned['avg_mapped_x'], aligned['avg_mapped_y'], c='yellow', s=100, marker='x', linewidths=3)
                ax.set_title(f"Frame {aligned['ui_frame']['frame_id']} | {aligned['gaze_count']} gaze points")
                ax.axis('off')
            except Exception as e:
                ax.text(0.5, 0.5, f'Error: {e}', ha='center', va='center', transform=ax.transAxes)
        ani = animation.FuncAnimation(fig, update, frames=len(self.aligned_data), interval=300)
        video_path = os.path.join(self.output_folder, output_name)
        ani.save(video_path, writer='ffmpeg', dpi=150)
        plt.close(fig)
        print(f"   ✅ gaze点视频保存到: {video_path}")
        return video_path

    def generate_heatmap_video(self, output_name='heatmap_dynamic.mp4'):
        """生成动态热力图视频（gaze分布随时间累积）"""
        print(f"🎬 生成动态热力图视频...")
        import matplotlib.animation as animation
        fig, ax = plt.subplots(figsize=(6, 4))
        all_x, all_y = [], []
        def update(frame_idx):
            ax.clear()
            aligned = self.aligned_data[frame_idx]
            try:
                img = Image.open(aligned['ui_frame']['filepath'])
                ax.imshow(img)
                all_x.extend(aligned['mapped_x'])
                all_y.extend(aligned['mapped_y'])
                heatmap, xedges, yedges = np.histogram2d(all_x, all_y, bins=40)
                # extent的y轴反向，origin设为upper
                ax.imshow(heatmap, cmap='jet', alpha=0.5, origin='upper', extent=[0, img.size[0], img.size[1], 0])
                ax.set_title(f"Heatmap up to Frame {aligned['ui_frame']['frame_id']}")
                ax.axis('off')
            except Exception as e:
                ax.text(0.5, 0.5, f'Error: {e}', ha='center', va='center', transform=ax.transAxes)
        ani = animation.FuncAnimation(fig, update, frames=len(self.aligned_data), interval=300)
        video_path = os.path.join(self.output_folder, output_name)
        ani.save(video_path, writer='ffmpeg', dpi=150)
        plt.close(fig)
        print(f"   ✅ 热力图视频保存到: {video_path}")
        return video_path


def list_available_data_folders():
    """列出所有可用的数据文件夹"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_collected_dir = os.path.join(current_dir, '..', 'data_collected')
    
    if not os.path.exists(data_collected_dir):
        print(f"❌ data_collected文件夹不存在: {data_collected_dir}")
        return []
    
    folders = []
    for item in os.listdir(data_collected_dir):
        folder_path = os.path.join(data_collected_dir, item)
        if os.path.isdir(folder_path) and item.startswith('carla_data_'):
            folders.append(item)
    
    folders.sort(reverse=True)  # 按时间排序（最新的在前）
    return folders


def main():
    """主函数 - 从data_collected文件夹中选择数据"""
    print("🎯 Gaze-UI对齐分析工具")
    print("=" * 50)
    
    print("\n📁 扫描data_collected文件夹...")
    available_folders = list_available_data_folders()
    
    if not available_folders:
        print("❌ 未找到任何CARLA数据文件夹")
        print("请确保data_collected文件夹下有以'carla_data_'开头的文件夹")
        return
    
    print(f"📁 找到 {len(available_folders)} 个数据文件夹:")
    for i, folder in enumerate(available_folders, 1):
        print(f"   {i}. {folder}")
    
    # 让用户选择
    selected_folder = None
    while True:
        try:
            choice = input(f"\n请输入文件夹编号 (1-{len(available_folders)}): ").strip()
            
            if choice.isdigit():
                folder_index = int(choice) - 1
                if 0 <= folder_index < len(available_folders):
                    selected_folder = available_folders[folder_index]
                    break
                else:
                    print(f"❌ 请输入1到{len(available_folders)}之间的数字")
            else:
                print("❌ 请输入数字")
                
        except KeyboardInterrupt:
            print("\n👋 用户取消")
            return
        except Exception as e:
            print(f"❌ 输入错误: {e}")
    
    # 构建完整路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_collected_dir = os.path.join(current_dir, '..', 'data_collected')
    full_path = os.path.join(data_collected_dir, selected_folder)
    
    print(f"\n📊 开始分析: {selected_folder}")
    
    try:
        # 创建分析器并运行
        aligner = GazeUIAligner(full_path, window_ms=50)
        results = aligner.run_full_analysis()
        
        if "error" not in results:
            print(f"\n🎉 分析完成!")
            print(f"📊 热力图: {os.path.basename(results['heatmap'])}")
            print(f"📋 报告: {os.path.basename(results['report'])}")
            print(f"💾 数据: {os.path.basename(results['csv'])}")
            print(f"📁 输出文件夹: {results['output_folder']}")
        else:
            print(f"❌ 分析失败: {results['error']}")
            
    except Exception as e:
        print(f"❌ 分析过程中出现错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    # ==================== 修改这里的路径 ====================
    data_path = r"C:\Users\USER\Desktop\CARLA_package\py code\35fps_Stable_Version\data_collected\carla_data_2025-10-18_15-59-00"
    # =====================================================
    print(f"📂 数据路径: {data_path}\n")
    try:
        aligner = GazeUIAligner(data_path, window_ms=50)
        results = aligner.run_full_analysis()
        # 自动生成视频
        if len(aligner.aligned_data) > 0:
            aligner.generate_gaze_video()
            aligner.generate_heatmap_video()
        if "error" not in results:
            print(f"\n🎉 分析完成!")
            print(f"📊 热力图: {os.path.basename(results['heatmap'])}")
            print(f"📋 报告: {os.path.basename(results['report'])}")
            print(f"💾 数据: {os.path.basename(results['csv'])}")
            print(f"📁 输出文件夹: {results['output_folder']}")
        else:
            print(f"❌ 分析失败: {results['error']}")
    except Exception as e:
        print(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()
