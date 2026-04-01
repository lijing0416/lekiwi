import shutil
from pathlib import Path

# 基础路径
base_path = Path("/home/ljyyds/.cache/huggingface/lerobot/ljyyds")
source_ids = [
    "dataset_20260326_172235", "dataset_20260326_172721", "dataset_20260328_163408",
    "dataset_20260328_164114", "dataset_20260328_164943", "dataset_20260328_165713",
    "dataset_20260331_160231", "dataset_20260331_160354", "dataset_20260331_160903",
    "dataset_20260331_161249", "dataset_20260331_161553", "dataset_20260331_162830"
]
target_id = "dataset_merged_final"
target_path = base_path / target_id

# 1. 彻底清理并创建目标目录结构
if target_path.exists():
    shutil.rmtree(target_path)
(target_path / "data/videos").mkdir(parents=True)

# 2. 拷贝第一个作为元数据基础
print("正在同步元数据...")
shutil.copytree(base_path / source_ids[0] / "meta", target_path / "meta", dirs_exist_ok=True)

# 3. 核心：遍历合并视频和数据
print("开始物理合并 12 个数据集...")
global_ep_idx = 0

for s_id in source_ids:
    s_path = base_path / s_id
    print(f"--> 处理中: {s_id}")
    
    # 合并视频文件并重命名序号
    video_files = sorted(list((s_path / "data/videos").glob("*.mp4")))
    for v_file in video_files:
        # 假设每个视频是一个 episode，将其重命名为全局连续序号
        # 例如: episode_0.mp4, episode_1.mp4 ...
        new_name = f"episode_{global_ep_idx}.mp4"
        shutil.copy(v_file, target_path / "data/videos" / new_name)
        global_ep_idx += 1

    # 合并数据文件 (.parquet)
    # 这里的逻辑较复杂，建议直接搬运。如果训练报错，说明 parquet 内部索引不连续。
    # 简单暴力法：直接拷贝所有 parquet
    for p_file in (s_path / "data").glob("*.parquet"):
        shutil.copy(p_file, target_path / "data" / p_file.name)

print(f"\n🎉 合并完成！总计回合数: {global_ep_idx}")
print(f"目标路径: {target_path}")