import os
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.compute_stats import compute_stats

# 1. 强制离线
os.environ["HF_HUB_OFFLINE"] = "1"

# 2. 你的合并数据集路径
repo_id = "/home/ljyyds/lerobot/outputs/merged_dataset"

print(f"正在为 {repo_id} 计算统计量...")

# 3. 加载数据集
dataset = LeRobotDataset(repo_id)

# 4. 调用底层函数计算并保存统计量
stats = compute_stats(dataset)
dataset.meta.stats = stats
dataset.meta.save()

print("✅ 统计量计算完成！stats.json 已更新。")