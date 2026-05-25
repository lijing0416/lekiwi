import os
import shutil
import torch
from pathlib import Path
import lerobot.common.datasets.lerobot_dataset as lerobot_ds

# 1. 强制离线模式
os.environ["HF_HUB_OFFLINE"] = "1"

base_dir = Path("/home/ljyyds/.cache/huggingface/lerobot/ljyyds")
output_path = "/home/ljyyds/lerobot/outputs/merged_dataset"

if os.path.exists(output_path):
    shutil.rmtree(output_path)

dataset_paths = sorted([str(p) for p in base_dir.glob("dataset_*")])
template_ds = lerobot_ds.LeRobotDataset(dataset_paths[0])

# 2. 创建数据集
merged_dataset = lerobot_ds.LeRobotDataset.create(
    repo_id=output_path, 
    fps=template_ds.fps,
    features=template_ds.features,
    robot_type="asus_tuf_robot"
)

# 3. 核心合并逻辑 (不依赖任何高级 API，只靠基础索引)
for path in dataset_paths:
    print(f"正在读取并逐帧合并: {path}")
    src_ds = lerobot_ds.LeRobotDataset(path)
    
    last_ep_idx = None
    
    for i in range(len(src_ds)):
        # 获取原始帧（此时包含所有原始字段）
        frame_data = src_ds[i]
        
        # 提取当前的 episode 编号，用于判断是否结束当前回合
        current_ep_idx = frame_data["episode_index"].item() if torch.is_tensor(frame_data["episode_index"]) else frame_data["episode_index"]
        
        # 如果切换了 episode，保存上一段
        if last_ep_idx is not None and current_ep_idx != last_ep_idx:
            merged_dataset.save_episode()
        
        last_ep_idx = current_ep_idx

        # --- 维度转置 (CHW -> HWC) ---
        for key in list(frame_data.keys()):
            if "observation.images" in key:
                img = frame_data[key]
                if isinstance(img, torch.Tensor):
                    frame_data[key] = img.permute(1, 2, 0)
        
        # --- 强制清理多余字段 ---
        # 必须把这些字段删干净，否则 add_frame 必报错
        forbidden_keys = ["frame_index", "timestamp", "task_index", "index", "episode_index"]
        for key in forbidden_keys:
            frame_data.pop(key, None)
        
        # 写入新数据集
        merged_dataset.add_frame(frame_data)

    # 每个源数据集读完后，确保最后一个 episode 被保存
    merged_dataset.save_episode()

print(f"\n✅ 任务完成！数据集已合并至: {output_path}")