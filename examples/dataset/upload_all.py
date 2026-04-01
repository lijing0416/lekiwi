from huggingface_hub import HfApi
import os

api = HfApi()

datasets = [
    "dataset_20260326_172235", "dataset_20260326_172721", "dataset_20260328_163408",
    "dataset_20260328_164114", "dataset_20260328_164943", "dataset_20260328_165713",
    "dataset_20260331_160231", "dataset_20260331_160354", "dataset_20260331_160903",
    "dataset_20260331_161249", "dataset_20260331_161553", "dataset_20260331_162830"
]

# 本地数据存放的根目录
local_root = "/home/ljyyds/.cache/huggingface/lerobot/ljyyds"

for ds in datasets:
    repo_id = f"ljyyds/{ds}"
    folder_path = os.path.join(local_root, ds)
    
    print(f"🚀 正在强制推送文件夹: {folder_path} -> {repo_id}")
    
    try:
        # 自动创建仓库（如果不存在）并上传所有文件
        api.create_repo(repo_id=repo_id, repo_type="dataset", exist_ok=True)
        api.upload_folder(
            folder_path=folder_path,
            repo_id=repo_id,
            repo_type="dataset",
        )
        print(f"✅ {ds} 上传成功！")
    except Exception as e:
        print(f"❌ {ds} 出错: {e}")

print("\n✨ 全部强制上传任务结束。")
