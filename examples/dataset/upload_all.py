import os
from huggingface_hub import HfApi

# ===================== 配置 =====================
YOUR_USERNAME = "ljyyds"
local_root = "/home/ljyyds/.cache/huggingface/lerobot/ljyyds"

dataset_folders = [
    "aloha_merged"
]
# ===============================================

api = HfApi()

# 上传每一个文件夹 → 变成独立仓库
for folder in dataset_folders:
    repo_id = f"{YOUR_USERNAME}/{folder}"  # 👈 关键：每个文件夹一个仓库
    local_path = os.path.join(local_root, folder)

    if not os.path.exists(local_path):
        print(f"跳过 {folder}")
        continue

    print(f"\n上传独立仓库：{repo_id}")
    
    # 创建仓库
    api.create_repo(repo_id=repo_id, repo_type="dataset", exist_ok=True)

    # 上传数据集
    api.upload_folder(
        folder_path=local_path,
        repo_id=repo_id,
        repo_type="dataset",
        path_in_repo=".",  # 上传到仓库根目录
    )

print("\n🎉 13 个独立数据集全部上传完成！")