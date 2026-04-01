import torch
import matplotlib.pyplot as plt
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from tqdm import tqdm
import numpy as np

# --- 配置路径 ---
CHECKPOINT_PATH = "outputs/train/diffusion_final_fixed/checkpoints/020000/pretrained_model"
REPO_ID = "ljyyds/dataset_combined"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

def evaluate_diffusion():
    # 1. 加载模型
    print(f"正在加载微型扩散模型...")
    policy = DiffusionPolicy.from_pretrained(CHECKPOINT_PATH).to(DEVICE)
    policy.eval()

    # 2. 加载数据集
    dataset = LeRobotDataset(REPO_ID)
    ep_info = dataset.meta.episodes[0]
    start_idx = ep_info["dataset_from_index"]
    end_idx = ep_info["dataset_to_index"]
    
    gt_actions = []
    pred_actions = []
    
    print(f"开始扩散去噪推理 (Horizon: {policy.config.horizon})...")
    
    with torch.no_grad():
        for i in tqdm(range(start_idx, end_idx)):
            item = dataset[i]
            
            # 准备 Observation
            observation = {}
            for k, v in item.items():
                if "observation.image" in k:
                    observation[k] = v.unsqueeze(0).to(DEVICE) # [1, C, H, W]
                elif "observation.state" in k:
                    observation[k] = v.unsqueeze(0).to(DEVICE) # [1, D]

            # 执行推理
            try:
                action_prediction = policy.select_action(observation)
            except RuntimeError:
                # 兼容性修复：如果模型要求特定维度
                for k in observation:
                    observation[k] = observation[k].reshape(-1, *observation[k].shape[2:])
                action_prediction = policy.select_action(observation)
            
            # --- 核心修复：提取 Action 并存入列表 ---
            # action_prediction 可能是 Tensor 或 Numpy，统一转为 Numpy
            if torch.is_tensor(action_prediction):
                action_prediction = action_prediction.cpu().numpy()

            # 自动处理维度 [Batch, Horizon, Dim] -> 取第一步 [Dim]
            if action_prediction.ndim == 3:
                pred_step = action_prediction[0, 0, :]
            elif action_prediction.ndim == 2:
                pred_step = action_prediction[0, :]
            else:
                pred_step = action_prediction.flatten()

            # 收集数据
            gt_actions.append(item["action"].numpy())
            pred_actions.append(pred_step)

    # 3. 转换为数组并确保维度为 (N, action_dim)
    gt_actions = np.array(gt_actions)
    pred_actions = np.array(pred_actions)

    if gt_actions.ndim == 1: gt_actions = gt_actions.reshape(-1, 1)
    if pred_actions.ndim == 1: pred_actions = pred_actions.reshape(-1, 1)

    # 4. 绘图与 MSE 计算
    action_dim = gt_actions.shape[1]
    fig, axes = plt.subplots(action_dim, 1, figsize=(12, 2 * action_dim))
    if action_dim == 1: axes = [axes]

    for d in range(action_dim):
        axes[d].plot(gt_actions[:, d], label="Demo (Real)", color="blue", alpha=0.5)
        axes[d].plot(pred_actions[:, d], label="Diffusion (Tiny)", color="green", linestyle="--")
        axes[d].set_ylabel(f"Dim {d}")
        axes[d].legend()
    
    mse = np.mean((gt_actions - pred_actions)**2)
    plt.suptitle(f"DIFFUSION EVAL | MSE: {mse:.6f}")
    plt.tight_layout()
    plt.savefig("diffusion_eval_result.png")
    
    print(f"\n推理完成！MSE: {mse:.6f}")
    print("对比图已保存至: diffusion_eval_result.png")
    plt.show()

if __name__ == "__main__":
    evaluate_diffusion()