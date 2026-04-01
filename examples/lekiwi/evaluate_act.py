import torch
from safetensors.torch import load_file
import matplotlib.pyplot as plt
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from tqdm import tqdm

# --- 配置路径 ---
CHECKPOINT_PATH = "outputs/train/act_final/checkpoints/020000/pretrained_model"
REPO_ID = "ljyyds/dataset_combined"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

def evaluate_and_plot():
    # 1. 加载模型
    print(f"正在加载模型权重...")
    policy = ACTPolicy.from_pretrained(CHECKPOINT_PATH).to(DEVICE)
    policy.eval()

    # 2. 手动加载反归一化参数 (带自动键名匹配)
    unnorm_path = f"{CHECKPOINT_PATH}/policy_postprocessor_step_0_unnormalizer_processor.safetensors"
    print(f"读取 Safetensors: {unnorm_path}")
    stats_data = load_file(unnorm_path)
    
    # --- 自动寻找键名 ---
    all_keys = list(stats_data.keys())
    print(f"文件内包含的键: {all_keys}")
    
    mean_key = [k for k in all_keys if 'mean' in k][0]
    std_key = [k for k in all_keys if 'std' in k][0]
    
    action_mean = stats_data[mean_key].to(DEVICE)
    action_std = stats_data[std_key].to(DEVICE)
    print(f"成功匹配键名: {mean_key}, {std_key}")

    # 3. 加载数据集
    dataset = LeRobotDataset(REPO_ID)
    ep_info = dataset.meta.episodes[0]
    from_idx, to_idx = ep_info["dataset_from_index"], ep_info["dataset_to_index"]
    
    gt_actions, pred_actions = [], []
    
    print(f"正在逐帧推理并反归一化...")
    with torch.no_grad():
        for i in tqdm(range(from_idx, to_idx)):
            item = dataset[i]
            # 构造输入
            observation = {k: v.unsqueeze(0).to(DEVICE) for k, v in item.items() if "observation" in k}
            
            # 推理获得归一化后的预测值 (形状通常是 [1, chunk, dim])
            output_action_norm = policy.select_action(observation)
            
            # 这里的核心：如果 select_action 返回的是已经反归一化的，那乘了就错。
            # 但你之前的 MSE 1589 证明它没反归一化，所以我们手动操作：
            # raw = norm * std + mean
            # 注意取第一帧动作 [dim]
            pred_raw = output_action_norm[0] * action_std + action_mean
            
            gt_actions.append(item["action"])
            pred_actions.append(pred_raw.cpu())

    gt_actions = torch.stack(gt_actions)
    pred_actions = torch.stack(pred_actions)

    # 4. 绘图
    action_dim = gt_actions.shape[1]
    fig, axes = plt.subplots(action_dim, 1, figsize=(12, 2 * action_dim))
    if action_dim == 1: axes = [axes]
    
    for d in range(action_dim):
        axes[d].plot(gt_actions[:, d].numpy(), label="Demo (Real)", color="blue", alpha=0.6)
        axes[d].plot(pred_actions[:, d].numpy(), label="ACT (Physical)", color="red", linestyle="--")
        axes[d].set_ylabel(f"Joint {d}")
        axes[d].legend()
    
    mse = torch.mean((gt_actions - pred_actions)**2).item()
    plt.suptitle(f"ACT FINAL PHYSICAL EVAL | MSE: {mse:.6f}")
    plt.tight_layout()
    plt.savefig("act_physical_final.png")
    
    print(f"\n物理量纲还原成功！MSE: {mse:.6f}")
    print("对比图已保存至: act_physical_final.png")
    plt.show()

if __name__ == "__main__":
    evaluate_and_plot()