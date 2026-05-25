import torch
import matplotlib.pyplot as plt
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.processor import PolicyProcessorPipeline
from lerobot.processor.converters import policy_action_to_transition, transition_to_policy_action
from tqdm import tqdm

# --- 配置路径 ---
CHECKPOINT_PATH = "/home/ljyyds/lerobot/outputs/train/ACT/010000/pretrained_model"
REPO_ID = "/home/ljyyds/.cache/huggingface/lerobot/ljyyds/aloha_merged_20260513_5datasets"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

def evaluate_and_plot():
    # 1. 加载模型
    print(f"正在加载模型权重...")
    policy = ACTPolicy.from_pretrained(CHECKPOINT_PATH).to(DEVICE)
    policy.eval()
    policy.reset()

    # 2. 加载训练时保存的处理器：
    # preprocessor 会给 observation 加 batch、搬到设备、并按训练统计量归一化；
    # postprocessor 会把模型输出的 action 反归一化回物理量纲并搬回 CPU。
    print("正在加载 policy preprocessor/postprocessor...")
    preprocessor = PolicyProcessorPipeline.from_pretrained(
        CHECKPOINT_PATH,
        config_filename="policy_preprocessor.json",
        overrides={"device_processor": {"device": DEVICE}},
    )
    postprocessor = PolicyProcessorPipeline.from_pretrained(
        CHECKPOINT_PATH,
        config_filename="policy_postprocessor.json",
        to_transition=policy_action_to_transition,
        to_output=transition_to_policy_action,
    )

    # 3. 加载数据集
    dataset = LeRobotDataset(REPO_ID)
    ep_info = dataset.meta.episodes[0]
    from_idx, to_idx = ep_info["dataset_from_index"], ep_info["dataset_to_index"]
    
    gt_actions, pred_actions = [], []
    
    print(f"正在逐帧推理并反归一化...")
    with torch.no_grad():
        for i in tqdm(range(from_idx, to_idx)):
            item = dataset[i]
            # 构造原始 observation，让 preprocessor 做训练一致的处理。
            observation = {k: v for k, v in item.items() if k.startswith("observation.")}

            observation = preprocessor(observation)
            output_action_norm = policy.select_action(observation)
            pred_raw = postprocessor(output_action_norm).squeeze(0)
            
            gt_actions.append(item["action"])
            pred_actions.append(pred_raw)

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
    
    error = gt_actions - pred_actions
    mse = torch.mean(error**2).item()
    rmse = torch.sqrt(torch.mean(error**2)).item()
    mae = torch.mean(torch.abs(error)).item()
    per_dim_rmse = torch.sqrt(torch.mean(error**2, dim=0))
    plt.suptitle(f"ACT FINAL PHYSICAL EVAL | MSE: {mse:.6f}")
    plt.tight_layout()
    plt.savefig("act_physical_final.png")
    
    print(f"\n物理量纲还原成功！MSE: {mse:.6f} | RMSE: {rmse:.6f} | MAE: {mae:.6f}")
    print(f"各维 RMSE: {[round(x, 6) for x in per_dim_rmse.tolist()]}")
    print("对比图已保存至: act_physical_final.png")
    plt.show()

if __name__ == "__main__":
    evaluate_and_plot()
