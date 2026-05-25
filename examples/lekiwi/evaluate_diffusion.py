import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors
from tqdm import tqdm
import numpy as np
import torchvision.transforms.functional as TF

CHECKPOINT_PATH = "/home/ljyyds/lerobot/outputs/train/train/diffusion_aloha_merged_20260517_11datasets_bs32_aug/checkpoints/050000/pretrained_model"
REPO_ID = "/home/ljyyds/.cache/huggingface/lerobot/ljyyds/aloha_merged_20260517_11datasets"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
MAX_SAMPLES = 800

def evaluate_diffusion_final():
    print("正在加载 Diffusion 模型...")
    policy = DiffusionPolicy.from_pretrained(CHECKPOINT_PATH).to(DEVICE)
    policy.eval()
    policy.config.device = DEVICE

    preprocessor, postprocessor = make_pre_post_processors(
        policy.config,
        pretrained_path=CHECKPOINT_PATH,
        preprocessor_overrides={"device_processor": {"device": DEVICE}},
        postprocessor_overrides={"device_processor": {"device": "cpu"}},
    )

    dataset = LeRobotDataset(REPO_ID)

    ep_info = dataset.meta.episodes[0]
    start_idx = ep_info["dataset_from_index"]
    end_idx = min(ep_info["dataset_to_index"], start_idx + MAX_SAMPLES)

    gt_actions, pred_actions = [], []

    prev_ep = None
    policy.reset()

    print("开始连续推理循环...")
    with torch.no_grad():
        for i in tqdm(range(start_idx, end_idx)):
            item = dataset[i]

            # ✅ episode 切换 reset（必须保留）
            current_ep = item["episode_index"]
            current_ep = current_ep.item() if isinstance(current_ep, torch.Tensor) else current_ep
            if prev_ep is not None and current_ep != prev_ep:
                policy.reset()
            prev_ep = current_ep

            obs_dict = {}

            # ===== 单帧输入 =====
            for key in policy.config.input_features:
                data = item[key]

                if "image" in key:
                    if not isinstance(data, torch.Tensor):
                        data = torch.from_numpy(data)

                    if data.ndim == 3 and data.shape[-1] == 3:
                        data = data.permute(2, 0, 1)

                    if data.dtype == torch.uint8:
                        data = data.float() / 255.0
                    elif data.max() > 1.0:
                        data = data.float() / 255.0
                    else:
                        data = data.float()

                    data = TF.resize(data, [224, 224], antialias=True)

                else:
                    data = torch.as_tensor(data, dtype=torch.float32)

                obs_dict[key] = data

            # ===== 推理 =====
            obs_dict = preprocessor(obs_dict)
            action = policy.select_action(obs_dict)  # (1,D), normalized
            action_phys = postprocessor(action)

            # ===== 🔥 打印范围（必须看）=====
            print("action range:", action.min().item(), action.max().item())

            gt_val = item["action"].numpy() if isinstance(item["action"], torch.Tensor) else item["action"]

            gt_actions.append(gt_val)
            pred_actions.append(action_phys.cpu().numpy())

    # ===== 后处理 =====
    gt_actions = np.array(gt_actions)
    pred_actions = np.array(pred_actions)

    if pred_actions.ndim == 3:
        pred_actions = pred_actions.squeeze(1)

    error = gt_actions - pred_actions
    mse = np.mean(error**2)
    rmse = np.sqrt(mse)
    mae = np.mean(np.abs(error))
    per_dim_rmse = np.sqrt(np.mean(error**2, axis=0))
    per_dim_mae = np.mean(np.abs(error), axis=0)

    print(f"\n物理量纲还原成功！MSE: {mse:.6f} | RMSE: {rmse:.6f} | MAE: {mae:.6f}")
    print("Per-dim RMSE:", np.array2string(per_dim_rmse, precision=4, suppress_small=True))
    print("Per-dim MAE :", np.array2string(per_dim_mae, precision=4, suppress_small=True))

    # ===== 画图 =====
    action_dim = gt_actions.shape[1]
    fig, axes = plt.subplots(action_dim, 1, figsize=(12, 2.5 * action_dim))
    if action_dim == 1:
        axes = [axes]

    for d in range(action_dim):
        axes[d].plot(gt_actions[:, d], label="GT", linewidth=1.5)
        axes[d].plot(pred_actions[:, d], "--", label="Pred")
        axes[d].set_ylabel(f"Dim {d}")
        axes[d].grid(True)
        if d == 0:
            axes[d].legend()

    plt.suptitle(f"MSE: {mse:.4f} | RMSE: {rmse:.4f} | MAE: {mae:.4f}")
    plt.tight_layout()
    plt.savefig("final_fixed_eval.png")

    print("图已保存: final_fixed_eval.png")


if __name__ == "__main__":
    evaluate_diffusion_final()
