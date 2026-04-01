import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as T
import os
import numpy as np
from tqdm import tqdm

# ================= 1. 数据增强（保守、正确） =================
train_transform = T.Compose([
    T.RandomResizedCrop(128, scale=(0.9, 1.0)),
    T.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2),
    T.Normalize(mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225])
])

# ================= 2. 数据集 =================
class BCDualCamDataset(Dataset):
    def __init__(self, data_dir="."):
        self.samples = []

        files = [f for f in os.listdir(data_dir) if f.endswith(".pt")]
        print(f"🔍 扫描数据文件: {files}")

        for f in files:
            try:
                data = torch.load(f, weights_only=False)
                if not isinstance(data, list):
                    continue

                for item in data:
                    if (
                        isinstance(item, dict)
                        and "observation" in item
                        and "image" in item["observation"]
                        and "image_bottom" in item["observation"]
                        and "state" in item["observation"]
                        and "action" in item
                    ):
                        self.samples.append(item)
            except Exception as e:
                print(f"⚠️ 跳过文件 {f}: {e}")

        print(f"📦 有效样本数: {len(self.samples)}")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        item = self.samples[idx]

        ACTION_STATS = torch.tensor([0.002, 0.002, 2.0, 2.0, 5.0, 10.0])

        # --- Top camera ---
        img_top = torch.from_numpy(
            item["observation"]["image"]
        ).permute(2, 0, 1).float() / 255.0
        img_top = train_transform(img_top)

        # --- Bottom camera ---
        img_bot = torch.from_numpy(
            item["observation"]["image_bottom"]
        ).permute(2, 0, 1).float() / 255.0
        img_bot = train_transform(img_bot)

        # --- State & Action ---
        state = torch.from_numpy(item["observation"]["state"]).float()
        action = torch.from_numpy(item["action"]).float()
        action = action / ACTION_STATS

        return img_top, img_bot, state, action

# ================= 3. 双目 BC 网络 =================
class DualCamPolicyNet(nn.Module):
    def __init__(self, state_dim=6, action_dim=6):
        super().__init__()

        def make_encoder():
            return nn.Sequential(
                nn.Conv2d(3, 32, 5, stride=2), nn.ReLU(),
                nn.Conv2d(32, 64, 3, stride=2), nn.ReLU(),
                nn.Conv2d(64, 128, 3, stride=2), nn.ReLU(),
                nn.AdaptiveAvgPool2d((4, 4)),
                nn.Flatten()
            )

        self.encoder_top = make_encoder()
        self.encoder_bot = make_encoder()

        self.mlp = nn.Sequential(
            nn.Linear(2048 * 2 + state_dim, 512), nn.ReLU(),
            nn.Linear(512, 256), nn.ReLU(),
            nn.Linear(256, action_dim)
        )

    def forward(self, img_top, img_bot, state):
        ft = self.encoder_top(img_top)
        fb = self.encoder_bot(img_bot)
        x = torch.cat([ft, fb, state], dim=1)
        return self.mlp(x)

# ================= 4. 训练 =================
def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"🚀 训练设备: {device}")

    dataset = BCDualCamDataset(".")
    assert len(dataset) > 0, "❌ 没有有效数据"

    loader = DataLoader(
        dataset,
        batch_size=64,
        shuffle=True,
        num_workers=4,
        pin_memory=True
    )

    model = DualCamPolicyNet().to(device)
    optimizer = optim.AdamW(model.parameters(), lr=3e-4, weight_decay=1e-4)
    criterion = nn.MSELoss()

    best_loss = float("inf")

    for epoch in range(100):
        model.train()
        total_loss = 0.0

        pbar = tqdm(loader, desc=f"Epoch {epoch+1}")
        for img_t, img_b, state, action in pbar:
            img_t = img_t.to(device)
            img_b = img_b.to(device)
            state = state.to(device)
            action = action.to(device)

            pred = model(img_t, img_b, state)
            loss = criterion(pred, action)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()
            pbar.set_postfix(loss=f"{loss.item():.6f}")

        avg_loss = total_loss / len(loader)
        print(f"📉 Epoch {epoch+1} Avg Loss: {avg_loss:.6f}")

        if avg_loss < best_loss:
            best_loss = avg_loss
            torch.save(model.state_dict(), "bc_dual_best.pth")

    torch.save(model.state_dict(), "bc_dual_last.pth")
    print(f"✅ 训练完成，Best Loss: {best_loss:.6f}")

if __name__ == "__main__":
    main()
