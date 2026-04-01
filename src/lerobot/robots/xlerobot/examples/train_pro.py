import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as T
import os
import numpy as np
from tqdm import tqdm

# ================= 1. 激进数据增强 (鲁棒性之魂) =================
train_transform = T.Compose([
    T.RandomResizedCrop(128, scale=(0.8, 1.0)), 
    T.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.3, hue=0.1),
    T.RandomGrayscale(p=0.1), # 10% 概率变黑白，防止颜色依赖
    T.GaussianBlur(kernel_size=(3, 3), sigma=(0.1, 1.5)), # 模拟镜头模糊
    T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

class BCDualCamDataset(Dataset):
    def __init__(self, data_dir=".", window_size=3):
        self.samples = []
        self.window_size = window_size # 堆叠 3 帧
        files = sorted([f for f in os.listdir(data_dir) if f.endswith('.pt') and 'dual' in f])
        
        print(f"🔍 正在加载序列数据...")
        for f in files:
            try:
                raw_data = torch.load(f, weights_only=False)
                # 必须保证单次录制的步数大于窗口大小
                if len(raw_data) > self.window_size:
                    for i in range(self.window_size - 1, len(raw_data)):
                        # 存储 (当前索引, 整个序列的引用)
                        self.samples.append((i, raw_data))
            except Exception as e:
                print(f"⚠️ 跳过损坏文件 {f}")
            
        print(f"📦 载入完成，有效序列样本: {len(self.samples)} 步")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        ACTION_STATS = torch.tensor([0.002, 0.002, 2.0, 2.0, 5.0, 10.0])
        curr_idx, sequence = self.samples[idx]
        
        # --- 帧堆叠逻辑 ---
        top_frames = []
        bot_frames = []
        
        for t in range(self.window_size - 1, -1, -1):
            item = sequence[curr_idx - t]
            
            t_img = torch.from_numpy(item['observation']['image']).permute(2, 0, 1).float() / 255.0
            b_img = torch.from_numpy(item['observation']['image_bottom']).permute(2, 0, 1).float() / 255.0
            
            top_frames.append(train_transform(t_img))
            bot_frames.append(train_transform(b_img))
        
        # 拼接成 [9, 128, 128] 的张量 (3帧 * 3通道)
        img_top_stack = torch.cat(top_frames, dim=0)
        img_bot_stack = torch.cat(bot_frames, dim=0)
        
        # 动作和状态只取当前帧
        target_item = sequence[curr_idx]
        state = torch.from_numpy(target_item['observation']['state']).float()
        action = torch.from_numpy(target_item['action']).float()
        
        return img_top_stack, img_bot_stack, state, action / ACTION_STATS

# ================= 2. 时间序感知神经网络 =================
class TimeSeriesPolicyNet(nn.Module):
    def __init__(self, state_dim=6, action_dim=6, window_size=3):
        super().__init__()
        
        def make_encoder():
            return nn.Sequential(
                nn.Conv2d(3 * window_size, 32, 5, stride=2), nn.ReLU(), # 输入通道为 9
                nn.Conv2d(32, 64, 3, stride=2), nn.ReLU(),
                nn.Conv2d(64, 128, 3, stride=2), nn.ReLU(),
                nn.AdaptiveAvgPool2d((4, 4)), nn.Flatten()
            )
        
        self.encoder_top = make_encoder()
        self.encoder_bot = make_encoder()
        
        self.mlp = nn.Sequential(
            nn.Linear(2048 + 2048 + state_dim, 512), nn.ReLU(),
            nn.Linear(512, 256), nn.ReLU(),
            nn.Linear(256, action_dim) 
        )

    def forward(self, img_top, img_bot, state):
        f_top = self.encoder_top(img_top)
        f_bot = self.encoder_bot(img_bot)
        combined = torch.cat([f_top, f_bot, state], dim=1)
        return self.mlp(combined)

# ================= 3. 训练主程序 (CUDA 优化) =================
def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.backends.cudnn.benchmark = True
    print(f"🚀 时间序强化训练启动 | 设备: {device}")

    dataset = BCDualCamDataset(".", window_size=3)
    # 增加 window 导致显存占用变大，建议调小 batch_size
    loader = DataLoader(dataset, batch_size=32, shuffle=True, num_workers=4, pin_memory=True)

    model = TimeSeriesPolicyNet().to(device)
    optimizer = optim.AdamW(model.parameters(), lr=1e-4, weight_decay=1e-3) # 减小学习率，增加权重衰减防止过拟合
    criterion = nn.MSELoss()

    for epoch in range(100):
        model.train()
        epoch_loss = 0
        pbar = tqdm(loader, desc=f"Epoch {epoch+1}")
        
        for i_top, i_bot, st, act in pbar:
            i_top, i_bot, st, act = i_top.to(device), i_bot.to(device), st.to(device), act.to(device)
            
            # State Dropout
            if np.random.rand() < 0.3: st = st * 0.0
            
            pred = model(i_top, i_bot, st)
            loss = criterion(pred, act)
            
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            epoch_loss += loss.item()
            pbar.set_postfix(loss=f"{loss.item():.6f}")

        if (epoch + 1) % 5 == 0:
            torch.save(model.state_dict(), f"bc_temporal_last.pth")
            print("✅ 最终模型已保存！")

if __name__ == "__main__":
    main()