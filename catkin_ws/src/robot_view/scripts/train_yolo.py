# filepath: /home/robot/catkin_ws/src/robot_view/scripts/train_yolo.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess
import rospy

class YOLOTrainer:
    def __init__(self):
        self.data_path = "/home/robot/catkin_ws/src/robot_view/data"
        self.darknet_path = "/home/robot/catkin_ws/src/darknet_ros/darknet"
        self.config_path = "/home/robot/catkin_ws/src/robot_view/config"
        
    def prepare_data(self):
        """准备训练数据"""
        # 创建训练和验证集列表
        train_txt = os.path.join(self.data_path, "train.txt")
        val_txt = os.path.join(self.data_path, "val.txt")
        
        # 生成图片路径列表
        images_dir = os.path.join(self.data_path, "images")
        if os.path.exists(images_dir):
            image_files = [os.path.join(images_dir, f) for f in os.listdir(images_dir) 
                          if f.endswith(('.jpg', '.jpeg', '.png'))]
            
            # 80% 训练，20% 验证
            split_idx = int(len(image_files) * 0.8)
            
            with open(train_txt, 'w') as f:
                for img in image_files[:split_idx]:
                    f.write(img + '\n')
                    
            with open(val_txt, 'w') as f:
                for img in image_files[split_idx:]:
                    f.write(img + '\n')
                    
        print(f"训练集: {split_idx} 张图片")
        print(f"验证集: {len(image_files) - split_idx} 张图片")
    
    def create_data_file(self):
        """创建 .data 文件"""
        data_content = f"""classes=1
train={self.data_path}/train.txt
valid={self.data_path}/val.txt
names={self.config_path}/drink.names
backup={self.data_path}/backup/
"""
        data_file = os.path.join(self.config_path, "drink.data")
        with open(data_file, 'w') as f:
            f.write(data_content)
            
        # 创建类别名称文件
        names_file = os.path.join(self.config_path, "drink.names")
        with open(names_file, 'w') as f:
            f.write("drink_can\n")
            
        # 创建备份目录
        backup_dir = os.path.join(self.data_path, "backup")
        os.makedirs(backup_dir, exist_ok=True)
        
        return data_file
    
    def train(self):
        """开始训练"""
        self.prepare_data()
        data_file = self.create_data_file()
        cfg_file = os.path.join(self.config_path, "drink_yolo.cfg")
        
        # 下载预训练权重
        weights_url = "https://pjreddie.com/media/files/darknet53.conv.74"
        weights_file = os.path.join(self.data_path, "darknet53.conv.74")
        
        if not os.path.exists(weights_file):
            print("下载预训练权重...")
            subprocess.run(["wget", weights_url, "-O", weights_file])
        
        # 训练命令
        train_cmd = [
            os.path.join(self.darknet_path, "darknet"),
            "detector", "train",
            data_file,
            cfg_file,
            weights_file
        ]
        
        print("开始训练...")
        print(" ".join(train_cmd))
        subprocess.run(train_cmd, cwd=self.darknet_path)

if __name__ == '__main__':
    trainer = YOLOTrainer()
    trainer.train()