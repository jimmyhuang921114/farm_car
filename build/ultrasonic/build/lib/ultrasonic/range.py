import matplotlib.pyplot as plt
import numpy as np

# 車體尺寸（單位：公分）
car_length = 125
car_width = 100

# ✅ 原點為「前輪中心」，感測器位置偏前 33 cm
offset_x = 0.0  # 前輪中心到車頭的偏移量

# 感測器位置與角度
sensor_positions = [
    (offset_x, -48),
    (offset_x, -24),
    (offset_x, 0),
    (offset_x, 24),
    (offset_x, 48)
]
sensor_angles = [-45, -15, 0, 15, 45]  # 每顆感測器的朝向（度）
# sensor_angles = [-60, -30, 0, 30, 60]  # 每顆感測器的朝向（度）
# sensor_angles = [-45, -22.5, 0, 22.5, 45]  # 每顆感測器的朝向（度）
sensor_range = 250  # 最大感測距離（公分）
sensor_fov = 40     # 感測器扇形角度（度）

# 初始化圖形
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_aspect('equal')
ax.set_title("Ultrasonic Range (Origin = Front Wheel Center)")
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")

# 畫出車體（以前輪中心為原點，車體向負X延伸）
car = plt.Rectangle((-car_length + offset_x, -car_width / 2), car_length, car_width,
                    edgecolor='black', facecolor='lightgray')
ax.add_patch(car)

# 畫出每個感測器的覆蓋範圍
for (x, y), angle in zip(sensor_positions, sensor_angles):
    theta = np.radians(angle)
    arc_theta = np.linspace(theta - np.radians(sensor_fov / 2),
                            theta + np.radians(sensor_fov / 2), 100)
    arc_x = x + sensor_range * np.cos(arc_theta)
    arc_y = y + sensor_range * np.sin(arc_theta)

    ax.plot([x] + arc_x.tolist() + [x], [y] + arc_y.tolist() + [y], color='blue', alpha=0.3)
    ax.fill([x] + arc_x.tolist() + [x], [y] + arc_y.tolist() + [y], color='blue', alpha=0.2)
    ax.plot(x, y, 'ro')  # 感測器點

# 調整圖面範圍
ax.set_xlim(-car_length + 50, 250)
ax.set_ylim(-car_width - 150, car_width + 150)

plt.grid(True)
plt.tight_layout()
plt.show()
