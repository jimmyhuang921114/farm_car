import numpy as np

# 實際位置與預測位置 (你可以替換成自己的資料)
true_positions = [
    (0.0, 120.0),
    (-85.0, 85.0),
    (-120.0, 0.0),
    (-85.0, -85.0),
    (0.0, -120.0),
    (85.0, -85.0),
    (120.0, 0.0),
    (85.0, 85.0)
]

predicted_positions = [
    (0.1, -0.1),
    (1.1, 0.0),
    (2.1, 0.1),
    (-0.1, 0.9),
    (1.0, 1.1),
    (2.0, 0.9),
    (0.2, 2.1),
    (0.9, 2.0)
]

# 計算每個點的歐幾里得誤差
errors = []
for true, pred in zip(true_positions, predicted_positions):
    dx = true[0] - pred[0]
    dy = true[1] - pred[1]
    error = np.sqrt(dx**2 + dy**2)
    errors.append(error)

# 計算誤差指標
errors = np.array(errors)
rmse = np.sqrt(np.mean(errors**2))
mae = np.mean(np.abs(errors))
max_error = np.max(errors)

# 顯示每個點的誤差
for i, error in enumerate(errors):
    print(f"Point {i+1}: Error = {error:.3f} meters")

# 顯示總體指標
print("\n--- Error Metrics ---")
print(f"RMSE      = {rmse:.3f} meters")
print(f"MAE       = {mae:.3f} meters")
print(f"Max Error = {max_error:.3f} meters")

