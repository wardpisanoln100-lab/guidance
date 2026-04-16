# -*- coding: utf-8 -*-
"""
风场模型可视化脚本
显示训练环境中使用的余弦阵风侧风模型：
  v_wind(t) = 16*(1 - cos(2*pi*(t-10)/10))/2   (m/s, t in [10, 20])
  beta_w(t) = v_wind(t) / V_inf                 (rad, 等效侧滑角)
"""
import numpy as np
import matplotlib.pyplot as plt

# ---------- 参数 ----------
V_inf = 80.0        # 巡航空速 (m/s)
dt = 0.01           # 仿真步长 (s)
T_total = 20.0      # 总时间 (s)
wind_peak = 16.0    # 峰值侧风速度 (m/s)
t_start = 0.0       # 阵风起始时间 (s)
t_end = 10.0        # 阵风结束时间 (s)
T_gust = t_end - t_start  # 阵风持续时间 (s)

# ---------- 生成时间序列 ----------
t = np.arange(0, T_total + dt, dt)

# 侧风速度 (m/s)
v_wind = np.zeros_like(t)
mask = (t >= t_start) & (t <= t_end)
v_wind[mask] = wind_peak * (1 - np.cos(2 * np.pi * (t[mask] - t_start) / T_gust)) / 2

# 等效侧滑角 (rad)
beta_w = v_wind / V_inf

# ---------- 绘图 ----------
fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
fig.canvas.manager.set_window_title("Wind Disturbance Model")

# 上图：侧风速度
axes[0].fill_between(t, v_wind, alpha=0.25, color="steelblue")
axes[0].plot(t, v_wind, linewidth=2, color="steelblue")
axes[0].set_ylabel("Crosswind Speed  $v_w$  (m/s)", fontsize=12)
axes[0].set_title("Cosine Gust — Crosswind Model for TD3 Training", fontsize=14, fontweight="bold")
axes[0].axhline(y=wind_peak, color="red", linestyle="--", linewidth=0.8, label=f"Peak = {wind_peak} m/s")
axes[0].axvspan(t_start, t_end, alpha=0.08, color="orange", label=f"Gust window [{t_start:.0f}s, {t_end:.0f}s]")
axes[0].legend(fontsize=10)
axes[0].grid(True, alpha=0.3)

# 下图：等效侧滑角
axes[1].fill_between(t, np.degrees(beta_w), alpha=0.25, color="coral")
axes[1].plot(t, np.degrees(beta_w), linewidth=2, color="coral")
axes[1].set_ylabel(r"Equivalent Sideslip  $\beta_w$  (°)", fontsize=12)
axes[1].set_xlabel("Time  (s)", fontsize=12)
peak_deg = np.degrees(wind_peak / V_inf)
axes[1].axhline(y=peak_deg, color="red", linestyle="--", linewidth=0.8,
                label=f"Peak = {peak_deg:.1f}° ({wind_peak/V_inf:.3f} rad)")
axes[1].axvspan(t_start, t_end, alpha=0.08, color="orange")
axes[1].legend(fontsize=10)
axes[1].grid(True, alpha=0.3)

fig.tight_layout()
plt.show()
