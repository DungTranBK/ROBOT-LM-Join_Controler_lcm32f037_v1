import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Wedge, Arc
import matplotlib.patheffects as pe

# Create high-res figure with dark technical theme
plt.style.use('dark_background')
fig, ax = plt.subplots(figsize=(16, 10), dpi=220)
fig.patch.set_facecolor('#080D1A')
ax.set_facecolor('#080D1A')

# Center and radii
cx, cy = 0, 0
r_shaft = 0.55
r_rotor = 2.5
r_airgap = 2.7
r_stator_in = 2.9
r_stator_teeth = 4.3
r_stator_out = 5.1
r_housing = 5.5

# 1. Outer housing and stator yoke
housing_outer = plt.Circle((cx, cy), r_housing, color='#141D2B', ec='#2B3D59', lw=3, zorder=1)
housing_inner = plt.Circle((cx, cy), r_stator_out, color='#0D1522', ec='#1E2B3E', lw=1.5, zorder=2)
ax.add_patch(housing_outer)
ax.add_patch(housing_inner)

# 2. Stator Teeth & Copper Windings (12 Teeth for classic 3-phase)
n_teeth = 12
phase_colors = ['#FFC107', '#FF3366', '#00E676'] # U (Yellow), V (Red/Pink), W (Green)
phase_names = ['U', 'V', 'W']

for i in range(n_teeth):
    angle_deg = i * (360 / n_teeth)
    angle_rad = np.deg2rad(angle_deg)
    p_idx = i % 3
    col = phase_colors[p_idx]
    
    # Tooth steel core
    tooth_w = 13 # degrees
    tooth_wedge = Wedge((cx, cy), r_stator_teeth, angle_deg - tooth_w/2, angle_deg + tooth_w/2,
                        width=r_stator_teeth - r_stator_in, facecolor='#162335', edgecolor='#2B3F5C', lw=1.5, zorder=3)
    ax.add_patch(tooth_wedge)
    
    # Copper coil on tooth
    coil_r = (r_stator_in + r_stator_teeth) / 2
    coil_x = coil_r * np.cos(angle_rad)
    coil_y = coil_r * np.sin(angle_rad)
    coil_box = patches.Circle((coil_x, coil_y), 0.38, facecolor=col, edgecolor='#FFFFFF', alpha=0.9, lw=1.2, zorder=4)
    ax.add_patch(coil_box)
    
    # Label Phase
    ax.text(coil_x, coil_y, f"{phase_names[p_idx]}", color='#000000', fontsize=9, weight='bold',
            ha='center', va='center', zorder=5)

# Stator inner bore circle
bore_circle = plt.Circle((cx, cy), r_stator_in, fill=False, ec='#394F70', ls=':', lw=1.5, zorder=5)
ax.add_patch(bore_circle)

# 3. Rotor Magnet (Rotated by angle theta = 30 degrees)
theta_deg = 30.0
theta_rad = np.deg2rad(theta_deg)

# North Pole (Red) from theta-90 to theta+90
wedge_n = Wedge((cx, cy), r_rotor, theta_deg - 90, theta_deg + 90,
                facecolor='#D32F2F', edgecolor='#FF5252', lw=2.5, zorder=6)
# South Pole (Blue) from theta+90 to theta+270
wedge_s = Wedge((cx, cy), r_rotor, theta_deg + 90, theta_deg + 270,
                facecolor='#1565C0', edgecolor='#42A5F5', lw=2.5, zorder=6)
ax.add_patch(wedge_n)
ax.add_patch(wedge_s)

# Magnet Pole Labels
n_label_r = 1.45
ax.text(n_label_r * np.cos(theta_rad), n_label_r * np.sin(theta_rad), 'N',
        color='#FFFFFF', fontsize=22, weight='bold', ha='center', va='center', zorder=8,
        path_effects=[pe.withStroke(linewidth=4, foreground='#7F0000')])

s_label_rad = theta_rad + np.pi
ax.text(n_label_r * np.cos(s_label_rad), n_label_r * np.sin(s_label_rad), 'S',
        color='#FFFFFF', fontsize=22, weight='bold', ha='center', va='center', zorder=8,
        path_effects=[pe.withStroke(linewidth=4, foreground='#0A2A66')])

# Center shaft
shaft_circle = plt.Circle((cx, cy), r_shaft, facecolor='#222E3F', edgecolor='#546E7A', lw=2.0, zorder=9)
shaft_center = plt.Circle((cx, cy), 0.18, facecolor='#78909C', zorder=10)
ax.add_patch(shaft_circle)
ax.add_patch(shaft_center)

# 4. Accurate Axes: d-axis and q-axis
# --- D-AXIS (Direct Axis: Along Rotor North Pole, theta = 30 deg) ---
d_len = 6.4
d_end_x = d_len * np.cos(theta_rad)
d_end_y = d_len * np.sin(theta_rad)

# Draw D-axis line and Arrow
ax.annotate('', xy=(d_end_x, d_end_y), xytext=(0, 0),
            arrowprops=dict(arrowstyle="-|>", color='#FF5252', lw=4.0, mutation_scale=28),
            zorder=15)
# Dashed negative d-axis line towards S pole
ax.plot([0, -3.5 * np.cos(theta_rad)], [0, -3.5 * np.sin(theta_rad)],
        color='#FF5252', ls='--', lw=2.2, alpha=0.7, zorder=14)

# --- Q-AXIS (Quadrature Axis: Exactly +90 degrees to d-axis, theta + 90 = 120 deg) ---
q_deg = theta_deg + 90.0
q_rad = np.deg2rad(q_deg)
q_len = 6.4
q_end_x = q_len * np.cos(q_rad)
q_end_y = q_len * np.sin(q_rad)

# Draw Q-axis line and Arrow (Stator Field Vector Bs / Iq direction)
ax.annotate('', xy=(q_end_x, q_end_y), xytext=(0, 0),
            arrowprops=dict(arrowstyle="-|>", color='#00E5FF', lw=4.5, mutation_scale=32),
            zorder=16)
# Dashed negative q-axis line
ax.plot([0, -3.5 * np.cos(q_rad)], [0, -3.5 * np.sin(q_rad)],
        color='#00E5FF', ls='--', lw=2.2, alpha=0.7, zorder=14)

# 5. Perfect 90-degree square right-angle symbol between d and q
sq_sz = 0.75
p1 = np.array([sq_sz * np.cos(theta_rad), sq_sz * np.sin(theta_rad)])
p2 = p1 + np.array([sq_sz * np.cos(q_rad), sq_sz * np.sin(q_rad)])
p3 = np.array([sq_sz * np.cos(q_rad), sq_sz * np.sin(q_rad)])
ax.plot([p1[0], p2[0], p3[0]], [p1[1], p2[1], p3[1]], color='#FFEB3B', lw=3.0, zorder=17)
ax.text(1.15 * np.cos(np.deg2rad(theta_deg + 45)), 1.15 * np.sin(np.deg2rad(theta_deg + 45)),
        '90°', color='#FFEB3B', fontsize=14, weight='bold', ha='center', va='center', zorder=18)

# 6. Torque & Pulling Action Arc
arc_r = 3.6
torque_arc = Arc((cx, cy), arc_r*2, arc_r*2, angle=0, theta1=theta_deg + 10, theta2=q_deg - 8,
                 color='#00E5FF', lw=3.2, ls='-', zorder=12)
ax.add_patch(torque_arc)
# Arrow tip on torque arc
arrow_angle = np.deg2rad(q_deg - 8)
ax.annotate('', xy=(arc_r * np.cos(arrow_angle), arc_r * np.sin(arrow_angle)),
            xytext=((arc_r + 0.05) * np.cos(arrow_angle - 0.15), (arc_r + 0.05) * np.sin(arrow_angle - 0.15)),
            arrowprops=dict(arrowstyle="-|>", color='#00E5FF', lw=3.2, mutation_scale=22),
            zorder=13)

# Torque label placed cleanly
ax.text(0.6, 5.2,
        'Lực kéo Mô-men xoắn (Torque Tem)\nKéo cực N đuổi theo từ trường Stator Bs',
        color='#00E5FF', fontsize=10.5, weight='bold', ha='center', va='center', zorder=19,
        bbox=dict(boxstyle='round,pad=0.4', facecolor='#091C28', edgecolor='#00E5FF', lw=1.5, alpha=0.95))

# 7. Labels and Annotations (Positioned without ANY overlap)
# D-Axis Callout Box (Right side)
ax.text(d_end_x + 0.5, d_end_y - 0.2,
        'TRỤC d (Direct Axis)\n'
        '─────────────────────────────\n'
        '• Hướng: Dọc theo cực Bắc (N) Rotor\n'
        '• Từ thông Rotor: Ψm\n'
        '• Mục tiêu FOC: Id = 0\n'
        '  (Triệt tiêu dòng từ hóa vô ích,\n'
        '   tránh làm nóng động cơ)',
        color='#FF8A80', fontsize=11, weight='bold', ha='left', va='center', zorder=20,
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#1F0F14', edgecolor='#FF5252', lw=2.0, alpha=0.95))

# Q-Axis Callout Box (Far Left side, completely clear of the cyan arrow)
ax.text(-6.8, 3.8,
        'TRỤC q (Quadrature Axis)\n'
        '─────────────────────────────\n'
        '• Hướng: VUÔNG GÓC ĐÚNG 90° với trục d\n'
        '• Vectơ từ trường Stator (Bs) đặt tại trục q\n'
        '• Sinh 100% Mô-men xoắn: Tem ∝ Iq\n'
        '  (Cánh tay đòn cực đại, kéo motor quay)',
        color='#80D8FF', fontsize=11, weight='bold', ha='center', va='center', zorder=20,
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#091C28', edgecolor='#00E5FF', lw=2.0, alpha=0.95))

# Header Title (Centered at the very top, with comfortable spacing)
ax.text(0, 8.1, 'NGUYÊN LÝ FOC: QUAN HỆ VUÔNG GÓC 90° TUYỆT ĐỐI GIỮA TRỤC d VÀ TRỤC q',
        color='#FFFFFF', fontsize=16, weight='bold', ha='center', va='center',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#121D33', edgecolor='#448AFF', lw=2.0))
ax.text(0, 7.3, 'Trục d trùng cực Bắc (N) Rotor  |  Trục q đi trước đúng 90° điện  |  Từ trường Stator Bs kéo Rotor quay',
        color='#90CAF9', fontsize=11.5, ha='center', va='center')

# Legend of winding colors at bottom - cleanly split into two lines
ax.text(0, -6.3, 'Cấu tạo cuộn dây Stator 3 pha (U, V, W đặt lệch nhau 120°):',
        color='#B0BEC5', fontsize=11, weight='bold', ha='center', va='center')

ax.plot([-3.5], [-7.0], 'o', color='#FFC107', ms=13)
ax.text(-3.0, -7.0, 'Pha U (Vàng)', color='#FFC107', fontsize=11, weight='bold', va='center')

ax.plot([-0.2], [-7.0], 'o', color='#FF3366', ms=13)
ax.text(0.3, -7.0, 'Pha V (Đỏ)', color='#FF3366', fontsize=11, weight='bold', va='center')

ax.plot([3.0], [-7.0], 'o', color='#00E676', ms=13)
ax.text(3.5, -7.0, 'Pha W (Xanh lá)', color='#00E676', fontsize=11, weight='bold', va='center')

# Configure plot boundaries
ax.set_xlim(-10.2, 10.2)
ax.set_ylim(-7.8, 8.8)
ax.set_aspect('equal')
ax.axis('off')

# Save to destination paths
out_jpg = r'd:\Lumi\LM_2026\ROBOT\.src\EC4012\LCM32F037_BLDC_Keil\docs\images\rotor_stator_interaction.jpg'
out_png = r'd:\Lumi\LM_2026\ROBOT\.src\EC4012\LCM32F037_BLDC_Keil\docs\images\rotor_stator_interaction.png'

plt.savefig(out_jpg, dpi=220, facecolor=fig.get_facecolor(), edgecolor='none', bbox_inches='tight')
plt.savefig(out_png, dpi=220, facecolor=fig.get_facecolor(), edgecolor='none', bbox_inches='tight')
print("Successfully generated finalized FOC d-q diagram!")
