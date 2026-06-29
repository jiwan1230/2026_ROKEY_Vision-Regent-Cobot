"""
Safety stop flowchart generator.
Run: python3 docs/generate_safety_flowchart.py
Output: docs/safety_flowchart.png
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib import font_manager

# 한글 폰트 등록
_FONT_PATH = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(_FONT_PATH)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=_FONT_PATH).get_name()
plt.rcParams['axes.unicode_minus'] = False

# ── 색상 팔레트 ────────────────────────────────────────────────────────────
C = {
    'hand_bg':    '#E8F5E9', 'hand_bd':    '#2E7D32', 'hand_txt':   '#1B5E20',
    'emg_bg':     '#FFEBEE', 'emg_bd':     '#B71C1C', 'emg_txt':    '#7F0000',
    'force_bg':   '#FFF3E0', 'force_bd':   '#E65100', 'force_txt':  '#BF360C',
    'ros_bg':     '#E3F2FD', 'ros_bd':     '#1565C0', 'ros_txt':    '#0D47A1',
    'robot_bg':   '#F3E5F5', 'robot_bd':   '#6A1B9A', 'robot_txt':  '#4A148C',
    'resume_bg':  '#E8F5E9', 'resume_bd':  '#388E3C', 'resume_txt': '#1B5E20',
    'hmi_bg':     '#FFF8E1', 'hmi_bd':     '#F57F17', 'hmi_txt':    '#E65100',
    'title_bg':   '#1A237E', 'title_txt':  'white',
    'arrow':      '#37474F',
    'manual_bg':  '#FCE4EC', 'manual_bd':  '#AD1457',
    'auto_bg':    '#E8F5E9', 'auto_bd':    '#2E7D32',
}

FIG_W, FIG_H = 18, 22
fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
ax.set_xlim(0, FIG_W)
ax.set_ylim(0, FIG_H)
ax.axis('off')
fig.patch.set_facecolor('white')

# ── 헬퍼 ──────────────────────────────────────────────────────────────────
def box(ax, x, y, w, h, text, bg, bd, tc, fontsize=9.5, bold=False,
        style='round,pad=0.1', alpha=1.0, linestyle='solid'):
    patch = FancyBboxPatch((x - w/2, y - h/2), w, h,
                           boxstyle=style, linewidth=1.5,
                           edgecolor=bd, facecolor=bg, alpha=alpha,
                           linestyle=linestyle, zorder=3)
    ax.add_patch(patch)
    weight = 'bold' if bold else 'normal'
    ax.text(x, y, text, ha='center', va='center', fontsize=fontsize,
            color=tc, weight=weight, zorder=4,
            multialignment='center', linespacing=1.4)

def diamond(ax, x, y, w, h, text, bg, bd, tc, fontsize=9):
    from matplotlib.patches import Polygon
    pts = [(x, y+h/2), (x+w/2, y), (x, y-h/2), (x-w/2, y)]
    poly = plt.Polygon(pts, closed=True, linewidth=1.5,
                       edgecolor=bd, facecolor=bg, zorder=3)
    ax.add_patch(poly)
    ax.text(x, y, text, ha='center', va='center', fontsize=fontsize,
            color=tc, weight='bold', zorder=4, multialignment='center',
            linespacing=1.4)

def arrow(ax, x1, y1, x2, y2, color='#37474F', lw=1.5, label='', label_side='right'):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=color,
                                lw=lw, mutation_scale=14),
                zorder=5)
    if label:
        mx, my = (x1+x2)/2, (y1+y2)/2
        dx = 0.15 if label_side == 'right' else -0.15
        ax.text(mx+dx, my, label, fontsize=7.5, color=color,
                ha='left' if label_side == 'right' else 'right',
                va='center', style='italic', zorder=6)

def curve_arrow(ax, xs, ys, xe, ye, color='#37474F', lw=1.4, label=''):
    import matplotlib.patches as mpa
    style = mpa.ArrowStyle('->', head_length=0.25, head_width=0.12)
    conn = mpa.ConnectionPatch((xs, ys), (xe, ye), 'data', 'data',
                               arrowstyle=style, color=color,
                               linewidth=lw,
                               connectionstyle='arc3,rad=0.3', zorder=5)
    ax.add_patch(conn)
    if label:
        ax.text((xs+xe)/2 - 0.3, (ys+ye)/2, label, fontsize=7.5, color=color,
                ha='center', va='bottom', style='italic', zorder=6)

# ══════════════════════════════════════════════════════════════════════════════
# 레이아웃 상수
# ══════════════════════════════════════════════════════════════════════════════
BW, BH = 4.4, 0.72     # 박스 기본 크기
DW, DH = 3.4, 0.60     # 다이아몬드 크기
GAP    = 0.38          # 박스 사이 세로 간격

# 컬럼 중심 x
CX = {
    'hand':  3.2,
    'emg':   9.0,
    'force': 14.8,
}

TOP = 20.8   # 첫 노드 y 위치

# ══════════════════════════════════════════════════════════════════════════════
# 제목
# ══════════════════════════════════════════════════════════════════════════════
box(ax, FIG_W/2, 21.6, 17.5, 0.8,
    'Vision AI Reagent QC — Safety Stop Data Flow',
    C['title_bg'], C['title_bg'], C['title_txt'],
    fontsize=14, bold=True, style='round,pad=0.15')

# ══════════════════════════════════════════════════════════════════════════════
# 컬럼 헤더
# ══════════════════════════════════════════════════════════════════════════════
for cx, lbl, bg, bd in [
    (CX['hand'],  '① 손 감지\n(Hand Detection)',       C['hand_bg'],  C['hand_bd']),
    (CX['emg'],   '② 비상 정지 버튼\n(Emergency Stop)', C['emg_bg'],   C['emg_bd']),
    (CX['force'], '③ 외력 감지\n(External Force)',      C['force_bg'], C['force_bd']),
]:
    box(ax, cx, TOP, BW+0.2, 0.85, lbl, bg, bd, 'black',
        fontsize=10.5, bold=True)

y = TOP - 0.85/2 - GAP  # 헤더 아래 시작

# ══════════════════════════════════════════════════════════════════════════════
# ① 손 감지 컬럼  (CX['hand'])
# ══════════════════════════════════════════════════════════════════════════════
cx = CX['hand']
ys = []   # 각 노드 y 기록

# 1. YOLOv8n 추론
y0 = y
box(ax, cx, y0, BW, BH, 'YOLOv8n 추론\n(매 프레임, ~119 FPS)',
    C['ros_bg'], C['ros_bd'], C['ros_txt'])
ys.append(y0)

y1 = y0 - BH - GAP
box(ax, cx, y1, BW, BH, "'hand' 클래스 검출\nROI 필터: y_center ≥ 250 px",
    C['hand_bg'], C['hand_bd'], C['hand_txt'])
ys.append(y1)
arrow(ax, cx, y0-BH/2, cx, y1+BH/2, C['hand_bd'])

# 2. 다이아몬드: ROI 통과?
y2 = y1 - BH/2 - DH/2 - GAP
diamond(ax, cx, y2, DW, DH+0.1, 'ROI 통과?',
        C['hand_bg'], C['hand_bd'], C['hand_txt'], fontsize=9.5)
ys.append(y2)
arrow(ax, cx, y1-BH/2, cx, y2+DH/2+0.05, C['hand_bd'])

# NO → 무시
box(ax, cx-2.4, y2, 1.8, BH, '무시\n(로봇 팔 오검출)',
    '#EEEEEE', '#9E9E9E', '#616161', fontsize=8.5)
arrow(ax, cx-DW/2, y2, cx-2.4+0.9, y2, '#9E9E9E', label='NO', label_side='left')

# YES → topic 발행
y3 = y2 - DH/2 - BH/2 - GAP
box(ax, cx, y3, BW, BH, '/vision/hand_detected = True\n→ main_decision_node',
    C['ros_bg'], C['ros_bd'], C['ros_txt'])
ys.append(y3)
arrow(ax, cx, y2-DH/2-0.05, cx, y3+BH/2, C['hand_bd'], label='YES')

y4 = y3 - BH - GAP
box(ax, cx, y4, BW, BH,
    'stop_task(stop=True,\nis_emergency=False)',
    C['ros_bg'], C['ros_bd'], C['ros_txt'])
ys.append(y4)
arrow(ax, cx, y3-BH/2, cx, y4+BH/2, C['ros_bd'])

y5 = y4 - BH - GAP
box(ax, cx, y5, BW, BH+0.1, '로봇 일시 정지\nSTATUS: PAUSED',
    C['robot_bg'], C['robot_bd'], C['robot_txt'], bold=True)
ys.append(y5)
arrow(ax, cx, y4-BH/2, cx, y5+(BH+0.1)/2, C['robot_bd'])

# 다이아몬드: 손 사라졌나?
y6 = y5 - (BH+0.1)/2 - DH/2 - GAP
diamond(ax, cx, y6, DW, DH+0.1, '손 사라짐?',
        C['hand_bg'], C['hand_bd'], C['hand_txt'], fontsize=9.5)
ys.append(y6)
arrow(ax, cx, y5-(BH+0.1)/2, cx, y6+DH/2+0.05, C['hand_bd'])

# NO → wait
box(ax, cx+2.5, y6, 1.6, BH, '대기\n(Wait)', '#EEEEEE', '#9E9E9E', '#616161', fontsize=8.5)
arrow(ax, cx+DW/2, y6, cx+2.5-0.8, y6, '#9E9E9E', label='NO')

# YES → 재개
y7 = y6 - DH/2 - BH/2 - GAP
box(ax, cx, y7, BW, BH,
    'stop_task(stop=False)\n→ 자동 재개',
    C['auto_bg'], C['auto_bd'], C['resume_txt'], bold=True)
ys.append(y7)
arrow(ax, cx, y6-DH/2-0.05, cx, y7+BH/2, C['hand_bd'], label='YES')

# ══════════════════════════════════════════════════════════════════════════════
# ② 비상 정지 버튼 컬럼  (CX['emg'])
# ══════════════════════════════════════════════════════════════════════════════
cx = CX['emg']

# 1. HMI 버튼
e0 = y
box(ax, cx, e0, BW, BH, 'HMI\nEmergency Stop 버튼 클릭',
    C['hmi_bg'], C['hmi_bd'], C['hmi_txt'])
arrow(ax, cx, e0-BH/2, cx, e0-BH/2-GAP-(BH/2), C['emg_bd'])

e1 = e0 - BH - GAP
box(ax, cx, e1, BW, BH,
    'stop_task(\n  stop=True,\n  is_emergency=True)',
    C['ros_bg'], C['ros_bd'], C['ros_txt'], fontsize=8.8)
arrow(ax, cx, e1-BH/2, cx, e1-BH/2-GAP-(BH/2), C['emg_bd'])

e2 = e1 - BH - GAP
box(ax, cx, e2, BW, BH+0.2,
    '로봇 즉시 정지\nSTATUS: EMERGENCY_STOP',
    C['emg_bg'], C['emg_bd'], C['emg_txt'], bold=True)
arrow(ax, cx, e2-(BH+0.2)/2, cx, e2-(BH+0.2)/2-GAP-(BH/2), C['emg_bd'])

e3 = e2 - (BH+0.2) - GAP
box(ax, cx, e3, BW, BH,
    'HMI: EMERGENCY 상태 표시\n(LED 빨강, 로그 기록)',
    C['hmi_bg'], C['hmi_bd'], C['hmi_txt'])
arrow(ax, cx, e3-BH/2, cx, e3-BH/2-GAP-(BH/2), C['emg_bd'])

e4 = e3 - BH - GAP
box(ax, cx, e4, BW, BH,
    '작업자 위험 요인 확인 및 제거',
    C['emg_bg'], C['emg_bd'], C['emg_txt'])
arrow(ax, cx, e4-BH/2, cx, e4-BH/2-GAP-(BH/2), C['emg_bd'])

e5 = e4 - BH - GAP
box(ax, cx, e5, BW, BH,
    'HMI: Start 버튼 클릭\n→ system_running = True',
    C['hmi_bg'], C['hmi_bd'], C['hmi_txt'])
arrow(ax, cx, e5-BH/2, cx, e5-BH/2-GAP-(BH/2), C['emg_bd'])

e6 = e5 - BH - GAP
box(ax, cx, e6, BW, BH,
    '시스템 재개\n(수동 재시작)',
    C['manual_bg'], C['manual_bd'], 'black', bold=True)

# e6 이후 빈 공간을 y7에 맞추기 위해 더미 화살표 없음
# 하단 라벨
ax.text(cx, e6-BH/2-0.25, '⚠ 수동 재개만 가능 (자동 재개 없음)',
        ha='center', va='top', fontsize=8.5,
        color=C['emg_bd'], style='italic', weight='bold', zorder=6)

# ══════════════════════════════════════════════════════════════════════════════
# ③ 외력 감지 컬럼  (CX['force'])
# ══════════════════════════════════════════════════════════════════════════════
cx = CX['force']

f0 = y
box(ax, cx, f0, BW, BH,
    'doosan_robot_control_node\nGetToolForce 폴링 (~7 Hz)',
    C['ros_bg'], C['ros_bd'], C['ros_txt'])
arrow(ax, cx, f0-BH/2, cx, f0-BH/2-GAP-(BH/2), C['force_bd'])

f1 = f0 - BH - GAP
box(ax, cx, f1, BW, BH,
    'force_norm = √(Fx²+Fy²+Fz²)\n→ /robot/force_norm 발행 (Float64)',
    C['ros_bg'], C['ros_bd'], C['ros_txt'], fontsize=8.8)
arrow(ax, cx, f1-BH/2, cx, f1-BH/2-GAP-(DH+0.1)/2, C['force_bd'])

f2 = f1 - BH/2 - (DH+0.1)/2 - GAP
diamond(ax, cx, f2, DW, DH+0.1, 'force_norm > 20 N?',
        C['force_bg'], C['force_bd'], C['force_txt'], fontsize=9.5)

# NO → 계속
box(ax, cx+2.6, f2, 1.8, BH,
    '정상\n(계속 발행)', '#EEEEEE', '#9E9E9E', '#616161', fontsize=8.5)
arrow(ax, cx+DW/2, f2, cx+2.6-0.9, f2, '#9E9E9E', label='NO')

# YES
arrow(ax, cx, f2-(DH+0.1)/2, cx, f2-(DH+0.1)/2-GAP-(BH/2), C['force_bd'], label='YES')

f3 = f2 - (DH+0.1)/2 - BH/2 - GAP
box(ax, cx, f3, BW, BH,
    '/robot/force_detected = True\n→ main_decision_node',
    C['ros_bg'], C['ros_bd'], C['ros_txt'])
arrow(ax, cx, f3-BH/2, cx, f3-BH/2-GAP-(BH/2)*2, C['force_bd'])

f4a = f3 - BH/2 - GAP - BH/2
box(ax, cx-1.3, f4a, 2.2, BH,
    'dispatch 차단\n(main_decision)',
    C['force_bg'], C['force_bd'], C['force_txt'], fontsize=8.5)
box(ax, cx+1.3, f4a, 2.2, BH,
    'stop_event 세팅\n(task_manager)',
    C['force_bg'], C['force_bd'], C['force_txt'], fontsize=8.5)

f5 = f4a - BH - GAP
box(ax, cx, f5, BW, BH+0.2,
    '로봇 즉시 정지\n(진행 중 movel 중단)',
    C['robot_bg'], C['robot_bd'], C['robot_txt'], bold=True)
arrow(ax, cx, f4a-BH/2, cx, f5+(BH+0.2)/2, C['force_bd'])

f6 = f5 - (BH+0.2) - GAP
box(ax, cx, f6, BW, BH,
    'HMI Force Alert 팝업\n(래치: Start 전까지 유지)',
    C['hmi_bg'], C['hmi_bd'], C['hmi_txt'])
arrow(ax, cx, f5-(BH+0.2)/2, cx, f6+BH/2, C['force_bd'])

f7 = f6 - BH - GAP
box(ax, cx, f7, BW, BH,
    '작업자 위험 요인 확인 및 제거',
    C['force_bg'], C['force_bd'], C['force_txt'])
arrow(ax, cx, f6-BH/2, cx, f7+BH/2, C['force_bd'])

f8 = f7 - BH - GAP
box(ax, cx, f8, BW, BH,
    'HMI: Start 버튼 클릭\n→ force_detected 리셋',
    C['hmi_bg'], C['hmi_bd'], C['hmi_txt'])
arrow(ax, cx, f7-BH/2, cx, f8+BH/2, C['force_bd'])

f9 = f8 - BH - GAP
box(ax, cx, f9, BW, BH,
    '시스템 재개\n(수동 재시작)',
    C['manual_bg'], C['manual_bd'], 'black', bold=True)
arrow(ax, cx, f8-BH/2, cx, f9+BH/2, C['force_bd'])
ax.text(cx, f9-BH/2-0.25, '⚠ 수동 재개만 가능 (자동 재개 없음)',
        ha='center', va='top', fontsize=8.5,
        color=C['force_bd'], style='italic', weight='bold', zorder=6)

# ══════════════════════════════════════════════════════════════════════════════
# 구분선 (컬럼 사이)
# ══════════════════════════════════════════════════════════════════════════════
for xd in [6.05, 11.95]:
    ax.plot([xd, xd], [1.2, 21.2], color='#BDBDBD', lw=1.0,
            linestyle='--', zorder=1)

# ══════════════════════════════════════════════════════════════════════════════
# 하단 범례
# ══════════════════════════════════════════════════════════════════════════════
legend_y = 0.95
legend_items = [
    (C['ros_bg'],    C['ros_bd'],    'ROS2 Topic / Service'),
    (C['robot_bg'],  C['robot_bd'],  '로봇 동작 상태'),
    (C['hmi_bg'],    C['hmi_bd'],    'HMI 이벤트'),
    (C['auto_bg'],   C['auto_bd'],   '자동 재개'),
    (C['manual_bg'], C['manual_bd'], '수동 재개 필요'),
]
ax.text(0.5, legend_y, '범례:', fontsize=9, color='#333', weight='bold', va='center')
for i, (bg, bd, lbl) in enumerate(legend_items):
    x0 = 1.5 + i * 3.2
    patch = FancyBboxPatch((x0, legend_y-0.22), 0.5, 0.44,
                           boxstyle='round,pad=0.05', linewidth=1.2,
                           edgecolor=bd, facecolor=bg, zorder=3)
    ax.add_patch(patch)
    ax.text(x0+0.65, legend_y, lbl, fontsize=8.5, va='center', color='#333', zorder=4)

# ══════════════════════════════════════════════════════════════════════════════
# 저장
# ══════════════════════════════════════════════════════════════════════════════
import os
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'safety_flowchart.png')
plt.tight_layout(pad=0.3)
plt.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
print(f'Saved: {out}')
