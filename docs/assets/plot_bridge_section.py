"""从现有 STL 提取顶板横截面；仅用于方案几何核对，不生成运行时地图。"""

import os
import struct
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/bridge-plan-matplotlib")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.font_manager import FontProperties


ROOT = Path(__file__).resolve().parents[2]
FONT = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")


def section_segments(path, cad_z_mm=1000.0):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    segments = []
    for index in range(count):
        values = struct.unpack_from("<12fH", data, 84 + 50 * index)
        vertices = [values[3:6], values[6:9], values[9:12]]
        points = []
        for a, b in zip(vertices, vertices[1:] + vertices[:1]):
            if min(a[2], b[2]) < cad_z_mm < max(a[2], b[2]):
                ratio = (cad_z_mm - a[2]) / (b[2] - a[2])
                points.append([(a[k] + ratio * (b[k] - a[k])) * 0.001 for k in (0, 1)])
        if len(points) == 2:
            segments.append(points)
    return segments


if __name__ == "__main__":
    segments = section_segments(ROOT / "gazebo_bridge/models/bridge/meshes/top_plane.stl")
    fig, ax = plt.subplots(figsize=(13, 3.9), layout="constrained")
    ax.add_collection(LineCollection(segments, colors="#334155", linewidths=1.3))
    ax.set(xlim=(-1.18, 1.18), ylim=(2.61, 3.12), aspect="equal")
    ax.set_xlabel("横桥向 X / m", fontproperties=FONT)
    ax.set_ylabel("世界坐标 Z / m", fontproperties=FONT)
    ax.set_title("现有 top_plane.stl 的中央区域横截面（世界 Y = −1 m）", fontproperties=FONT, fontsize=14)
    for x, name in [(0.0, "中央通道"), (0.59988, "相邻通道")]:
        ax.plot([x, x], [2.84, 2.95], color="#2563eb", linestyle="--", linewidth=1)
        ax.text(x, 2.81, name, ha="center", color="#2563eb", fontproperties=FONT)
    ax.annotate("", xy=(0.14997, 3.035), xytext=(-0.14997, 3.035), arrowprops={"arrowstyle": "<->", "color": "#059669"})
    ax.text(0, 3.047, "根部间距约 0.30 m", ha="center", fontproperties=FONT, color="#059669")
    ax.annotate("", xy=(0.59988, 2.66), xytext=(0, 2.66), arrowprops={"arrowstyle": "<->", "color": "#2563eb"})
    ax.text(0.3, 2.625, "通道中心节距约 0.60 m", ha="center", fontproperties=FONT, color="#2563eb")
    ax.text(-0.63, 2.65, "U 肋向箱内凸出约 0.26 m", ha="center", fontproperties=FONT, color="#475569")
    ax.text(0.83, 3.025, "顶板存在横向坡度", ha="center", fontproperties=FONT, color="#475569")
    ax.grid(alpha=0.15)
    for extension in ("png", "svg"):
        fig.savefig(Path(__file__).with_name(f"bridge_top_section.{extension}"), dpi=180)
    print(f"截取 {len(segments)} 条三角网格交线；已生成 PNG 和 SVG。")
