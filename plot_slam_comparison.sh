#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

cmake --build build --target vortex_tests -j"$(nproc)"

./build/tests/vortex_tests \
  --gtest_filter="OptimizationTrajectoryTest.GivenNoisyTrajectory_ExpectConvergenceNearGroundTruth" \
  > /tmp/slam_comparison_output.txt

grep -E '^idx|^-?[0-9]' /tmp/slam_comparison_output.txt > slam_comparison.csv

python3 <<'EOF'
import csv

import matplotlib.pyplot as plt

idx, ref_x, ref_y, init_x, init_y, pose_x, pose_y = [], [], [], [], [], [], []
with open("slam_comparison.csv") as f:
    for row in csv.DictReader(f):
        idx.append(int(row["idx"]))
        ref_x.append(float(row["ref_x"]))
        ref_y.append(float(row["ref_y"]))
        init_x.append(float(row["init_x"]))
        init_y.append(float(row["init_y"]))
        pose_x.append(float(row["pose_x"]))
        pose_y.append(float(row["pose_y"]))

fig, ax = plt.subplots(figsize=(10, 6))
for x0, y0, x1, y1 in zip(init_x, init_y, pose_x, pose_y):
    ax.plot([x0, x1], [y0, y1], "-", color="gray", alpha=0.3, linewidth=0.8, zorder=1)
ax.plot(ref_x, ref_y, "o-", label="reference", color="tab:blue", zorder=3)
ax.plot(init_x, init_y, "o", label="initial", color="tab:green", alpha=0.5, zorder=2)
ax.plot(pose_x, pose_y, "o-", label="optimized", color="tab:orange", zorder=3)

for i, x, y in zip(idx, ref_x, ref_y):
    ax.annotate(str(i), (x, y), textcoords="offset points", xytext=(0, 8),
                ha="center", fontsize=7, color="tab:blue")
for i, x, y in zip(idx, pose_x, pose_y):
    ax.annotate(str(i), (x, y), textcoords="offset points", xytext=(0, -12),
                ha="center", fontsize=7, color="tab:orange")

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_title("SLAM trajectory: reference vs optimized")
ax.legend()
ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig("slam_comparison.png", dpi=150)
print("Saved slam_comparison.png")
EOF

echo "Wrote slam_comparison.csv and slam_comparison.png"
