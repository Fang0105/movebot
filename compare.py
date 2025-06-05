import os
import glob
import matplotlib.pyplot as plt

def read_and_average_times(folder):
    pattern = os.path.join(folder, "execution_time_*.txt")
    files = glob.glob(pattern)
    mv_times = []
    fk_times = []
    cc_times = []
    for file in files:
        with open(file, "r") as f:
            for line in f:
                if line.startswith("Motion validation avg time:"):
                    try:
                        mv_times.append(float(line.split(":")[1].strip().replace(" ms", "")))
                    except ValueError:
                        continue
                elif line.startswith("Forward kinematic avg time:"):
                    try:
                        fk_times.append(float(line.split(":")[1].strip().replace(" ms", "")))
                    except ValueError:
                        continue
                elif line.startswith("Collision check avg time:"):
                    try:
                        cc_times.append(float(line.split(":")[1].strip().replace(" ms", "")))
                    except ValueError:
                        continue
    mv_avg = sum(mv_times) / len(mv_times) if mv_times else 0
    fk_avg = sum(fk_times) / len(fk_times) if fk_times else 0
    cc_avg = sum(cc_times) / len(cc_times) if cc_times else 0
    return mv_avg, fk_avg, cc_avg

folders = {
    "VAMP": "vamp/output",
    "Baseline": "movebot/output"
}

labels = ["motion validation", "forward kinematic", "collision check"]
x = range(len(labels))
bar_width = 0.35
mv_avgs = []
fk_avgs = []
cc_avgs = []
colors = ["#1e90ff", "#40e0d0"]

for label, folder in folders.items():
    mv_avg, fk_avg, cc_avg = read_and_average_times(folder)
    mv_avgs.append(mv_avg)
    fk_avgs.append(fk_avg)
    cc_avgs.append(cc_avg)

fig, ax = plt.subplots()
for i, label in enumerate(folders.keys()):
    ax.bar([p + i * bar_width for p in x], [mv_avgs[i], fk_avgs[i], cc_avgs[i]], width=bar_width, label=label, color=colors[i])

print(f"VAMP - Motion validation avg: {mv_avgs[0]} ms, Forward kinematic avg: {fk_avgs[0]} ms, Collision check avg: {cc_avgs[0]} ms")
print(f"Movebot - Motion validation avg: {mv_avgs[1]} ms, Forward kinematic avg: {fk_avgs[1]} ms, Collision check avg: {cc_avgs[1]} ms")
print(f"difference: Motion validation: x{(mv_avgs[1] / mv_avgs[0]).6f}, Forward kinematic: x{fk_avgs[1] / fk_avgs[0]}, Collision check: x{cc_avgs[1] / cc_avgs[0]}")

ax.set_xticks([p + bar_width / 2 for p in x])
ax.set_xticklabels(labels)
ax.set_ylabel("Average Time (ms)")
ax.set_title("Average Time Comparison")
ax.legend()
plt.tight_layout()
plt.savefig("compare.png")