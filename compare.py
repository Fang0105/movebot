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
    "vamp": "vamp/output",
    "movebot": "movebot/output"
}

labels = ["Motion validation", "Forward kinematic", "Collision check"]
x = range(len(labels))
bar_width = 0.35

fig, ax = plt.subplots()
for i, (label, folder) in enumerate(folders.items()):
    mv_avg, fk_avg, cc_avg = read_and_average_times(folder)
    ax.bar([p + i * bar_width for p in x], [mv_avg, fk_avg, cc_avg], width=bar_width, label=label)

ax.set_xticks([p + bar_width / 2 for p in x])
ax.set_xticklabels(labels)
ax.set_ylabel("Average Time (ms)")
ax.set_title("Average Time Comparison")
ax.legend()
plt.tight_layout()
plt.show()