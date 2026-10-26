import os
import re
import matplotlib.pyplot as plt
import glob

base_dir = r"D:\Users\Desktop\localization\outdoor_test_nav2_once"
log_files = glob.glob(os.path.join(base_dir, "*.txt"))

# Try searching in logs/ subdirectory if no .txt files are found in base_dir
if not log_files:
    log_dir = os.path.join(base_dir, "logs")
    if os.path.exists(log_dir):
        log_files = glob.glob(os.path.join(log_dir, "*.txt"))

all_paths = {}

for log_file in log_files:
    filename = os.path.basename(log_file)
    x_vals = []
    y_vals = []
    
    with open(log_file, 'r', encoding='utf-8') as f:
        is_global_icp = False
        for line in f:
            if "[Global ICP]" in line:
                is_global_icp = True
            elif is_global_icp and "Pos -> X:" in line:
                match = re.search(r"Pos -> X:\s+([-\d.]+),\s+Y:\s+([-\d.]+)", line)
                if match:
                    x_vals.append(float(match.group(1)))
                    y_vals.append(float(match.group(2)))
                is_global_icp = False
                
    if x_vals and y_vals:
        all_paths[filename] = (x_vals, y_vals)
        
        # Plot individual
        plt.figure(figsize=(10, 8))
        plt.plot(x_vals, y_vals, marker='o', markersize=2, linestyle='-', label=filename)
        plt.title(f"Trajectory for {filename}")
        plt.xlabel("Map X (m)")
        plt.ylabel("Map Y (m)")
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        out_name = f"plot_{os.path.splitext(filename)[0]}.png"
        plt.savefig(os.path.join(base_dir, out_name))
        plt.close()
        print(f"Saved {out_name}")

# Plot overall
if all_paths:
    plt.figure(figsize=(12, 10))
    for filename, (x_vals, y_vals) in all_paths.items():
        plt.plot(x_vals, y_vals, marker='o', markersize=2, linestyle='-', label=filename)
    
    plt.title("Overall Trajectory")
    plt.xlabel("Map X (m)")
    plt.ylabel("Map Y (m)")
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    overall_out = "plot_overall_trajectory.png"
    plt.savefig(os.path.join(base_dir, overall_out))
    plt.close()
    print(f"Saved {overall_out}")
