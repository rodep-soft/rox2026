#!/usr/bin/env python3
import sys
import time
import os
import psutil


def main():
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    interval = 1.0
    print(
        f"📊 [RDK X5 Benchmark] Starting CPU & Memory benchmark for {duration} seconds..."
    )
    print("=" * 75)
    print(
        f"{'Time':<8} | {'Total CPU%':<10} | {'RAM Used (MB)':<14} | {'Top Active ROS Node / Process'}"
    )
    print("-" * 75)

    cpu_samples = []

    # Initial read to warm up psutil cpu_percent
    psutil.cpu_percent(interval=None)

    for i in range(duration):
        time.sleep(interval)
        total_cpu = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory()
        cpu_samples.append(total_cpu)

        # Find top CPU consuming process
        procs = []
        for p in psutil.process_iter(["name", "cmdline", "cpu_percent"]):
            try:
                c = p.info["cpu_percent"]
                name = p.info["name"]
                cmd = " ".join(p.info["cmdline"]) if p.info["cmdline"] else name
                if (
                    "ros" in cmd.lower()
                    or "apriltag" in cmd.lower()
                    or "mipi" in cmd.lower()
                    or "v4l2" in cmd.lower()
                    or "ekf" in cmd.lower()
                    or "mecanum" in cmd.lower()
                    or "bno" in cmd.lower()
                ):
                    procs.append((c, name, cmd))
                elif c > 5.0:
                    procs.append((c, name, cmd))
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

        procs.sort(key=lambda x: x[0], reverse=True)
        top_proc_str = "None"
        if procs:
            top = procs[0]
            short_cmd = top[1]
            if len(top[2].split()) > 0:
                short_cmd = os.path.basename(top[2].split()[0])
            top_proc_str = f"{short_cmd} ({top[0]}% CPU)"

        ram_mb = mem.used / (1024 * 1024)
        print(
            f"{i+1:02d}s     | {total_cpu:6.1f}%    | {ram_mb:8.1f} MB     | {top_proc_str}"
        )

    print("=" * 75)
    avg_cpu = sum(cpu_samples) / len(cpu_samples) if cpu_samples else 0.0
    max_cpu = max(cpu_samples) if cpu_samples else 0.0
    min_cpu = min(cpu_samples) if cpu_samples else 0.0

    print("📈 [Benchmark Summary]")
    print(f"  • Average CPU Usage : {avg_cpu:.1f} %")
    print(f"  • Peak CPU Usage    : {max_cpu:.1f} %")
    print(f"  • Min CPU Usage     : {min_cpu:.1f} %")
    print(f"  • Total CPU Cores   : {psutil.cpu_count(logical=True)} Cores")
    print("=" * 75)


if __name__ == "__main__":
    main()
