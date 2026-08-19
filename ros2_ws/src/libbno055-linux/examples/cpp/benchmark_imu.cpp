#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

#include "libbno055-linux/bno055.hpp"

int main(int argc, char** argv) {
    std::string device = "/dev/i2c-1";
    int pin = -1;

    if (argc > 1) {
        device = argv[1];
    }
    bno055lib::OpMode mode = bno055lib::OpMode::NDOF;
    std::string mode_str = "ndof";

    if (argc > 2) {
        mode_str = argv[2];
        if (mode_str == "amg") {
            mode = bno055lib::OpMode::AMG;
        } else if (mode_str == "imu") {
            mode = bno055lib::OpMode::IMUPlus;
        } else if (mode_str == "ndof") {
            mode = bno055lib::OpMode::NDOF;
        }
    }
    if (argc > 3) {
        pin = std::stoi(argv[3]);
    }

    std::cout << "Mode: " << mode_str << "\n";
    std::cout << "Interrupt GPIO Pin: " << pin << "\n";

    bno055lib::BNO055 imu(0x28, device);

    if (!imu.begin(mode)) {
        std::cerr << "[-] Failed to initialize BNO055 in " << mode_str << " Mode.\n";
        return 1;
    }
    std::cout << "[+] BNO055 initialized in " << mode_str << " Mode.\n";

    constexpr int NUM_SAMPLES = 2000; // Benchmark for 2000 samples (~1 second at 2kHz)
    std::vector<double> io_latencies;
    std::vector<double> intervals;
    io_latencies.reserve(NUM_SAMPLES);
    intervals.reserve(NUM_SAMPLES);

    std::mutex mtx;
    std::condition_variable cv;
    int sample_count = 0;
    auto last_time = std::chrono::high_resolution_clock::now();

    std::cout << "[+] Starting benchmark. Collecting " << NUM_SAMPLES << " samples...\n";

    // Set callback triggered by GPIO hardware interrupt
    auto callback = [&](const bno055lib::BNO055::RawSensorData& data) {
        auto now = std::chrono::high_resolution_clock::now();
        (void)data;

        // Measure I2C transaction latency (measure getRawSensorDataNoexcept execution time)
        auto start_io = std::chrono::high_resolution_clock::now();
        auto raw_opt = imu.getRawSensorDataNoexcept();
        auto end_io = std::chrono::high_resolution_clock::now();

        std::lock_guard<std::mutex> lock(mtx);
        if (sample_count > 0) {
            std::chrono::duration<double, std::micro> diff = now - last_time;
            intervals.push_back(diff.count());
        }
        if (raw_opt) {
            std::chrono::duration<double, std::micro> io_diff = end_io - start_io;
            io_latencies.push_back(io_diff.count());
        }
        
        last_time = now;
        sample_count++;
        
        if (sample_count >= NUM_SAMPLES) {
            cv.notify_one();
        }
    };

    auto start_bench = std::chrono::high_resolution_clock::now();
    
    // Attempt GPIO interrupt mode if pin >= 0, or fallback to polling loop
    bool is_interrupt_started = false;
    if (pin >= 0) {
        is_interrupt_started = imu.startInterruptDrivenReading(pin, callback);
    }
    if (!is_interrupt_started) {
        std::cout << "[!] GPIO interrupt pin not available or failed. Falling back to software polling mode...\n";
        
        while (sample_count < NUM_SAMPLES) {
            auto now = std::chrono::high_resolution_clock::now();
            auto start_io = std::chrono::high_resolution_clock::now();
            auto quat = imu.getQuaternionNoexcept();
            auto end_io = std::chrono::high_resolution_clock::now();

            if (sample_count > 0) {
                std::chrono::duration<double, std::micro> diff = now - last_time;
                intervals.push_back(diff.count());
            }
            if (quat) {
                std::chrono::duration<double, std::micro> io_diff = end_io - start_io;
                io_latencies.push_back(io_diff.count());
            }
            
            last_time = now;
            sample_count++;
            
            // Limit loop rate depending on mode (100Hz for NDOF/IMU, 1kHz for AMG)
            int sleep_ms = (mode == bno055lib::OpMode::AMG) ? 1 : 10;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    } else {
        // Wait until interrupt collection completes
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [&]() { return sample_count >= NUM_SAMPLES; });
        imu.stopInterruptDrivenReading();
    }

    auto end_bench = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> total_duration = end_bench - start_bench;

    std::cout << "[+] Collection complete!\n\n";

    // Calculate metrics
    double avg_io = std::accumulate(io_latencies.begin(), io_latencies.end(), 0.0) / io_latencies.size();
    double max_io = *std::max_element(io_latencies.begin(), io_latencies.end());
    double min_io = *std::min_element(io_latencies.begin(), io_latencies.end());

    double avg_interval = std::accumulate(intervals.begin(), intervals.end(), 0.0) / intervals.size();
    double max_interval = *std::max_element(intervals.begin(), intervals.end());
    double min_interval = *std::min_element(intervals.begin(), intervals.end());

    // Calculate Standard Deviation (Jitter) of data intervals
    double sq_sum = std::inner_product(intervals.begin(), intervals.end(), intervals.begin(), 0.0);
    double mean = avg_interval;
    double variance = (sq_sum / intervals.size()) - (mean * mean);
    double jitter_stddev = std::sqrt(std::max(0.0, variance));

    // Expected period at 2000Hz is 500 microseconds
    double target_period = 500.0; 

    std::cout << "=========================================================\n";
    std::cout << "                    BENCHMARK RESULTS                    \n";
    std::cout << "=========================================================\n";
    std::cout << "Total Samples:      " << sample_count << "\n";
    std::cout << "Total Time:         " << total_duration.count() << " seconds\n";
    std::cout << "Actual Frequency:   " << (sample_count / total_duration.count()) << " Hz\n\n";

    std::cout << "--- I/O Burst-Read Latency (18-Byte Sequential Read) ---\n";
    std::cout << "Average Latency:    " << std::fixed << std::setprecision(2) << avg_io << " us\n";
    std::cout << "Minimum Latency:    " << min_io << " us\n";
    std::cout << "Maximum Latency:    " << max_io << " us\n\n";

    std::cout << "--- Timing Jitter & Period Stability (Target: " << target_period << " us) ---\n";
    std::cout << "Average Interval:   " << avg_interval << " us\n";
    std::cout << "Jitter (StdDev):    " << jitter_stddev << " us (Lower is better!)\n";
    std::cout << "Min/Max Interval:   " << min_interval << " us / " << max_interval << " us\n";
    std::cout << "Max Deviation:      " << std::max(std::abs(max_interval - target_period), std::abs(min_interval - target_period)) << " us\n";
    std::cout << "=========================================================\n";

    return 0;
}
