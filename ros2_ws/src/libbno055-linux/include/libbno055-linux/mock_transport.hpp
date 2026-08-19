#ifndef LIBBNO055_LINUX_MOCK_TRANSPORT_HPP
#define LIBBNO055_LINUX_MOCK_TRANSPORT_HPP

#include <cstring>
#include <functional>

#include "libbno055-linux/transport.hpp"

namespace bno055lib {

/// Mock transport for unit testing without real hardware.
/// Simulates BNO055 register reads/writes with a 256-byte register map.
class MockTransport : public Transport {
public:
    MockTransport() {
        std::memset(registers_, 0, sizeof(registers_));
        // Initialize with BNO055 chip ID at register 0x00
        registers_[0x00] = 0xA0;  // BNO055_ID
        // Set default quaternion W to 16384 (=1.0 after scaling by 1/16384)
        registers_[0x20] = 0x00;  // QUATERNION_DATA_W_LSB
        registers_[0x21] = 0x40;  // QUATERNION_DATA_W_MSB (16384 = 0x4000)
    }

    bool open() override {
        open_ = true;
        return !fail_open_;
    }

    void close() override { open_ = false; }

    bool isOpen() const override { return open_; }

    bool write8(uint8_t reg, uint8_t value) override {
        if (!open_ || fail_writes_) return false;
        registers_[reg] = value;
        write_count_++;
        if (on_write_) on_write_(reg, value);
        return true;
    }

    bool writeLen(uint8_t reg, const uint8_t* buffer, uint8_t len) override {
        if (!open_ || fail_writes_) return false;
        for (uint8_t i = 0; i < len; ++i) {
            registers_[static_cast<uint8_t>(reg + i)] = buffer[i];
        }
        write_count_++;
        return true;
    }

    bool read8(uint8_t reg, uint8_t& value) override {
        if (!open_ || fail_reads_) return false;
        value = registers_[reg];
        read_count_++;
        return true;
    }

    bool readLen(uint8_t reg, uint8_t* buffer, uint8_t len) override {
        if (!open_ || fail_reads_) return false;
        for (uint8_t i = 0; i < len; ++i) {
            buffer[i] = registers_[static_cast<uint8_t>(reg + i)];
        }
        read_count_++;
        return true;
    }

    // --- Test control methods ---

    /// Set a register value directly (for setting up test scenarios).
    void setRegister(uint8_t reg, uint8_t value) { registers_[reg] = value; }

    /// Get a register value directly.
    uint8_t getRegister(uint8_t reg) const { return registers_[reg]; }

    /// Set a 16-bit little-endian value at a register pair.
    void setRegister16LE(uint8_t reg, int16_t value) {
        registers_[reg] = static_cast<uint8_t>(value & 0xFF);
        registers_[static_cast<uint8_t>(reg + 1)] = static_cast<uint8_t>((value >> 8) & 0xFF);
    }

    /// Configure the mock to fail open() calls.
    void setFailOpen(bool fail) { fail_open_ = fail; }

    /// Configure the mock to fail all read operations.
    void setFailReads(bool fail) { fail_reads_ = fail; }

    /// Configure the mock to fail all write operations.
    void setFailWrites(bool fail) { fail_writes_ = fail; }

    /// Get the number of write operations performed.
    uint32_t getWriteCount() const { return write_count_; }

    /// Get the number of read operations performed.
    uint32_t getReadCount() const { return read_count_; }

    /// Reset all counters, failure modes, and register values.
    void reset() {
        std::memset(registers_, 0, sizeof(registers_));
        registers_[0x00] = 0xA0;  // BNO055_ID
        registers_[0x20] = 0x00;
        registers_[0x21] = 0x40;  // Quaternion W = 16384
        fail_open_ = false;
        fail_reads_ = false;
        fail_writes_ = false;
        write_count_ = 0;
        read_count_ = 0;
        open_ = false;
    }

    /// Set a callback for write operations (useful for verifying register writes).
    void setOnWrite(std::function<void(uint8_t reg, uint8_t value)> callback) { on_write_ = std::move(callback); }

private:
    uint8_t registers_[256]{0};
    bool open_{false};
    bool fail_open_{false};
    bool fail_reads_{false};
    bool fail_writes_{false};
    uint32_t write_count_{0};
    uint32_t read_count_{0};
    std::function<void(uint8_t reg, uint8_t value)> on_write_;
};

}  // namespace bno055lib

#endif  // LIBBNO055_LINUX_MOCK_TRANSPORT_HPP
