#ifndef LIBBNO055_LINUX_TRANSPORT_HPP
#define LIBBNO055_LINUX_TRANSPORT_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace bno055lib {

/// Abstract transport interface for BNO055 communication.
/// Allows dependency injection for testing without real hardware.
class Transport {
public:
    virtual ~Transport() = default;

    /// Open the connection to the device.
    virtual bool open() = 0;

    /// Close the connection.
    virtual void close() = 0;

    /// Check if the connection is open.
    virtual bool isOpen() const = 0;

    /// Write a single byte to a register.
    virtual bool write8(uint8_t reg, uint8_t value) = 0;

    /// Write multiple bytes starting at a register.
    virtual bool writeLen(uint8_t reg, const uint8_t* buffer, uint8_t len) = 0;

    /// Read a single byte from a register.
    virtual bool read8(uint8_t reg, uint8_t& value) = 0;

    /// Read multiple bytes starting at a register.
    virtual bool readLen(uint8_t reg, uint8_t* buffer, uint8_t len) = 0;
};

}  // namespace bno055lib

#endif  // LIBBNO055_LINUX_TRANSPORT_HPP
