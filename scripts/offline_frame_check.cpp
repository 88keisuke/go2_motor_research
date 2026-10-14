// Offline serializer check only. No SerialPort, device access, or transmission.
#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include "unitreeMotor/unitreeMotor.h"

// Independent bitwise implementation; upstream uses a CRC lookup table.
static uint16_t crc(uint16_t value, const uint8_t* data, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        value ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            value = (value & 1) ? (value >> 1) ^ 0x8408 : value >> 1;
    }
    return value;
}

int main() {
    int checks = 0;
    for (const auto type : {MotorType::GO_2, MotorType::GO_M8010_6}) {
        for (unsigned short id : {0, 14, 15}) {
            MotorCmd cmd;
            cmd.motorType = type;
            cmd.id = id;
            cmd.mode = 7;
            cmd.q = cmd.dq = cmd.tau = cmd.kp = cmd.kd = 0;
            cmd.modify_data(&cmd);
            std::array<uint8_t, 17> expected{};
            expected[0] = 0xFE;
            expected[1] = 0xEE;
            expected[2] = static_cast<uint8_t>(0x70 | id);
            const uint16_t value = crc(type == MotorType::GO_2 ? 0x2CBB : 0,
                                       expected.data(), 15);
            expected[15] = static_cast<uint8_t>(value);
            expected[16] = static_cast<uint8_t>(value >> 8);
            if (cmd.hex_len != 17 ||
                std::memcmp(cmd.get_motor_send_data(), expected.data(), 17) != 0) {
                std::cerr << "FAIL: frame mismatch for id " << id << '\n';
                return 1;
            }
            ++checks;
        }
    }
    std::cout << "PASS: " << checks
              << " offline frame cases (Go2/GO-M8010-6, IDs 0/14/15, CRC). No I/O.\n";
}
