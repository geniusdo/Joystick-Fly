#include "OISInputManager.h"
#include "OISJoyStick.h"
#include "OISException.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <atomic>

using namespace OIS;

//using namespace OIS;

// --- CRC8 Function ---
// Standard CRC-8 (Polynomial: 0x07)
uint8_t calculate_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

#pragma pack(push, 1)
struct serial_packet {
    uint16_t magic; // Remove 'const' and default '=' here
    uint16_t status;
    uint32_t sec;
    uint32_t nsec;
    uint32_t id;
    float throttle;
    float yaw_speed;
    float pitch_speed;
    float roll_speed;
    uint8_t  crc;
};
#pragma pack(pop)

// --- Configuration ---
const int   DEADZONE = 2500;
const float MAX_PR_RAD_S = 8.72f * 0.25;
const float MAX_YAW_RAD_S = 3.14f * 0.25;
const float THR_MIN = 0.1f;
const float THR_MAX = 0.8f;

std::atomic<bool> appRunning(true);
std::atomic<bool> isArmed(false);
std::atomic<float> f_throttle(THR_MIN), f_yaw(0.0f), f_pitch(0.0f), f_roll(0.0f);

int setup_serial(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return -1;
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);
options.c_cflag &= ~PARENB;        // No parity
options.c_cflag &= ~CSTOPB;        // 1 stop bit
options.c_cflag &= ~CSIZE;         // Clear current size
options.c_cflag |= CS8;            // Explicitly set 8 data bits
options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

void serialWorker() {
    int fd = setup_serial("/dev/ttyUSB0");
    if (fd < 0) {
        std::cerr << "\n[!] Serial Error /dev/ttyUSB0" << std::endl;
        appRunning = false;
        return;
    }

    serial_packet pkg;
    pkg.magic = 0xE0FD;
    uint32_t p_id = 0;
    auto next_run = std::chrono::steady_clock::now();

    while (appRunning) {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        pkg.magic = 0xE0FD;
        pkg.status = isArmed ? 0 : 1; 
        pkg.sec = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
        pkg.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1000000000;
        pkg.id = p_id++;
        pkg.throttle = isArmed ? f_throttle.load() : 0.0f;
        pkg.yaw_speed = f_yaw.load();
        pkg.pitch_speed = f_pitch.load();
        pkg.roll_speed = f_roll.load();

        // Calculate CRC on all members EXCEPT the crc field itself
        pkg.crc = calculate_crc8((uint8_t*)&pkg, sizeof(serial_packet) - 1);

        write(fd, &pkg, sizeof(pkg));
        printf("\r\033[K[TX HEX]: ");
uint8_t* raw = (uint8_t*)&pkg;
for(size_t i = 0; i < sizeof(pkg); ++i) {
    printf("%02X ", raw[i]);
}
fflush(stdout);
        // Added \033[K to clear old text ghosts
        printf("\r\033[KID:%-6u | ST:%d | THR:%4.2f | Y:%5.2f P:%5.2f R:%5.2f | CRC:0x%02X", 
               pkg.id, pkg.status, pkg.throttle, 
               pkg.yaw_speed, pkg.pitch_speed, pkg.roll_speed, pkg.crc);
        fflush(stdout);

        next_run += std::chrono::milliseconds(10); 
        std::this_thread::sleep_until(next_run);
    }
    close(fd);
}


float scale_axis(int raw, float max_rate) {
    if (std::abs(raw) < DEADZONE) return 0.0f;
    float usable_range = 32767.0f - DEADZONE;
    float normalized = (std::abs(raw) - DEADZONE) / usable_range;
    return normalized * ((raw > 0) ? 1.0f : -1.0f) * max_rate;
}

class FPVHandler : public JoyStickListener {
public:
    bool buttonPressed(const JoyStickEvent &arg, int button) {
        if (button == 0) { 
            isArmed = !isArmed;
            std::cout << "\n[ SYSTEM " << (isArmed ? "ARMED" : "DISARMED") << " ]" << std::endl;
        }
        return true;
    }

    bool axisMoved(const JoyStickEvent &arg, int axis) {
        const JoyStickState &s = arg.state;
        
        int raw_thr = s.mAxes[1].abs;
        if (raw_thr >= 0) f_throttle = THR_MIN;
        else f_throttle = THR_MIN + (((float)raw_thr / -32768.0f) * (THR_MAX - THR_MIN));

        f_yaw   = scale_axis(-s.mAxes[0].abs, MAX_YAW_RAD_S);
        f_pitch = scale_axis(s.mAxes[4].abs, MAX_PR_RAD_S);
        f_roll  = scale_axis(-s.mAxes[3].abs, MAX_PR_RAD_S);
        
        return true;
    }
    // ... rest of boilerplate listeners ...
    bool buttonReleased(const JoyStickEvent &arg, int b) { return true; }
    bool povMoved(const JoyStickEvent &arg, int i) { return true; }
    bool vector3Moved(const JoyStickEvent &arg, int i) { return true; }
    bool sliderMoved(const JoyStickEvent &arg, int i) { return true; }
};

int main() {
    InputManager *inputManager = nullptr;
    JoyStick *joystick = nullptr;
    FPVHandler handler;

    // Start Serial Thread
    std::thread senderThread(serialWorker);

    try {
        ParamList pl;
        inputManager = InputManager::createInputSystem(pl);
        if (inputManager->getNumberOfDevices(OISJoyStick) > 0) {
            joystick = (JoyStick*)inputManager->createInputObject(OISJoyStick, true);
            joystick->setEventCallback(&handler);
            
            while (appRunning) {
                joystick->capture();
                usleep(5000); // Poll stick at 200Hz for low latency
            }
        }
    } catch (const Exception &ex) { std::cerr << "OIS Error: " << ex.eText << std::endl; }

    appRunning = false;
    if (senderThread.joinable()) senderThread.join();
    if (inputManager) InputManager::destroyInputSystem(inputManager);
    return 0;
}