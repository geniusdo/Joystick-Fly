#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#pragma pack(push, 1)
struct serial_packet {
    uint16_t magic;      // Target: 0xE0FD (Memory: FD E0)
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

int main() {
    int fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);
    if (fd < 0) { 
        perror("Failed to open /dev/ttyUSB1"); 
        return 1; 
    }

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);
    
    // 8N1 Raw Mode configuration
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &tty);

    struct serial_packet pkt;
    uint8_t *window = (uint8_t*)&pkt;
    const size_t PKT_SIZE = sizeof(struct serial_packet);

    printf("Receiver Active. Synchronizing on 0xE0FD...\n");

    while (1) {
        // 1. Shift existing data left by 1 byte
        memmove(window, window + 1, PKT_SIZE - 1);

        // 2. Read 1 new byte into the very last position
        if (read(fd, &window[PKT_SIZE - 1], 1) <= 0) continue;

        // 3. Check for the Magic Header [FD E0] at the start of our window
        if (window[0] == 0xFD && window[1] == 0xE0) {
            
            // 4. Validate the CRC (calculated on all bytes except the last one)
            uint8_t computed_crc = calculate_crc8(window, PKT_SIZE - 1);
            
            if (computed_crc == pkt.crc) {
                // SUCCESS: Frame synchronized and data validated
                printf("\r\033[K[OK] ID:%-6u | ST:%d | THR:%4.2f | Y:%5.2f P:%5.2f R:%5.2f", 
                       pkt.id, pkt.status, pkt.throttle, 
                       pkt.yaw_speed, pkt.pitch_speed, pkt.roll_speed);
                fflush(stdout);
            }
        }
    }

    close(fd);
    return 0;
}