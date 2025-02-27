#include <iostream>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdint>
#include <chrono>
#include <thread>
#include <string>

bool setServoTarget(int fd, uint8_t channel, uint16_t target) {
    
    uint8_t command[4];
    command[0] = 0x84;  
    command[1] = channel;
    command[2] = target & 0x7F;  
    command[3] = (target >> 7) & 0x7F;  

    ssize_t bytesWritten = write(fd, command, 4);
    if (bytesWritten != 4) {
        std::cerr << "Failed to write command for channel " << static_cast<int>(channel) << "\n";
        return false;
    }

}

int main() {
    const char* portName = "/dev/ttyACM0";

    int fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << portName << "\n";
        return 1;
    }

    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        std::cerr << "Error getting port attributes\n";
        close(fd);
        return 1;
    }

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag &= ~PARENB;  
    options.c_cflag &= ~CSTOPB;  
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;  
    options.c_cflag |= CREAD | CLOCAL;  

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        std::cerr << "Error setting port attributes\n";
        close(fd);
        return 1;
    }

    std::string input;
    std::cout << "Maestro Servo Controller\n";
    std::cout << "Enter commands in the format: <channel> <pulse_width_microseconds>\n";
    std::cout << "Example: 0 1500\n";
    std::cout << "Enter 'q' to quit.\n";

    while (true) {
        std::cout << "\nCommand> ";
        std::getline(std::cin, input);

        if (input == "q" || input == "quit") break;

        std::istringstream iss(input);
        int channel, pulse_us;
        if (!(iss >> channel >> pulse_us)) {
            std::cerr << "Invalid input. Please enter a channel and a pulse width.\n";
            continue;
        }

        if (channel < 0 || channel > 23) {
            std::cerr << "Channel must be between 0 and 23.\n";
            continue;
        }

        uint16_t target = static_cast<uint16_t>(pulse_us * 4);

        if (!setServoTarget(fd, static_cast<uint8_t>(channel), target)) {
            std::cerr << "Failed to set servo target.\n";
            continue;
        }

        std::cout << "Servo " << channel << " set to " << pulse_us << " microseconds (target value: " << target << ").\n";
    }

    close(fd);
    return 0;
}

