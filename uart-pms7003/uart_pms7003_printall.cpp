/* 
 * Created by J. Yun, SCH Univ.
 * Read a dust sensor (PMS 7003) via USB-to-TTL convertor (PL2303)
 * Need to modify the path to USB port (e.g., /dev/ttyUSB0)
 * Just print all received data
 */

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <iomanip>
using namespace std;

int main(int argc, char *argv[]) {
    int dust;

    // Open the serial port
    if ((dust = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        perror("UART: Failed to open the serial port.\n");
        return -1;
    }

    // Configure the serial port settings
    struct termios options;
    tcgetattr(dust, &options);

    // Modified 25/4/18
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag &= ~PARENB; // Disable parity
    options.c_cflag &= ~CSTOPB; // Use 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 data bits
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines
    tcflush(dust, TCIFLUSH);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);  // make reads non-blocking
    tcsetattr(dust, TCSANOW, &options);
    
    // Used before 25/4/18
    // options.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
    // options.c_iflag = IGNPAR | ICRNL;
    // tcflush(dust, TCIFLUSH);
    // fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);  // make reads non-blocking
    // tcsetattr(dust, TCSANOW, &options);
    // tcsetattr(dust, TCSANOW, &options);

    // >>>>>> Set the sensor to Active mode
    unsigned char cmdActive[7] = {0x42, 0x4d, 0xe1, 0x00, 0x01, 0x01, 0x71};
    write(dust, cmdActive, 7);
    usleep(100000); // Wait 0.1 second for the sensor to switch to active mode
    // <<<<<<

    unsigned char data[32];

    while (1) {
        if (read(dust, data, 32) > 0) {
            // Print the first 16 bytes
            for (int i = 0; i < 16; i++) {
                cout << hex << setw(2) << setfill('0') << (int)data[i] << " ";
            }
            cout << endl;

            // Print the next 16 bytes
            for (int i = 16; i < 32; i++) {
                cout << hex << setw(2) << setfill('0') << (int)data[i] << " ";
            }
            cout << endl;

            usleep(1000000); // Sleep for 1 second to prevent excessive output
        }
    }

    close(dust);
    return 0;
}
