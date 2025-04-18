/* 
 * Created by J. Yun, SCH Univ.
 * Read a dust sensor (PMS 7003) via USB-to-TTL convertor (PL2303)
 * Need to modify the path to USB port (e.g., /dev/ttyUSB0)
 * Passive mode: get data every 5 seconds
 */

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
using namespace std;

int main(int argc, char *argv[]){
    int dust;

    if ((dust = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY)) < 0){
        perror("UART: Failed to open the file.\n");
        return -1;
    }
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

    unsigned char cmdPassive[7] = {0x42, 0x4d, 0xe1, 0x00, 0x00, 0x01, 0x70};
    unsigned char cmdRead[7] = {0x42, 0x4d, 0xe2, 0x00, 0x00, 0x01, 0x71};
    write(dust, cmdPassive, 7);

    unsigned char c;
    unsigned char data[32];
    int bit_start = 0;
    int count = 0;
    int pm1_0, pm2_5, pm10;

    write(dust, cmdRead, 7);

    while (1) {
        usleep(1000);
        read(dust, &c, 1);
        if (bit_start == 0) {
            if (c == 0x42) {
                bit_start = 1;
                data[0] = c;
                count = 1;
            }

        } else if (bit_start == 1) {
            if (c == 0x4d) {
                bit_start = 2;
                data[1] = c;
                count = 2;
            }
        } else if (bit_start == 2) {
            data[count++] = c;
            if (count >= 32) {
                auto timenow = chrono::system_clock::to_time_t(chrono::system_clock::now()); 
                cout << ctime(&timenow); 
                for (int i = 0; i < 16; i++) {
                    cout << hex << setw(2) << setfill('0') << (int)data[i] << " ";
                }
                cout << endl;
                for (int i = 16; i < 32; i++) {
                    cout << hex << setw(2) << setfill('0') << (int)data[i] << " ";
                }
                cout << endl;

                pm1_0 = (int)((data[10] << 8) | data[11]);
                pm2_5 = (int)((data[12] << 8) | data[13]);
                pm10 = (int)((data[14] << 8) | data[15]);
                cout << "pm1.0: " << dec << pm1_0 << " pm2.5: " << pm2_5 << " pm10: " << pm10 << endl;

                count = 0;
                bit_start = 0;
                for (int i = 0; i < 32; i++)
                    data[i] = 0;

                usleep(5000000);
                write(dust, cmdRead, 7);
            }
        }
    }

    close(dust);
    return 0;
}
