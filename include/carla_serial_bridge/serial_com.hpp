
#ifndef SERIAL_COM_
#define SERIAL_COM_

#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>



class SerialCom{
    public:

        SerialCom();
        SerialCom(char * port_name, __uint16_t baudrate);
        ~SerialCom();

        char * readSerialPort();
        void writeSerialPort(char * );

    private:

        char * port_name;
        __uint16_t baudrate;

        int fd;

        struct termios tty;

        char buffer[64];
        int buffer_size;


};

#endif  // SERIAL_COM_