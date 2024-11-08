
#include <carla_serial_bridge/serial_com.hpp>



SerialCom :: SerialCom(){
}

SerialCom :: ~SerialCom(){
    close(this->fd);
    std::cout << "Serial Com closed" << std::endl;
}

SerialCom::SerialCom(char * port_name, __uint16_t baudrate){
    this->port_name = port_name;
    this->baudrate = baudrate;

    this->fd = open(this->port_name, O_RDWR | O_NOCTTY | O_SYNC);
    
    if (this->fd < 0){
            std::cout << "1 Error opening " << this->port_name << ": " << strerror(errno) << std::endl;
            this->~SerialCom();
    }

   
    if (tcgetattr(this->fd, &tty) != 0) {
        std::cout << "2 Error from tcgetattr: " << strerror(errno) << std::endl;
        this->~SerialCom();
    }

    cfsetospeed(&tty, this->baudrate);
    cfsetispeed(&tty, this->baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signaling chars, no echo, no
                     // canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN] = 0; // read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF
                     | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag
        |= (CLOCAL | CREAD); // ignore modem controls,
                             // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cout << "3 Error from tcsetattr: " << strerror(errno) << std::endl;
        this->~SerialCom();
    }

    this->buffer_size = sizeof(this->buffer);

}

char * SerialCom::readSerialPort(){
    if(read(fd, buffer, buffer_size)){
        return buffer;
    }
    else{
        std::cout << "4 Error reading from serial port: " << strerror(errno) << std::endl;
        strcpy(buffer, "");
        return buffer;
    }
}

void SerialCom::writeSerialPort(char * tx_msg){
    bool write_status = write(fd, tx_msg, 22);

    if(!write_status){
        std::cout << "5 Error writing in serial port: " << strerror(errno) << std::endl;
    }
}