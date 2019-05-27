#include "t_serial.h"

t_serial::t_serial(){
    fd = -1;
    isConnected = false;
    nLength = 0;
    pCallback = NULL;
    pCallbackArg = NULL;
}

t_serial::~t_serial(){
    //pthread_exit(NULL);
    Close();
}

extern "C" void *tSerialThread(void*);

void *tSerialThread(void *p){
    t_serial *pSerial = (t_serial*)p;
    if (pSerial->isConnected) pSerial->Read();
    pthread_exit(NULL);
}

bool t_serial::Open(char *device, int baudrate){ // open the serial port
    memset(&newtio, 0, sizeof(newtio));
    fd = open(device, O_RDWR | O_SYNC | O_NONBLOCK | O_NOCTTY, S_IRUSR | S_IWUSR ); // open(device, O_RDWR|O_NOCTTY|O_NONBLOCK);
    if (fd < 0){
        cout << "Device open fail : " << device << endl;
        return false;
    }

    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_cflag = CS8 | CLOCAL | CREAD; // CRTSCTS | CS8 | CLOCAL | CREAD;

    switch (baudrate) {
    case 115200:
        newtio.c_cflag |= B115200;
        break;
    case 57600:
        newtio.c_cflag |= B57600;
        break;
    case 38400:
        newtio.c_cflag |= B38400;
        break;
    case 19200:
        newtio.c_cflag |= B19200;
        break;
    case 9600:
        newtio.c_cflag |= B9600;
        break;
    case 4800:
        newtio.c_cflag |= B4800;
        break;
    case 2400:
        newtio.c_cflag |= B2400;
        break;
    default:
        newtio.c_cflag |= B115200;
        break;
    }

    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 1; // blocking read until x chars received
    tcflush(fd,TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);

    isConnected = true;

    pthread_create(&pthread_serial, NULL, tSerialThread, (void*)this);

    return true;
}

void t_serial::Close(){
    if(fd != -1)    close(fd);
    fd = -1;
}

void t_serial::Reset(){
    nLength = 0;
}

void t_serial::Write(unsigned char *p, int n){
    if(!isConnected)    return;
    write(fd, p, n);
}

void t_serial::Writeb(unsigned char b){
    if(!isConnected)    return;
    write(fd, &b, 1);
}

void t_serial::Read(){
    fd_set fc;
    char ch[2] = {' ', '\0'};

    //struct timespec sleeptime;
    //sleeptime.tv_sec = 0;
    //sleeptime.tv_nsec = 1000;   // 1000ns = 1us

    while(isConnected){
        FD_ZERO(&fc);
        FD_SET(fd, &fc);

        if(select(fd+1, &fc, NULL, NULL, NULL)== -1){
            //qDebug() << "error ?? ";    // for test
            cout << "Select Error" << endl;
            return;
        }

        if(FD_ISSET(fd, &fc)){
            int rc = read(fd, &ch[0], 1);
            if(rc >= 0){
                pBuffer[nLength++] = ch[0];
                if(nLength >= MAX_BUFFER) Reset();
                if(pCallback) pCallback(pCallbackArg);
            }
        }
    }
}
