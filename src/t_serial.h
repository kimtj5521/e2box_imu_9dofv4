/*
 *  linux-serial class
 *  date : 2017.11.23
 *  made by Taejin Kim - KAIST_USRG
 */

#ifndef T_SERIAL_H
#define T_SERIAL_H

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <string.h>
#include <termios.h>
#include <linux/serial.h>
#include <pthread.h>

#ifndef TRUE
#define TRUE true
#endif

#ifndef FALSE
#define FALSE false
#endif

typedef unsigned long   DWORD;
typedef unsigned short  WORD;
typedef unsigned char   byte;
typedef unsigned char   BYTE;
typedef bool BOOL;
typedef std::string CString;

#define MAX_BUFFER 4096

using namespace std;

class t_serial{
public:
    t_serial();
    virtual ~t_serial();

private:
    int fd;
    int nLength;
    pthread_t pthread_serial;

public:
    bool isConnected;
    unsigned char pBuffer[MAX_BUFFER];
    struct termios newtio;
    void (*pCallback)(void *);
    void* pCallbackArg;
    unsigned char *GetBuffer() { return pBuffer; }
    int GetLength() { return nLength; }

    // function
    bool Open(char*, int);
    void Close();
    void Write(unsigned char*, int);
    void Writeb(unsigned char);
    void Read();
    void Reset();
};

#endif // T_SERIAL_H
