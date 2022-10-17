#include "SimpleSerial.h"
//#include "stdafx.h"
#include <Windows.h>
#include <iostream>
//#include "targetver.h"
DWORD dwSize = 1;
DWORD dwBytesWritten;
LPOVERLAPPED ov;

void Serial::InitCOM(LPCTSTR sPortName)
{
    hSerial = ::CreateFile(sPortName,GENERIC_READ | GENERIC_WRITE,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
    if (hSerial == INVALID_HANDLE_VALUE)
        cout<<"serial port open error"<<endl;
    else
        cout<<"serial port open success"<<endl;

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        //    cout << "getting state error\n";
    }
    dcbSerialParams.BaudRate=CBR_115200;
    dcbSerialParams.ByteSize=8;
    dcbSerialParams.StopBits=ONESTOPBIT;
    dcbSerialParams.Parity=NOPARITY;
//    dcbSerialParams.
    if(!SetCommState(hSerial, &dcbSerialParams))
    {
        //    cout << "error setting serial port state\n";
    }
    char data[] = "H";
    dwSize = sizeof(data);
    dwBytesWritten;
    ov;

    DCB SetDCB;
    GetCommState(hSerial, &SetDCB);
    COMMTIMEOUTS timeouts;
    if(GetCommTimeouts(hSerial, &timeouts)) {
        cout<<"my timeout: "<<timeouts.ReadTotalTimeoutConstant<<endl;
        timeouts.ReadTotalTimeoutConstant = 10;
        SetCommTimeouts(hSerial, &timeouts);
    }
//    BOOL iRet = WriteFile (hSerial,TEXT("H"),dwSize,&dwBytesWritten ,NULL);
}

void Serial::write(char c)
{
    dwSize = 1;
    WriteFile (hSerial,&c,dwSize,&dwBytesWritten ,NULL);
}

string Serial::ReadCOM()
{
    string ret;
    DWORD iSize;
    char mes[10];
    DWORD bytes;
    if(ReadFile(hSerial, mes, 3, &iSize, 0)) {
//        cout << iSize << endl;
        ret.append(mes, iSize);
    }
    return ret;
}

void Serial::close()
{
    FindClose(hSerial);
}