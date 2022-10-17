//modified from http://blablacode.ru/programmirovanie/392
#ifndef SERIAL_DLL
#define SERIAL_DLL
//#include "stdafx.h"
//#include "targetver.h"
#include <string>
#include <Windows.h>
using namespace std;
class Serial
{
public:
    HANDLE hSerial;
    void  InitCOM(LPCTSTR );
    string  ReadCOM();
    void close();
    void write(char c);
};
#endif // SERIAL_H