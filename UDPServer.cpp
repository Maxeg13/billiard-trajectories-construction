#include<stdio.h>
#include<iostream>
using namespace std;
#include<winsock2.h>
#include <string>
#include <math.h>

#pragma comment(lib,"ws2_32.lib") //Winsock Library

#define BUFLEN 512	//Max length of buffer
#define PORT 8888	//The port on which to listen for incoming data
extern float main_rads;
extern float hue_rads;
int UDPServer()
{
    SOCKET s;
    struct sockaddr_in server, si_other;
    int slen , recv_len;
    char buf[BUFLEN];
    WSADATA wsa;

    slen = sizeof(si_other) ;

    //Initialise winsock
    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        printf("Failed. Error Code : %d",WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    printf("Initialised.\n");

    //Create a socket
    if((s = socket(AF_INET , SOCK_DGRAM , 0 )) == INVALID_SOCKET)
    {
        printf("Could not create socket : %d" , WSAGetLastError());
    }
    printf("Socket created.\n");

    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( PORT );

    //Bind
    if( bind(s ,(struct sockaddr *)&server , sizeof(server)) == SOCKET_ERROR)
    {
        printf("Bind failed with error code : %d" , WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    puts("Bind done");

    //keep listening for data
    while(1)
    {
        printf("Waiting for data...");
        fflush(stdout);

        //clear the buffer by filling null, it might have previously received data
        memset(buf,'\0', BUFLEN);

        //try to receive some data, this is a blocking call
        if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == SOCKET_ERROR)
        {
            printf("recvfrom() failed with error code : %d" , WSAGetLastError());
            exit(EXIT_FAILURE);
        }

        //print details of the client/peer and the data received
//        printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
//        printf("Data: %s\n" , buf);

        string s(buf);
        std::string delimiter = ",";
        size_t pos = 0;
        int cnt=0;
        std::string token;
        float gx, gy, gz, leny, lenx;
        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            if(cnt==10)
                gx=stof(token);
            if(cnt==11)
                gy=stof(token);
            s.erase(0, pos + delimiter.length());
            cnt++;
        }
        gz=stof(s);
//        std::cout << s << std::endl;
        cout<<endl<<gx<<"\t"<<gy<<"\t"<<gz<<endl;
        leny = (float)gx*gx + (float)gz*gz;
        leny = sqrt(leny);
        lenx = (float)gx*gx + (float)gy*gy;
        lenx = sqrt(lenx);
        if(fabs(gz)>0.1)
            main_rads = asin(gz/leny)-0.4;

        hue_rads = asin(gy/lenx);
        cout << "main angle: "<< main_rads * 180/3.14 << "\thue: "<<hue_rads * 180/3.14 << endl;

    }

    closesocket(s);
    WSACleanup();

    return 0;
}