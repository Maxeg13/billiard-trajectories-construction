//#include<stdio.h>
//#include<iostream>
//using namespace std;
//#include<winsock2.h>
//#include <string>
//#include <math.h>
//
//#pragma comment(lib,"ws2_32.lib") //Winsock Library
//
//#define BUFLEN 512	//Max length of buffer
//#define PORT 8888	//The port on which to listen for incoming data
//extern float main_rads;
//extern float hue_rads;
int UDPServer()
{
//    SOCKET s;
//    struct sockaddr_in server, si_other;
//    int slen , recv_len;
//    char buf[BUFLEN];
//    WSADATA wsa;
//
//    slen = sizeof(si_other) ;
//
//    //Initialise winsock
//    printf("\nInitialising Winsock...");
//    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
//    {
//        printf("Failed. Error Code : %d",WSAGetLastError());
//        exit(EXIT_FAILURE);
//    }
//    printf("Initialised.\n");
//
//    //Create a socket
//    if((s = socket(AF_INET , SOCK_DGRAM , 0 )) == INVALID_SOCKET)
//    {
//        printf("Could not create socket : %d" , WSAGetLastError());
//    }
//    printf("Socket created.\n");
//
//    //Prepare the sockaddr_in structure
//    server.sin_family = AF_INET;
//    server.sin_addr.s_addr = INADDR_ANY;
//    server.sin_port = htons( PORT );
//
//    //Bind
//    if( bind(s ,(struct sockaddr *)&server , sizeof(server)) == SOCKET_ERROR)
//    {
//        printf("Bind failed with error code : %d" , WSAGetLastError());
//        exit(EXIT_FAILURE);
//    }
//    puts("Bind done");
//
//    //keep listening for data
//    while(1)
//    {
//        printf("Waiting for data...");
//        fflush(stdout);
//
//        //clear the buffer by filling null, it might have previously received data
//        memset(buf,'\0', BUFLEN);
//
//        //try to receive some data, this is a blocking call
//        if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == SOCKET_ERROR)
//        {
//            printf("recvfrom() failed with error code : %d" , WSAGetLastError());
//            exit(EXIT_FAILURE);
//        }
//
//        //print details of the client/peer and the data received
//        printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
//        printf("Data: %s\n" , buf);
//
//        string s(buf);
//        std::string delimiter = ",";
//        size_t pos = 0;
//        int cnt=0;
//        std::string token;
//        float gx, gy, gz, leny, lenx;
//        while ((pos = s.find(delimiter)) != std::string::npos) {
//            token = s.substr(0, pos);
//            if(cnt==10)
//                gx=stof(token);
//            if(cnt==11)
//                gy=stof(token);
//            s.erase(0, pos + delimiter.length());
//            cnt++;
//        }
//        if(cnt==12)
//            gz=stof(s);
////        std::cout << s << std::endl;
//        cout<<endl<<gx<<"\t"<<gy<<"\t"<<gz<<endl;
//        leny = (float)gx*gx + (float)gz*gz;
//        leny = sqrt(leny);
//        lenx = (float)gx*gx + (float)gy*gy;
//        lenx = sqrt(lenx);
//        if(fabs(gz)>0.1)
//            main_rads = asin(gz/leny)-0.4;
//
//        hue_rads = asin(gy/lenx);
//        cout << "main angle: "<< main_rads * 180/3.14 << "\thue: "<<hue_rads * 180/3.14 << endl;
//
//    }
//
//    closesocket(s);
//    WSACleanup();

    return 0;
}

#include "curl/curl.h"
#include <string>
#include <iostream>
using namespace std;
float gx, gy, gz, leny, lenx;
extern float main_rads;
extern float hue_rads;


struct memory {
    char *response;
    size_t size;
};

static size_t cb(void *data, size_t size, size_t nmemb, void *userp)
{
    size_t realsize = size * nmemb;
    struct memory *mem = (struct memory *)userp;

    char *ptr = (char*)realloc(mem->response, mem->size + realsize + 1);
    if(ptr == NULL)
        return 0;  /* out of memory! */

    mem->response = ptr;
    memcpy(&(mem->response[mem->size]), data, realsize);
    mem->size += realsize;
    mem->response[mem->size] = 0;

    return realsize;
}

#include <math.h>
#include <vector>
#include <algorithm>
#include <thread>

void curl_main() {
    CURL *curl;
    CURLcode res;

    const string temp = "http://192.168.0.100:8080/sensors.json";

    curl = curl_easy_init();

    if(curl) {

        curl_easy_setopt(curl, CURLOPT_URL, temp.c_str());
        struct memory chunk = {0};

        /* send all data to this function  */
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, cb);

        /* we pass our 'chunk' struct to the callback function */
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);

        for(;;) {
            /* Perform the request, res will get the return code */
            res = curl_easy_perform(curl);
            /* Check for errors */
            if (res != CURLE_OK)
                fprintf(stderr, "curl_easy_perform() failed: %s\n",
                        curl_easy_strerror(res));
            string str(chunk.response + chunk.size - 40, 40 - 5);
            str.erase(std::remove(str.begin(), str.end(), '['), str.end());
            printf("\n\nmy data: %s\n", str.c_str());

            //tokenize
            vector<string> tokens;
            size_t start;
            size_t end = str.size();

            for (int step = 0; step < 3; step++) {
                start = str.find_last_of(",", end);
                start += 1;
                printf("%d\t%d\n", start, end);
                tokens.push_back(str.substr(start, end - start));
                end = start - 2;
                printf("%s\n", tokens.back().c_str());
            }
            gx = stof(tokens[2]);
            gy = stof(tokens[1]);
            gz = stof(tokens[0]);
            cout << endl << gx << "\t" << gy << "\t" << gz << endl;
            leny = (float) gx * gx + (float) gz * gz;
            leny = sqrt(leny);
            lenx = (float) gx * gx + (float) gy * gy;
            lenx = sqrt(lenx);
            if (fabs(gz) > 0.1)
                main_rads = asin(gz / leny)+0.1;

            hue_rads = -asin(gy / lenx);
            cout << "main angle: " << main_rads * 180 / 3.14 << "\thue: " << hue_rads * 180 / 3.14 << endl;
//
            /* always cleanup */
//            curl_easy_cleanup(curl);
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }
    }
}