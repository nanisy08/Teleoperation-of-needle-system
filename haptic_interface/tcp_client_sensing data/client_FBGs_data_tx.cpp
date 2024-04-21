/*
File    : Client_I4 Interrogator.cpp
Author  : Sooyeon Kim
Date    : June 06, 2023
Update  : April 14, 2024
Description : C++98
Protocol    : TCP/IP Client for I4 Interrogator

This C++ program acts as a client for an I4 Interrogator device, communicating over TCP/IP.
It receives data packets from the I4 device, processes them, and sends the processed data
to a main server. The packets include header information, payload data (peak or timestamped peaks),
and error information. The program then sends the processed data to a main server.
*/


#include <WinSock2.h>
#include <Ws2tcpip.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <conio.h> // For _kbhit and _getch
#include <stdint.h>

#include <thread>
#include <chrono>

#pragma comment(lib, "ws2_32.lib")

#define PORT 4578
#define PACKET_SIZE 11  // int8_t 3개, double 1개 송신
#define SERVER_IP "192.168.0.140"

#define PORT_I4 9931
#define SERVER_I4_IP "10.100.51.16"

// I4 packet size
#define HEADER_SIZE 16
#define ERROR_PAYLOAD_SIZE 8
#define PEAK_PAYLOAD_SIZE 8
#define TSPEAK_PAYLOAD_SIZE 12
#define FLAG_SIZE 8

#pragma pack(1)
struct ts_peak_payload_t {
    uint32_t ts_peak_payload_LSB : 32;
    uint32_t ts_peak_payload_MSB : 32;
    uint32_t time_stamp : 32;
};
struct peak_payload_t {
    uint32_t peak_payload_LSB : 32;
    uint32_t peak_payload_MSB : 32;
};
struct error_payload_t {
    uint32_t error_id : 32;
    uint32_t error_description : 32;
};
struct I4PacketHeader {
    uint16_t info : 16; // packetCounter(12) + sweepingType(3) + triggerMode(1)
    uint16_t dataOffset : 16;
    uint32_t dataLength : 32;
    uint64_t timeStamp : 64;
};
struct I4PacketFlag {
    uint32_t sweep_counter : 32;
    uint32_t reserved : 32;
};
#pragma pack()

// function redefinition
typedef uint32_t peak_data_t[2];
typedef uint32_t ts_peak_data_t[3];

int processPacket_Header(char* buffer_header, int* sweep_type, int* DO, int* DL);
int processPacket_Payload(char* buffer_payload, uint8_t* channel, uint8_t* fiber, uint8_t* sensor, double* wavelength_data);
int processPacket_tsPayload(char* buffer_payload, uint8_t* channel, uint8_t* fiber, uint8_t* sensor, double* wavelength_data);
int processPacket_errorPayload(char* error_payload);

double wavelength(const uint32_t* const peak_data);
uint8_t channel_id(const uint32_t* const peak_data);
uint8_t fiber_id(const uint32_t* const peak_data);
uint8_t sensor_id(const uint32_t* const peak_data);
double time_stamp(const uint32_t* const peak_data);


/* =============================================================================
 *
 * Main Function
 *
 * =============================================================================
 */

int main() {
    /*****************************************/
    /**** Initialize TCP/IP communication ****/
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        fprintf(stderr, "WSAStartup failed.\n");
        return 1;
    }

    SOCKET hSocket, hSocket_I4;
    hSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (hSocket == INVALID_SOCKET) {
        fprintf(stderr, "Socket creation failed for main server.\n");
        WSACleanup();
        return 1;
    }
    hSocket_I4 = socket(AF_INET, SOCK_STREAM, 0);
    if (hSocket_I4 == INVALID_SOCKET) {
        fprintf(stderr, "Socket creation failed for I4.\n");
        closesocket(hSocket);
        WSACleanup();
        return 1;
    }

    SOCKADDR_IN serverAddr, serverAddr_I4;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) <= 0) {
        fprintf(stderr, "inet_pton failed.\n");
        closesocket(hSocket);
        WSACleanup();
        return 1;
    }
    if (connect(hSocket, (SOCKADDR*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        fprintf(stderr, "Connection failed for main server.\n");
        closesocket(hSocket);
        WSACleanup();
        return 1;
    }
    printf("Connected to main server\n");

    serverAddr_I4.sin_family = AF_INET;
    serverAddr_I4.sin_port = htons(PORT_I4);
    if (inet_pton(AF_INET, SERVER_I4_IP, &serverAddr_I4.sin_addr) <= 0) {
        fprintf(stderr, "inet_pton failed.\n");
        closesocket(hSocket_I4);
        WSACleanup();
        return 1;
    }
    if (connect(hSocket_I4, (SOCKADDR*)&serverAddr_I4, sizeof(serverAddr_I4)) == SOCKET_ERROR) {
        fprintf(stderr, "Connection failed for I4.\n");
        closesocket(hSocket_I4);
        WSACleanup();
        return 1;
    }
    printf("Connected to I4\n");

    /*****************************************************************/
    /**** Receiving data from I4, and sending data to main server ****/

    while (1) {
        if (_kbhit()) { // Loop until ESC key is pressed
            int ch = _getch();
            if (ch == 27) {
                printf("Exiting program.\n");
                closesocket(hSocket);
                closesocket(hSocket_I4);
                break;
            }
        }

        /* 1. Receiving header packet */
        char buffer_header[HEADER_SIZE] = { 0 };
        int hbytesRead = recv(hSocket_I4, buffer_header, HEADER_SIZE, 0);

        if (hbytesRead == SOCKET_ERROR) {
            perror("Receive failed");
        }
        else if (hbytesRead == 0) {
            printf("Client disconnected\n");
        }
        if (hbytesRead != HEADER_SIZE) {
            perror("Packet header size error");
        }

        int sweep_type, DO, DL; // offset for error handling
        processPacket_Header(buffer_header, &sweep_type, &DO, &DL);


        uint8_t channel = 0, fiber = 0, sensor = 0;
        double wavelength_data = 0, force = 0;


        /* 2. Receiving payload packet */
        if (DO == 16) { // or 0X0010
            // receiving peak payload
            if (sweep_type == 0) { // peak
                char buffer_payload[PEAK_PAYLOAD_SIZE] = { 0 };
                for (int i = 0; i < (DL / PEAK_PAYLOAD_SIZE); i++) {
                    int pbytesRead = recv(hSocket_I4, buffer_payload, PEAK_PAYLOAD_SIZE, 0);

                    if (pbytesRead == SOCKET_ERROR) {
                        perror("Receive failed");
                    }
                    else if (pbytesRead == 0) {
                        printf("Client disconnected\n");
                    }
                    processPacket_Payload(buffer_payload, &channel, &fiber, &sensor, &wavelength_data);

                    uint8_t int_data[3] = { channel, fiber, sensor };
                    char cBuffer[PACKET_SIZE];
                    //memset(cBuffer, 0, PACKET_SIZE);

                    memcpy(cBuffer, int_data, sizeof(int_data));
                    memcpy(cBuffer + sizeof(int_data), &wavelength_data, sizeof(wavelength_data));
                    send(hSocket, cBuffer, PACKET_SIZE, 0);

                    printf("Sent data %u - Channel#%u, Sensor#%u, Wavelength: %.5f nm\n",
                        fiber, channel, sensor, wavelength_data);

                }
            }
            // receiving time-stamped peak payload
            else if (sweep_type == 2) { // peak with timestamps
                char buffer_payload[TSPEAK_PAYLOAD_SIZE] = { 0 };

                for (int i = 0; i < (DL / TSPEAK_PAYLOAD_SIZE); i++) {
                    int pbytesRead = recv(hSocket_I4, buffer_payload, TSPEAK_PAYLOAD_SIZE, 0);

                    if (pbytesRead == SOCKET_ERROR) {
                        perror("Receive failed");
                    }
                    else if (pbytesRead == 0) {
                        printf("Client disconnected\n");
                    }
                    processPacket_Payload(buffer_payload, &channel, &fiber, &sensor, &wavelength_data);

                    //printf("sensor#%u, ", sensor);
                    //printf("fiber#%u, ", fiber);
                    //printf("channel#%u\t", channel);
                    //printf("wavelength::%.3f nm\n", wavelength_data);

                    uint8_t int_data[3] = { channel, fiber, sensor };
                    char cBuffer[PACKET_SIZE];
                    //memset(cBuffer, 0, PACKET_SIZE);

                    memcpy(cBuffer, int_data, sizeof(int_data));
                    memcpy(cBuffer + sizeof(int_data), &wavelength_data, sizeof(wavelength_data));
                    send(hSocket, cBuffer, PACKET_SIZE, 0);

                    printf("Sent data %u - Channel#%u, Sensor#%u, Wavelength: %.5f nm\n",
                        fiber, channel, sensor, wavelength_data);

                }
            }
        }

        else { // if error exists..
            // receiving error payload
            char error_payload[ERROR_PAYLOAD_SIZE] = { 0 };
            int ebytesRead = recv(hSocket_I4, error_payload, ERROR_PAYLOAD_SIZE, 0);
            if (ebytesRead == SOCKET_ERROR) {
                perror("Error receive failed");
            }
            else if (ebytesRead == 0) {
                printf("Client disconnected\n");
            }
            processPacket_errorPayload(error_payload);

            // receiving peak payload
            if (sweep_type == 0) { // peak
                char buffer_payload[PEAK_PAYLOAD_SIZE] = { 0 };
                for (int i = 0; i < (DL / PEAK_PAYLOAD_SIZE); i++) {
                    int pbytesRead = recv(hSocket_I4, buffer_payload, PEAK_PAYLOAD_SIZE, 0);

                    if (pbytesRead == SOCKET_ERROR) {
                        perror("Receive failed");
                    }
                    else if (pbytesRead == 0) {
                        printf("Client disconnected\n");
                    }
                    processPacket_Payload(buffer_payload, &channel, &fiber, &sensor, &wavelength_data);

                    uint8_t int_data[3] = { channel, fiber, sensor };
                    char cBuffer[PACKET_SIZE];
                    //memset(cBuffer, 0, PACKET_SIZE);

                    memcpy(cBuffer, int_data, sizeof(int_data));
                    memcpy(cBuffer + sizeof(int_data), &wavelength_data, sizeof(wavelength_data));
                    send(hSocket, cBuffer, PACKET_SIZE, 0);

                    printf("Sent data %u - Channel#%u, Sensor#%u, Wavelength: %.5f nm\n",
                        fiber, channel, sensor, wavelength_data);

                }
            }
            // receiving time-stamped peak payload
            else if (sweep_type == 2) { // peak with timestamps
                char buffer_payload[TSPEAK_PAYLOAD_SIZE] = { 0 };

                for (int i = 0; i < (DL / TSPEAK_PAYLOAD_SIZE); i++) {
                    int pbytesRead = recv(hSocket_I4, buffer_payload, TSPEAK_PAYLOAD_SIZE, 0);

                    if (pbytesRead == SOCKET_ERROR) {
                        perror("Receive failed");
                    }
                    else if (pbytesRead == 0) {
                        printf("Client disconnected\n");
                    }
                    processPacket_Payload(buffer_payload, &channel, &fiber, &sensor, &wavelength_data);

                    uint8_t int_data[3] = { channel, fiber, sensor };
                    char cBuffer[PACKET_SIZE];
                    //memset(cBuffer, 0, PACKET_SIZE);

                    memcpy(cBuffer, int_data, sizeof(int_data));
                    memcpy(cBuffer + sizeof(int_data), &wavelength_data, sizeof(wavelength_data));
                    send(hSocket, cBuffer, PACKET_SIZE, 0);

                    printf("Sent data %u - Channel#%u, Sensor#%u, Wavelength: %.5f nm\n",
                        fiber, channel, sensor, wavelength_data);

                }
            }
        }

        /* 3. Receiving flag packet */
        char buffer_flag[FLAG_SIZE] = { 0 };
        int fbytesRead = recv(hSocket_I4, buffer_flag, FLAG_SIZE, 0);

        if (fbytesRead == SOCKET_ERROR) {
            perror("Flag receive failed");
        }
        else if (fbytesRead == 0) {
            printf("Client disconnected\n");
        }

    }

    // Close TCP/IP communication
    WSACleanup();

    return 0;
}



/* =============================================================================
* Function Descriptions :
* ------------------------------------------------------------------------------
* processPacket_Header : Extracts packet header info(counter, type, offset, length).
* processPacket_tsPayload : Extracts timestamp payload info(channel, fiber, sensor, wavelength).
* processPacket_Payload : Extracts peak payload info(channel, fiber, sensor, wavelength).
* processPacket_errorPayload : Processes error payload and provides error details.
* wavelength : Extracts wavelength in meters from peak data.
* channel_id, fiber_id, sensor_id : Extract respective IDs from peak data.
* time_stamp : Extracts time stamp in seconds from peak data.
* ==============================================================================
*/

int processPacket_Header(char* buffer_header, int* sweep_type, int* DO, int* DL) {
    struct I4PacketHeader* header = (struct I4PacketHeader*)(buffer_header);

    // Packet Header (bit field mask)
    uint16_t packetCounterMask = 0xFFF; // lower 12 bits
    uint16_t sweepingTypeMask = 0x7000; // middle 3 bits
    uint16_t triggerModeMask = 0x8000;  // top 1 bit

    uint16_t packetCounter = header->info & packetCounterMask;
    uint16_t sweepingType = (header->info & sweepingTypeMask) >> 12;
    uint16_t triggerMode = (header->info & triggerModeMask) >> 15;

    uint16_t dataOffset = header->dataOffset; // 0~65535 byte offset
    uint32_t dataLength = header->dataLength; // bytes
    uint32_t totalPacketSize = dataOffset + dataLength + 8;

    *sweep_type = sweepingType;
    *DO = dataOffset;
    *DL = dataLength;

    return 0;
}

int processPacket_tsPayload(char* buffer_payload, uint8_t* channel, uint8_t* fiber, uint8_t* sensor, double* wavelength_data) {
    // 초기화
    memset(channel, 0, sizeof(uint8_t));
    memset(fiber, 0, sizeof(uint8_t));
    memset(sensor, 0, sizeof(uint8_t));
    memset(wavelength_data, 0, sizeof(double));
    
    struct ts_peak_payload_t* ts_payload = (struct ts_peak_payload_t*)(buffer_payload);

    ts_peak_data_t peak_data;
    peak_data[0] = ts_payload->ts_peak_payload_LSB;
    peak_data[1] = ts_payload->ts_peak_payload_MSB;
    peak_data[2] = ts_payload->time_stamp;

    *channel = channel_id(peak_data);
    *fiber = fiber_id(peak_data);
    *sensor = sensor_id(peak_data);
    *wavelength_data = 1e9 * wavelength(peak_data); //nm

    return 0;
}

int processPacket_Payload(char* buffer_payload, uint8_t* channel, uint8_t* fiber, uint8_t* sensor, double* wavelength_data) {
    // 초기화
    memset(channel, 0, sizeof(uint8_t));
    memset(fiber, 0, sizeof(uint8_t));
    memset(sensor, 0, sizeof(uint8_t));
    memset(wavelength_data, 0, sizeof(double));
    
    struct peak_payload_t* payload = (struct peak_payload_t*)(buffer_payload);

    peak_data_t peak_data;
    peak_data[0] = payload->peak_payload_LSB;
    peak_data[1] = payload->peak_payload_MSB;

    *channel = channel_id(peak_data);
    *fiber = fiber_id(peak_data);
    *sensor = sensor_id(peak_data);
    *wavelength_data = 1e9 * wavelength(peak_data); //nm

    return 0;
}

int processPacket_errorPayload(char* error_payload) {
    struct error_payload_t* payload = (struct error_payload_t*)(error_payload);

    peak_data_t data;
    data[1] = payload->error_id;
    data[0] = payload->error_description;

    if (data[1] == 500) {
        printf("Error type : Missing Peak\n");
        printf("Sensor #%u, Fiber #%u, Channel #%u\n",
            sensor_id(data), fiber_id(data), channel_id(data));
        printf("Error source: No peak was detected where one was "
            "expected from the interrogator configuration.\n"
            "Possible causes include :\n"
            "* Misconfiguration of sensor wavelength range or threshold\n"
            "* disconnected sensor\n");
    }
    else if (data[1] == 501) {
        printf("Error type : Multiple Peaks\n");
        printf("Sensor #%u, Fiber #%u, Channel #%u\n",
            sensor_id(data), fiber_id(data), channel_id(data));
        printf("Error source : More than the expected number of peaks was "
            "detected in the sensors wavelength range.\n"
            "Possible causes include :\n"
            "* Misconfiguration of sensor wavelength range or threshold\n");
    }
    else {
        printf("Error type : Internal Error\n");
        printf("Error source: Internal error\n"
            "Possible causes include :\n"
            "* Transient mismatch of configuration and data stream\n"
            "* Internal failure\n");
    }

    return 0;
}

/* extract wavelength value in meters */
double wavelength(const uint32_t* const peak_data) {
    uint32_t peak_value[2];
    peak_value[0] = (peak_data[0] & ~0xffff) | 0x7fff; /* mask out id bits */
    peak_value[1] = peak_data[1];
    return *(double*)&peak_value;
}

uint8_t channel_id(const uint32_t* const peak_data) {
    return (uint8_t)(peak_data[0] >> 12) & 0x0f;
}

uint8_t fiber_id(const uint32_t* const peak_data) {
    return (uint8_t)(peak_data[0] >> 8) & 0x0f;
}

uint8_t sensor_id(const uint32_t* const peak_data) {
    return (uint8_t)(peak_data[0] & 0xff);
}

/* extract time stamp value in seconds */
double time_stamp(const uint32_t* const peak_data) {
    return (double)peak_data[2] * 5e-10;
}
