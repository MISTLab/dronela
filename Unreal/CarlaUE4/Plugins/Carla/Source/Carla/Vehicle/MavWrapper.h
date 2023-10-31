#pragma once

#include <iostream>
#include <mavlink/common/mavlink.h>
#include <mavlink/common/common.h>
#include <iomanip>

// Assuming you're using POSIX sockets for the TCP connection
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include <chrono>
#include <cmath>
#include<thread>
#include <random>


#include "MavWrapper.generated.h"

UCLASS()
class CARLA_API UMavlinkConnection: public UObject
{
    GENERATED_BODY()

private:
    int sockfd;
    int newsockfd;
 
public:
    UMavlinkConnection();
    ~UMavlinkConnection();

    void connect();
    bool receiveAndCheckMessage(int expected_msg_id);
    void checkIncomingMessages(bool blocking = false);

    void sendSystemTimeMessage(uint64_t t_abs__us, uint64_t t_boot__us);
    void sendHILGPSMessage(uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, 
                        uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, 
                        int16_t vd, uint16_t cog, uint8_t satellites_visible, uint8_t the_id, float yaw);
    void sendHILSensorMessage(uint64_t time_usec, float xacc, float yacc, float zacc, 
                            float xgyro, float ygyro, float zgyro,
                            float xmag, float ymag, float zmag,
                            float abs_pressure, float diff_pressure, float pressure_alt,
                            float temperature, uint32_t fields_updated, uint8_t the_id);
    void sendHeartbeatMessage(
        uint8_t the_type,
        uint8_t autopilot,
        uint8_t base_mode,
        uint32_t custom_mode,
        uint8_t system_status);

    mavlink_message_t receiveMessage(bool blocking = false);
    
};

