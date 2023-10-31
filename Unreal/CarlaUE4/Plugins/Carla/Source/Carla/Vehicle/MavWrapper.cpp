#include "MavWrapper.h"



UMavlinkConnection::UMavlinkConnection() {}

void UMavlinkConnection::connect() {
    sockaddr_in serv_addr = {};
    sockfd = socket(AF_INET, SOCK_STREAM, 0); // Change to SOCK_STREAM for TCP
    if (sockfd < 0) {
       
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(4560);
    serv_addr.sin_addr.s_addr = INADDR_ANY;  // Listen on any address

    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        close(sockfd);  
    }

    listen(sockfd, 5); // Listen with a backlog of 5

    struct sockaddr_in cli_addr;
    socklen_t clilen = sizeof(cli_addr);
    UMavlinkConnection::newsockfd = accept(UMavlinkConnection::sockfd, (struct sockaddr *) &cli_addr, &clilen);

    if (newsockfd < 0) {
    
        close(sockfd);
        
    }

    mavlink_message_t msg;
    mavlink_status_t status;

    if (!receiveAndCheckMessage(MAVLINK_MSG_ID_COMMAND_LONG)) {
        
    }

    if (!receiveAndCheckMessage(MAVLINK_MSG_ID_HEARTBEAT)) {
      
    }
    int flags = fcntl(newsockfd, F_GETFL, 0);
    fcntl(newsockfd, F_SETFL, flags | O_NONBLOCK);


}



void UMavlinkConnection::sendSystemTimeMessage(uint64_t t_abs__us, uint64_t t_boot__us) {
    uint64_t since_boot__us = t_abs__us - t_boot__us;
    uint32_t since_boot__ms = static_cast<uint32_t>(round(since_boot__us / 1000.0));

    uint64_t time_unix_usec = t_abs__us;
    uint32_t time_boot_ms = since_boot__ms;

    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_system_time_pack(
        1,  // system_id: 1 as an example. Replace with appropriate value.
        0,  // component_id: 0 as an example. Replace with appropriate value.
        &msg,
        time_unix_usec,
        time_boot_ms
    );

    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    if (write(newsockfd, buffer, len) != len) {
        std::cerr << "Failed to send system time message" << std::endl;
    }

    std::cout << "--> SYSTEM_TIME {"
                << " time_unix_usec: " << time_unix_usec
                << ", time_boot_ms: " << time_boot_ms
                << " }" << std::endl;
}

bool UMavlinkConnection::receiveAndCheckMessage(int expected_msg_id) {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;
    while (read(newsockfd, &byte, 1) > 0) { // Changed sockfd to newsockfd here
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            std::cout << "<== " << msg.msgid << std::endl;
            return true;
        }
    }
    return false;
}


void UMavlinkConnection::sendHILGPSMessage(uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, 
                    uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, 
                    int16_t vd, uint16_t cog, uint8_t satellites_visible, uint8_t the_id, float yaw) {
    mavlink_hil_gps_t hil_gps = {};

    // Populate hil_gps with the provided data
    hil_gps.time_usec = time_usec;
    hil_gps.fix_type = fix_type;
    hil_gps.lat = lat;
    hil_gps.lon = lon;
    hil_gps.alt = alt;
    hil_gps.eph = eph;
    hil_gps.epv = epv;
    hil_gps.vel = vel;
    hil_gps.vn = vn;
    hil_gps.ve = ve;
    hil_gps.vd = vd;
    hil_gps.cog = cog;
    hil_gps.satellites_visible = satellites_visible;
    hil_gps.id = the_id;
    hil_gps.yaw = yaw;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Encode the message
    mavlink_msg_hil_gps_encode(1, 200, &msg, &hil_gps);

    // Serialize the message
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message over the socket
    ssize_t bytes_sent = send(newsockfd, buf, len, 0);
    if (bytes_sent < 0) {
        std::cerr << "ERROR sending HIL_GPS message" << std::endl;
    }
}


void UMavlinkConnection::sendHILSensorMessage(uint64_t time_usec, float xacc, float yacc, float zacc, 
                        float xgyro, float ygyro, float zgyro,
                        float xmag, float ymag, float zmag,
                        float abs_pressure, float diff_pressure, float pressure_alt,
                        float temperature, uint32_t fields_updated, uint8_t the_id) {
    mavlink_hil_sensor_t hil_sensor = {};

    // Populate hil_sensor with the provided data
    hil_sensor.time_usec = time_usec;
    hil_sensor.xacc = xacc;
    hil_sensor.yacc = yacc;
    hil_sensor.zacc = zacc;
    hil_sensor.xgyro = xgyro;
    hil_sensor.ygyro = ygyro;
    hil_sensor.zgyro = zgyro;
    hil_sensor.xmag = xmag;
    hil_sensor.ymag = ymag;
    hil_sensor.zmag = zmag;
    hil_sensor.abs_pressure = abs_pressure;
    hil_sensor.diff_pressure = diff_pressure;
    hil_sensor.pressure_alt = pressure_alt;
    hil_sensor.temperature = temperature;
    hil_sensor.fields_updated = fields_updated;
    hil_sensor.id = the_id;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Encode the message
    mavlink_msg_hil_sensor_encode(1, 200, &msg, &hil_sensor);

    // Serialize the message
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message over the socket
    ssize_t bytes_sent = send(newsockfd, buf, len, 0);
    if (bytes_sent < 0) {
        std::cerr << "ERROR sending HIL_SENSOR message" << std::endl;
    }
}




void UMavlinkConnection::sendHeartbeatMessage(
    uint8_t the_type,
    uint8_t autopilot,
    uint8_t base_mode,
    uint32_t custom_mode,
    uint8_t system_status) 
    {
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        1,  // system_id: 1 as an example. Replace with appropriate value.
        0,  // component_id: 0 as an example. Replace with appropriate value.
        &msg,
        the_type,
        autopilot,
        base_mode,
        custom_mode,
        system_status
    );

    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    if (write(newsockfd, buffer, len) != len) {
        std::cerr << "Failed to send heartbeat message" << std::endl;
    }
}




mavlink_message_t UMavlinkConnection::receiveMessage(bool blocking) {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;
    while (true) {
        ssize_t res = read(newsockfd, &byte, 1);
        if (res <= 0) {
            if (!blocking) return {};  // If non-blocking and no data, return an empty message
            continue;  // If blocking, continue trying to read
        }

        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            return msg;  // Return the parsed message
        }
    }
}

void UMavlinkConnection::checkIncomingMessages(bool blocking) {
    mavlink_message_t msg = receiveMessage(blocking);
    if (msg.msgid != 0) {
        std::cout << "<== " << " (" << msg.msgid << ")" << std::endl;
    }
}



UMavlinkConnection::~UMavlinkConnection() {
    close(newsockfd);
    close(sockfd);
}


