#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef int socklen_t;
    #define close closesocket
    #define usleep(us) Sleep((us)/1000)
#else
    #include <unistd.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
#endif

#include "common/mavlink.h"

#define TCP_PORT 10000
#define MAVLINK_PORT 14550
#define BUFFER_SIZE 2048

static double last_lat = 0, last_lon = 0, last_alt = 0;
static double last_speed = 0, last_course = 0;
static int last_sats = 0;
static int has_fix = 0;

unsigned char nmea_checksum(const char *sentence) {
    unsigned char cs = 0;
    const char *p = sentence;
    if (*p == '$') p++;
    while (*p && *p != '*') {
        cs ^= *p;
        p++;
    }
    return cs;
}

void format_latlon(double val, int is_lat, char *out, char *dir) {
    int deg = (int)fabs(val);
    double min = (fabs(val) - deg) * 60.0;
    if (is_lat) {
        sprintf(out, "%02d%07.4f", deg, min);
        *dir = (val >= 0) ? 'N' : 'S';
    } else {
        sprintf(out, "%03d%07.4f", deg, min);
        *dir = (val >= 0) ? 'E' : 'W';
    }
}

void send_nmea(int client_fd) {
    if (!has_fix) return;
    
    time_t now = time(NULL);
    struct tm *tm = gmtime(&now);
    
    char time_str[16], date_str[16];
    sprintf(time_str, "%02d%02d%02d.00", tm->tm_hour, tm->tm_min, tm->tm_sec);
    sprintf(date_str, "%02d%02d%02d", tm->tm_mday, tm->tm_mon + 1, tm->tm_year % 100);
    
    char lat_str[16], lon_str[16];
    char lat_dir, lon_dir;
    format_latlon(last_lat, 1, lat_str, &lat_dir);
    format_latlon(last_lon, 0, lon_str, &lon_dir);
    
    double speed_knots = last_speed * 1.94384;
    
    char gga[256];
    sprintf(gga, "$GPGGA,%s,%s,%c,%s,%c,1,%02d,0.9,%.1f,M,0.0,M,,",
            time_str, lat_str, lat_dir, lon_str, lon_dir, last_sats, last_alt);
    unsigned char cs_gga = nmea_checksum(gga);
    char gga_full[256];
    sprintf(gga_full, "%s*%02X\r\n", gga, cs_gga);
    
    char rmc[256];
    sprintf(rmc, "$GPRMC,%s,A,%s,%c,%s,%c,%.1f,%.1f,%s,,",
            time_str, lat_str, lat_dir, lon_str, lon_dir, speed_knots, last_course, date_str);
    unsigned char cs_rmc = nmea_checksum(rmc);
    char rmc_full[256];
    sprintf(rmc_full, "%s*%02X\r\n", rmc, cs_rmc);
    
    send(client_fd, gga_full, (int)strlen(gga_full), 0);
    send(client_fd, rmc_full, (int)strlen(rmc_full), 0);
}

void parse_mavlink(const uint8_t *buf, int len) {
    mavlink_message_t msg;
    mavlink_status_t status;
    
    for (int i = 0; i < len; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
                mavlink_gps_raw_int_t gps;
                mavlink_msg_gps_raw_int_decode(&msg, &gps);
                
                if (gps.fix_type >= 3) {
                    last_lat = gps.lat / 1e7;
                    last_lon = gps.lon / 1e7;
                    last_alt = gps.alt / 1000.0;
                    last_speed = gps.vel / 100.0;
                    last_course = gps.cog / 100.0;
                    last_sats = gps.satellites_visible;
                    has_fix = 1;
                }
            }
        }
    }
}

int main() {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        fprintf(stderr, "WSAStartup failed\n");
        return 1;
    }
#endif

    printf("=== MAVLink to NMEA Bridge ===\n");
    
    int mav_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (mav_sock < 0) {
        perror("MAVLink socket failed");
        return 1;
    }
    
    struct sockaddr_in mav_addr;
    memset(&mav_addr, 0, sizeof(mav_addr));
    mav_addr.sin_family = AF_INET;
    mav_addr.sin_port = htons(MAVLINK_PORT);
    mav_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    
    if (bind(mav_sock, (struct sockaddr *)&mav_addr, sizeof(mav_addr)) < 0) {
        perror("MAVLink bind failed");
        close(mav_sock);
        return 1;
    }
    printf("Listening MAVLink on UDP:%d\n", MAVLINK_PORT);
    
    int tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_sock < 0) {
        perror("TCP socket failed");
        close(mav_sock);
        return 1;
    }
    
    int opt = 1;
    setsockopt(tcp_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));
    
    struct sockaddr_in tcp_addr;
    memset(&tcp_addr, 0, sizeof(tcp_addr));
    tcp_addr.sin_family = AF_INET;
    tcp_addr.sin_port = htons(TCP_PORT);
    tcp_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(tcp_sock, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr)) < 0) {
        perror("TCP bind failed");
        close(mav_sock);
        close(tcp_sock);
        return 1;
    }
    
    listen(tcp_sock, 1);
    printf("TCP server on port %d. Waiting for Alpine Quest...\n", TCP_PORT);
    
    int client_fd = -1;
    uint8_t buf[BUFFER_SIZE];
    fd_set readfds;
    
    while (1) {
        FD_ZERO(&readfds);
        FD_SET(mav_sock, &readfds);
        int max_fd = mav_sock;
        
        if (client_fd >= 0) {
            FD_SET(client_fd, &readfds);
            if (client_fd > max_fd) max_fd = client_fd;
        }
        
        struct timeval tv = {1, 0};
        int activity = select(max_fd + 1, &readfds, NULL, NULL, &tv);
        
        if (activity < 0) {
            perror("select error");
            break;
        }
        
        if (FD_ISSET(mav_sock, &readfds)) {
            int len = recvfrom(mav_sock, (char*)buf, BUFFER_SIZE, 0, NULL, NULL);
            if (len > 0) {
                parse_mavlink(buf, len);
            }
        }
        
        if (client_fd < 0 && FD_ISSET(tcp_sock, &readfds)) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            client_fd = accept(tcp_sock, (struct sockaddr *)&client_addr, &client_len);
            if (client_fd >= 0) {
                printf("Alpine Quest connected! IP: %s\n", inet_ntoa(client_addr.sin_addr));
            }
        }
        
        if (client_fd >= 0 && has_fix) {
            send_nmea(client_fd);
            usleep(200000);
        }
    }
    
    close(mav_sock);
    if (client_fd >= 0) close(client_fd);
    close(tcp_sock);
    
#ifdef _WIN32
    WSACleanup();
#endif
    
    return 0;
}