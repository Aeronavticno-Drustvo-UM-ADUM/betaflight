/* Reference frames built with the firmware MAVLink headers, compared with mavlink_gen.py by check_mavlink_gen.sh */
#include <stdio.h>
#include <string.h>
#include <math.h>
#define cos_approx cosf
#define sin_approx sinf
#include "adum/mavlink.h"
static void dump(const char *n, mavlink_message_t *m){uint8_t b[300];int l=mavlink_msg_to_send_buffer(b,m);printf("%s:",n);for(int i=0;i<l;i++)printf("%02x",b[i]);printf("\n");}
int main(void){mavlink_message_t m;
 mavlink_msg_heartbeat_pack(1,1,&m,0,0,0x80,0,4); dump("hb",&m);
 uint16_t motor[3]={1500,1600,1700}, servo[8]={1100,1200,1300,1400,1500,65535,65535,65535};
 mavlink_msg_adum_aux_channels_pack(1,1,&m,1,motor,servo); dump("aux",&m);
 mavlink_msg_adum_gps_pvt_pack(1,1,&m,1700000000123456ULL,473977000,85432100,430000,477000,1000,-2000,300,12345678,2500,3100,450,123,6,17,7); dump("pvt",&m);return 0;}
