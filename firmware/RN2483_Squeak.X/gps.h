#ifndef GPS_H
#define	GPS_H

void gps_initialize();
inline void gps_parse(uint8_t data);
void gps_sleep();


#endif