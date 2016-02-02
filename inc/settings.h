#ifndef SETTINGS_H_INCLUDED
#define SETTINGS_H_INCLUDED

#define SETTINGS_CNT    10

unsigned short getParam(unsigned short num);
void setParam(unsigned short addr, unsigned short value);
void read_settings(void);

#endif /* SETTINGS_H_INCLUDED */
