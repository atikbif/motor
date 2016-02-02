#ifndef MODBUS_H_INCLUDED
#define MODBUS_H_INCLUDED

typedef struct
{
	unsigned short addr;
	unsigned long laddr;
	unsigned short cnt;
	unsigned char* tx_buf;
	unsigned char* rx_buf;
	unsigned char can_name;
	unsigned char mode;
}request;

unsigned short read_holdregs(request* req);
unsigned short write_single_reg(request* req);
unsigned short write_multi_regs(request* req);

#endif /* MODBUS_H_INCLUDED */
