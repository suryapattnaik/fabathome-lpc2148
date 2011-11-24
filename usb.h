#include <string.h>
#include "global.h"			// include our global project settings
#include "lpc2000.h"		// include LPC210x defines
#include "usbapi.h"
#include "buffer.h"

#define INT_IN_EP		0x81        //0b10000001
#define BULK_OUT_EP		0x05 | 0x02 //0b00000111
#define BULK_IN_EP		0x82        //0b10000010

#define MAX_PACKET_SIZE	64

#define LE_WORD(x)		((x)&0xFF),((x)>>8)

// CDC definitions
#define CS_INTERFACE			0x24
#define CS_ENDPOINT				0x25

#define	SET_LINE_CODING			0x20
#define	GET_LINE_CODING			0x21
#define	SET_CONTROL_LINE_STATE	0x22

#define EOF (-1)

#define UART_TX_BUFFER_SIZE	MAX_PACKET_SIZE
#define UART_RX_BUFFER_SIZE	MAX_PACKET_SIZE

#define	INT_VECT_NUM	0

void usbSetup(void);
int VCOM_getchar(void);
void VCOM_init(void);
int VCOM_putchar(int c);
void *usbPrintChar(unsigned char p);
void SendData2Host( BYTE str[], BYTE len );

void SendNextBulkIn(U8 bEP);

