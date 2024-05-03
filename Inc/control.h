
#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "system.h"
#include "USART.h"

// ���������
#define VARR_STATE      0      // R: ���������
//--------------------------------------------------
//#define CMD_PAR         20  // 0x14: �������� ��������� (����� �� ������)
#define CMD_SET_PAR     21  // 0x15: ��������� ���������
#define CMD_GET_PAR     22  // 0x16: ������ �������� ���������

#define CMD_RESET		0		// ����� �������� CMD
//--------------------------------------------------
#define STATUS_NULL  	    0x0000	// Null
#define STATUS_LVL1_SET    	0x0001	// �������� ������� 1
#define STATUS_LVL2_SET 	0x0002	// �������� ������� 2
#define STATUS_LVL3_SET  	0x0004	// �������� ������� 3
#define STATUS_LVL4_SET    	0x0008	// �������� ������� 4
#define STATUS_LVL_ON  	    0x0010	// �������� ������� ON
#define STATUS_LVL_OFF 	    0x0020	// �������� ������� OFF
#define STATUS_LVL1_SHOW   	0x0100	// �������� ������� 1
#define STATUS_LVL2_SHOW 	0x0200	// �������� ������� 2
#define STATUS_LVL3_SHOW 	0x0400	// �������� ������� 3
#define STATUS_LVL4_SHOW   	0x0800	// �������� ������� 4
//--------------------------------------------------
#define PAR_STATE  		0x0001	//0 ������ ��������� 0


// ���������� ��������
enum CONTROL_VAR
{
	CONTROL_STAT,			// ���������
	CONTROL_LVLNET1,		// ������� ext 1
	CONTROL_LVLNET2,		// ������� ext 2
	CONTROL_LVLNET3,		// ������� ext 3
	CONTROL_LVLNET4,		// ������� ext 4
	CONTROL_CHAN1,			// ������� 1
	CONTROL_CHAN2,			// ������� 2
	CONTROL_CHAN3,			// ������� 3
	CONTROL_CHAN4,			// ������� 4
	CONTROL_BAT,			// Battery level (V)

	CONTROL_VAR_NUM    		// ���������� ����������
};
// ���������� ��������
enum PARAM_VAR
{
	PARAM_IDSEND1=0,	// ID ���������� 1
	PARAM_IDSEND2,		// ID ���������� 2
	PARAM_IDSEND3,		// ID ���������� 3
	PARAM_IDSEND4,		// ID ���������� 4
	PARAM_UNITSND1,		// ���� ���������� 1
	PARAM_UNITSND2,		// ���� ���������� 2
	PARAM_UNITSND3,		// ���� ���������� 3
	PARAM_UNITSND4,		// ���� ���������� 4
	PARAM_PARSND1,		// ���-� ���������� 1
	PARAM_PARSND2,		// ���-� ���������� 2
	PARAM_PARSND3,		// ���-� ���������� 3
	PARAM_PARSND4,		// ���-� ���������� 4
	PARAM_BCOEF,		// Bright LED
	PARAM_TIME_SHOW,	// ����� ������ (mS)
	PARAM_TIME_OFF,	    // ����� ���������� (mS)
	PARAM_MKBUS,	    // MKBUS ��������

	PARAM_VAR_NUM    		// ���������� ����������
};

#pragma pack (1)
typedef struct
{
    u8 level[4];
    u8 lvl_curr[4];
    u16 state;
	u16 start;		// ������� ������
    u16 send_off;
    s16 adc_det[5];
    u16 kbus;
	uint16_t rxState;
	float vdev;		// ���������� ������� ������� (V)
	u32 realtime;   
	u32 count_pkt_in;   
	u32 count_pkt_out;   
	u32 count_pkt_err;
}
VAR_PAR;
#pragma pack(4)

extern u32 mTimeout_show;
extern u32 mTimeout_off;
extern VAR_PAR dev_var;

//----------------------------------------------------------------
uint8_t Control_prop(uint8_t par, uint8_t * name, uint8_t * prop, uint8_t * nbyte);
void Control_get(uint8_t par, void * value);
void Control_set(uint8_t par, void * value);
//----------------------------------------------------------------
uint8_t Params_prop(uint8_t par, uint8_t * name, uint8_t * prop, uint8_t * nbyte);
void Params_get(uint8_t par, void * value);
void Params_set(uint8_t par, void * value);
//----------------------------------------------------------------

u8 calc_CS(u8 * addr, u16 size);
u16 conv_baud_uint(u16 baud);

// ������� ���������
void CONTROL_Proc(void);

#endif // __CONTROL_H
