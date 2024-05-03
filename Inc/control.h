
#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "system.h"
#include "USART.h"

// Параметры
#define VARR_STATE      0      // R: состояние
//--------------------------------------------------
//#define CMD_PAR         20  // 0x14: значения параметра (ответ на запрос)
#define CMD_SET_PAR     21  // 0x15: установка параметра
#define CMD_GET_PAR     22  // 0x16: запрос значения параметра

#define CMD_RESET		0		// Сброс регистра CMD
//--------------------------------------------------
#define STATUS_NULL  	    0x0000	// Null
#define STATUS_LVL1_SET    	0x0001	// Заданный уровень 1
#define STATUS_LVL2_SET 	0x0002	// Заданный уровень 2
#define STATUS_LVL3_SET  	0x0004	// Заданный уровень 3
#define STATUS_LVL4_SET    	0x0008	// Заданный уровень 4
#define STATUS_LVL_ON  	    0x0010	// Заданный уровень ON
#define STATUS_LVL_OFF 	    0x0020	// Заданный уровень OFF
#define STATUS_LVL1_SHOW   	0x0100	// Показать уровень 1
#define STATUS_LVL2_SHOW 	0x0200	// Показать уровень 2
#define STATUS_LVL3_SHOW 	0x0400	// Показать уровень 3
#define STATUS_LVL4_SHOW   	0x0800	// Показать уровень 4
//--------------------------------------------------
#define PAR_STATE  		0x0001	//0 запрос параметра 0


// Переменные настроек
enum CONTROL_VAR
{
	CONTROL_STAT,			// состояние
	CONTROL_LVLNET1,		// Уровень ext 1
	CONTROL_LVLNET2,		// Уровень ext 2
	CONTROL_LVLNET3,		// Уровень ext 3
	CONTROL_LVLNET4,		// Уровень ext 4
	CONTROL_CHAN1,			// Уровень 1
	CONTROL_CHAN2,			// Уровень 2
	CONTROL_CHAN3,			// Уровень 3
	CONTROL_CHAN4,			// Уровень 4
	CONTROL_BAT,			// Battery level (V)

	CONTROL_VAR_NUM    		// количество переменных
};
// Переменные настроек
enum PARAM_VAR
{
	PARAM_IDSEND1=0,	// ID получателя 1
	PARAM_IDSEND2,		// ID получателя 2
	PARAM_IDSEND3,		// ID получателя 3
	PARAM_IDSEND4,		// ID получателя 4
	PARAM_UNITSND1,		// Узел получателя 1
	PARAM_UNITSND2,		// Узел получателя 2
	PARAM_UNITSND3,		// Узел получателя 3
	PARAM_UNITSND4,		// Узел получателя 4
	PARAM_PARSND1,		// Пар-р получателя 1
	PARAM_PARSND2,		// Пар-р получателя 2
	PARAM_PARSND3,		// Пар-р получателя 3
	PARAM_PARSND4,		// Пар-р получателя 4
	PARAM_BCOEF,		// Bright LED
	PARAM_TIME_SHOW,	// Время показа (mS)
	PARAM_TIME_OFF,	    // Время отключения (mS)
	PARAM_MKBUS,	    // MKBUS протокол

	PARAM_VAR_NUM    		// количество переменных
};

#pragma pack (1)
typedef struct
{
    u8 level[4];
    u8 lvl_curr[4];
    u16 state;
	u16 start;		// Регистр Старта
    u16 send_off;
    s16 adc_det[5];
    u16 kbus;
	uint16_t rxState;
	float vdev;		// Напряжение питания прибора (V)
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

// Функция обработки
void CONTROL_Proc(void);

#endif // __CONTROL_H
