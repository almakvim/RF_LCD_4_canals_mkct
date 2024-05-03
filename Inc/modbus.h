#ifndef __modbus_H
#define __modbus_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "system.h"

#define WAITING_ADDR    0
#define WAITING_DATA    1

#define ANSWER    20
     
#define MBBUF_SIZE      256
     
#define MBBUF_MASK (MBBUF_SIZE - 1)
#if    (MBBUF_SIZE & MBBUF_MASK)
#error  MBBUF_SIZE is not a power of 2
#endif
     
#define AVER_NUM_16 16
#define AVER_NUM_32 32
#define AVER_NUM_64 64
#define AVER_NUM_128 128
#define AVER_NUM_256 256

// Переменные настроек 
enum MODBUS_VAR
{
	MODBUS_VAR_SlaveID,			// Адрес устройства
	MODBUS_VAR_FUNC,			// Функциональный код
	MODBUS_VAR_ADDR,			// Адрес первого регистра
	MODBUS_VAR_NREG,			// Количество регистров 

	MODBUS_VAR_OUT_R1,			// Запись в регистр 1
	MODBUS_VAR_OUT_R2,			// Запись в регистр 2
	MODBUS_VAR_OUT_R3,			// Запись в регистр 3
	MODBUS_VAR_OUT_R4,			// Запись в регистр 4

	MODBUS_VAR_IN_R1,			// Чтение регистра 1
	MODBUS_VAR_IN_R2,			// Чтение регистра 2
	MODBUS_VAR_IN_R3,			// Чтение регистра 3
	MODBUS_VAR_IN_R4,			// Чтение регистра 4

	MODBUS_VAR_REPPER,			// Время повтора (mS; 0=off)
	MODBUS_VAR_SENCMD,			// Отправить запрос
	MODBUS_VAR_ANSW_ERR,		// Есть ответ на запрос
    
	MODBUS_VAR_NUM          	// количество переменных
};

// Переменные настроек 
enum MODBUS_SYS
{
	MODBUS_SYS_Slave_1,			// Адрес устр-во 1 (У1)
	MODBUS_SYS_Slave_2,			// Адрес устр-во 2 (У2)
	MODBUS_SYS_FUNC,			// Функ-й код запроса
	MODBUS_SYS_ADDR_1,			// Адрес регистра на У1
	MODBUS_SYS_ADDR_2,			// Адрес регистра на У2
	MODBUS_SYS_NREG_1,			// Кол-во регистров на У1 
	MODBUS_SYS_NREG_2,			// Кол-во регистров на У2 

	MODBUS_SYS_REPPER,			// Повтор общ. запроса (mS; 0=off)
	MODBUS_SYS_SENCMD_1,		// Отправить запрос на У1
	MODBUS_SYS_SENCMD_2,		// Отправить запрос на У2
	MODBUS_SYS_ANSW_ERR_1,		// Есть ответ на запрос от У1
	MODBUS_SYS_ANSW_ERR_2,		// Есть ответ на запрос от У2
    
	MODBUS_SYS_NUM          	// количество переменных
};

#pragma pack (1)
typedef struct
{
    u8 sl_addr[2];
	u8 func;	
    u16 ad_reg[2];
    u16 n_reg[2];
    DATA_16 reg[2];
    DATA_16 inreg[2];
    u16 rep_time;
    u16 start;
    u16 nDev;
    u16 answ_not[2];
    u16 timeout;
    u16 timeout_rx;
    
    uint16_t state;
    DATA_2 addr;
    uint16_t len;
    uint16_t crc;
    uint16_t RxTail;
    uint16_t RxHead;
    uint8_t RxBuf[MBBUF_SIZE];
    
    uint8_t TxBuf[8];
}
MODBUS;
#pragma pack(4)

extern MODBUS modbus;
extern float Aver_M1[2][AVER_NUM_64 + 2];
extern float Aver_M2[2][AVER_NUM_64 + 2];

// Функции поддержки конфигурации
//Uint16 MODBUS_VarGet ( Uint16 nodeid, Uint16 varid, void * value );
//Uint16 MODBUS_VarSet ( Uint16 nodeid, Uint16 varid, void * value );
//Uint16 MODBUS_VarProp ( Uint16 nodeid, Uint16 varid, uint8_t * name, Uint16 * prop );

// Функции поддержки конфигурации
Uint16 MODBUS_SysGet ( Uint16 nodeid, Uint16 varid, void * value );
Uint16 MODBUS_SysSet ( Uint16 nodeid, Uint16 varid, void * value );
Uint16 MODBUS_SysProp ( Uint16 nodeid, Uint16 varid, uint8_t * name, Uint16 * prop );

void Send_pkt_uart(void * addr, u16 len);
void Send_pkt_func(uint16_t func, uint16_t dev);

void MODBUS_Rx(uint8_t dt);
void MODBUS_Proc(void);

#ifdef __cplusplus
}
#endif
#endif /*__ modbus_H */

/**
  * @}
  */

/**
  * @}
  */
