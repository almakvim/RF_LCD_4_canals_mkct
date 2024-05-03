
#include "string.h"
#include "mk_conf_tree.h"
#include "setup.h"
#include "system.h"
#include "USART.h"
#include "control.h"

VAR_PAR dev_var;
u32 mTimeout_show = 0;
u32 mTimeout_off = 0;
u32 mTimeout_cntrl = 0;
u32 mTimeout_shift = 0;
//============================================================================
uint8_t Control_prop( uint8_t par, uint8_t * name, uint8_t * prop, uint8_t * nbyte )
{
	char * str;
    uint8_t size = 0;
	if( name ) { switch( par )
        {
		case CONTROL_STAT       :	str ="Статус"; break;
		case CONTROL_CHAN1      :	str ="Уровень 1"; break;
		case CONTROL_CHAN2      :	str ="Уровень 2"; break;
		case CONTROL_CHAN3      :	str ="Уровень 3"; break;
		case CONTROL_CHAN4      :	str ="Уровень 4"; break;
		case CONTROL_LVLNET1    :	str ="Заданный уровень 1"; break;
		case CONTROL_LVLNET2    :	str ="Заданный уровень 2"; break;
		case CONTROL_LVLNET3    :	str ="Заданный уровень 3"; break;
		case CONTROL_LVLNET4    :	str ="Заданный уровень 4"; break;
		case CONTROL_BAT        :	str ="Battery (V)"; break;
        default: return 0;
        }
        while( *str ) { *name++ = *str++; size++; } *name = 0; 
    }
	if( prop ) switch( par )
        {
		case CONTROL_STAT       :	*prop = HEX; break;
        case CONTROL_BAT        :   *prop = REAL; break;
        case CONTROL_LVLNET1    :   
        case CONTROL_LVLNET2    :   
        case CONTROL_LVLNET3    :   
        case CONTROL_LVLNET4    :   *prop = UINT; break;  
        case CONTROL_CHAN1      :   
        case CONTROL_CHAN2      :   
        case CONTROL_CHAN3      :   
        case CONTROL_CHAN4      :	*prop = UINT | RO; break;   
        default: return 0;
        }
	if( nbyte ) switch( par )
        {
        case CONTROL_BAT        :   *nbyte = 4; break;
		case CONTROL_STAT       :	*nbyte = 2; break;
        case CONTROL_LVLNET1    :   
        case CONTROL_LVLNET2    :   
        case CONTROL_LVLNET3    :   
        case CONTROL_LVLNET4    :   
        case CONTROL_CHAN1      :   
        case CONTROL_CHAN2      :   
        case CONTROL_CHAN3      :   
        case CONTROL_CHAN4      :	*nbyte = 1; break;   
        default: return 0;
        }
    return size;
}
//============================================================================
void Control_get(uint8_t par, void * value)
{
    switch( par )
    {
			case CONTROL_CHAN1     	:	*(Uint32*)value = dev_var.level[0]; break;
			case CONTROL_CHAN2     	:	*(Uint32*)value = dev_var.level[1]; break;
			case CONTROL_CHAN3     	:	*(Uint32*)value = dev_var.level[2]; break;
			case CONTROL_CHAN4     	:	*(Uint32*)value = dev_var.level[3]; break;
			case CONTROL_LVLNET1   	:	*(Uint32*)value = dev_var.lvl_curr[0]; break;
			case CONTROL_LVLNET2   	:	*(Uint32*)value = dev_var.lvl_curr[1]; break;
			case CONTROL_LVLNET3   	:	*(Uint32*)value = dev_var.lvl_curr[2]; break;
			case CONTROL_LVLNET4   	:	*(Uint32*)value = dev_var.lvl_curr[3]; break;
			case CONTROL_BAT     	:	*(float*)value = dev_var.vdev; break;
			case CONTROL_STAT     	:	*(Uint32*)value = dev_var.state; break;
        default:;
    }
}
//============================================================================
void Control_set(uint8_t par, void * value)
{
	u8 i = *(u8*)value;
	float f = *(float*)value;
    switch( par )
    {
		case CONTROL_BAT	 :	f = f/(float)dev_var.adc_det[4]; Setup.bt_coef = f;
                                //SETUP_Save();	
                                break;
		case CONTROL_STAT    :  dev_var.state = i; break;
		case CONTROL_LVLNET1 :  dev_var.lvl_curr[0] = i;
                                dev_var.state |= STATUS_LVL1_SET;
                                break;
		case CONTROL_LVLNET2 :  dev_var.lvl_curr[1] = i; 
                                dev_var.state |= STATUS_LVL2_SET;
                                break;
		case CONTROL_LVLNET3 :  dev_var.lvl_curr[2] = i; 
                                dev_var.state |= STATUS_LVL3_SET;
                                break;
		case CONTROL_LVLNET4 :  dev_var.lvl_curr[3] = i; 
                                dev_var.state |= STATUS_LVL4_SET;
                                break;
        default:;
    }
}
//============================================================================
uint8_t Params_prop( uint8_t par, uint8_t * name, uint8_t * prop, uint8_t * nbyte )
{
	char * str;
    uint8_t size = 0;
	if( name ) {switch( par )
        {
	case PARAM_IDSEND1     :	str ="ID получателя 1"; break;
	case PARAM_IDSEND2     :	str ="ID получателя 2"; break;
	case PARAM_IDSEND3     :	str ="ID получателя 3"; break;
	case PARAM_IDSEND4     :	str ="ID получателя 4"; break;
	case PARAM_PARSND1     :	str ="Пар-р получателя 1"; break;
	case PARAM_PARSND2     :	str ="Пар-р получателя 2"; break;
	case PARAM_PARSND3     :	str ="Пар-р получателя 3"; break;
	case PARAM_PARSND4     :	str ="Пар-р получателя 4"; break;
	case PARAM_UNITSND1    :	str ="Узел получателя 1"; break;
	case PARAM_UNITSND2    :	str ="Узел получателя 2"; break;
	case PARAM_UNITSND3    :	str ="Узел получателя 3"; break;
	case PARAM_UNITSND4    :	str ="Узел получателя 4"; break;
	case PARAM_BCOEF       :	str ="Battery coef"; break;
	case PARAM_TIME_SHOW   :	str ="Время показа (mS)"; break;
	case PARAM_TIME_OFF    :	str ="Время отключения (mS)"; break;
	case PARAM_MKBUS       :	str ="MKBUS протокол"; break;
        default: return 0;
        }
        while( *str ) { *name++ = *str++; size++; } *name = 0; 
    }
	if( prop ) switch( par )
        {
    case PARAM_TIME_SHOW  :	
    case PARAM_TIME_OFF   :	
    case PARAM_IDSEND1    :	
    case PARAM_IDSEND2    :	
    case PARAM_IDSEND3    :	
    case PARAM_IDSEND4    :	
    case PARAM_PARSND1    :	
    case PARAM_PARSND2    :	
    case PARAM_PARSND3    :	
    case PARAM_PARSND4    :	
    case PARAM_UNITSND1   :	
    case PARAM_UNITSND2   :	
    case PARAM_UNITSND3   :	
    case PARAM_UNITSND4   :	*prop = UINT; break;
    case PARAM_BCOEF      :	*prop = REAL; break;
    case PARAM_MKBUS      :	*prop = BOOL; break;
        default: return 0;
        }
	if( nbyte ) switch( par )
        {
    case PARAM_TIME_SHOW  :	
    case PARAM_TIME_OFF   :	
    case PARAM_IDSEND1    :	
    case PARAM_IDSEND2    :	
    case PARAM_IDSEND3    :	
    case PARAM_IDSEND4    :	*nbyte = 2; break;
    case PARAM_PARSND1    :	
    case PARAM_PARSND2    :	
    case PARAM_PARSND3    :	
    case PARAM_PARSND4    :	
    case PARAM_UNITSND1   :	
    case PARAM_UNITSND2   :	
    case PARAM_UNITSND3   :	
    case PARAM_UNITSND4   :	
    case PARAM_MKBUS      :	*nbyte = 1; break;
    case PARAM_BCOEF      :	*nbyte = 4; break;
        default: return 0;
        }
    return size;
}
//============================================================================
void Params_get(uint8_t par, void * value)
{
    switch( par )
    {
		case PARAM_TIME_SHOW   	:	*(u16*)value = Setup.time_show; break;
		case PARAM_TIME_OFF   	:	*(u16*)value = Setup.time_off; break;
		case PARAM_IDSEND1     	:	*(u16*)value = Setup.id_send[0]; break;
		case PARAM_IDSEND2     	:	*(u16*)value = Setup.id_send[1]; break;
		case PARAM_IDSEND3     	:	*(u16*)value = Setup.id_send[2]; break;
		case PARAM_IDSEND4     	:	*(u16*)value = Setup.id_send[3]; break;
		case PARAM_PARSND1    	:	*(u8*)value = Setup.par_send[0]; break;
		case PARAM_PARSND2    	:	*(u8*)value = Setup.par_send[1]; break;
		case PARAM_PARSND3    	:	*(u8*)value = Setup.par_send[2]; break;
		case PARAM_PARSND4    	:	*(u8*)value = Setup.par_send[3]; break;
		case PARAM_UNITSND1    	:	*(u8*)value = Setup.unit_send[0]; break;
		case PARAM_UNITSND2    	:	*(u8*)value = Setup.unit_send[1]; break;
		case PARAM_UNITSND3    	:	*(u8*)value = Setup.unit_send[2]; break;
		case PARAM_UNITSND4    	:	*(u8*)value = Setup.unit_send[3]; break;
		case PARAM_BCOEF     	:	*(float*)value = Setup.bt_coef; break;
		case PARAM_MKBUS    	:	*(u8*)value = Setup.mkbus; break;
        default:;
    }
}
//============================================================================
void Params_set(uint8_t par, void * value)
{
	u16 i = *(u16*)value;
	u8 i8 = *(u8*)value;
	float f = *(float*)value;
    switch( par )
    {
		case PARAM_TIME_SHOW  :	Setup.time_show = i; break;
		case PARAM_TIME_OFF  :	Setup.time_off = i; break;
		case PARAM_IDSEND1	  :	Setup.id_send[0] = i; break;
		case PARAM_IDSEND2	  :	Setup.id_send[1] = i; break;
		case PARAM_IDSEND3	  :	Setup.id_send[2] = i; break;
		case PARAM_IDSEND4	  :	Setup.id_send[3] = i; break;
		case PARAM_PARSND1	  :	Setup.par_send[0] = i8; break;
		case PARAM_PARSND2	  :	Setup.par_send[1] = i8; break;
		case PARAM_PARSND3	  :	Setup.par_send[2] = i8; break;
		case PARAM_PARSND4	  :	Setup.par_send[3] = i8; break;
		case PARAM_UNITSND1	  :	Setup.unit_send[0] = i8; break;
		case PARAM_UNITSND2	  :	Setup.unit_send[1] = i8; break;
		case PARAM_UNITSND3	  :	Setup.unit_send[2] = i8; break;
		case PARAM_UNITSND4	  :	Setup.unit_send[3] = i8; break;
		case PARAM_BCOEF	  :	Setup.bt_coef = f; break;
		case PARAM_MKBUS      :	Setup.mkbus = i8; break;
        default:;
    }
}
//=========================================================================
void send_level_to_device(u16 level, u16 num)
{
    TX_CMD tx_cmd;
    
    dev_var.rxState = WAIT_HEAD;
    tx_cmd.id_dev = Setup.id_send[num];
    tx_cmd.n_par = Setup.par_send[num];
    tx_cmd.l_par = 1;
    tx_cmd.data.dt_16[0] = level;
    
    if(Setup.mkbus){
        tx_cmd.id_unit = Setup.unit_send[num];
        tx_cmd.cmd = 5;
        mkbus_config_send(&tx_cmd);
    } else {
        tx_cmd.id_unit = Setup.idunit_send[num];
        tx_cmd.cmd = 0x11;
        sbus_config_send(&tx_cmd);
    }
}
//=========================================================================
// Функция обработки
void CONTROL_Proc(void)
{
	static Uint32 timeout_type = 0;
	static Uint32 timeout_zero = 0;
	static Int16 lvl_old[4];
	static Int16 clvl_old[4];
	static Int16 slvl_old[4];
	static Int16 count_zero = 0;
    u16 delta = 0;

	if(dev_var.start == 0)
	{
        dev_var.start = 1;
        lvl_old[0] = 0;
        lvl_old[1] = 0;
        lvl_old[2] = 0;
        lvl_old[3] = 0;
        dev_var.state = 0;
	}

	if(timeout_type+150 <= HAL_GetTick())
	{
        timeout_type = HAL_GetTick();
        
        if((dev_var.state & STATUS_LVL1_SET)==0) dev_var.lvl_curr[0] = dev_var.level[0];
        if((dev_var.state & STATUS_LVL2_SET)==0) dev_var.lvl_curr[1] = dev_var.level[1];
        if((dev_var.state & STATUS_LVL3_SET)==0) dev_var.lvl_curr[2] = dev_var.level[2];
        if((dev_var.state & STATUS_LVL4_SET)==0) dev_var.lvl_curr[3] = dev_var.level[3];
        
        if(dev_var.state & STATUS_LVL_ON) 
        {
            dev_var.lvl_curr[0] = 100;
            dev_var.lvl_curr[1] = 100;
            dev_var.lvl_curr[2] = 100;
            dev_var.lvl_curr[3] = 100;
            dev_var.state &= ~(STATUS_LVL_ON+STATUS_LVL_OFF);
            dev_var.state |= STATUS_LVL1_SET+STATUS_LVL2_SET+STATUS_LVL3_SET+STATUS_LVL4_SET;
        }
        if(dev_var.state & STATUS_LVL_OFF) 
        {
            dev_var.lvl_curr[0] = 0;
            dev_var.lvl_curr[1] = 0;
            dev_var.lvl_curr[2] = 0;
            dev_var.lvl_curr[3] = 0;
            dev_var.state &= ~(STATUS_LVL_ON+STATUS_LVL_OFF);
            dev_var.state |= STATUS_LVL1_SET+STATUS_LVL2_SET+STATUS_LVL3_SET+STATUS_LVL4_SET;
        }
    //------------------------------------------
        for(u16 i=0;i<4;i++)
        {
            delta = dev_var.level[i]-lvl_old[i]; 
            if(lvl_old[i]>=dev_var.level[i]) delta = lvl_old[i]-dev_var.level[i];
            if(delta>1)
            {
                lvl_old[i] = dev_var.level[i];
                u16 inv = 1<<i;
                dev_var.state &= ~inv;
            }
        }
    //------------------------------------------
        for(u16 i=0;i<4;i++)
        {
            delta = dev_var.lvl_curr[i]-clvl_old[i]; 
            if(clvl_old[i]>=dev_var.lvl_curr[i]) delta = clvl_old[i]-dev_var.lvl_curr[i];
            if(delta>1)
            {
                clvl_old[i] = dev_var.lvl_curr[i];
                send_level_to_device(dev_var.lvl_curr[i], i);
            }
        }
	}
    //------------------------------------------
        for(u16 i=0;i<4;i++)
        {
            delta = dev_var.lvl_curr[i]-slvl_old[i]; 
            if(slvl_old[i]>=dev_var.lvl_curr[i]) delta = slvl_old[i]-dev_var.lvl_curr[i];
            if(delta>=1)
            {
                slvl_old[i] = dev_var.lvl_curr[i];
                dev_var.state &= ~(STATUS_LVL1_SHOW+STATUS_LVL2_SHOW+STATUS_LVL3_SHOW+STATUS_LVL4_SHOW);
                u16 inv = 1<<(i+8);
                dev_var.state |= inv;
                mTimeout_show = Setup.time_show;
                dev_var.send_off = 1;
            }
        }
        if(mTimeout_show == 0)
        {
            dev_var.state &= ~(STATUS_LVL1_SHOW+STATUS_LVL2_SHOW+STATUS_LVL3_SHOW+STATUS_LVL4_SHOW);
        }
    //------------------------------------------
	if(timeout_zero+150 <= HAL_GetTick())
	{
        timeout_zero = HAL_GetTick();
        if(dev_var.send_off)
        {
            send_level_to_device(dev_var.lvl_curr[count_zero&3], count_zero&3);
            count_zero++;
            if(count_zero >= 12) dev_var.send_off = 0; 
        }
        else count_zero = 0;
    }    
    //------------------------------------------
	if(dev_var.realtime+500 <= HAL_GetTick())
	{
		dev_var.kbus = 0;
	}
}
//=========================================================================

