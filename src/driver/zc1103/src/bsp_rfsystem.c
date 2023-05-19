#include "bsp_rfsystem.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "pindef.h"
#include "Drv_Spi.h"
#include "delay.h"
#include "stm32f10x_exti.h"


#define RF_RST_HIGH()       GPIO_WriteBit(RF_RST_Port, RF_RST_Pin, Bit_SET)
#define RF_RST_LOW()        GPIO_WriteBit(RF_RST_Port, RF_RST_Pin, Bit_RESET)

#define RF_SDN_HIGH()       GPIO_WriteBit(RF_SDN_Port, RF_SDN_Pin, Bit_SET)
#define RF_SDN_LOW()        GPIO_WriteBit(RF_SDN_Port, RF_SDN_Pin, Bit_RESET)

#define  RF_IRQ_INPUT()     GPIO_ReadInputDataBit(RF_IRQ_Port, RF_IRQ_Pin)

#define  RfCsHigh()         SPI_NSS_OUT_High
#define  RfCsLow()           SPI_NSS_OUT_Low


volatile uint32_t preamble_timeout = 0;
volatile uint8_t  rece_falg = 0;
static int systemStatus = 0;
unsigned char g_paValue = 10;

volatile unsigned char rf_interrupt_pending = 0;



/**
 * \brief  配置rf相关gpio
 * \param   None
 * \retval  None
 */
static void RfGpioConfigure(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	EXTI_InitTypeDef  	EXTI_InitStruct;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO  ,ENABLE);
	
    //SDN;
    RCC_APB2PeriphClockCmd( RF_SDN_RCCLK  ,ENABLE);
	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = RF_SDN_Pin;
    GPIO_Init( RF_SDN_Port ,&GPIO_InitStructure);  
    
    //IRQ 
    RCC_APB2PeriphClockCmd( RF_IRQ_RCCLK  ,ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = RF_IRQ_Pin;
    GPIO_Init( RF_IRQ_Port ,&GPIO_InitStructure);
    //RST 
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA ,ENABLE);
    GPIO_InitStructure.GPIO_Pin = RF_RST_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_Init( RF_RST_Port ,&GPIO_InitStructure);
   
	SWSPI_GPIO_Init();
	
	RF_RST_HIGH();
	RF_SDN_HIGH();
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);
	
	EXTI_StructInit(&EXTI_InitStruct);
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Line = EXTI_Line5;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	
	EXTI_Init(&EXTI_InitStruct);
	
	EXTI_ClearFlag(GPIO_PinSource5);
	
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	
	
	
	return;
}

/**
 * \brief  SPI master控制器初始化
 * \param   None
 * \retval  None
 */
static void RfSpiConfigure(void)
{

	return;
}

/**
 * \brief  复位射频芯片
 * \param   None
 * \retval  None
 */
static void RfReset(void)
{

    unsigned int loopCount;
    unsigned int i;
    /*复位寄存器*/
    RF_RST_LOW();
    for(i = 10;i >0;i--){ 
      for (loopCount = 0xfff; loopCount != 0; loopCount--);
    }
	RF_RST_HIGH();
	RF_SDN_LOW();
    for(i = 10;i >0;i--){
      for (loopCount = 0xfff; loopCount != 0; loopCount--);
    }
}

/**
 * \brief  通过spi传输一个字节
 * \param  [IN] byte 发送的字节
 * \retval  接收的字节
 */
static unsigned char RfSendByte(unsigned char byte)
{
   unsigned char tmp=0;
    tmp = Spi_Read_Write_Byte(byte);
    return tmp;
}

/**
 * \brief  写RF寄存器
 * \param[IN] addr 寄存器地址 取值0x00 - 0x7F
 * \param[IN] val  写入的值
 * \retval  None
 */
void RfRegisterWrite(const unsigned char addr,const unsigned char val)
{
    RfCsLow();
    RfSendByte(addr&0x7f);
    RfSendByte(val);
    RfCsHigh();
}
/**
 * \brief  读RF寄存器
 * \param[IN] addr 寄存器地址 取值0x00 - 0x7F
 * \retval  读取寄存器的值
 */
unsigned char RfRegisterRead(const unsigned char addr)
{
    unsigned char readdat;

    RfCsLow();
    RfSendByte(addr|0x80);
    readdat = RfSendByte(0xff);
    RfCsHigh();
    return readdat;
}
/**
 * \brief  初始化rf寄存器
 * \param  None
 * \retval  None
 */
static void RfRegisterInit(void)
{
    unsigned int i=0;
    unsigned int loopCount = 0;

    RfRegisterWrite(0x09,0x08);/*Debug*/
    RfRegisterWrite(0x0c,0x03);
    RfRegisterWrite(0x0e,0xA1);
    RfRegisterWrite(0x0F,0x0A);
    RfRegisterWrite(0x10,0x54);
    RfRegisterWrite(0x1b,0x25);

   
    RfRegisterWrite(0x20,0xa4);            
    RfRegisterWrite(0x21,0x37);         
    RfRegisterWrite(0x22,0x3a);         /*VCO Config  3a*/  //3a -> 0azhangjun 20200612    
    RfRegisterWrite(0x23,0x36);         /*SYN Config   bit[7]enable wideband */ 
    /*RfRegisterWrite(0x23, ((RfRegisterRead(0x23)&0x8F)|0x50));  //bit[6-4] Vco ldo output voltage */
    RfRegisterWrite(0x2F,0xe0);         /*rx rssi threshold*/
    RfRegisterWrite(0x2E,0x00);      

    RfRegisterWrite(0x30,0x00);         /*ber optimize 0x40->0x00 by 20211126 juner*/
    RfRegisterWrite(0x31,0x00);         
    RfRegisterWrite(0x32,0x00); 
    RfRegisterWrite(0x33,0x00); 
    RfRegisterWrite(0x34,0x00); 
    RfRegisterWrite(0x35,0x00); 
    RfRegisterWrite(0x36,0x00); 


    RfRegisterWrite(0x39,0x74); /*enable demode reset */
    RfRegisterWrite(0x3A,0x61); 
    RfRegisterWrite(0x4a,0x60);
    RfRegisterWrite(0x4d,0x0b);
    RfRegisterWrite(0x4e,0x7c);/*ber optimize 0x6c->0x7c by 20211126 juner*/
    RfRegisterWrite(0x4f,0xc5);
    ////10kps
    RfRegisterWrite(0x74,0x9d);/*bit[7-6] ADC clock select*/
    RfRegisterWrite(0x08,0x01);/*方法1设置频偏25k   */  
    RfRegisterWrite(0x24,0x19);/*中频设置[7-0]      */
    RfRegisterWrite(0x3D,0x53);/*中频设置[7-0]      */

    RfRegisterWrite(0x38,0x56);
    RfRegisterWrite(0x3C,0xD1);
    RfRegisterWrite(0x3E,0x83);
    RfRegisterWrite(0x3F,0x08);

    RfRegisterWrite(0x58,0x00);
    RfRegisterWrite(0x59,0x07);
    RfRegisterWrite(0x5A,0x08);
    RfRegisterWrite(0x5B,0x09);
    RfRegisterWrite(0x5C,0x03);
    RfRegisterWrite(0x5D,0x71);
    RfRegisterWrite(0x5e,0x00);
    RfRegisterWrite(0x5f,0xDF);


    //25k->20k   0x40->0xC0
    //25k->20k   0x66->0x51
    //25k->20k   0x66->0xEC
    
    RfRegisterWrite(0x78,0xC0);  //方法2设置频偏10k --未使用 25->20K
    RfRegisterWrite(0x79,0x51);  //25->20K
    RfRegisterWrite(0x7a,0xEC);  //25->20K

    RfRegisterWrite(0x7b,0x5A);
    RfRegisterWrite(0x7c,0x7C);
    RfRegisterWrite(0x7d,0x01);
    RfRegisterWrite(0x7e,0x00);
    RfRegisterWrite(0x7f,0x70);
    
    RfRegisterWrite(0x15,0x21);
    RfRegisterWrite(0x07,0x5d);
    RfRegisterWrite(0x18,0x20);
    RfRegisterWrite(0x2a,0x14);
    RfRegisterWrite(0x37,0x99);

    RfRegisterWrite(0x06,0x3a);//0x3a /*syncwordlen = 2bytes,length = 1byte,CRC,SCramble_on*/   bit[3] share fifo
    RfRegisterWrite(0x04,0x50);/*preamble length 80 bytes*/

    for(i = 10;i >0;i--)
    {
        for (loopCount = 0xffff; loopCount != 0; loopCount--);
    }
}
/**
 * \brief  设置频率
 * \param [IN]  freq 频率值
 * \retval  None
 */
 void  RFSetRefFreq(const double freq)
{
    unsigned int Fre = 0;
    unsigned char reg73 = 0,reg72 = 0,reg71 = 0,reg70 = 0;
    Fre = (unsigned int)(freq * pow(2.0,24.0));
    
    reg73 =(unsigned char)(Fre & 0xFF);
    reg72 =(unsigned char)((Fre >> 8) & 0xFF);
    reg71 =(unsigned char)((Fre >> 16) & 0xFF);
    reg70 =(unsigned char)((Fre >> 24) & 0xFF);
    
    RfRegisterWrite(0x73,reg73);
    RfRegisterWrite(0x72,reg72);
    RfRegisterWrite(0x71,reg71);
    RfRegisterWrite(0x70,reg70);
}
/**
 * \brief  设置PA增益
 * \param [IN]  x_dBm 增益
 * \retval  None
 */
 void RfSetPA(PA_LEVEL x_dBm)
{
    unsigned char r_reg;
#if 0   
 code   unsigned char vReg25Tbl[] =      {0x3a,0x2b,0x25,0x1a,0x0d,0x0b,0x09,0x07,0x06,0x04,0x03,0x01,0x4c,
						0x4a,0x46,0x04,0x04,0x05,0x04,0x01,0X01,0X01,0X00,0X00,0X02,0X01,0X00};
 code   unsigned char vReg26Tbl[] =      {0xdd,0xdd,0xbf,0xdd,0xdd,0xdd,0xdd,0xbf,0xbf,0xbf,0xbf,0xbf,0x3a,
						0x3a,0x30,0x35,0x25,0x11,0X0e,0X17,0X11,0X0c,0X11,0X0a,0X02,0x03,0x03};
 code   unsigned char vReg25Tbl_h3[] ={ 0x3f,0xff,0xa6,0x95,0x94,0xce,0x91,0x09,0x08,0x03,\
                                    0x7f,0x3f,0x2f,0x1f,0x1c,0x15,0x13,0x11,0x10,0x10,\
                                    0x07,0x05,0x05,0x03,0x03,0x03,0x03};

 code   unsigned char vReg26Tbl_h3[] ={ 0x81,0x81,0x83,0x9a,0x82,0x82,0x80,0x83,0x9f,0x98,\
                                    0x7f,0x64,0x67,0x7d,0x7d,0x7c,0x77,0x74,0x75,0x65,\
                                    0x6c,0x77,0x67,0x7d,0x6b,0x5e,0x56};
#else    
 const  unsigned char vReg25Tbl_h4[] = {0x3f,0x38,0x25,0x1a,0x0f,0x0d,0x0b,0x0a,0x09,0x08,0x04,0x03,0x86,
                                                0x82,0x01,0x01,0x02,0x02,0x00,0x00,0x24,0x20,0x16,0x14,0x11,0x0d,0x0d};

 const   unsigned char vReg26Tbl_h4[] = {0xb0,0xb0,0xbf,0xbf,0xbf,0xbf,0xbf,0xbf,0x9f,0x9f,0xbf,0x8f,0x80,
    						0xbf,0x9f,0x84,0x81,0x80,0xad,0x88,0x7f,0x7f,0x7f,0x7f,0x7f,0x72,0x62};

    
    
 const   unsigned char vReg25Tbl_h31[] ={ 0xbf,0xff,0x1d,0x1c,0x0f,0x0e,0x07,0x06,0x05,0x04,\
                                    0x03,0x3f,0x2f,0x2f,0x1d,0x17,0x13,0x11,0x10,0x10,\
                                    0x07,0x05,0x05,0x03,0x03,0x03,0x03};

 const   unsigned char vReg26Tbl_h31[] ={ 0x83,0x81,0xaf,0x82,0x8f,0x85,0x95,0xbf,0x81,0xab,\
                                    0x81,0x68,0x6b,0x5c,0x75,0x72,0x7b,0x79,0x7a,0x68,\
                                    0x6f,0x7c,0x6a,0x7f,0x6e,0x62,0x58};
#endif    
    g_paValue = x_dBm;
    r_reg = RfRegisterRead(0x47);
    if( (r_reg & 0x07) == 0x04 )
    {
      RfRegisterWrite(0x25,vReg25Tbl_h4[x_dBm]);
      RfRegisterWrite(0x26,vReg26Tbl_h4[x_dBm]);
    }
    else
    {
      RfRegisterWrite(0x25,vReg25Tbl_h31[x_dBm]);
      RfRegisterWrite(0x26,vReg26Tbl_h31[x_dBm]);
    }
}

/**
  * \brief  使能接收到同步字后锁定rssi
  * \param  None
  * \retval  None
  */
static void RfSetSyncLockRssi(void)
{
  RfRegisterWrite(0x3e,RfRegisterRead(0x3e)|0x40);
}

static void RfSetVcoFreq(const double freq)
{
    unsigned  int Fre = 0;
    unsigned char reg77 = 0,reg76 = 0,reg75 = 0,reg74 = 0,temp = 0;
    Fre = (unsigned int)(freq * pow(2.0,20.0));
    
    reg77 =(unsigned char)(Fre & 0xFF);
    reg76 =(unsigned char)((Fre >> 8) & 0xFF);
    reg75 =(unsigned char)((Fre >> 16) & 0xFF);
    reg74 =(unsigned char)(((Fre >> 24) & 0xFF)| (RfRegisterRead(0x74)&0xc0));

    temp = RfRegisterRead(0x00);
    RfRegisterWrite(0x00,(0x80 | temp));
  
    RfRegisterWrite(0x77,reg77);
    RfRegisterWrite(0x76,reg76);
    RfRegisterWrite(0x75,reg75);
    RfRegisterWrite(0x74,reg74);
}

static void  RfSetFreq_N(const unsigned char N)
{
    if(N > 0x7F) 
        return;
    RfRegisterWrite(0x00,(0x80 | N));
}


static void RfSetFreqStep(double step)
{
    unsigned int fre = 0;
    unsigned char reg1 = 0,reg2 = 0,reg3 = 0;
    fre = (unsigned int)(step * pow(2.0,20.0));
    reg3 = (unsigned char)(fre & 0xFF);
    reg2 = (unsigned char)((fre >> 8)  & 0xFF);
    reg1 = (unsigned char)((fre >> 16) & 0x7F);
    RfRegisterWrite(0x03,reg3);
    RfRegisterWrite(0x02,reg2);
    RfRegisterWrite(0x01,reg1);
}

double g_freq = 476.3;
void RfFreqSet(const double f0,const unsigned char N,const double step)
{
    
    g_freq = f0;
    RfSetVcoFreq(f0);
    RfSetFreq_N(N);
    RfSetFreqStep(step);  
}

/**
  * \brief  清空发送区域
  * \param  None
  * \retval  None
  */
static void RfClrTxFifoWrPtr(void)
{
      RfRegisterWrite(0x53,0x80);      /*Reset FIFO write Pointer*/
}

/**
  * \brief  获取包状态
  * \param  None
  * \retval  
  */
unsigned char RfGetPktStatus(void)
{
   
     if(RF_IRQ_INPUT() ) {
        return 1;
     }
     else {
        return 0;
     }
	
}

/**
  * \brief  读取Rssi值
  * \param  None
  * \retval  
  */
unsigned char RfReadRssi(void)
{
    unsigned char r_reg;

    r_reg = RfRegisterRead(0x43); 
    return r_reg/2;
}

/**
  * \brief  发送数据
  * \param [IN] SrcBuf 待发送数据
  * \param [IN] len 待发送数据长度
  * \retval None 
  */
static void RfWriteFIFO(const unsigned char* SrcBuf,unsigned char len)
{
    unsigned char i = 0;

    RfCsLow();
    RfSendByte(0x55 & 0x7F);
    for(i = 0;i < len;i++)
    {
       RfSendByte(*(SrcBuf++));
    }
    RfCsHigh(); 
}

/**
  * \brief  读数据
  * \param [OUT] StoreBuf 保存数据地址
  * \param [IN] len 读取长度
  * \retval None 
  */
void RfReadFIFO(unsigned char *StoreBuf,unsigned char Len)
{
    unsigned char i = 0;
    RfCsLow();
    RfSendByte(0x52|0x80);      
    for(i = 0;i < Len;i++)
    {
        *(StoreBuf + i) =  RfSendByte(0xFF);
    }
    RfCsHigh(); 
}
/**
  * \brief  RF 当前状态
  * \param  None
  * \retval rf芯片状态 
  */

int RfSystemStatus(void)
{
  return systemStatus;
  /*
    switch(RfRegisterRead(0x46))
    {
      case 0x80:
         return systemStatus;
      break;
      case 0x20:
        return 2;
      break;
      case 0x40:
        return 1;
      break;
      case 0x10:
        return 6;
      break;
      case 0x08:
        return 7;
      case 0x04:
        return 8;
      case 0x02:
        return 9;
      case 0x01:
        return 10;
      break;
      default:
        break;
    }*/
}

/**
  * \brief  使能IDLE 模式
  * \param  None
  * \retval None
  */
void RfIdleEn(void)
{
	int i=0;
    RfRegisterWrite(0x60,0xff); 
    while(RfRegisterRead(0x46) != 0x80){
		if(i++ > 256){
			systemStatus = 0xff;
			return;
		}
	}
    systemStatus = 0;
}
/**
  * \brief  使能接收模式
  * \param  None
  * \retval None
  */
void RfRecEn(void)
{
	int i = 0;
   RfRegisterWrite(0x51,0x80); 
   RfIdleEn();
   RfRegisterWrite(0x66,0xff); 
   while(RfRegisterRead(0x46) != 0x20){
	   if(i++ > 256){
			systemStatus = 0xff;
			return;
		}
   }
   systemStatus = 2;
}
/**
* \brief  切换到发送状态
  * \param  None
  * \retval None
  */
void RfTranEn(void)
{
   RfIdleEn(); 
   RfRegisterWrite(0x65,0xff); 
   while(RfRegisterRead(0x46) != 0x40);
   systemStatus = 1;
}
/**
  * \brief  切换到睡眠状态
  * \param  None
  * \retval None
  */
void RfSleepEn(void)
{
   RfIdleEn(); 
   RfRegisterWrite(0x67,0xff); 
   systemStatus = 3;
}
/**
  * \brief  切换到待机状态
  * \param  None
  * \retval None
  */
void RfStandByEn(void)
{
   RfIdleEn(); 
   RfRegisterWrite(0x68,0xff); 
   systemStatus = 4;
}

/**
* \brief  发送单音载波
  * \param  None
  * \retval None
  */
void RF_TxCW(void)
{
    RfIdleEn();
    RfRegisterWrite(0x24,(RfRegisterRead(0x24)|0x80));
    RfRegisterWrite(0x06,(RfRegisterRead(0x06)&0xFC));
    RfTranEn();
}


void RfTestPackageSend(const unsigned char *buffer, const unsigned char size)
{

    RfIdleEn();
    RfClrTxFifoWrPtr();
    RfWriteFIFO(&buffer[0],size);
    RfTranEn();
  
}

/**
  * \brief  发送数据包
  * \param [IN] buffer 发送数据
  * \param [IN] size   发送数数据长度
  * \retval None
  */
void RfDataPackageSend(const unsigned char *buffer, const unsigned char size)
{

    /*Fix SPI concurrency conflicts, disable irq */
    if(size > 0)
    {
      unsigned char buf[264]={0};
      
	
      buf[0] = size;
      memcpy(buf+1,buffer,size);
      RfIdleEn();
      RfClrTxFifoWrPtr();
      RfWriteFIFO(&buf[0],size+1);
      RfTranEn();

    }
    
}
/**
  * \brief  接收数据包
  * \param [OUT] buf 接收数据
  * \retval 接收数据长度
  */
int RfPackageRecv(char *buf)
{
    int len;

    unsigned char rx_rssi=0;

    RfRegisterWrite(0x51,0x80);
    len = RfRegisterRead(0x52|0x80);
    if (len == 0)
    {
        RfRecEn();
        return -3;
    }
    else
    {
        rx_rssi = RfReadRssi();
        RfReadFIFO((uint8_t*)buf,len);	
        RfRecEn();
        #if 0
        printf("rece data len = %d  rssi = -%bd dB\r\n", len,rx_rssi);
        for(i=0; i<len; i++){
            printf("0x%bx\t",buf[i] );
        }
        #endif
        return len;
    }
}



/**
  * \brief RF芯片配置
  * \param None
  * \retval None
  */
void RfConfigure(void)
{
    unsigned int i=0;


    RfGpioConfigure();

    RfCsHigh();

    RfSpiConfigure();
    //重启芯片
    delay_ms(200);
    RfReset();
    //初始化rf 参数

    RfRegisterInit();
    //设置参考频率
    RfRegisterWrite(0x70,0x12);
    RfRegisterWrite(0x71,0x14);
    RfRegisterWrite(0x72,0x7A); // 6D 48    A1  7A 18.08
    RfRegisterWrite(0x73,0xE1); // 32 00    84  E1 
    //设置锁频
    RfSetSyncLockRssi();
    //设置中心频点
    RfFreqSet(476.0,0,0); 
    //设置发射功率
    RfSetPA(DBM20);
    //设置为接收态
    RfRecEn();
    //打印初始化参数
	
    for(i=0; i<=0x7f; i++)
    {
    	printf("read  reg0x%02x = %2.2x \n",i,RfRegisterRead(i));
    }
}


/**
 * \brief   rf 中断底半段
 * \param   None
 * \retval  None
 */
void rf_isr(void)
{
        unsigned char tmp = 0;
        
        tmp = RfRegisterRead(0x40);
    
        printf("%2.2x\r\n", tmp);
    
        if(tmp & (1<<6)){          /*接收到正确的preamble*/    
			if(!(tmp & (1<<7))){   /*接收到正确的syncword*/
				preamble_timeout = 200;		
						
			}
			else if (!(tmp & (1<<5))) { /*Crc 错误指示 */			
				preamble_timeout = 0;
				rece_falg = 0x01;  
			}
			else{
				rece_falg = 0x02; 
				//RfRecEn();
				preamble_timeout = 0;
			}
        }
        else{ /*发送完成*/
            preamble_timeout = 0;
            // RfRecEn();
			RfIdleEn();
        }
		
}
/**
  * \brief   rf中断顶半段
  * \param   None
  * \retval  None
  */
void rf_ist_0(void)
{
     rf_interrupt_pending = 1;
}

/**
  * \brief  外部检查是否有中断发生
  * \param   None
  * \retval  0 没有rf中断 1有rf中断
  */
unsigned char is_there_a_rf_interrupt_pending(void)
{
    return rf_interrupt_pending;
}
/**
  * \brief   清除中断标记
  * \param   None
  * \retval  None
  */
void clear_rf_interrupt_flags(void)
{
      rf_interrupt_pending =0 ;
}
#if 0
/**
 * \brief   timer2 中断服务
 * \param 	None
 * \retval  None
 */
void TIM2_IRQHandler(void)
{
    if (TRUE == Bt_GetIntFlag(TIM2)){
		
		 Bt_ClearIntFlag(TIM2);
		
		  if(preamble_timeout > 0){
            preamble_timeout--;
        }
        if(preamble_timeout  == 1){
			
        }
    }
}

#endif
