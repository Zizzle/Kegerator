#include "FreeRTOS.h"
#include "ili_lcd_general.h"
#include <stdio.h>
#include "ili9320_font.h"

// Compatible list:
// ili9320 ili9325 ili9328
// LG4531

#ifdef __CC_ARM                			 /* ARM Compiler 	*/
#define lcd_inline   				static __inline
#elif defined (__ICCARM__)        		/* for IAR Compiler */
#define lcd_inline 					inline
#elif defined (__GNUC__)        		/* GNU GCC Compiler */
#define lcd_inline 					static __inline
#else
#define lcd_inline                  static
#endif

#define rw_data_prepare()               write_cmd(34)


/********* control ***********/
#include "stm32f10x.h"
//#include "board.h"

/* LCD is connected to the FSMC_Bank1_NOR/SRAM2 and NE2 is used as ship select signal */
/* RS <==> A2 */
#define LCD_REG              (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define LCD_RAM              (*((volatile unsigned short *) 0x60020000)) /* RS = 1 */

static void display_ON(void);
static void display_OFF(void);
static void gamma_SET(void);

static void LCD_FSMCConfig(void)
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  p;

    /*-- FSMC Configuration ------------------------------------------------------*/
    p.FSMC_AddressSetupTime = 2;             /* 地址建立时间  */
    p.FSMC_AddressHoldTime = 1;              /* 地址保持时间  */
    p.FSMC_DataSetupTime = 3;                /* 数据建立时间  */
    p.FSMC_BusTurnAroundDuration = 0;        /* 总线返转时间  */
    p.FSMC_CLKDivision = 0;                  /* 时钟分频      */
    p.FSMC_DataLatency = 0;                  /* 数据保持时间  */
    p.FSMC_AccessMode = FSMC_AccessMode_A;   /* FSMC 访问模式 */

    /* Color LCD configuration ------------------------------------
       LCD configured as follow:
          - Data/Address MUX = Disable
          - Memory Type = SRAM
          - Data Width = 16bit
          - Write Operation = Enable
          - Extended Mode = Enable
          - Asynchronous Wait = Disable */
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}

static void delay(int cnt)
{
    volatile unsigned int dl;
    while(cnt--)
    {
        for(dl=0; dl<100; dl++)asm("nop");
    }
}



static void lcd_port_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | 
                           RCC_APB2Periph_GPIOE | 
                           RCC_APB2Periph_GPIOF | 
                           RCC_APB2Periph_GPIOG, 
                           ENABLE);
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /*
      FSMC_D0 ~ FSMC_D3
      PD14 FSMC_D0   PD15 FSMC_D1   PD0  FSMC_D2   PD1  FSMC_D3

      FSMC_A16 PD11 - RS
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |
        GPIO_Pin_1 | 
        GPIO_Pin_14 | 
        GPIO_Pin_15 | 
        GPIO_Pin_11;
    GPIO_Init(GPIOD,&GPIO_InitStructure);

    /*
      FSMC_D4 ~ FSMC_D12
      PE7 ~ PE15  FSMC_D4 ~ FSMC_D12
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 |
        GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10| 
        GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | 
        GPIO_Pin_14 | GPIO_Pin_15;

    GPIO_Init(GPIOE,&GPIO_InitStructure);

    /* FSMC_D13 ~ FSMC_D15   PD8 ~ PD10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOD,&GPIO_InitStructure);


    /* RD-PD4 WR-PD5 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOD,&GPIO_InitStructure);
    
    /* NE1/NCE2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOD,&GPIO_InitStructure);


//    GPIO_WriteBit(GPIOE, GPIO_Pin_6, Bit_SET);
    
    //lcd_rst();	 
    GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    vTaskDelay(10/portTICK_RATE_MS);					   
    GPIO_SetBits(GPIOE, GPIO_Pin_1 );		 //	 
    vTaskDelay(10/portTICK_RATE_MS);					   

    LCD_FSMCConfig();
}

lcd_inline void write_cmd(unsigned short cmd)
{
    LCD_REG = cmd;
}

lcd_inline unsigned short read_data(void)
{
    return LCD_RAM;
}

lcd_inline void write_data(unsigned short data_code )
{
    LCD_RAM = data_code;
}

lcd_inline void write_reg(unsigned char reg_addr,unsigned short reg_val)
{
    write_cmd(reg_addr);
    write_data(reg_val);
}

lcd_inline unsigned short read_reg(unsigned char reg_addr)
{
    unsigned short val=0;
    write_cmd(reg_addr);
    val = read_data();
    return (val);
}

/********* control <只移植以上函数即可> ***********/

static unsigned short deviceid=0;//设置一个静态变量用来保存LCD的ID

//返回LCD的ID
unsigned int lcd_getdeviceid(void)
{
    return deviceid;
}

unsigned short BGR2RGB(unsigned short c)
{
    u16  r, g, b, rgb;

    b = (c>>0)  & 0x1f;
    g = (c>>5)  & 0x3f;
    r = (c>>11) & 0x1f;

    rgb =  (b<<11) + (g<<5) + (r<<0);

    return( rgb );
}

void lcd_SetCursor(unsigned int x,unsigned int y)
{
    write_reg(32,x);    /* 0-239 */
    write_reg(33,y);    /* 0-319 */
}

/* 读取指定地址的GRAM */
unsigned short lcd_read_gram(unsigned int x,unsigned int y)
{
    unsigned short temp;
    lcd_SetCursor(x,y);
    rw_data_prepare();
    /* dummy read */
    temp = read_data();
    temp = read_data();
    return temp;
}

void lcd_clear(unsigned short Color)
{
    unsigned int index=0;
    lcd_SetCursor(0,0);
    rw_data_prepare();                      /* Prepare to write GRAM */
    for (index=0; index<(LCD_WIDTH*LCD_HEIGHT); index++)
    {
        write_data(Color);
    }
}

void lcd_data_bus_test(void)
{
    unsigned short temp1;
    unsigned short temp2;
    printf("bus test\r\n");

    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) );
    //printf("A\r\n");

    /* wirte */
    lcd_SetCursor(0,0);
    //   printf("B\r\n");
    rw_data_prepare();
    //  printf("C\r\n");
    write_data(0x5555);
    //  printf("D\r\n");
      write_data(0xAAAA);
    //  printf("E\r\n");

    /* read */
    lcd_SetCursor(0,0);
    if (
        (deviceid ==0x9325)
        || (deviceid ==0x9328)
        || (deviceid ==0x9320)
    )
    {
        temp1 = BGR2RGB( lcd_read_gram(0,0) );
        temp2 = BGR2RGB( lcd_read_gram(1,0) );
    }
    else if( deviceid ==0x4532 )
    {
        //printf("F\r\n");
        temp1 = lcd_read_gram(0,0);
        temp2 = lcd_read_gram(1,0);
//        temp1 = BGR2RGB( lcd_read_gram(0,0) );
//        temp2 = BGR2RGB( lcd_read_gram(1,0) );
    }

    if( (temp1 == 0x5555) && (temp2 == 0xAAAA) )
    {
        printf("Data bus test pass!\r\n");
    }
    else
    {
        printf("Data bus test error: %04X %04X\r\n",temp1,temp2);
    }
}

void lcd_gram_test(void)
{
    unsigned short temp;
    unsigned int test_x;
    unsigned int test_y;

    printf("LCD GRAM test....\r\n");

    /* write */
    temp=0;
    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) );
    lcd_SetCursor(0,0);
    rw_data_prepare();
    for(test_y=0; test_y<76800; test_y++)
    {
        write_data(temp);
        temp++;
    }

    /* read */
    temp=0;

    if (
        (deviceid ==0x9320)
        || (deviceid ==0x9325)
        || (deviceid ==0x9328)
    )
    {
        for(test_y=0; test_y<320; test_y++)
        {
            for(test_x=0; test_x<240; test_x++)
            {
                if( BGR2RGB( lcd_read_gram(test_x,test_y) ) != temp++)
                {
                    printf("  LCD GRAM ERR!!");
                    
                    while(1);
                }
            }
        }
        printf("GRAM TEST PASS!\r\n");
    }
    else if( deviceid ==0x4532 )
    {
        for(test_y=0; test_y<320; test_y++)
        {
            for(test_x=0; test_x<240; test_x++)
            {
                if(  lcd_read_gram(test_x,test_y) != temp++)
                {
                    printf("LCD GRAM ERR!!");
                    // while(1);
                }
            }
        }
        printf("TEST PASS!\r\n");
    }
}


//#define LCD_WR_CMD(a,b) write_reg(a,b)

//void Delay(__IO uint32_t nCount)
//{
//  for(; nCount != 0; nCount--);
//}
#define Delay(x)  vTaskDelay( (x)/portTICK_RATE_MS )


static void power_SET(void)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    vTaskDelay(10/portTICK_RATE_MS);
    GPIO_SetBits(GPIOE, GPIO_Pin_1 );		 //	 
    vTaskDelay(10/portTICK_RATE_MS);    
    
   

//############# void Power_Set(void) ################//
    write_reg(0x0000,0x0001); //Start Oscillation
    Delay(10);

    write_reg(0x0015,0x0030);// internal voltage reg at 0.65*Vci
    write_reg(0x0011,0x0040);//Power Control Setup Reg 1 
                              //Step-Up Circuit 1,2 = Fosc/128,
                              // VciOut - 1 * Vci
    write_reg(0x0010,0x1628);//Power Control Setup Reg 1
    write_reg(0x0012,0x0000);//Power Control Setup Reg 3
    write_reg(0x0013,0x104d);//Power Control Setup Reg 4
    Delay(10);
    write_reg(0x0012,0x0010);//VREGout = 1.47
    Delay(10);
    write_reg(0x0010,0x2620);//Power Control Setup Reg1
    write_reg(0x0013,0x344d); //304d
    Delay(10);
    
    write_reg(0x0001,0x0100);//Driver Output Control
    write_reg(0x0002,0x0300);//Driving Range Control
    write_reg(0x0003,0x1030);//Entry Mode BGR, Horizontal, then vertical
    write_reg(0x0008,0x0604);//Display Control, first 4 and last 6
                              //lines blank
    write_reg(0x0009,0x0000);//Display Control
    write_reg(0x000A,0x0008);//Output FMARK every 1 Frame
    
    write_reg(0x0041,0x0002);
    write_reg(0x0060,0x2700);
    write_reg(0x0061,0x0001);
    write_reg(0x0090,0x0182);
    write_reg(0x0093,0x0001);
    write_reg(0x00a3,0x0010);
    Delay(10);
}

static void gamma_SET(void){
    Delay(10);
    write_reg(0x30,0x0000);		
    write_reg(0x31,0x0502);		
    write_reg(0x32,0x0307);		
    write_reg(0x33,0x0305);		
    write_reg(0x34,0x0004);		
    write_reg(0x35,0x0402);		
    write_reg(0x36,0x0707);		
    write_reg(0x37,0x0503);		
    write_reg(0x38,0x1505);		
    write_reg(0x39,0x1505);
    Delay(10);
}
static void display_ON(void)
{   
    Delay(10);
    write_reg(0x0007,0x0001);
    Delay(10);
    write_reg(0x0007,0x0021);
    write_reg(0x0007,0x0023);
    Delay(10);
    write_reg(0x0007,0x0033);
    Delay(10);
    write_reg(0x0007,0x0133);
}

static void display_OFF(void)
{
    Delay(10);
    write_reg(0x0007,0x0001);
    Delay(10);
}

void lcd_Initializtion(void)
{
    lcd_port_init();

    deviceid = read_reg(0x00);
    
    //printf("Device: %x\r\n", deviceid);
    
    /* deviceid check */
    if(deviceid != 0x4532)
    {
        printf("Invalid LCD ID:%08X\r\n",deviceid);
        printf("Please check you hardware and configure.");
        while(1);
    }
    else
    {
        printf("\r\nLCD Device ID : %04X ",deviceid);
    }

        
    power_SET(); //Set up the power Registers

    gamma_SET(); //Set up the Gamma Registers
    
    display_OFF(); //Switch off the display for tests
    lcd_data_bus_test();
    lcd_gram_test(); 
    display_ON();  //Switch on the display
        
    lcd_clear( White );
    lcd_clear( Blue );
    lcd_clear( Red  );
    lcd_clear( White );

  
}


void lcd_DrawHLine(int x1, int x2, int col, int y ) 
{
    uint16_t ptr;
    
       
    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) );
    
    lcd_SetCursor(x1, y);
    rw_data_prepare(); /* Prepare to write GRAM */
    while (x1 <= x2)
    {
        write_data(col);
        x1 ++;
        
    }

}


void lcd_DrawVLine(int y1, int y2, int col, int x)
{
 unsigned short p;

    /* get color pixel */
   
    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    write_reg(0x0003,(1<<12)|(1<<5)|(0<<4) | (1<<3) );

    lcd_SetCursor(x, y1);
    rw_data_prepare(); /* Prepare to write GRAM */
    while (y1 <= y2)
    {
        write_data(col);
        y1++;
    }
}

void lcd_DrawRect(int x1, int y1, int x2, int y2, int col)
{
    lcd_DrawVLine(y1, y2, col, x1);
    lcd_DrawVLine(y1, y2, col, x2);
    lcd_DrawHLine(x1, x2, col, y1);
    lcd_DrawHLine(x1, x2, col, y2);
}

void lcd_DrawPixel(int x, int y, int col){
    vTaskSuspendAll();
    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) ); // set up
    rw_data_prepare(); /* Prepare to write GRAM */
    lcd_SetCursor(x, y);
    write_data(col);
    xTaskResumeAll();
}

void lcd_PutChar(unsigned int x,unsigned int y,unsigned char c,unsigned int  charColor,unsigned int bkColor)   
{   
    unsigned int  i=0;   
    unsigned int  j=0;   
    unsigned int d = 0;
    unsigned char tmp_char=0;   
    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) ); // set up
                                                       // orientation
    // printf("orientation set up\r\n");
    
    
    for (i=0;i<16;i++)   
    {   
        
        lcd_SetCursor(x,y+i); //move down each line    
        tmp_char=ascii_8x16[((c-0x20)*16)+i];   //get the char from lookup
        for (j=0;j<8;j++)   
        {   
            if ( (tmp_char >> 7-j) & 0x01 == 0x01) //text pixel
            {   
                rw_data_prepare();
                write_data(charColor);
                //printf("ascii pixel created\r\n");
            }   
            else   //background pixel
            {   
                rw_data_prepare();
                write_data(bkColor);
                //printf("background pixel created\r\n");
            }   
        }   
    }   
}   


void lcd_PutString(unsigned int x, unsigned int y, unsigned char * s, unsigned int textColor, unsigned int bkColor)
{
    unsigned char * temp = s;
    unsigned int cnt = 0;

    while (*temp != '\0')
    {
        lcd_PutChar(x+cnt, y, *temp, textColor, bkColor);
        cnt+=8;
        temp++;

    }
}


/*******************************************************************************
* Function Name  : LCD_DrawCircle
* Description    : Displays a circle.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position.
*                  - Height: display rectangle height.
*                  - Width: display rectangle width.
* Output         : None
* Return         : None
*******************************************************************************/
void lcd_DrawCircle(unsigned char Xpos, unsigned int Ypos, unsigned int Radius)
{
    int  D;/* Decision Variable */
    unsigned int  CurX;/* Current X Value */
    unsigned int  CurY;/* Current Y Value */
    
    D = 3 - (Radius << 1);
    CurX = 0;
    CurY = Radius;
    
    while (CurX <= CurY)
    {
        rw_data_prepare();
        lcd_SetCursor(Xpos + CurX, Ypos + CurY);
        write_data(Black);
        
        lcd_SetCursor(Xpos + CurX, Ypos - CurY);
        write_data(Black);
        
        lcd_SetCursor(Xpos - CurX, Ypos + CurY);
        write_data(Black);
        
        lcd_SetCursor(Xpos - CurX, Ypos - CurY);
        write_data(Black);
        
        lcd_SetCursor(Xpos + CurY, Ypos + CurX);
        write_data(Black);
        
        lcd_SetCursor(Xpos + CurY, Ypos - CurX);
        write_data(Black);
        
        lcd_SetCursor(Xpos - CurY, Ypos + CurX);
        write_data(Black);
        
        lcd_SetCursor(Xpos - CurY, Ypos - CurX);
        write_data(Black);
        
        if (D < 0)
        {
            D += (CurX << 2) + 6;
        }
        else
        {
            D += ((CurX - CurY) << 2) + 10;
            CurY--;
        }
        CurX++;
    }
}


void lcd_DrawBMP(unsigned portCHAR *Pict)
{

}


#if 0 //This code was in MAIN... taken for referenc for columns and
     //lines etc
int putChar( int ch )
{
static unsigned portSHORT usColumn = 0, usRefColumn = mainCOLUMN_START;
static unsigned portCHAR ucLine = 0;

	if( ( usColumn == 0 ) && ( ucLine == 0 ) )
	{
            lcd_clear(White);
	}

	if( ch != '\n' )
	{
		// Display one character on LCD 
		//LCD_DisplayChar( ucLine, usRefColumn, (u8) ch );

		// Decrement the column position by 16 */
		usRefColumn -= mainCOLUMN_INCREMENT;

		/* Increment the character counter */
		usColumn++;
		if( usColumn == mainMAX_COLUMN )
		{
			ucLine += mainROW_INCREMENT;
			usRefColumn = mainCOLUMN_START;
			usColumn = 0;
		}
	}
	else
	{
		/* Move back to the first column of the next line. */
		ucLine += mainROW_INCREMENT;
		usRefColumn = mainCOLUMN_START;
		usColumn = 0;
	}

	/* Wrap back to the top of the display. */
	if( ucLine >= mainMAX_LINE )
	{
		ucLine = 0;
	}

	return ch;
}


#endif
