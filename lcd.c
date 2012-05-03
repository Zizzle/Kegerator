#include <stdio.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "console.h"
#include "menu.h"
#include "lcd.h"
#include "touch.h"
#include "images.h"
#include "ili9320_font.h"
#include "semphr.h"

// Compatible list:
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

//#define Delay(x)  ;

xSemaphoreHandle xLcdSemaphore;

void Delay(unsigned i)
{
	unsigned n;
	for(;i;i--)
	{
		for(n=0;n<3100;n++)
		{
			asm("nop");
		}
	}
}


//vTaskDelay( (x)/portTICK_RATE_MS )

/********* control ***********/
#include "stm32f10x.h"


/* LCD is connected to the FSMC_Bank1_NOR/SRAM2 
   and NE2 is used as ship select signal */
/* RS <==> A2 */
#define LCD_REG (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define LCD_RAM (*((volatile unsigned short *) 0x60020000)) /* RS = 1 */
//---------------------------------------------------------------------/
//               Static Functions and prototypes
//---------------------------------------------------------------------/
static void display_ON(void);
static void display_OFF(void);
static void gamma_SET(void);
void lcd_PutString(unsigned int x, 
                          unsigned int y, 
                          const char * s, 
                          unsigned int textColor, 
                          unsigned int bkColor);

static void lcd_PutChar(unsigned int  x,
                        unsigned int  y,
                        unsigned char c,
                        unsigned int  charColor,
                        unsigned int  bkColor);   

static void lcd_DrawCircleFill(unsigned char Xpos, 
                               unsigned int Ypos, 
                               unsigned int Radius, 
                               unsigned int Col);
static void lcd_DrawCircle(unsigned char Xpos, 
                           unsigned int Ypos, 
                           unsigned int Radius);
static void lcd_data_bus_test(void);
static void lcd_gram_test(void);
static void lcd_port_init(void);
static void power_SET(void);
static unsigned short deviceid=0;

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



//---------------------------------------------------------------------/
//                       LCD RTOS TASK
//---------------------------------------------------------------------/


//---------------------------------------------------------------------/
//                       INIT
//---------------------------------------------------------------------/
void lcd_init(void)
{
    xLcdSemaphore = xSemaphoreCreateMutex();

    printf("LCD init %x\r\n", xLcdSemaphore);

    lcd_port_init(); //initialise IO Registers

    /* deviceid check */    
    deviceid = read_reg(0x00); 
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
    
    //SET UP//
    power_SET(); //Set up the power Registers
    gamma_SET(); //Set up the Gamma Registers

#if 1
    //TEST//
    display_OFF(); //Switch off the display for tests
    lcd_data_bus_test();
    lcd_gram_test(); 
    display_ON();  //Switch on the display
#endif

#if 1

    //Eye Candy//
    Delay(10);      
    lcd_clear( Red );
    Delay(100);      
    lcd_clear( Black );
    Delay(100);
    lcd_clear( Red );
    Delay(100);
    lcd_clear( Black );

    lcd_PutString(5, 10, "Colour LCD Touch\0", Cyan, Black );
    lcd_PutString(5, 26, "Testing FreeRTOS\0", Blue2, Black);
    lcd_PutString(5, 44, "ARM Cortex CM3 Processor\0", Black, White);
    lcd_PutString(5, 60, "STM32F103VE 512k Flash\0", Black, White);

    Delay(1000);     
#endif
    printf("Lcd_init done\r\n");
}

//---------------------------------------------------------------------/
//                       FMSC Setup
//---------------------------------------------------------------------/
static void LCD_FSMCConfig(void)
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  p;

    /*-- FSMC Configuration ----------------------------------------------*/
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

//---------------------------------------------------------------------/
//                       Hardware (I/O) Setup
//---------------------------------------------------------------------/
static void lcd_port_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
        
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
                         RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG |
                         RCC_APB2Periph_AFIO, ENABLE);



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

    GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    Delay(10/portTICK_RATE_MS);					   
    GPIO_SetBits(GPIOE, GPIO_Pin_1 );		 //	 
    Delay(10/portTICK_RATE_MS);					   

    LCD_FSMCConfig();
}

//---------------------------------------------------------------------/
//                       REGISTER SET UP
//---------------------------------------------------------------------/
static void power_SET(void)
{
    //Toggle Reset Pin
    GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    Delay(10/portTICK_RATE_MS);
    GPIO_SetBits(GPIOE, GPIO_Pin_1 );		 //	 
    Delay(10/portTICK_RATE_MS);    
    
   
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

static void lcd_data_bus_test(void)
{
    unsigned short temp1;
    unsigned short temp2;
    printf("bus test\r\n");

    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) );

    /* Write Alternating Bit Values */
    lcd_SetCursor(0,0);
    rw_data_prepare();
    write_data(0x5555);
    write_data(0xAAAA);
  

    /* Read it back and check*/
    lcd_SetCursor(0,0);
    temp1 = lcd_read_gram(0,0);
    temp2 = lcd_read_gram(1,0);
    if( (temp1 == 0x5555) && (temp2 == 0xAAAA) )
    {
        printf("Data bus test pass!\r\n");
    }
    else
    {
        printf("Data bus test error: %04X %04X\r\n",temp1,temp2);
    }
}

static void lcd_gram_test(void)
{
    unsigned short temp; //Temp value to put in GRAM
    unsigned int test_x;
    unsigned int test_y;

    printf("LCD GRAM test....\r\n");

    /* write */
    temp=0;
    
    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) );
    lcd_SetCursor(0,0);
    rw_data_prepare();
    for(test_y=0; test_y<76800; test_y++) //test 320*240 Memory locations
    {
        write_data(temp); //put temp in GRAM
        temp++;
    }

    /* Read it back from GRAM and Test */
    temp=0;
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




//---------------------------------------------------------------------/
//                      Interface Functions 
//---------------------------------------------------------------------/



static void lcd_DrawHLine(int x1, int x2, int col, int y ) 
{
    uint16_t ptr;
    //Set up registers for horizontal scanning
    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) );
    
    lcd_SetCursor(x1, y); //start position
    rw_data_prepare(); /* Prepare to write GRAM */
    portENTER_CRITICAL();
    while (x1 <= x2)
    {
        write_data(col);
        x1 ++;
        
    }
    portEXIT_CRITICAL();
}


static void lcd_DrawVLine(int y1, int y2, int col, int x)
{
    unsigned short p;
    
    //Set up registers for vertical scanning 
    write_reg(0x0003,(1<<12)|(1<<5)|(0<<4) | (1<<3) );
    
    lcd_SetCursor(x, y1); //start position
    rw_data_prepare(); /* Prepare to write GRAM */
    portENTER_CRITICAL();
    while (y1 <= y2)
    {
        write_data(col);
        y1++;
    }
    portEXIT_CRITICAL();
}

void lcd_DrawRect(int x1, int y1, int x2, int y2, int col)
{
    lcd_DrawVLine(y1, y2, col, x1);
    lcd_DrawVLine(y1, y2, col, x2);
    lcd_DrawHLine(x1, x2, col, y1);
    lcd_DrawHLine(x1, x2, col, y2);
}

static void lcd_DrawPixel(int x, int y, int col){
   
    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) ); // set up
    
    lcd_SetCursor(x, y);
    rw_data_prepare(); /* Prepare to write GRAM */    
    write_data(col);
   
}

static void lcd_PutChar(unsigned int x,unsigned int y,unsigned char c,unsigned int  charColor,unsigned int bkColor)   
{   
    unsigned int  i=0;   
    unsigned int  j=0;   
    unsigned int d = 0;
    unsigned char tmp_char=0;   
    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) ); // set up
                                                       // orientation
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
            }   
            else   //background pixel
            {   
                rw_data_prepare();
                write_data(bkColor);
            }   
        }   
    }   
}   


void lcd_PutString(unsigned int x, unsigned int y, const char * s, unsigned int textColor, unsigned int bkColor)
{
    const char * temp = s;
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
static void lcd_DrawCircle(unsigned char Xpos, unsigned int Ypos, unsigned int Radius)
{
    int  D;/* Decision Variable */
    unsigned int  CurX;/* Current X Value */
    unsigned int  CurY;/* Current Y Value */
    
    D = 3 - (Radius << 1);
    CurX = 0;
    CurY = Radius;
    
    while (CurX <= CurY)
    {
        
        lcd_SetCursor(Xpos + CurX, Ypos + CurY);
        rw_data_prepare();
        write_data(Black);
        
        lcd_SetCursor(Xpos + CurX, Ypos - CurY);
        rw_data_prepare();
        write_data(Black);
        
        lcd_SetCursor(Xpos - CurX, Ypos + CurY);
        rw_data_prepare();
        write_data(Black);
        
        lcd_SetCursor(Xpos - CurX, Ypos - CurY);
        rw_data_prepare();
        write_data(Black);
        
        lcd_SetCursor(Xpos + CurY, Ypos + CurX);
        rw_data_prepare();
        write_data(Black);
        
        lcd_SetCursor(Xpos + CurY, Ypos - CurX);
        rw_data_prepare();
        write_data(Black);
        
        lcd_SetCursor(Xpos - CurY, Ypos + CurX);
        rw_data_prepare();
        write_data(Black);
        
        lcd_SetCursor(Xpos - CurY, Ypos - CurX);
        rw_data_prepare();
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


/*******************************************************************************
* Function Name  : lcd_DrawCircleFull
* Description    : Displays a circle filled with the color.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position.
*                  - Height: display rectangle height.
*                  - Width: display rectangle width.
* Output         : None
* Return         : None
*******************************************************************************/
static void lcd_DrawCircleFill(unsigned char Xpos, unsigned int Ypos, unsigned int Radius, unsigned int Col)
{
    int  D;/* Decision Variable */
    unsigned int  CurX;/* Current X Value */
    unsigned int  CurY;/* Current Y Value */
    unsigned int dec = Radius;
    while ((dec)>0){
        Radius = dec;
        D = 3 - (Radius << 1);
        CurX = 0; 
        CurY=Radius; 
        while (CurX <= CurY)
        {
            
            lcd_SetCursor(Xpos + CurX, Ypos + CurY);
            rw_data_prepare();
            write_data(Col);
            
            lcd_SetCursor(Xpos + CurX, Ypos - CurY);
            rw_data_prepare();
            write_data(Col);
            
            lcd_SetCursor(Xpos - CurX, Ypos + CurY);
            rw_data_prepare();
            write_data(Col);
            
            lcd_SetCursor(Xpos - CurX, Ypos - CurY);
            rw_data_prepare();
            write_data(Col);
            
            lcd_SetCursor(Xpos + CurY, Ypos + CurX);
            rw_data_prepare();
            write_data(Col);
            
            lcd_SetCursor(Xpos + CurY, Ypos - CurX);
            rw_data_prepare();
            write_data(Col);
            
            lcd_SetCursor(Xpos - CurY, Ypos + CurX);
            rw_data_prepare();
            write_data(Col);
            
            lcd_SetCursor(Xpos - CurY, Ypos - CurX);
            rw_data_prepare();
            write_data(Col);
            
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
        dec--;
    }
}

//display the menu text into the cells
void lcd_text_menu(uint16_t x_pos, uint16_t cell, const char * text)
{
    if (cell>6) cell = 1;
    uint16_t cellpos = (cell*50)-(25+8);
    //uint16_t rowpos = x_pos*(240/8);
    lcd_PutString(x_pos, cellpos, text, Red, Black);
}


//prints the menu 
void lcd_menu_update(struct menu * menu){
    unsigned char ii = 0;

    lcd_clear(Black);
    //  vTaskDelay(100);
    for (ii = 0; ii < 6 && menu[ii].text; ii++)
    {
        lcd_text_menu(10, ii + 1, menu[ii].text);
    }

    lcd_DrawRect(0, 0, 150, 50, Red);
    lcd_DrawRect(0, 50, 150, 100, Red);
    lcd_DrawRect(0, 100, 150, 150, Red);
    lcd_DrawRect(0, 150, 150, 200, Red);
    lcd_DrawRect(0, 200, 150, 250, Red);
    lcd_DrawRect(0, 250, 150, 300, Red);
    lcd_DrawRect(160, 0, 230, 100, Red);
    lcd_DrawRect(160, 100, 230, 200, Red);
    lcd_DrawRect(160, 200, 230, 300, Red);
    lcd_DrawCircleFill(195, 150, 20, Green);
    lcd_DrawCircleFill(195, 250, 20, Red);
    lcd_PutString(179, 46, "BACK", Red, Black); 
   
}


void lcd_draw_buttons(void) {
 
    lcd_DrawRect(160, 0, 230, 100, Red);
    lcd_DrawRect(160, 100, 230, 200, Red);
    lcd_DrawRect(160, 200, 230, 300, Red);
    lcd_DrawCircleFill(195, 150, 20, Green);
    lcd_DrawCircleFill(195, 250, 20, Red);
    lcd_PutString(179, 46, "BACK", Red, Black); 

}

void lcd_draw_back_button(void){
    lcd_DrawRect(160, 0, 230, 100, Red);
    lcd_PutString(179, 46, "BACK", Red, Black); 
}
void lcd_draw_applet_options(const char * text_1, char * text_2, char * text_3, char * text_4)
{

    lcd_DrawRect(0, 0, 150, 50, Red);
    lcd_DrawRect(0, 50, 150, 100, Red);
    lcd_DrawRect(0, 140, 120, 220, Red);
    lcd_DrawRect(120, 140, 239, 220, Red);
    lcd_DrawRect(0, 220, 120, 300, Red);
    lcd_DrawRect(120, 220, 239, 300, Red);
    
    if (text_1 != NULL)
        lcd_PutString(2, 141, text_1, Cyan, Black);
    if (text_2 != NULL)
        lcd_PutString(121, 141, text_2, Cyan, Black);
    if (text_3 != NULL)
        lcd_PutString(2, 221, text_3, Cyan, Black);
    if (text_4 != NULL)
        lcd_PutString(121, 221, text_4, Cyan, Black);
    
    
}


/**
  * @brief  Copy 4 bytes from bitmap array to 32Bit buffer
  * @param  ptrBitmap - Bitmap pointer
  * @param  ptr32BitBuffer - 32Bit buffer to fill
  * @retval None
  */
void BmpBuffer32BitRead(uint32_t* ptr32BitBuffer, uint8_t* ptrBitmap)
{
  *ptr32BitBuffer = 0;
  *ptr32BitBuffer = (*ptrBitmap);
  *ptr32BitBuffer += (*(ptrBitmap + 1)) << 8;
  *ptr32BitBuffer += (*(ptrBitmap + 2)) << 16;
  *ptr32BitBuffer += (*(ptrBitmap + 3)) << 24;
}

/*******************************************************************************
* Function Name  : LCD_SetDisplayWindow
* Description    : Sets a display window
* Input          : - Xpos: specifies the X buttom left position.
*                  - Ypos: specifies the Y buttom left position.
*                  - Height: display window height.
*                  - Width: display window width.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
{
  /* Horizontal GRAM Start Address */
  if(Xpos >= Height)
  {
    write_reg(0x0080, (Xpos - Height + 1));
    write_reg(0x0080, (Xpos));
  }
  else
  {
      printf("outside region\r\n");
    write_reg(0x0080, 0);
  }
  /* Horizontal GRAM End Address */
  write_reg(0x0082, Ypos);
  /* Vertical GRAM Start Address */
  if(Ypos >= Width)
  {
    write_reg(0x0081, (Ypos));
  }
  else
  {
      printf("outside region\r\n");
    write_reg(0x0081, 0);
  }
  /* Vertical GRAM End Address */
  write_reg(0x0082, Ypos+Height);

  lcd_SetCursor(Xpos, Ypos);
}

void DrawBMP(uint8_t* ptrBitmap)
{
  uint32_t uDataAddr = 0, uBmpSize = 0;
  uint16_t uBmpData;
  uint8_t ii;
  BmpBuffer32BitRead(&uBmpSize, ptrBitmap + 2);
  BmpBuffer32BitRead(&uDataAddr, ptrBitmap + 10);

  /* Set GRAM write direction and BGR = 1 */
  /* I/D=00 (Horizontal : decrement, Vertical : decrement) */
  /* AM=1 (address is updated in vertical writing direction) */
  write_reg(0x0003, 0x1030);
  rw_data_prepare(); /* Prepare to write GRAM */
  
  /* Read bitmap data and write them to LCD */
 
     for (; uDataAddr < uBmpSize; uDataAddr += 2)
     {
     
             uBmpData = (uint16_t)(*(ptrBitmap + uDataAddr)) + (uint16_t)((*(ptrBitmap + uDataAddr + 1)) << 8);
             write_data( uBmpData );
     
     
     }

/*  if ( pLcdHwParam.LCD_Connection_Mode == GL_SPI )
    GL_LCD_CtrlLinesWrite(pLcdHwParam.LCD_Ctrl_Port_NCS, pLcdHwParam.LCD_Ctrl_Pin_NCS, GL_HIGH);
*/
  /* Set GRAM write direction and BGR = 1 */
  /* I/D = 01 (Horizontal : increment, Vertical : decrement) */
  /* AM = 1 (address is updated in vertical writing direction) */
      write_reg(0x0003, 0x1031);
}




#define MAX_X 319
#define MAX_Y 239


static uint16_t bg_col;
static volatile xTaskHandle lcdUsingTask = NULL;

void lcd_lock()
{
    printf("Locking %x\r\n", xLcdSemaphore);
	while( xSemaphoreTake( xLcdSemaphore, ( portTickType ) 100 ) != pdTRUE )
	{
		printf("Waiting a long time for LCD\r\n");
	}
	lcdUsingTask = xTaskGetCurrentTaskHandle();
}

void lcd_release()
{
    printf("Releasing %x\r\n", xLcdSemaphore);
	xSemaphoreGive(xLcdSemaphore);	
	lcdUsingTask = NULL;
}

static unsigned char const AsciiLib[95][16] = {
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*" ",0*/
		{0x00,0x00,0x00,0x18,0x3C,0x3C,0x3C,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00},/*"!",1*/
		{0x00,0x00,0x00,0x66,0x66,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*""",2*/
		{0x00,0x00,0x00,0x36,0x36,0x7F,0x36,0x36,0x36,0x7F,0x36,0x36,0x00,0x00,0x00,0x00},/*"#",3*/
		{0x00,0x18,0x18,0x3C,0x66,0x60,0x30,0x18,0x0C,0x06,0x66,0x3C,0x18,0x18,0x00,0x00},/*"$",4*/
		{0x00,0x00,0x70,0xD8,0xDA,0x76,0x0C,0x18,0x30,0x6E,0x5B,0x1B,0x0E,0x00,0x00,0x00},/*"%",5*/
		{0x00,0x00,0x00,0x38,0x6C,0x6C,0x38,0x60,0x6F,0x66,0x66,0x3B,0x00,0x00,0x00,0x00},/*"&",6*/
		{0x00,0x00,0x00,0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"'",7*/
		{0x00,0x00,0x00,0x0C,0x18,0x18,0x30,0x30,0x30,0x30,0x30,0x18,0x18,0x0C,0x00,0x00},/*"(",8*/
		{0x00,0x00,0x00,0x30,0x18,0x18,0x0C,0x0C,0x0C,0x0C,0x0C,0x18,0x18,0x30,0x00,0x00},/*")",9*/
		{0x00,0x00,0x00,0x00,0x00,0x36,0x1C,0x7F,0x1C,0x36,0x00,0x00,0x00,0x00,0x00,0x00},/*"*",10*/
		{0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00},/*"+",11*/
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x0C,0x18,0x00,0x00},/*",",12*/
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"-",13*/
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x00,0x00},/*".",14*/
		{0x00,0x00,0x00,0x06,0x06,0x0C,0x0C,0x18,0x18,0x30,0x30,0x60,0x60,0x00,0x00,0x00},/*"/",15*/
		{0x00,0x00,0x00,0x1E,0x33,0x37,0x37,0x33,0x3B,0x3B,0x33,0x1E,0x00,0x00,0x00,0x00},/*"0",16*/
		{0x00,0x00,0x00,0x0C,0x1C,0x7C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x00,0x00,0x00,0x00},/*"1",17*/
		{0x00,0x00,0x00,0x3C,0x66,0x66,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00,0x00,0x00,0x00},/*"2",18*/
		{0x00,0x00,0x00,0x3C,0x66,0x66,0x06,0x1C,0x06,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},/*"3",19*/
		{0x00,0x00,0x00,0x30,0x30,0x36,0x36,0x36,0x66,0x7F,0x06,0x06,0x00,0x00,0x00,0x00},/*"4",20*/
		{0x00,0x00,0x00,0x7E,0x60,0x60,0x60,0x7C,0x06,0x06,0x0C,0x78,0x00,0x00,0x00,0x00},/*"5",21*/
		{0x00,0x00,0x00,0x1C,0x18,0x30,0x7C,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},/*"6",22*/
		{0x00,0x00,0x00,0x7E,0x06,0x0C,0x0C,0x18,0x18,0x30,0x30,0x30,0x00,0x00,0x00,0x00},/*"7",23*/
		{0x00,0x00,0x00,0x3C,0x66,0x66,0x76,0x3C,0x6E,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},/*"8",24*/
		{0x00,0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x3E,0x0C,0x18,0x38,0x00,0x00,0x00,0x00},/*"9",25*/
		{0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x00,0x00},/*":",26*/
		{0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x00,0x1C,0x1C,0x0C,0x18,0x00,0x00},/*";",27*/
		{0x00,0x00,0x00,0x06,0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x06,0x00,0x00,0x00,0x00},/*"<",28*/
		{0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"=",29*/
		{0x00,0x00,0x00,0x60,0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x60,0x00,0x00,0x00,0x00},/*">",30*/
		{0x00,0x00,0x00,0x3C,0x66,0x66,0x0C,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00},/*"?",31*/
		{0x00,0x00,0x00,0x7E,0xC3,0xC3,0xCF,0xDB,0xDB,0xCF,0xC0,0x7F,0x00,0x00,0x00,0x00},/*"@",32*/
		{0x00,0x00,0x00,0x18,0x3C,0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00,0x00,0x00,0x00},/*"A",33*/
		{0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x7C,0x66,0x66,0x66,0x7C,0x00,0x00,0x00,0x00},/*"B",34*/
		{0x00,0x00,0x00,0x3C,0x66,0x66,0x60,0x60,0x60,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},/*"C",35*/
		{0x00,0x00,0x00,0x78,0x6C,0x66,0x66,0x66,0x66,0x66,0x6C,0x78,0x00,0x00,0x00,0x00},/*"D",36*/
		{0x00,0x00,0x00,0x7E,0x60,0x60,0x60,0x7C,0x60,0x60,0x60,0x7E,0x00,0x00,0x00,0x00},/*"E",37*/
		{0x00,0x00,0x00,0x7E,0x60,0x60,0x60,0x7C,0x60,0x60,0x60,0x60,0x00,0x00,0x00,0x00},/*"F",38*/
		{0x00,0x00,0x00,0x3C,0x66,0x66,0x60,0x60,0x6E,0x66,0x66,0x3E,0x00,0x00,0x00,0x00},/*"G",39*/
		{0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00},/*"H",40*/
		{0x00,0x00,0x00,0x3C,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00},/*"I",41*/
		{0x00,0x00,0x00,0x06,0x06,0x06,0x06,0x06,0x06,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},/*"J",42*/
		{0x00,0x00,0x00,0x66,0x66,0x6C,0x6C,0x78,0x6C,0x6C,0x66,0x66,0x00,0x00,0x00,0x00},/*"K",43*/
		{0x00,0x00,0x00,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00,0x00,0x00,0x00},/*"L",44*/
		{0x00,0x00,0x00,0x63,0x63,0x77,0x6B,0x6B,0x6B,0x63,0x63,0x63,0x00,0x00,0x00,0x00},/*"M",45*/
		{0x00,0x00,0x00,0x63,0x63,0x73,0x7B,0x6F,0x67,0x63,0x63,0x63,0x00,0x00,0x00,0x00},/*"N",46*/
		{0x00,0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},/*"O",47*/
		{0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x7C,0x60,0x60,0x60,0x60,0x00,0x00,0x00,0x00},/*"P",48*/
		{0x00,0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x0C,0x06,0x00,0x00},/*"Q",49*/
		{0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x7C,0x6C,0x66,0x66,0x66,0x00,0x00,0x00,0x00},/*"R",50*/
		{0x00,0x00,0x00,0x3C,0x66,0x60,0x30,0x18,0x0C,0x06,0x66,0x3C,0x00,0x00,0x00,0x00},/*"S",51*/
		{0x00,0x00,0x00,0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00},/*"T",52*/
		{0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},/*"U",53*/
		{0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00,0x00,0x00,0x00},/*"V",54*/
		{0x00,0x00,0x00,0x63,0x63,0x63,0x6B,0x6B,0x6B,0x36,0x36,0x36,0x00,0x00,0x00,0x00},/*"W",55*/
		{0x00,0x00,0x00,0x66,0x66,0x34,0x18,0x18,0x2C,0x66,0x66,0x66,0x00,0x00,0x00,0x00},/*"X",56*/
		{0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00},/*"Y",57*/
		{0x00,0x00,0x00,0x7E,0x06,0x06,0x0C,0x18,0x30,0x60,0x60,0x7E,0x00,0x00,0x00,0x00},/*"Z",58*/
		{0x00,0x00,0x00,0x3C,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x3C,0x00},/*"[",59*/
		{0x00,0x00,0x00,0x60,0x60,0x30,0x30,0x18,0x18,0x0C,0x0C,0x06,0x06,0x00,0x00,0x00},/*"\",60*/
		{0x00,0x00,0x00,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00},/*"]",61*/
		{0x00,0x18,0x3C,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"^",62*/
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00},/*"_",63*/
		{0x00,0x00,0x00,0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"'",64*/
		{0x00,0x00,0x00,0x00,0x00,0x3C,0x06,0x06,0x3E,0x66,0x66,0x3E,0x00,0x00,0x00,0x00},/*"a",65*/
		{0x00,0x00,0x00,0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x66,0x7C,0x00,0x00,0x00,0x00},/*"b",66*/
		{0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00,0x00,0x00,0x00},/*"c",67*/
		{0x00,0x00,0x00,0x06,0x06,0x3E,0x66,0x66,0x66,0x66,0x66,0x3E,0x00,0x00,0x00,0x00},/*"d",68*/
		{0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x66,0x7E,0x60,0x60,0x3C,0x00,0x00,0x00,0x00},/*"e",69*/
		{0x00,0x00,0x00,0x1E,0x30,0x30,0x30,0x7E,0x30,0x30,0x30,0x30,0x00,0x00,0x00,0x00},/*"f",70*/
		{0x00,0x00,0x00,0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x66,0x3E,0x06,0x06,0x7C,0x00},/*"g",71*/
		{0x00,0x00,0x00,0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00},/*"h",72*/
		{0x00,0x00,0x18,0x18,0x00,0x78,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00},/*"i",73*/
		{0x00,0x00,0x0C,0x0C,0x00,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x78,0x00},/*"j",74*/
		{0x00,0x00,0x00,0x60,0x60,0x66,0x66,0x6C,0x78,0x6C,0x66,0x66,0x00,0x00,0x00,0x00},/*"k",75*/
		{0x00,0x00,0x00,0x78,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00},/*"l",76*/
		{0x00,0x00,0x00,0x00,0x00,0x7E,0x6B,0x6B,0x6B,0x6B,0x6B,0x63,0x00,0x00,0x00,0x00},/*"m",77*/
		{0x00,0x00,0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00},/*"n",78*/
		{0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},/*"o",79*/
		{0x00,0x00,0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x66,0x7C,0x60,0x60,0x60,0x00},/*"p",80*/
		{0x00,0x00,0x00,0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x66,0x3E,0x06,0x06,0x06,0x00},/*"q",81*/
		{0x00,0x00,0x00,0x00,0x00,0x66,0x6E,0x70,0x60,0x60,0x60,0x60,0x00,0x00,0x00,0x00},/*"r",82*/
		{0x00,0x00,0x00,0x00,0x00,0x3E,0x60,0x60,0x3C,0x06,0x06,0x7C,0x00,0x00,0x00,0x00},/*"s",83*/
		{0x00,0x00,0x00,0x30,0x30,0x7E,0x30,0x30,0x30,0x30,0x30,0x1E,0x00,0x00,0x00,0x00},/*"t",84*/
		{0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x3E,0x00,0x00,0x00,0x00},/*"u",85*/
		{0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00,0x00,0x00,0x00},/*"v",86*/
		{0x00,0x00,0x00,0x00,0x00,0x63,0x6B,0x6B,0x6B,0x6B,0x36,0x36,0x00,0x00,0x00,0x00},/*"w",87*/
		{0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00,0x00,0x00,0x00},/*"x",88*/
		{0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x0C,0x18,0xF0,0x00},/*"y",89*/
		{0x00,0x00,0x00,0x00,0x00,0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00,0x00,0x00,0x00},/*"z",90*/
		{0x00,0x00,0x00,0x0C,0x18,0x18,0x18,0x30,0x60,0x30,0x18,0x18,0x18,0x0C,0x00,0x00},/*"{",91*/
		{0x00,0x00,0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00},/*"|",92*/
		{0x00,0x00,0x00,0x30,0x18,0x18,0x18,0x0C,0x06,0x0C,0x18,0x18,0x18,0x30,0x00,0x00},/*"}",93*/
		{0x00,0x00,0x00,0x71,0xDB,0x8E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"~",94*/
};

static __inline void lcd_write_ram_prepare(void)
{
	write_cmd(0x22);
}

static void lcd_set_cursor(uint16_t Xpos,uint16_t Ypos)
{
	write_reg(32, Ypos); /* Row */
	write_reg(33, Xpos); /* Line */ 
}

static void lcd_char_xy(unsigned short Xpos,unsigned short Ypos,unsigned char c,unsigned short charColor,unsigned short bkColor)
{
	unsigned short i=0;
	unsigned short j=0;
	const unsigned char *buffer = AsciiLib[(c - 32)] ;
	unsigned char tmp_char=0;
	for (i=0;i<16;i++)
	{
		lcd_set_cursor(Xpos,Ypos+i);
		lcd_write_ram_prepare();

		tmp_char=buffer[i];
		for (j=0;j<8;j++)
		{
			uint16_t col = ( (tmp_char >> 7-j) & 0x01) ? charColor : bkColor;
			write_data(col);
		}
	}
}

#define LCD_LOCK char auto_lock = 0;if (lcdUsingTask != xTaskGetCurrentTaskHandle()){ lcd_lock(); auto_lock = 1; }
#define LCD_UNLOCK if (auto_lock) lcd_release()

void lcd_text_xy(uint16_t Xpos, uint16_t Ypos, const char *str,uint16_t Color, uint16_t bkColor)
{
	LCD_LOCK;
	uint8_t TempChar;
	
//	printf("lcd text %d,%d %s\r\n", Xpos, Ypos, str);
	
	while ((TempChar=*str++))
	{
		lcd_char_xy(Xpos,Ypos,TempChar,Color,bkColor);    
		if (Xpos < MAX_X - 8)
		{
			Xpos+=8;
		} 
		else if (Ypos < MAX_Y - 16)
		{
			Xpos=0;
			Ypos+=16;
		}   
		else
		{
			Xpos=0;
			Ypos=0;
		}    
	}
	LCD_UNLOCK;
}

void lcd_text(uint8_t col, uint8_t row, const char *text)
{
	lcd_text_xy(col * 8, row * 16, text, 0xFFFF, bg_col);
}

void lcd_fill(uint16_t xx, uint16_t yy, uint16_t ww, uint16_t hh, uint16_t color)
{
	LCD_LOCK;
	for (int ii = 0; ii < hh; ii++)
	{	  
		lcd_set_cursor(xx, yy + ii);
		lcd_write_ram_prepare();
		for (int jj = 0; jj < ww; jj++)
		{
			write_data(color);
		}
	}	
	LCD_UNLOCK;
}

#include <stdarg.h>
void lcd_printf(uint8_t col, uint8_t row, uint8_t ww, const char *fmt, ...)
{
	LCD_LOCK;
	char message[31];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(message, sizeof(message) - 1, fmt, ap);
    va_end(ap);

    while (len < ww && len < sizeof(message) - 2)
    {
    	message[len++] = ' ';
    }
    message[len] = 0;
    
    lcd_text(col, row, message);

    LCD_UNLOCK;
}

void lcd_clear(uint16_t Color)
{
	LCD_LOCK;	
	uint32_t index=0;
	lcd_set_cursor(0,0); 
	lcd_write_ram_prepare(); /* Prepare to write GRAM */
	for(index=0;index<76800;index++)
	{
		write_data(Color);
	}
	LCD_UNLOCK;
}

void lcd_background(uint16_t color)
{
	bg_col = color;
}
