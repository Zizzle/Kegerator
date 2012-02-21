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
#define Delay(x)  vTaskDelay( (x)/portTICK_RATE_MS )

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

void vLCDTask( void *pvParameters )
{
    
    unsigned int ii = 0;
    portTickType xLastExecutionTime = xTaskGetTickCount();
    portBASE_TYPE xStatus;
    portTickType xTicksToWait = 50/portTICK_RATE_MS;
    char buf[30] = "Message From LCD Task\r\n";
    
    /* Initialise the LCD and display a startup message. */
    lcd_init();          
    lcd_PutString(5, 10, "Colour LCD Touch\0", Cyan, Black );
    lcd_PutString(5, 26, "Testing FreeRTOS\0", Blue2, Black);
    lcd_PutString(5, 44, "ARM Cortex CM3 Processor\0", Black, White);
    lcd_PutString(5, 60, "STM32F103VE 512k Flash\0", Black, White);
    // LCD_SetDisplayWindow(100,100, 20,20);
    //DrawBMP(RButtonA);
    vTaskDelay(1000);
    menu_update();

    //Drop the priority back now... 
    vTaskPrioritySet(NULL, tskIDLE_PRIORITY+2);
    
    for( ;; )
    {
        //DO NOTHING AT THE MOMENT
        vTaskDelayUntil(&xLastExecutionTime, 200/portTICK_RATE_MS );
    
        /*if (xStatus == pdTRUE) //if we dont have something to print, leave
        {
            lcd_DrawHLine(rxStruct.uiX, rxStruct.uiX, Black, rxStruct.uiY);
            xQueueSendToBack(xConsoleQueue, &buf, 0); //send message
                                                      //to console
                                                      }
                                              */
        // PRINT THE SCROLLING BOTTOM LINE
        lcd_PutString(0, 305, "                              ", Cyan, Black);
        lcd_PutString(ii, 305, "Brew Machine MKIV", Cyan, Black);
        ii++;
        if (ii > 100) ii=0;
        taskYIELD();
    }   
    
}

//---------------------------------------------------------------------/
//                       INIT
//---------------------------------------------------------------------/
void lcd_init(void)
{
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

    //TEST//
    display_OFF(); //Switch off the display for tests
    lcd_data_bus_test();
    lcd_gram_test(); 
    display_ON();  //Switch on the display

    //Eye Candy//
    Delay(10);      
    lcd_clear( Red );
    Delay(100);      
    lcd_clear( Black );
    Delay(100);
    lcd_clear( Red );
    Delay(100);
    lcd_clear( Black );
   
  
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


    GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    vTaskDelay(10/portTICK_RATE_MS);					   
    GPIO_SetBits(GPIOE, GPIO_Pin_1 );		 //	 
    vTaskDelay(10/portTICK_RATE_MS);					   

    LCD_FSMCConfig();
}

//---------------------------------------------------------------------/
//                       REGISTER SET UP
//---------------------------------------------------------------------/
static void power_SET(void)
{
    //Toggle Reset Pin
    GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    vTaskDelay(10/portTICK_RATE_MS);
    GPIO_SetBits(GPIOE, GPIO_Pin_1 );		 //	 
    vTaskDelay(10/portTICK_RATE_MS);    
    
   
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



void lcd_clear(unsigned short Color)
{
    unsigned int index=0;
    lcd_SetCursor(0,0);
    rw_data_prepare();                      /* Prepare to write GRAM */
    portENTER_CRITICAL();
    for (index=0; index<(LCD_WIDTH*LCD_HEIGHT); index++)
    {
        write_data(Color);
    }
    portEXIT_CRITICAL();
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
