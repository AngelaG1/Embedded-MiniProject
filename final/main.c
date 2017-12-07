//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Blinky
// Application Overview - The objective of this application is to showcase the
//                        GPIO control using Driverlib api calls. The LEDs
//                        connected to the GPIOs on the LP are used to indicate
//                        the GPIO output. The GPIOs are driven high-low
//                        periodically in order to turn on-off the LEDs.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_Blinky_Application
// or
// docs\examples\CC32xx_Blinky_Application.pdf
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup blinky
//! @{
//
//****************************************************************************

// Standard includes
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "uart.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"

#include "pin_mux_config.h"
//SPI
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
//Timer
#include "timer_if.h"
#include "gpio_if.h"

// Common interface includes
#include "uart_if.h"


//
#define APPLICATION_VERSION     "1.1.1"
#define SPI_IF_BIT_RATE  100000
#define GSPI_BASE               0x44021000
#define MAP_PRCMPeripheralClockGet \
        PRCMPeripheralClockGet
#define PRCM_GSPI                 0x00000003
#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100
#define TIMER_CFG_PERIODIC                0x00000022
#define TIMER_A         0x000000ff

#define CONSOLE              UARTA0_BASE
#define UartGetChar()        MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)       MAP_UARTCharPut(CONSOLE,c)

static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulADCBase;
static volatile unsigned long ADCIntNum = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long MotorIntNum;

#define FIFO_ELEMENTS 20
#define FIFO_SIZE (FIFO_ELEMENTS + 1)
int FIFO[FIFO_SIZE];
int FIFO_In, FIFO_Out;

int DataArray[3000];
int DataIndex = -1;
int TotalSamples=2800;
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
void FIFO_Init(void);
int FIFOPush(int);
int FIFOPop();
int FIFORead(int);
int PushRoutine(int);
int SaturateSpeed(int, int);
void MotorTurner(int);
void ADC_Read();
int DifferenceChecker();
static void BoardInit(void);
void TimerMotorIntHandler(void);
void TimerADCIntHandler(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

//*****************************************************************************
//
//! Configures the pins as GPIOs and peroidically toggles the lines
//!
//! \param None
//!
//! This function
//!    1. Configures 3 lines connected to LEDs as GPIO
//!    2. Sets up the GPIO pins as output
//!    3. Periodically toggles each LED one by one by toggling the GPIO line
//!
//! \return None
//
//*****************************************************************************
void FIFO_Init(void)
{
    FIFO_In= 0;
    FIFO_Out= 0;
}

int FIFOPush(int new)
{
    if(FIFO_In == (( FIFO_Out - 1 + FIFO_SIZE) % FIFO_SIZE))
    {
        return -1; /* FIFO Full*/
    }

    FIFO[FIFO_In] = new;

    FIFO_In = (FIFO_In + 1) % FIFO_SIZE;

    return 0; // No errors
}

int FIFOPop()
{
    if(FIFO_In == FIFO_Out)
    {
        return -1; /* FIFO Empty - nothing to get*/
    }

    FIFO_Out = (FIFO_Out + 1) % FIFO_SIZE;

    return 0; // No errors
}

int FIFORead(int element) //element from 0 to FIFO_SIZE-1
{
    if(FIFO_In == FIFO_Out)
    {
        return -1; /* FIFO Empty - nothing to get*/
    }

    if(element>9)
    {
        return -1; /* Invalid index */
    }

    return FIFO[(FIFO_Out+element)% FIFO_SIZE];
}

int PushRoutine(int currentVal)
{
    if(GPIOPinRead(GPIOA1_BASE, 0x20)){
        currentVal++;
        if(currentVal>7)
            currentVal=7;
        else
            while(GPIOPinRead(GPIOA1_BASE, 0x20));
    }
    else if(GPIOPinRead(GPIOA2_BASE, 0x40)){
        currentVal--;
        if(currentVal<0)
            currentVal=0;
        else
            while(GPIOPinRead(GPIOA2_BASE, 0x40));
    }

    if(currentVal & 0x01)
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    else
        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    if(currentVal & 0x02)
        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
    else
        GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
    if(currentVal & 0x04)
        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    else
        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    return currentVal;
}

void MotorTurner(int mode){
    if(mode==0)
        GPIOPinWrite(GPIOA0_BASE, 0x78, 0x50);
    else if(mode==1)
        GPIOPinWrite(GPIOA0_BASE, 0x78, 0x48);
    else if(mode==2)
        GPIOPinWrite(GPIOA0_BASE, 0x78, 0x28);
    else if(mode==3)
        GPIOPinWrite(GPIOA0_BASE, 0x78, 0x30);
}


int SaturateSpeed(int current, int finall){

    if(current < finall){
        current=current+1;
        if(current > finall)
            current=finall;
    }

    if(current > finall){
        current=current-1;
        if(current < finall)
            current=finall;
    }

    return current;
}

void ADC_Read(){
    unsigned char rx1,rx2;
    GPIOPinWrite(GPIOA0_BASE, 0x80, 0x00);
    SPITransfer(GSPI_BASE,0,&rx1,0x01,1);
    SPITransfer(GSPI_BASE,0,&rx2,0x01,1);
    //FIFOPop();
    //FIFOPush( (((int)rx1 & 0x1F)<<5) + ((int)rx2>>3));
    char str[6];
    sprintf(str, "%d", (((int)rx1 & 0x1F)<<5) + ((int)rx2>>3));
    Message(str);
    Message("\r\n");
    //UartPutChar('\n');

    GPIOPinWrite(GPIOA0_BASE, 0x80, 0xFF);
}

void TimerMotorIntHandler(void){
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulBase);

    MotorTurner(MotorIntNum%4);
    MotorIntNum ++;

}

void TimerADCIntHandler(void){
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulADCBase);

    ADC_Read();
    ADCIntNum ++;

}


int DifferenceChecker(){
    int array[10];
    int i,sum=0;

    //first a more smooth curve is created using average over 10 samples
    for(i=9; i<18; i++){
        sum+= FIFORead(i);
    }
    array[9]=sum/10;
    for(i=8; i>=0; i--){
        sum+= FIFORead(i)-FIFORead(i+10);
        array[i]= sum/10;
    }

    //
    printf("arr0=%d\n",array[0]);
    printf("arr9=%d\n",array[9]);
    if ((array[0]-array[9]) >14){
        return ADCIntNum;
    }
    return -1;
}



//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif

    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//! This function
//!    1. Invokes the LEDBlinkyTask
//!
//! \return None.
//
//****************************************************************************
int
main()
{
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Power on the corresponding GPIO port B for 9,10,11.
    // Set up the GPIO lines to mode 0 (GPIO)
    //
    PinMuxConfig();
    //
        // Reset SPI
        //
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);
    MAP_SPITransfer(GSPI_BASE,0,0,50,SPI_CS_ENABLE|SPI_CS_DISABLE);

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);


    //terminal
    InitTerm();
    ClearTerm();

    //Timer Interrupt
    g_ulBase = TIMERA0_BASE;
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerMotorIntHandler);
    Timer_IF_Start(g_ulBase, TIMER_A, 25); //25ms for each 7 degrees: 1round per second

    g_ulADCBase = TIMERA1_BASE;
    Timer_IF_Init(PRCM_TIMERA1, g_ulADCBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(g_ulADCBase, TIMER_A, TimerADCIntHandler);
    Timer_IF_Start(g_ulADCBase, TIMER_A, 10); //10ms= 100 Hz sampling frequency

    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_ALL_LED_IND);

    //
    // Start the Code
    //

    GPIOPinWrite(GPIOA0_BASE, 0x80, 0xFF);

    char cCharacter[12];
    int Counter=0;
    int CurrFreq=1, FinalFreq=1;

    while(ADCIntNum<22);

    while(1){

        Counter= 0;
        while(!(cCharacter[Counter] == '\r' || cCharacter[Counter] == '\n')){
            cCharacter[Counter] = UartGetChar();
            Counter++;
            printf("Counter=%d\n",Counter);
            printf("Val=%c\n",cCharacter[Counter]);
        }

        FinalFreq= atoi(cCharacter);
        CurrFreq= SaturateSpeed(CurrFreq, FinalFreq);
        if(CurrFreq){
            Timer_IF_ReLoad(g_ulBase, TIMER_A, 25/CurrFreq);
        }
    }

    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
