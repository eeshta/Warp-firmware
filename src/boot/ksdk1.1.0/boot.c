/*
	Authored 2016-2018. Phillip Stanley-Marbell.
	Additional contributions, 2018 onwards: See git blame.
	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:
	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.
	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.
	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"
#include "fsl_interrupt_manager.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devMMA8451Q.h"
#include "devSSD1331.h"
#include "pedometer.h"

#define WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF

volatile WarpI2CDeviceState            deviceMMA8451QState;

volatile i2c_master_state_t i2cMasterState;
volatile spi_master_state_t spiMasterState;
volatile spi_master_user_config_t spiUserConfig;

volatile uint32_t gWarpI2cBaudRateKbps = 200;
volatile uint32_t gWarpSpiBaudRateKbps = 200;
volatile uint32_t gWarpI2cTimeoutMilliseconds = 5;
volatile uint32_t gWarpSpiTimeoutMicroseconds = 5;

// Define some standard colours
#define WHITE           0xFFFFFF
#define RED             0xFF0000
#define GREEN           0x00FF00
#define BLUE            0x0000FF
#define CYAN            0x00FFFF

// Define a dimmed brightness
#define DIM             0x0A0A0A

// User defined thresholds for 'completing rings'
#define STEP_THRESHOLD  10
#define CAL_THRESHOLD   10
#define SPEED_THRESHOLD 10

// Modes
#define REST            0
#define WALK            1
#define RUN             2

// Used defined parameters
#define HEIGHT          168         // Height in cm
#define WEIGHT          65          // Weight in kg


void enableSPIpins(void)
{
    CLOCK_SYS_EnableSpiClock(0);

    // PTA8 -> MOSI
    PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);

    // PTA9 -> SCK
    PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

    /*
     *    Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
     *
     */
    uint32_t calculatedBaudRate;
    spiUserConfig.polarity = kSpiClockPolarity_ActiveHigh;
    spiUserConfig.phase = kSpiClockPhase_FirstEdge;
    spiUserConfig.direction = kSpiMsbFirst;
    spiUserConfig.bitsPerSec = gWarpSpiBaudRateKbps * 1000;
    SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
    SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}

void
disableSPIpins(void)
{
    SPI_DRV_MasterDeinit(0);


    /*    Warp KL03_SPI_MISO    --> PTA6    (GPI)        */
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

    /*    Warp KL03_SPI_MOSI    --> PTA7    (GPIO)        */
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

    /*    Warp KL03_SPI_SCK    --> PTB0    (GPIO)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);


    CLOCK_SYS_DisableSpiClock(0);
}



void
enableI2Cpins(uint8_t pullupValue)
{
    CLOCK_SYS_EnableI2cClock(0);

    /*    Warp KL03_I2C0_SCL    --> PTB3    (ALT2 == I2C)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);

    /*    Warp KL03_I2C0_SDA    --> PTB4    (ALT2 == I2C)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);


    I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
}

void
disableI2Cpins(void)
{
    I2C_DRV_MasterDeinit(0 /* I2C instance */);


    /*    Warp KL03_I2C0_SCL    --> PTB3    (GPIO)            */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);

    /*    Warp KL03_I2C0_SDA    --> PTB4    (GPIO)            */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

    /*
     *    Drive the I2C pins low
     */
    CLOCK_SYS_DisableI2cClock(0);
}



// Draw the background
void displayBackground(uint8_t mode, uint8_t setting)
{
    uint32_t text_colour;
    uint32_t line_colour;
    
    if(mode == REST)
    {
        text_colour = WHITE & DIM;      // Bitwise AND colour with DIM for dimmed colours
        line_colour = CYAN & DIM;       // (only works for primary and secondary colours but will do for use here)
    }
    else
    {
        text_colour = WHITE;
        line_colour = CYAN;
    }
    
    // STEPS
    writeCharacter(2, 63, 'S', text_colour);
    writeCharacter(10, 63, 'T', text_colour);
    writeCharacter(18, 63, 'E', text_colour);
    writeCharacter(26, 63, 'P', text_colour);
    writeCharacter(34, 63, 'S', text_colour);
    
    if (setting == 1) {
	    // CALS
	    writeCharacter(57, 63, 'C', text_colour);
	    writeCharacter(65, 63, 'A', text_colour);
	    writeCharacter(73, 63, 'L', text_colour);
	    writeCharacter(81, 63, 'S', text_colour);
    }
    
    else if (setting == 2) {
	    // CALS
	    writeCharacter(55, 63, 'S', text_colour);
	    writeCharacter(63, 63, 'P', text_colour);
	    writeCharacter(71, 63, 'E', text_colour);
	    writeCharacter(79, 63, 'E', text_colour);
	    writeCharacter(87, 63, 'D', text_colour);
    }
    // Draw Line
    writeCommand(kSSD1331CommandDRAWLINE);
    writeCommand(2);             // Col start
    writeCommand(63-19);         // Row start
    writeCommand(92);            // Col end
    writeCommand(63-19);         // Row end
    writeCommand((uint8_t)(line_colour >> 16) & 0xFF);          // Line red
    writeCommand((uint8_t)(line_colour >> 8) & 0xFF);           // Line green
    writeCommand((uint8_t)line_colour & 0xFF);                  // Line blue
}


// Draw the current mode
void displayMode(uint8_t mode)
{
    
    clearSection(20, 11, 76, 10);
    
    switch(mode)
    {
    case 0:
    {
    // ---
    writeCharacter(36, 11, '-', WHITE & DIM);
    writeCharacter(44, 11, '-', WHITE & DIM);
    writeCharacter(52, 11, '-', WHITE & DIM);
        
    break;
    }
    
    case 1:
    {
    // WALKING
    writeCharacter(20, 11, 'W', WHITE);
    writeCharacter(28, 11, 'A', WHITE);
    writeCharacter(36, 11, 'L', WHITE);
    writeCharacter(44, 11, 'K', WHITE);
    writeCharacter(52, 11, 'I', WHITE);
    writeCharacter(60, 11, 'N', WHITE);
    writeCharacter(68, 11, 'G', WHITE);
        
    break;
    }
    
    case 2:
    {
    // RUNNING
    writeCharacter(20, 11, 'R', RED);
    writeCharacter(28, 11, 'U', RED);
    writeCharacter(36, 11, 'N', RED);
    writeCharacter(44, 11, 'N', RED);
    writeCharacter(52, 11, 'I', RED);
    writeCharacter(60, 11, 'N', RED);
    writeCharacter(68, 11, 'G', RED);
    
    break;
    }
    }
}

// Draw the various counts - keep centred wit number of digits
void drawCount(uint8_t column, uint8_t row, uint32_t count, uint32_t colour)
{
    
    clearSection(column, row, 45, 10);
    
    if(count < 10)
    {
        writeDigit(column + 18, row, count, colour);
    }
    else if(count < 100)
    {
        writeDigit(column + 23, row, count % 10, colour);
        writeDigit(column + 14, row, count / 10, colour);
        
    }
    else if(count < 1000)
    {
        writeDigit(column + 28, row, count % 10, colour);
        writeDigit(column + 19, row, (count / 10) %  10, colour);
        writeDigit(column + 10, row, count / 100, colour);
    }
    else if(count < 10000)
    {
        writeDigit(column + 32, row, count % 10, colour);
        writeDigit(column + 23, row, count / 10 % 10, colour);
        writeDigit(column + 14, row, count / 100 % 10, colour);
        writeDigit(column + 5, row, count / 1000, colour);
    }
    else if(count < 100000)
    {
        writeDigit(column + 37, row, count % 10, colour);
        writeDigit(column + 28, row, count / 10 % 10, colour);
        writeDigit(column + 19, row, count / 100 % 10, colour);
        writeDigit(column + 10, row, count / 1000 % 10, colour);
        writeDigit(column + 1, row, count / 10000, colour);
    }
    else
    {
    SEGGER_RTT_WriteString(0, "\nERROR: Count Overflow");
    }
    
}

// Draw step count using drawCount
void drawSteps(uint8_t step_count, uint8_t mode)
{
    uint32_t colour;
    
    if(step_count >= STEP_THRESHOLD)
    {
        colour = GREEN;
    }
    else{
        colour = WHITE;
    }
    
    if(mode == REST)
    {
        colour = colour & DIM;
    }
    
    drawCount(0, 42, step_count, colour);
}

void drawSpeed(uint8_t speed, uint8_t mode)
{
    uint32_t colour;
    
    if(speed >= SPEED_THRESHOLD)
    {
        colour = GREEN;
    }
    else{
        colour = WHITE;
    }
    
    if(mode == REST)
    {
        colour = colour & DIM;
    }
    
    drawCount(51, 42, speed, colour);
}

// Draw cal count using drawCount
void drawCals(uint32_t cals, uint8_t mode)
{
    uint32_t colour;
    
    // Divide by 1000 to get back into Kcals
    cals = cals / 1000;
    
    
    if(cals >= CAL_THRESHOLD)
    {
        colour = GREEN;
    }
    else{
        colour = WHITE;
    }
    
    if(mode == REST)
    {
        colour = colour & DIM;
    }
    
    drawCount(51, 42, cals, colour);
}


int main(void)
{
    uint16_t                menuI2cPullupValue = 32768;
    /*
     *    Enable clock for I/O PORT A and PORT B
     */
    CLOCK_SYS_EnablePortClock(0);
    CLOCK_SYS_EnablePortClock(1);

    /*
     *    Setup board clock source.
     */
    g_xtal0ClkFreq = 32768U;

    /*
     *    Initialize KSDK Operating System Abstraction layer (OSA) layer.
     */
    OSA_Init();

    /*
     *    Setup SEGGER RTT to output as much as fits in buffers.
     *
     *    Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
     *    we might have SWD disabled at time of blockage.
     */
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

    SEGGER_RTT_WriteString(0, "\n\n\n\rBooting Step Counter in 3... ");
    OSA_TimeDelay(200);
    SEGGER_RTT_WriteString(0, "2... ");
    OSA_TimeDelay(200);
    SEGGER_RTT_WriteString(0, "1...\n\r");
    OSA_TimeDelay(200);

    /*
     *    Initialize the GPIO pins with the appropriate pull-up, etc.,
     *    defined in the inputPins and outputPins arrays (gpio_pins.c).
     *
     *    See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
     */
    GPIO_DRV_Init(inputPins /* input pins */, outputPins /* output pins */);

    /*
     *    Toggle LED3 (kWarpPinSI4705_nRST)
     */
    GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);

    // Enable SPI and I2C
    enableSPIpins();
    enableI2Cpins(menuI2cPullupValue);

    // Initialise and configure all devices
    devSSD1331init();
    initMMA8451Q(    0x1D    /* i2cAddress */,    &deviceMMA8451QState    );


    // Configure accelerometer - set high pass filter to remove gravity offset
    configureSensorMMA8451Q(0x00,/* Payload: Disable FIFO */
                            0x00,/* Normal read 14bit, 800Hz, normal, standby mode to write HP */
                            0x11,/* 4G Scale, HPF data enabled */
                            0x00,/* 16Hz cutoff */
                            0x01,/* Normal read 14bit, 800Hz, normal, active mode*/
                            menuI2cPullupValue
                            );

    
    uint32_t    step_count          = 0;            // Tracks step count
    uint32_t    last_step_count     = 0;            // Tracks last step count
    uint16_t    speed		    = 0;            // Dist in m/s
    uint32_t    cal_count           = 0;            // Tracks calories (Kcal / 1000)
    uint8_t     mode                = 0;            // Tracks mode
    uint8_t     last_mode           = 0;            // Tracks last mode
    uint32_t    start_time          = 0;            // Time at start of cycle
    uint8_t     run_time            = 0;            // Time for one cycle to run
    uint32_t    last_step_time      = 0;            // Last step time
    uint8_t     ticks               = 0;            // Tracks seconds as measured by 50 cycles
    uint8_t     setting             = 2;            // 1: CALS; 2: SPEED  
    
    
    // Initialise display information
    displayBackground(mode,setting);
    displayMode(mode);
    drawSteps(step_count,mode);
    
    if (setting == 1 ){
        drawCals(cal_count,mode);
    }

    
    else if (setting == 2){
        drawSpeed(speed,mode);
    }


    SEGGER_RTT_WriteString(0, "\nRunning...\n\r");
    
    while(1){
                // Measure start time
                start_time = OSA_TimeGetMsec();
                ticks ++;

                // Reset steps when 100000 is reached to avoid overflow
                if(step_count >= 100000)
                {
                    step_count = 0;
                    cal_count = 0;
                }
                
        
                // Count steps
                step_count = countSteps(step_count);
                mode = modeSelector(mode, last_step_time);
                
                // Count and update cals every second (every 50 cycles)
                if(ticks >= 50)
                {
                    cal_count = countCals(cal_count, HEIGHT, WEIGHT);
                    speed = calcSpeed();
                    
                    if (setting == 1){
                    	drawCals(cal_count, mode);
                    }
                    
                    else if (setting == 2){
                    	drawSpeed(speed, mode);
                    }
                    ticks = 0;
                }
        

                // Update steps and reset step timer
                if(step_count != last_step_count)
                {
                    drawSteps(step_count, mode);
                    
                    last_step_count = step_count;
                    last_step_time = OSA_TimeGetMsec();
                }
                
                // Update mode
                if(last_mode != mode)
                {
               	    if (setting == 1){
                    	drawCals(cal_count, mode);
                    }
                    
                    else if (setting == 2){
                    	drawSpeed(speed, mode);
                    }
                    displayBackground(mode, setting);
                    displayMode(mode);
                    drawSteps(step_count, mode);
                    
                    last_mode = mode;
                }
            
        
                // Run at 50Hz so delay for 20ms minus amount of time for code to run - loop rarely runs for >20ms
                run_time = OSA_TimeGetMsec() - start_time;
                if(run_time < 20)
                {
                    OSA_TimeDelay(20-run_time);
                }
    }
    return 0;
}


