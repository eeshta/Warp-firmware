/*
	Authored 2016-2021. Phillip Stanley-Marbell.
	
	Additional contributions, 2018: Jan Heck, Chatura Samarakoon, Youchao Wang, Sam Willis.
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

#include <stdint.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"
//#include "devSSD1331_extra.h"

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 11),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	//OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	warpEnableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 11u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);



	/*
	 *	Any post-initialization drawing commands go here.
	 */
	//...
	
	return 0;
}

void rectdraw(uint8_t col, uint8_t row, uint8_t endcol, uint8_t endrow)
{
	writeCommand(kSSD1331CommandDRAWRECT);
	writeCommand(col);
	writeCommand(row);
	writeCommand(endcol);
	writeCommand(endrow);				

}

void textcol(uint8_t red, uint8_t green, uint8_t blue){
	writeCommand(red);
	writeCommand(green);
	writeCommand(blue);
	writeCommand(red);
	writeCommand(green);
	writeCommand(blue);					
}
void fillscreen(uint8_t red, uint8_t green, uint8_t blue){
	rectdraw(0x00, 0x00, 0x5F, 0x3F);
	textcol(red, green, blue);
}

void clearScreen(void)
{
    writeCommand(kSSD1331CommandCLEAR);
    writeCommand(0x00);
    writeCommand(0x00);
    writeCommand(0x5F);
    writeCommand(0x3F);
    
    return;
}

/*
Helper functions for Drawing the Pedometer interface defined in pedometer.c
*/

void clearSection(uint8_t column, uint8_t row, uint8_t across, uint8_t down){
    
    // Screen is upside down
    row = 63 - row;
    
    writeCommand(kSSD1331CommandCLEAR);
    writeCommand(column); // Column start address
    writeCommand(row); // Row start address
    writeCommand(column + across);   // Column end address
    writeCommand(row + down);   // Row end address
    return;
}

void drawLine(uint8_t column, uint8_t row, uint8_t across, uint8_t down, uint32_t colour){
    
    uint8_t red     = (colour >> 16) & 0xFF;
    uint8_t green   = (colour >> 8) & 0xFF;
    uint8_t blue    = colour & 0xFF;
    
    writeCommand(kSSD1331CommandDRAWLINE);
    writeCommand(column);           // Column start address
    writeCommand(row);              // Row start address
    writeCommand(column + across);  // Column end address
    writeCommand(row + down);       // Row end address
    writeCommand(red);             // Red
    writeCommand(green);             // Green
    writeCommand(blue);             // Blue
}


// Draws digits 0-9 in a 7x11 shaped box starting at the top left coordinates given
void writeDigit(uint8_t column, uint8_t row, uint8_t digit, uint32_t colour)
{
    clearSection(column, row, 6, 10);
    
    row = 63 - row; // Screen is upside down
    
    switch (digit)
    {
    case 0:
    {
        drawLine(column + 1, row, 4, 0, colour);
        drawLine(column, row + 1, 0, 8, colour);
        drawLine(column + 6, row + 1, 0, 8, colour);
        drawLine(column + 1, row + 10, 4, 0, colour);
        drawLine(column + 1, row + 9, 4, -8, colour);
    
        break;
    }
    case 1:
    {
        drawLine(column + 3, row, 0, 10, colour);
        drawLine(column + 3, row, -2, 2, colour);
        drawLine(column + 1, row + 10, 4, 0, colour);

        break;
    }
    case 2:
    {
        drawLine(column + 1, row, 4, 0, colour);
        drawLine(column, row + 1, 0, 0, colour);
        drawLine(column + 6, row + 1, 0, 3, colour);
        drawLine(column, row + 10, 5, -5, colour);
        drawLine(column, row + 10, 6, 0, colour);

        break;
    }
    case 3:
    {
        drawLine(column + 1, row, 4, 0, colour);
        drawLine(column, row + 1, 0, 0, colour);
        drawLine(column + 6, row + 1, 0, 3, colour);
        drawLine(column + 6, row + 6, 0, 3, colour);
        drawLine(column + 1, row + 10, 4, 0, colour);
        drawLine(column, row + 9, 0, 0, colour);
        drawLine(column + 1, row + 5, 4, 0, colour);

        break;
    }
    case 4:
    {
        drawLine(column + 5, row, 0, 10, colour);
        drawLine(column, row + 5, 6, 0, colour);
        drawLine(column, row + 5, 5, -5, colour);
        
        break;
    }

    case 5:
    {
        drawLine(column, row, 6, 0, colour);
        drawLine(column, row, 0, 5, colour);
        drawLine(column, row + 5, 4, 0, colour);
        drawLine(column + 4, row + 5, 2, 2, colour);
        drawLine(column + 6, row + 7, 0, 2, colour);
        drawLine(column + 4, row + 10, 2, -2, colour);
        drawLine(column + 1, row + 10, 3, 0, colour);
        drawLine(column, row + 9, 0, 0, colour);
        
        break;
    }
    case 6:
    {
        drawLine(column + 1, row, 4, 0, colour);
        drawLine(column + 6, row + 1, 0, 0, colour);
        drawLine(column, row + 1, 0, 8, colour);
        drawLine(column + 1, row + 10, 4, 0, colour);
        drawLine(column + 6, row + 9, 0, -3, colour);
        drawLine(column + 1, row + 5, 4, 0, colour);

        break;
    }
    case 7:
    {
        drawLine(column, row, 6, 0, colour);
        drawLine(column + 6, row, -3, 6, colour);
        drawLine(column + 3, row + 6, 0, 4, colour);

        break;
    }
    case 8:
    {
        drawLine(column + 1, row, 4, 0, colour);
        drawLine(column, row + 1, 0, 3, colour);
        drawLine(column + 6, row + 1, 0, 3, colour);
        drawLine(column + 6, row + 6, 0, 3, colour);
        drawLine(column + 1, row + 10, 4, 0, colour);
        drawLine(column, row + 9, 0, -3, colour);
        drawLine(column + 1, row + 5, 4, 0, colour);

        break;
    }
    case 9:
    {
        drawLine(column + 1, row, 4, 0, colour);
        drawLine(column, row + 1, 0, 3, colour);
        drawLine(column + 6, row + 1, 0, 3, colour);
        drawLine(column + 6, row + 6, 0, 3, colour);
        drawLine(column + 1, row + 10, 4, 0, colour);
        drawLine(column, row + 9, 0, 0, colour);
        drawLine(column + 1, row + 5, 4, 0, colour);

        break;
    }
    }
    return;
}

// Writes a character symbol in a 7x11 sized box. Only the characters used in the implementation have been worked out
void writeCharacter(uint8_t column, uint8_t row, char character, uint32_t colour)
{
    clearSection(column, row, 6, 10);
    
    row = 63 - row; // Screen is upside down
    
    switch (character)
    {
    case 'A':
    {
        drawLine(column, row + 5, 3, -5, colour);
        drawLine(column + 3, row, 3, 5, colour);
        drawLine(column, row + 5, 5, 0, colour);
        drawLine(column, row + 5, 0, 5, colour);
        drawLine(column + 6, row + 5, 0, 5, colour);
        
        break;
    }
    case 'C':
    {
        drawLine(column + 2, row, 2, 0, colour);
        drawLine(column + 4, row, 2, 2, colour);
        drawLine(column, row + 2, 2, -2, colour);
        drawLine(column, row + 2, 0, 6, colour);
        drawLine(column, row + 8, 2, 2, colour);
        drawLine(column + 2, row + 10, 2, 0, colour);
        drawLine(column + 4, row + 10, 2, -2, colour);

        break;
    }
    case 'D':
    {
        
        drawLine(column, row, 0, 10, colour);
        drawLine(column, row, 3, 0, colour);
        drawLine(column, row + 10, 3, 0, colour);
        drawLine(column + 6, row + 2, 0, 6, colour);
        drawLine(column + 4, row, 2, 2, colour);
        drawLine(column + 4, row + 10, 2, -2, colour);

        break;
    }
    case 'E':
    {
        drawLine(column, row, 6, 0, colour);
        drawLine(column, row + 5, 6, 0, colour);
        drawLine(column, row + 10, 6 , 0, colour);
        drawLine(column, row, 0, 10, colour);

        break;
    }
    case 'G':
    {
        drawLine(column + 2, row, 2, 0, colour);
        drawLine(column + 4, row, 2, 2, colour);
        drawLine(column, row + 2, 2, -2, colour);
        drawLine(column, row + 2, 0, 6, colour);
        drawLine(column, row + 8, 2, 2, colour);
        drawLine(column + 2, row + 10, 2, 0, colour);
        drawLine(column + 4, row + 10, 2, -2, colour);
        drawLine(column + 3, row + 5, 3, 0, colour);
        drawLine(column + 6, row + 5, 0, 3, colour);


        break;
    }
    case 'I':
    {
        drawLine(column + 3, row, 0, 10, colour);
        drawLine(column + 1, row, 4, 0, colour);
        drawLine(column + 1, row + 10, 4, 0, colour);

        break;
    }
    case 'K':
    {
        drawLine(column, row, 0, 10, colour);
        drawLine(column + 1, row + 5, 5, -5, colour);
        drawLine(column + 1, row + 5, 5, 5, colour);
        
        break;
    }
    case 'L':
    {
        drawLine(column, row, 0, 10, colour);
        drawLine(column, row + 10, 6, 0, colour);
        
        break;
    }
    case 'N':
    {
        drawLine(column, row, 0, 10, colour);
        drawLine(column + 6, row, 0, 10, colour);
        drawLine(column, row, 6, 10, colour);
        
        break;
    }
    
    case 'P':
    {
        drawLine(column, row, 0, 10, colour);
        drawLine(column, row, 3, 0, colour);
        drawLine(column, row + 5, 3, 0, colour);
        drawLine(column + 6, row + 2, 0, 2, colour);
        drawLine(column + 4, row, 2, 2, colour);
        drawLine(column + 4, row + 5, 2, -2, colour);
        
        break;
    }
            
    case 'R':
    {
        drawLine(column, row, 0, 10, colour);
        drawLine(column, row, 3, 0, colour);
        drawLine(column, row + 5, 3, 0, colour);
        drawLine(column + 6, row + 2, 0, 2, colour);
        drawLine(column + 4, row, 2, 2, colour);
        drawLine(column + 4, row + 5, 2, -2, colour);
        drawLine(column + 3, row + 5, 3, 5, colour);
        
        break;
    }
            
    case 'S':
    {
        drawLine(column + 1, row, 4, 0, colour);
        drawLine(column + 6, row + 1, 0, 0, colour);
        drawLine(column, row + 1, 0, 3, colour);
        drawLine(column + 1, row + 5, 4, 0, colour);
        drawLine(column + 6, row + 6, 0, 3, colour);
        drawLine(column + 1, row + 10, 4, 0, colour);
        drawLine(column, row + 9, 0, 0, colour);
            
        break;
    }
            
    case 'T':
    {
        drawLine(column, row, 6, 0, colour);
        drawLine(column + 3, row, 0, 10, colour);
                
        break;
    }
            
    case 'U':
    {
        drawLine(column, row, 0, 7, colour);
        drawLine(column, row + 8, 2, 2, colour);
        drawLine(column + 2, row + 10, 2, 0, colour);
        drawLine(column + 4, row + 10, 2, -2, colour);
        drawLine(column + 6, row, 0, 7, colour);
        
                    
        break;
    }
            
    case 'W':
    {
        drawLine(column, row, 0, 10, colour);
        drawLine(column + 6, row, 0, 10, colour);
        drawLine(column, row + 10, 3, -3, colour);
        drawLine(column + 6, row + 10, -3, -3, colour);
                        
        break;
    }
            
    case '-':
    {
        drawLine(column, row + 5, 6, 0, colour);
        break;
    }
            
    }
        
    return;
}
