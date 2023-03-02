
#include <stdlib.h>
#include <math.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devMMA8451Q.h"
#include "pedometer.h"
#include "devSSD1331.h"

#define BUFF_LENGTH             9
#define STEP_BUFF_LENGTH        150             // Record steps for last 3s for mode selection
#define THRESH                  2000            // Step threshold for LPF data
#define DERIV_THRESH            250             // Step threshold for deriv data
#define RUNNING_THRESH          8               // Threshold for running mode - 8 steps in 3s = 2.66Hz
#define REST_TIME               2500            // Enter rest mode after 5s


int16_t     diff_coeff[BUFF_LENGTH]     =   {0,0,0,1,0,-1,0,0,0};               // FIR derivative
int16_t     lpf_coeff[BUFF_LENGTH]      =   {1,6,22,44,54,44,22,6,1};           // FIR LPF filter designed using MATLAB

uint32_t    data_buff[BUFF_LENGTH]      =   {0};                                // Data buffer
uint32_t    lpf_buff[BUFF_LENGTH]       =   {0};                                // Buffer to store lpf signal
int32_t     deriv_buff[BUFF_LENGTH]     =   {0};                                // Buffer to store derivative of signal
bool        step_buff[STEP_BUFF_LENGTH] =   {0};                                // Buffer to hold recent previous steps

int8_t      n                           =   BUFF_LENGTH - 1;                    // Index of last number in buffer
uint8_t steps_in_buffer                 =   0;                                  // Keep track of how many steps in buffer for mode selection

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];


/*
 *	Override Warp firmware's use of these pins and define new aliases for our build.
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


// Combine the stream from x,y,z by squaring, adding and square-rooting
int16_t  combine_stream(int16_t x_data, int16_t y_data, int16_t z_data){
    
    int16_t comb_data = (int16_t)sqrt(x_data*x_data + y_data*y_data + z_data*z_data);
    
    warpPrint(" %d,", x_data);
    warpPrint(" %d,", y_data);
    warpPrint(" %d,", z_data);
    warpPrint("\n");
    //SEGGER_RTT_printf(0, "%d\n", comb_data);
    
    
    return comb_data;
}


// FIR Low Pass Filter
void lpf(void){
    
    uint32_t moving_lpf = 0;

    for(uint8_t i = 0; i < BUFF_LENGTH; i++){
            moving_lpf += lpf_coeff[i]*data_buff[n - i];
    }
    // Store in LPF buffer
    lpf_buff[n] = moving_lpf / 8;       // Divide to avoid overflow - resolution ok for step counting
}


// FIR derivative
void  diff(void){
    
    int32_t     moving_deriv = 0;

    for(uint8_t i = 0; i < BUFF_LENGTH; i++){
            moving_deriv += diff_coeff[i]*lpf_buff[n - i];
    }
    // Store in derivative buffer
    deriv_buff[n] = moving_deriv;
}

// Calculate Stride length
float calcStride(uint8_t height){
    uint8_t steps_in_2s = 0;

    steps_in_2s = steps_in_buffer/1.5;

    if (steps_in_2s <= 2){
        return height/5;
    }
    else if (steps_in_2s > 2 && steps_in_2s <= 3){
        return height/4;
    }
    else if (steps_in_2s > 3 && steps_in_2s <= 4){
        return height/3;
    }
    else if (steps_in_2s > 4 && steps_in_2s <= 5){
        return height/2;
    }
    else if (steps_in_2s > 5 && steps_in_2s <= 6){
        return height/1.2;
    }
    else if (steps_in_2s > 6 && steps_in_2s <= 8){
        return height;
    }
    else if (steps_in_2s >= 8) {
        return height*1.2;
    }
}

// Calculate Total distance travelled
uint32_t calcDistance(uint32_t distance){
    float stride = 0;

    stride = calcStride(HEIGHT)/100;
    distance += round(stride*steps_in_buffer);
    return distance;
}

// Calculate Current Speed
uint16_t calcSpeed(void){
    float stride = 0;

    stride = calcStride(HEIGHT)/100;
    return round((steps_in_buffer*stride)/3);
}

// Main function to count steps
uint32_t countSteps(uint32_t step_count){

    // Shift the elements in the buffers left to the left
    for(int i=0; i < STEP_BUFF_LENGTH - 1; i++){
        
        if(i < n)
        {
            data_buff[i]    = data_buff[i+1];
            lpf_buff[i]     = lpf_buff[i+1];
            deriv_buff[i]   = deriv_buff[i+1];
        }
        
        step_buff[i]    = step_buff[i+1];

    }


    // Set last element in array to new data point
    data_buff[n] = combine_stream(readAxis_x(),readAxis_y(),readAxis_z());
    
    // Low pass filter the data
    lpf();
    
    // Differentiate
    diff();
    

    // Check there haven't been any steps in the last buffer period (as repeat counts in the period are too fast to be additional steps)
    bool recent_step = 0;
    
    for(int i=1; i <= BUFF_LENGTH; i++){
        if(step_buff[STEP_BUFF_LENGTH - i] != 0){
            recent_step = 1;
            break;
        }
    }
    
    // Count the steps using derivative and spread of data points
    if((deriv_buff[n] * deriv_buff[n-1]) < 0 && lpf_buff[n-4] > THRESH && (deriv_buff[n-4] - deriv_buff[n]) > DERIV_THRESH && recent_step == 0){
        step_count ++;
        step_buff[STEP_BUFF_LENGTH - 1] = 1;
    }
    else{
        step_buff[STEP_BUFF_LENGTH - 1] = 0;
    }
    
    // Keep track of steps in buffer based on entering and leaving steps
    steps_in_buffer = steps_in_buffer + step_buff[STEP_BUFF_LENGTH - 1] - step_buff[0];
    
    return step_count;
}


// Count calories based on speed, weight and height
uint32_t countCals(uint32_t cal_count, uint8_t height, uint8_t weight)
{
    
    // Uses equations outlined in 'Full-Featured Pedometer Design Realized with 3-Axis Digital Accelerometer' - Neil Zhao
    
    uint8_t cals;
    uint8_t speed;
    uint8_t stride;
    
    
    // Calculate stride length from steps per 3s
    if(steps_in_buffer < 3)
    {
        stride = height / 5;
    }
    
    else if(steps_in_buffer < 4)
    {
        stride = height / 4;
    }
    
    else if(steps_in_buffer < 6)
    {
        stride = height / 3;
    }
    
    else if(steps_in_buffer < 7)
    {
        stride = height / 2;
    }
    
    else if(steps_in_buffer < 9)
    {
        stride = height / 1.2;
    }
    
    else if(steps_in_buffer < 12)
    {
        stride = height;
    }
    
    else
    {
        stride = height * 1.2;
    }
    
    // Calculate speed in cm/s
    speed = steps_in_buffer * stride / 3;       // Speed = number of steps * stride length / time (3s)
    
    /*
    Calculate calories per second (cals used i.e. Kcals x1000)
    kCals/kg/h = 1.25 x running speed (km/h)
    Cals/s = 1.25 * weight (kg) * running speed (m/s) = 1.25 * weight * running speed (cm/s) / 100
    */
    
    cals = 1.25 * weight * speed / 100;
    
    cal_count += cals;
    
    return(cal_count);
}

// Select mode based on steps
uint8_t modeSelector(uint8_t mode, uint32_t last_step_time)
{
    // Rest mode if no steps in REST_TIME
    if(OSA_TimeGetMsec() - last_step_time > REST_TIME)
    {
        return 0;
    }
    // Running mode if number of steps in buffer > RUNNING_THRESH
    else if(steps_in_buffer > RUNNING_THRESH)
    {
        return 2;
    }
    // Otherwise walking mode
    else
    {
        return 1;
    }
}

/*
UI and UX for OLED borrowed from Adam Goldney https://github.com/adamgoldney/Warp-Pedometer
*/

// Draw the background
void displayBackground(uint8_t mode, uint8_t setting)
{
    uint32_t text_colour;
    uint32_t line_colour;
    
    if(mode == REST)
    {
        text_colour = WHITE & DIM;      // Bitwise AND colour with DIM for dimmed colours
        line_colour = PINK & DIM;       // (only works for primary and secondary colours but will do for use here)
    }
    else
    {
        text_colour = WHITE;
        line_colour = PINK;
    }
    
    // STEPS
    writeCharacter(2, 63, 'S', text_colour);
    writeCharacter(10, 63, 'T', text_colour);
    writeCharacter(18, 63, 'E', text_colour);
    writeCharacter(26, 63, 'P', text_colour);
    writeCharacter(34, 63, 'S', text_colour);

    if (setting == 1){

        // CALS
    writeCharacter(57, 63, 'C', text_colour);
    writeCharacter(65, 63, 'A', text_colour);
    writeCharacter(73, 63, 'L', text_colour);
    writeCharacter(81, 63, 'S', text_colour);

    }
    else if (setting == 2){

        // DIST
    writeCharacter(57, 63, 'D', text_colour);
    writeCharacter(65, 63, 'I', text_colour);
    writeCharacter(73, 63, 'S', text_colour);
    writeCharacter(81, 63, 'T', text_colour);

    }

    else if (setting == 3){ 

        // SPEED
    writeCharacter(55, 63, 'S', text_colour);
    writeCharacter(63, 63, 'P', text_colour);
    writeCharacter(71, 63, 'E', text_colour);
    writeCharacter(79, 63, 'E', text_colour);
    writeCharacter(87, 63, 'D', text_colour);

    }
    
    else if (setting == 4) {
	    // BMI
	    //writeCharacter(55, 63, 'B', text_colour);
	    //writeCharacter(63, 63, 'M', text_colour);
	    writeCharacter(71, 63, 'I', text_colour);
    }
    
    // Draw Line
    writeCommand(kSSD1331CommandDRAWLINE);
    writeCommand(47);             // Col start
    writeCommand(63-63);         // Row start
    writeCommand(47);            // Col end
    writeCommand(63-31);         // Row end
    writeCommand((uint8_t)(line_colour >> 16) & 0xFF);          // Line red
    writeCommand((uint8_t)(line_colour >> 8) & 0xFF);           // Line green
    writeCommand((uint8_t)line_colour & 0xFF);                  // Line blue

   // Draw horizontal line
    writeCommand(kSSD1331CommandDRAWLINE);
    writeCommand(1);             // Col start
    writeCommand(63-20);         // Row start
    writeCommand(96);            // Col end
    writeCommand(63-20);         // Row end
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
    writeCharacter(34, 11, 'R', WHITE & DIM);
    writeCharacter(42, 11, 'E', WHITE & DIM);
    writeCharacter(50, 11, 'S', WHITE & DIM);
    writeCharacter(58, 11, 'T', WHITE & DIM);
        
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

// Draw the various counts - keep centred with number of digits
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

void drawDist(uint32_t distance, uint8_t mode)
{
    uint32_t colour;
    
    if(distance >= DIST_THRESHOLD)
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
    
    drawCount(51, 42, distance, colour);
}

void drawSpeed(uint32_t speed, uint8_t mode)
{
    uint32_t colour = WHITE;
    
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

void drawBMI(uint8_t bmi, uint8_t mode)
{
    uint32_t colour;

    colour = WHITE;

    if(mode == REST)
    {
        colour = colour & DIM;
    }

    drawCount(51, 42, bmi, colour);
}


