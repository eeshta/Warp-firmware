// Define some standard colours
#define PINK		0XFF10F0
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
#define DIST_THRESHOLD  10
#define SPEED_THRESHOLD 12.43

// Modes
#define REST            0
#define WALK            1
#define RUN             2

// User defined parameters
#define HEIGHT          168         // Height in cm
#define WEIGHT          65          // Weight in kg

int16_t combine_stream(int16_t x_data, int16_t y_data, int16_t z_data);
void    lpf(void);
void    diff(void);

float calcStride(uint8_t height);
uint32_t calcDistance(uint32_t distance);
uint16_t calcSpeed(void);

uint32_t countSteps(uint32_t step_count);
uint32_t countCals(uint32_t cal_count, uint8_t height, uint8_t weight);
uint8_t modeSelector(uint8_t mode, uint32_t last_step_time);

void displayBackground(uint8_t mode, uint8_t setting);
void displayMode(uint8_t mode);
void drawCount(uint8_t column, uint8_t row, uint32_t count, uint32_t colour);
void drawSteps(uint8_t step_count, uint8_t mode);
void drawDist(uint32_t distance, uint8_t mode);
void drawSpeed(uint32_t speed, uint8_t mode);
void drawCals(uint32_t cals, uint8_t mode);
