#ifndef SSD1306_H
#define SSD1306_H

//#include "stm32f0xx_hal_conf.h"
#include "stm32f0xx_hal.h"

/******************************************************************************
 * I2C Comunication 
 * Addressing
 * +----+----+----+----+----+----+-----+-----+
 * | B7 | B6 | B5 | B4 | B3 | B2 | B1  | B0  |
 * +----+----+----+----+----+----+-----+-----+
 * |  0 |  1 |  1 |  1 |  1 |  0 | SA0 | R/W |
 * +----+----+----+----+----+----+-----+-----+
 * SA0: 1 or 0 can be selected for the SSD1306
 * R/W: set operation mode
 * 
 * 
 *****************************************************************************/
#define SSD1306_WIDTH				128		// Screen and GDDRAM width 
#define SSD1306_HEIGHT				64		// Screen and GDDRAM height 

#define SSD1306_ADDR        		0x78    // Device address. Can be 0x7A if SA0 is high

#define SSD1306_READ_BIT    		0x01    // 8'bxxxxxxxR/W
#define SSD1306_WRITE_BIT   		0x00    // 8'bxxxxxxxR/W

#define SSD1306_CONTUNUATION_BIT    0x80	// If O, transmission will contain data only
#define SSD1306_DATA_COMAND_BIT		0x40	// 0: command, 1: data to store in GDDRAM

#define SSD1306_CONTROL_BYTE		0x00	// next bytes will be a CMD
/******************************************************************************
 * Comand table
 *****************************************************************************/
#define SSD1306_CMD_COMAND					SSD1306_CONTROL_BYTE 	// next bytes will be a CMD
// Fundamental commands
#define SSD1306_CMD_SET_CONTRAST			0x81	// Set contrast between 1 and 256 with next byte, reset = 0x7F
#define SSD1306_CMD_ENTIRE_DISPLAY_ON_RAM	0xA4	// Output follows ram content (reset)
#define SSD1306_CMD_ENTIRE_DISPLAY_ON_I_RAM	0xA5	// Output ignores ram content
#define SSD1306_CMD_NORMAL_DISPLAY			0xA6	// None inverted ram to display (reset)
#define SSD1306_CMD_INVERSE_DISPLAY			0xA6	// Inverted ram to display
#define SSD1306_CMD_DISPLAY_OFF				0xAE	// Display off (sleep mode) (reset)
#define SSD1306_CMD_DISPLAY_ON				0xAF	// Display on (in normal mode)

// Scrolling commands
// ToDo: add scrolling comands

// Addressing Settings Command Table
// ToDo: Set Lower Column Start Address for Page Addressing Mode CMD
// ToDo: Set Higher Column Start Address for Page Addressing Mode CMD
#define SSD1306_CMD_MEMORY_ADDR_MODE 		0x20	// Follow with CMD byte SSD1306_MEMORY_MODE setting
#define SSD1306_CMD_COLUMN_ADDR				0x21	// Follow with CMD bytes [6:0] start and end addr, reset = 0 to 127
#define SSD1306_CMD_PAGE_ADDR				0x22	// Follow with CMD bytes [2:0] start and end addr, reset = 0 to 7
// ToDo: set Page Start Address for Page Addressing Mode CMD

// Hardware Configuration 
// ToDo: add hardweare CMDs
#define SSD1306_CMD_SEGMENT_RE_MAP			0xA0	// X0: 0 col0 is mapped to seg0 (reset), 1 col127 mapped to seg0
#define SSD1306_CMD_DISPLAY_START_LINE		0x40	// 0b01XXXXXX where XXXXXX is 0-63d offset	(reset = 0 offset)
#define SSD1306_CMD_MULTIPLEX_RATIO			0xA8	// Follow with CMD byte [5:0] + 1 (0 to 14 invalid)(reset = 0x3F)
#define SSD1306_CMD_COM_OUTPUT_SCAN_DIR		0xC0	// X3: 0 normal mode (reset), 1 remaped mode
#define SSD1306_CMD_DISPLAY_OFFSET			0xD3	// Follow with CMD byte 0-63d for vertical COM offset (reset = 0x00) 
#define SSD1306_CMD_COM_PINS_HW_CONFIG		0xDA	// ?? Follow with CMD byte SSD1306_COM_PINS_HW_CONFIG

// Timing & Driving Scheme setting commands 
// ToDo: add Timing & Driving Scheme CMDs
#define SSD1306_CMD_DISPLAY_CLK_DIV_RATIO	0xD5	// Follow with CMD byte [3:0] divide ratio = [3:0] + 1, [7:4] oscillator frq reset = 0b1000

// Charge Pump commands
#define SSD1306_CMD_CHARGE_PUMP				0x8D	// Follow with CMD byte 0b**010A00 A: 0 (reset) disable, 1 enable (aka 0x14h) to enable CP

// Memory addressing modes
#define SSD1306_MEMORY_MODE_HORIZONTAL 		0x00	// Col0Page0 -> C1P0 -...-> C127P0 -> C0P1 ect 
#define SSD1306_MEMORY_MODE_VERTICAL 		0x01	// Col0Page1 -> C0P2 -...-> C0P63 -> C1P0 ect 
#define SSD1306_MEMORY_MODE_PAGE	 		0x10	// Col0Page0 -> C1P0 -...-> C127P0 -> Stops, manually go to C0P1 to continue (reset)

// Hardweare settings
#define SSD1306_MULTIPLEX_RATIO				0xFF
#define SSD1306_DISPLAY_OFFSET				0x00
#define SSD1306_COM_PINS_HW_CONFIG			0x12

// Timing Settings
#define SSD1306_DISPLAY_CLK_DIV_128_64		0x80	// Defult value for 128 x 64 SSD1306_CMD_DISPLAY_CLK_DIV_RATIO

// Charge pump settings
#define SSD1306_CHARGE_PUMP_ENABLE			0x14	// Enable charge pump

/******************************************************************************
 * Data
 *****************************************************************************/
#define SSD1306_DATA_BYTE					SSD1306_DATA_COMAND_BIT		// Is this nessasarry
#define SSD1306_PAGE_SIZE					8	// Size in bits
#define SSD1306_COLUMN                      SSD1306_WIDTH
#define SSD1306_PAGE						SSD1306_HEIGHT/SSD1306_PAGE_SIZE

// For turning pixels on or off
typedef enum{
    PIXLE_OFF   = 0x00U,
    PIXEL_ON    = 0x01U
}pixelColor_t;

// Text size options
typedef enum{
    TEXT_0      = 0x00U,
    TEXT_1      = 0x01U
}textSize_t;

/******************************************************************************
 * Functions
 *****************************************************************************/
//ToDo: make ajustable display diver
/**
 * @brief Initilises 128x64 I2c Oled display
 * 
 * @param hi2c is the i2c device used. i needs to be set up with TX with DMA
 * @return HAL_StatusTypeDef HAL_OK if succes
 */
void SSD1306_init(I2C_HandleTypeDef *hi2cIn);
//void SSD1306_deInit();
//void SSD1306_comand(uint8_t cmd);
//void SSD1306_setContrast(uint8_t contrast);
//void SSD1306_invertDisplay();

//void SSD1306_drawPixle(uint8_t x, uint8_t y, pixelColor_t colour);
//void SSD1306_drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, pixelColor_t colour);
//void SSD1306_print(char * text, uint16_t size, uint8_t x, uint8_t y, pixelColor_t colour, textSize_t fontSize);

/**
 * @brief clears the frame buffer
 * 
 * @param colour 
 */
void SSD1306_clear(pixelColor_t colour);

/**
 * @brief draws the current frame buffer to the display
 * 
 */
void SSD1306_update(I2C_HandleTypeDef *hi2c);

#endif