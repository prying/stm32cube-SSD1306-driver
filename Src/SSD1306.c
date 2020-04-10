#include "SSD1306.h"
#include "memory.h"

static uint8_t frameBuffer[SSD1306_COLUMN * SSD1306_PAGE] = {0};
// Should i be using a global variable??
static I2C_HandleTypeDef *hi2c;

void sendComand(uint8_t cmd);
void sendByte(uint8_t byte);
void sendBytes(uint8_t *byte, uint16_t size);
void sendBytesDMA(uint8_t *byte, uint16_t size);


void SSD1306_init(I2C_HandleTypeDef *hi2cIn)
{
	hi2c = hi2cIn;
    while( HAL_I2C_IsDeviceReady(hi2c, SSD1306_ADDR, 1, 10) != HAL_OK);
	

    sendComand(SSD1306_CMD_DISPLAY_CLK_DIV_RATIO);
	sendComand(SSD1306_DISPLAY_CLK_DIV_128_64);
	// Set MUX ratio
	sendComand(SSD1306_CMD_MULTIPLEX_RATIO);
	sendComand(SSD1306_MULTIPLEX_RATIO);
	// Set display start line 40
	sendComand(SSD1306_CMD_DISPLAY_START_LINE);
	// Set segment re-map A0 or A1
	sendComand(SSD1306_CMD_MEMORY_ADDR_MODE);
	sendComand(SSD1306_MEMORY_MODE_HORIZONTAL); //SSD1306_MEMORY_MODE_PAGE
	sendComand(SSD1306_CMD_SEGMENT_RE_MAP | 0x1);
	// Set COM ouput scan direction C0 or C8
	sendComand(SSD1306_CMD_COM_OUTPUT_SCAN_DIR|0x8);
	// set COM pins hardware config
	sendComand(SSD1306_CMD_COM_PINS_HW_CONFIG);
	sendComand(SSD1306_COM_PINS_HW_CONFIG);
	// Set contrast 
	sendComand(SSD1306_CMD_SET_CONTRAST);
	sendComand(0x7F);
	// Disable entire display on ??
	sendComand(SSD1306_CMD_DISPLAY_OFF);
	// set normal display
	sendComand(SSD1306_CMD_NORMAL_DISPLAY);
	// set osc freq
	sendComand(SSD1306_CMD_DISPLAY_OFFSET);
	sendComand(SSD1306_DISPLAY_OFFSET);
	// enable charge pump
	sendComand(SSD1306_CMD_CHARGE_PUMP);
	sendComand(SSD1306_CHARGE_PUMP_ENABLE);
	// Display on
	sendComand(SSD1306_CMD_DISPLAY_ON);

	// scrolling d sendComand(hi2c, 0x2E);

	SSD1306_clear(PIXLE_OFF);
	SSD1306_update(hi2c);
    // ToDo capture errors
    return;
}


void SSD1306_deInit(I2C_HandleTypeDef *hi2c)
{
	while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY);

	// turn off display
	SSD1306_clear(PIXLE_OFF);
	sendComand(SSD1306_CMD_DISPLAY_OFF);

	// Remove reffrence to display
	hi2c = NULL;
}

void SSD1306_comand(uint8_t cmd)
{
	sendComand(cmd);
}

void SSD1306_setContrast(uint8_t contrast)
{
	// Any uint8 will work as it IS the range 0-256
	sendComand(SSD1306_CMD_SET_CONTRAST);
	sendComand(contrast);
}
//void SSD1306_invertDisplay();

//void SSD1306_drawPixle(uint8_t x, uint8_t y, pixelColor_t colour);
//void SSD1306_drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, pixelColor_t colour);
//void SSD1306_print(char * text, uint16_t size, uint8_t x, uint8_t y, pixelColor_t colour, textSize_t fontSize);

void SSD1306_clear(pixelColor_t colour)
{
	memset(frameBuffer, colour, sizeof(frameBuffer));
}

void SSD1306_update(I2C_HandleTypeDef *hi2c)
{
	// Go to home
	sendComand(SSD1306_CMD_COLUMN_ADDR);
	sendComand(0x00);
	sendComand(0x7F);
	sendComand(SSD1306_CMD_PAGE_ADDR);
	sendComand(0x00);
	sendComand(0x07);

	// Update hole display
	for (int page = 0; page < SSD1306_PAGE; page++)
	{
		sendBytes(&frameBuffer[page*SSD1306_COLUMN], SSD1306_COLUMN);
	}
}

/**
 * @brief Sends a command to SSD1306
 * 
 * @param hi2c i2c bus to use
 * @param cmd SSD1306_CMD_ comands
 */
void sendComand(uint8_t cmd)
{
	uint8_t cmdByte[2] = {SSD1306_CMD_COMAND, cmd};
	HAL_I2C_Master_Transmit(hi2c, SSD1306_ADDR, cmdByte, sizeof(cmdByte), 10);
}

void sendByte(uint8_t byte)
{
	uint8_t cmdByte[2] = {SSD1306_DATA_BYTE, byte};
	HAL_I2C_Master_Transmit(hi2c, SSD1306_ADDR, &byte, sizeof(cmdByte), 10);
}

void sendBytes(uint8_t *byte, uint16_t size)
{
	uint8_t data[SSD1306_COLUMN + 1];
	data[0] = SSD1306_DATA_BYTE;
	for (int i = 0; i < size; i++)
	{
		data[i+1] = byte[i];
	}
	HAL_I2C_Master_Transmit(hi2c, SSD1306_ADDR, data, size + 1, 40);
}

void sendBytesDMA(uint8_t *byte, uint16_t size)
{
	uint8_t data[SSD1306_COLUMN + 1];
	data[0] = SSD1306_DATA_BYTE;
	for (int i = 0; i < size; i++)
	{
		data[i+1] = byte[i];
	}
	HAL_I2C_Master_Transmit_DMA(hi2c, SSD1306_ADDR, data, size + 1);
}