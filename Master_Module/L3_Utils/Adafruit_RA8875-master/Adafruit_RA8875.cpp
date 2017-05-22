/**************************************************************************/
/*!
    @file     Adafruit_RA8875.cpp
    @author   Limor Friend/Ladyada, K.Townsend/KTOWN for Adafruit Industries
    @license  BSD license, all text above and below must be included in
              any redistribution

 This is the library for the Adafruit RA8875 Driver board for TFT displays
 ---------------> http://www.adafruit.com/products/1590
 The RA8875 is a TFT driver for up to 800x480 dotclock'd displays
 It is tested to work with displays in the Adafruit shop. Other displays
 may need timing adjustments and are not guanteed to work.
 
 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source hardware
 by purchasing products from Adafruit!
 
 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, check license.txt for more information.
 All text above must be included in any redistribution.

    @section  HISTORY
    
    v1.0 - First release
*/
/**************************************************************************/
#include <stdio.h>
#include "ssp1.h"
#include "utilities.h"
#include "Adafruit_RA8875.h"
#include "out_rgb.h"

#if defined (ARDUINO_ARCH_ARC32)
  uint32_t spi_speed = 12000000;
#else
  uint32_t spi_speed = 4000000;
#endif

enum mode
{
	LOW = 0,
	HIGH,
};

void digitalWrite(enum mode m)
{
	if(m == LOW)
		LPC_GPIO0->FIOCLR = (1 << 26);
	else
		LPC_GPIO0->FIOSET = (1 << 26);
}

// If the SPI library has transaction support, these functions
// establish settings and protect from interference from other
// libraries.  Otherwise, they simply do nothing.
#ifdef SPI_HAS_TRANSACTION
    static inline void spi_begin(void) __attribute__((always_inline));
    static inline void spi_begin(void) {
        // max speed!
        SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
    }
    static inline void spi_end(void) __attribute__((always_inline));
    static inline void spi_end(void) {
        SPI.endTransaction();
    }
#else
    #define spi_begin()
    #define spi_end()
#endif


/**************************************************************************/
/*!
      Constructor for a new RA8875 instance
      
      @args CS[in]  Location of the SPI chip select pin
      @args RST[in] Location of the reset pin
*/
/**************************************************************************/
Adafruit_RA8875::Adafruit_RA8875(uint8_t CS, uint8_t RST) : Adafruit_GFX(800, 480) {
  _cs = CS;
  _rst = RST;
}


/**************************************************************************/
/*!
      Initialises the LCD driver and any HW required by the display
      
      @args s[in] The display size, which can be either:
                  'RA8875_480x272' (4.3" displays) r
                  'RA8875_800x480' (5" and 7" displays)
*/
/**************************************************************************/
bool Adafruit_RA8875::begin(enum RA8875sizes s) {
  _size = s;

  if (_size == RA8875_480x272) {
    _width = 480;
    _height = 272;
  } 
  else if (_size == RA8875_800x480) {
    _width = 800;
    _height = 480;
  }
  else {
    return false;
  }

  //TODO:
  //pinMode(_cs, OUTPUT);
  //digitalWrite(_cs, HIGH);
  //pinMode(_rst, OUTPUT);
    
  //digitalWrite(_rst, LOW);
  //delay_ms(100);
  //digitalWrite(_rst, HIGH);
  //delay_ms(100);

  //P0.26 = CS
  //P2.4 = RST
  LPC_PINCON->PINSEL1 &= ~(3 << 20); // Select type as GPIO P0.26
  LPC_PINCON->PINSEL4 &= ~(3 << 8); // Select type as GPIO P2.4

  LPC_GPIO0->FIODIR |= (1<<26);  // Configure P0.26 as output
  LPC_GPIO2->FIODIR |= (1<<4);  // Configure P2.4 as output

  digitalWrite(HIGH);				// Set P2.3
  LPC_GPIO2->FIOCLR = (1 << 4);  	// Clear p2.4
  delay_ms(100);
  LPC_GPIO2->FIOSET = (1 << 4);		// Set p2.4
  delay_ms(100);

  
  //SPI.begin();
  ssp1_init();
    
#ifdef SPI_HAS_TRANSACTION
    #if defined (ARDUINO_ARCH_ARC32)
        spi_speed = 2000000;
    #else
        spi_speed = 125000;
    #endif
#else
    #ifdef __AVR__
        SPI.setClockDivider(SPI_CLOCK_DIV128);
        SPI.setDataMode(SPI_MODE0);
    #endif
#endif

    uint8_t x = readReg(0);
//    Serial.print("x = 0x"); Serial.println(x,HEX);
    if (x != 0x75) {
    	printf("Inside.......................\n");
        return false;
    }

  initialize();

#ifdef SPI_HAS_TRANSACTION
    #if defined (ARDUINO_ARCH_ARC32)
        spi_speed = 12000000L;
    #else
        spi_speed = 4000000L;
    #endif
#else
    #ifdef __AVR__
        SPI.setClockDivider(SPI_CLOCK_DIV4);
    #endif
#endif
    
  return true;
}

/************************* Initialization *********************************/

/**************************************************************************/
/*!
      Performs a SW-based reset of the RA8875
*/
/**************************************************************************/
void Adafruit_RA8875::softReset(void) {
  writeCommand(RA8875_PWRR);
  writeData(RA8875_PWRR_SOFTRESET);
  writeData(RA8875_PWRR_NORMAL);
  delay_ms(1);
}

/**************************************************************************/
/*!
      Initialise the PLL
*/
/**************************************************************************/
void Adafruit_RA8875::PLLinit(void) {
  if (_size == RA8875_480x272) {
    writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
    delay_ms(1);
    writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
    delay_ms(1);
  }
  else /* (_size == RA8875_800x480) */ {
    writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
    delay_ms(1);
    writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
    delay_ms(1);
  }
}

/**************************************************************************/
/*!
      Initialises the driver IC (clock setup, etc.)
*/
/**************************************************************************/
void Adafruit_RA8875::initialize(void) {
  PLLinit();
  //writeReg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);
  writeReg(RA8875_SYSR, RA8875_SYSR_8BPP | RA8875_SYSR_MCU8);

  /* Timing values */
  uint8_t pixclk;
  uint8_t hsync_start;
  uint8_t hsync_pw;
  uint8_t hsync_finetune;
  uint8_t hsync_nondisp;
  uint8_t vsync_pw; 
  uint16_t vsync_nondisp;
  uint16_t vsync_start;

	_DPCR_Reg = 0b00000000;
	_maxLayers = 2;
	_currentLayer = 0;
	_useMultiLayers = false;//starts with one layer only
	_hasLayerLimits = false;
	_color_bpp = 16;
	_colorIndex = 0;



  /* Set the correct values for the display being used */  
  if (_size == RA8875_480x272) 
  {
    pixclk          = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
    hsync_nondisp   = 10;
    hsync_start     = 8;
    hsync_pw        = 48;
    hsync_finetune  = 0;
    vsync_nondisp   = 3;
    vsync_start     = 8;
    vsync_pw        = 10;
  } 
  else // (_size == RA8875_800x480)
  {
    pixclk          = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
    hsync_nondisp   = 26;
    hsync_start     = 32;
    hsync_pw        = 96;
    hsync_finetune  = 0;
    vsync_nondisp   = 32;
    vsync_start     = 23;
    vsync_pw        = 2;
	_hasLayerLimits = true;
  }

  writeReg(RA8875_PCSR, pixclk);
  delay_ms(1);
  


  /* Horizontal settings registers */
  writeReg(RA8875_HDWR, (_width / 8) - 1);                          // H width: (HDWR + 1) * 8 = 480
  writeReg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
  writeReg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2)/8);    // H non-display: HNDR * 8 + HNDFTR + 2 = 10
  writeReg(RA8875_HSTR, hsync_start/8 - 1);                         // Hsync start: (HSTR + 1)*8 
  writeReg(RA8875_HPWR, RA8875_HPWR_LOW + (hsync_pw/8 - 1));        // HSync pulse width = (HPWR+1) * 8
  
  /* Vertical settings registers */
  writeReg(RA8875_VDHR0, (uint16_t)(_height - 1) & 0xFF);
  writeReg(RA8875_VDHR1, (uint16_t)(_height - 1) >> 8);
  writeReg(RA8875_VNDR0, vsync_nondisp-1);                          // V non-display period = VNDR + 1
  writeReg(RA8875_VNDR1, vsync_nondisp >> 8);
  writeReg(RA8875_VSTR0, vsync_start-1);                            // Vsync start position = VSTR + 1
  writeReg(RA8875_VSTR1, vsync_start >> 8);
  writeReg(RA8875_VPWR, RA8875_VPWR_LOW + vsync_pw - 1);            // Vsync pulse width = VPWR + 1
  
  /* Set active window X */
  writeReg(RA8875_HSAW0, 0);                                        // horizontal start point
  writeReg(RA8875_HSAW1, 0);
  writeReg(RA8875_HEAW0, (uint16_t)(_width - 1) & 0xFF);            // horizontal end point
  writeReg(RA8875_HEAW1, (uint16_t)(_width - 1) >> 8);
  
  /* Set active window Y */
  writeReg(RA8875_VSAW0, 0);                                        // vertical start point
  writeReg(RA8875_VSAW1, 0);  
  writeReg(RA8875_VEAW0, (uint16_t)(_height - 1) & 0xFF);           // horizontal end point
  writeReg(RA8875_VEAW1, (uint16_t)(_height - 1) >> 8);
  
  /* ToDo: Setup touch panel? */
  
  /* Clear the entire window */
  writeReg(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
  delay_ms(500);
}

/**************************************************************************/
/*!
      Returns the display width in pixels
      
      @returns  The 1-based display width in pixels
*/
/**************************************************************************/
uint16_t Adafruit_RA8875::width(void) { return _width; }

/**************************************************************************/
/*!
      Returns the display height in pixels

      @returns  The 1-based display height in pixels
*/
/**************************************************************************/
uint16_t Adafruit_RA8875::height(void) { return _height; }

/************************* Text Mode ***********************************/

/**************************************************************************/
/*!
      Sets the display in text mode (as opposed to graphics mode)
*/
/**************************************************************************/
void Adafruit_RA8875::textMode(void) 
{
  /* Set text mode */
  writeCommand(RA8875_MWCR0);
  uint8_t temp = readData();
  temp |= RA8875_MWCR0_TXTMODE; // Set bit 7
  writeData(temp);
  
  /* Select the internal (ROM) font */
  writeCommand(0x21);
  temp = readData();
  temp &= ~((1<<7) | (1<<5)); // Clear bits 7 and 5
  writeData(temp);
}

/**************************************************************************/
/*!
      Sets the display in text mode (as opposed to graphics mode)
      
      @args x[in] The x position of the cursor (in pixels, 0..1023)
      @args y[in] The y position of the cursor (in pixels, 0..511)
*/
/**************************************************************************/
void Adafruit_RA8875::textSetCursor(uint16_t x, uint16_t y) 
{
  /* Set cursor location */
  writeCommand(0x2A);
  writeData(x & 0xFF);
  writeCommand(0x2B);
  writeData(x >> 8);
  writeCommand(0x2C);
  writeData(y & 0xFF);
  writeCommand(0x2D);
  writeData(y >> 8);
}

/**************************************************************************/
/*!
      Sets the fore and background color when rendering text
      
      @args foreColor[in] The RGB565 color to use when rendering the text
      @args bgColor[in]   The RGB565 colot to use for the background
*/
/**************************************************************************/
void Adafruit_RA8875::textColor(uint16_t foreColor, uint16_t bgColor)
{
  /* Set Fore Color */
  writeCommand(0x63);
  writeData((foreColor & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((foreColor & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((foreColor & 0x001f));
  
  /* Set Background Color */

  /*writeCommand(0x60);
  writeData((bgColor & 0xf800) >> 11);
  writeCommand(0x61);
  writeData((bgColor & 0x07e0) >> 5);
  writeCommand(0x62);
  writeData((bgColor & 0x001f));*/
  

  /* Clear transparency flag */
  writeCommand(0x22);
  uint8_t temp = readData();
  temp &= ~(1<<6); // Clear bit 6
  //temp |= (1<<6); // Set bit 6
  writeData(temp);
}

/**************************************************************************/
/*!
      Sets the fore color when rendering text with a transparent bg
      
      @args foreColor[in] The RGB565 color to use when rendering the text
*/
/**************************************************************************/
void Adafruit_RA8875::textTransparent(uint16_t foreColor)
{
  /* Set Fore Color */
  writeCommand(0x63);
  writeData((foreColor & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((foreColor & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((foreColor & 0x001f));

  /* Set transparency flag */
  writeCommand(0x22);
  uint8_t temp = readData();
  temp |= (1<<6); // Set bit 6
  writeData(temp);  
}

/**************************************************************************/
/*!
      Sets the text enlarge settings, using one of the following values:
      
      0 = 1x zoom
      1 = 2x zoom
      2 = 3x zoom
      3 = 4x zoom
      
      @args scale[in]   The zoom factor (0..3 for 1-4x zoom)
*/
/**************************************************************************/
void Adafruit_RA8875::textEnlarge(uint8_t scale)
{
  if (scale > 3) scale = 3;

  /* Set font size flags */
  writeCommand(0x22);
  uint8_t temp = readData();
  temp &= ~(0xF); // Clears bits 0..3
  temp |= scale << 2;
  temp |= scale;
  writeData(temp);  

  _textScale = scale;
}

/**************************************************************************/
/*!
      Renders some text on the screen when in text mode
      
      @args buffer[in]    The buffer containing the characters to render
      @args len[in]       The size of the buffer in bytes
*/
/**************************************************************************/
void Adafruit_RA8875::textWrite(const char* buffer, uint16_t len) 
{
  if (len == 0) len = strlen(buffer);
  writeCommand(RA8875_MRWC);
  for (uint16_t i=0;i<len;i++)
  {
    writeData(buffer[i]);
#if defined(__AVR__)
    if (_textScale > 1) delay(1);
#elif defined(__arm__)
    // This delay is needed with textEnlarge(1) because
    // Teensy 3.X is much faster than Arduino Uno
    if (_textScale > 0) delay_ms(1);
#endif
  }
}

/************************* Graphics ***********************************/

/**************************************************************************/
/*!
      Sets the display in graphics mode (as opposed to text mode)
*/
/**************************************************************************/
void Adafruit_RA8875::graphicsMode(void) {
  writeCommand(RA8875_MWCR0);
  uint8_t temp = readData();
  temp &= ~RA8875_MWCR0_TXTMODE; // bit #7
  writeData(temp);
}

/**************************************************************************/
/*!
      Waits for screen to finish by polling the status!
*/
/**************************************************************************/
bool Adafruit_RA8875::waitPoll(uint8_t regname, uint8_t waitflag) {
  /* Wait for the command to finish */
  while (1)
  {
    uint8_t temp = readReg(regname);
    if (!(temp & waitflag))
      return true;
  }  
  return false; // MEMEFIX: yeah i know, unreached! - add timeout?
}


/**************************************************************************/
/*!
      Sets the current X/Y position on the display before drawing
      
      @args x[in] The 0-based x location
      @args y[in] The 0-base y location
*/
/**************************************************************************/
void Adafruit_RA8875::setXY(uint16_t x, uint16_t y) {
  writeReg(RA8875_CURH0, x);
  writeReg(RA8875_CURH1, x >> 8);
  writeReg(RA8875_CURV0, y);
  writeReg(RA8875_CURV1, y >> 8);  
}

/**************************************************************************/
/*!
      HW accelerated function to push a chunk of raw pixel data
      
      @args num[in] The number of pixels to push
      @args p[in]   The pixel color to use
*/
/**************************************************************************/
void Adafruit_RA8875::pushPixels(uint32_t num, uint16_t p) {
  //TODO Replace digitalWrite with chip select
  //digitalWrite(_cs, LOW);
  digitalWrite(LOW);
  //SPI.transfer(RA8875_DATAWRITE);
  ssp1_exchange_byte(RA8875_DATAWRITE);
  while (num--) {
    //SPI.transfer(p >> 8);
    //SPI.transfer(p);
	ssp1_exchange_byte(p >> 8);
	ssp1_exchange_byte(p);
  }
  digitalWrite(HIGH);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void Adafruit_RA8875::fillRect(void) {
  writeCommand(RA8875_DCR);
  writeData(RA8875_DCR_LINESQUTRI_STOP | RA8875_DCR_DRAWSQUARE);
  writeData(RA8875_DCR_LINESQUTRI_START | RA8875_DCR_FILL | RA8875_DCR_DRAWSQUARE);
}

/**************************************************************************/
/*!
      Draws a single pixel at the specified location

      @args x[in]     The 0-based x location
      @args y[in]     The 0-base y location
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawPixel (int16_t x, int16_t y, uint16_t color)
{
  writeReg(RA8875_CURH0, x);
  writeReg(RA8875_CURH1, x >> 8);
  writeReg(RA8875_CURV0, y);
  writeReg(RA8875_CURV1, y >> 8);
  writeCommand(RA8875_MRWC);
  digitalWrite(LOW);

  //SPI.transfer(RA8875_DATAWRITE);
  //SPI.transfer(color >> 8);
  //SPI.transfer(color);
  ssp1_exchange_byte(RA8875_DATAWRITE);
  ssp1_exchange_byte(color >> 8);
  ssp1_exchange_byte(color);

  digitalWrite(HIGH);
}

void Adafruit_RA8875::drawPixel_8bit (int16_t x, int16_t y, uint8_t color)
{
  writeReg(RA8875_CURH0, x);
  writeReg(RA8875_CURH1, x >> 8);
  writeReg(RA8875_CURV0, y);
  writeReg(RA8875_CURV1, y >> 8);
  writeCommand(RA8875_MRWC);
  digitalWrite(LOW);

  ssp1_exchange_byte(RA8875_DATAWRITE);
  ssp1_exchange_byte(color);

  digitalWrite(HIGH);
}



/**************************************************************************/
/*!
      Draws a HW accelerated line on the display
    
      @args x0[in]    The 0-based starting x location
      @args y0[in]    The 0-base starting y location
      @args x1[in]    The 0-based ending x location
      @args y1[in]    The 0-base ending y location
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  /* Set X */
  writeCommand(0x91);
  writeData(x0);
  writeCommand(0x92);
  writeData(x0 >> 8);
  
  /* Set Y */
  writeCommand(0x93);
  writeData(y0); 
  writeCommand(0x94);
  writeData(y0 >> 8);
  
  /* Set X1 */
  writeCommand(0x95);
  writeData(x1);
  writeCommand(0x96);
  writeData((x1) >> 8);
  
  /* Set Y1 */
  writeCommand(0x97);
  writeData(y1); 
  writeCommand(0x98);
  writeData((y1) >> 8);
  
  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_DCR);
  writeData(0x80);
  
  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void Adafruit_RA8875::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
  drawLine(x, y, x, y+h, color);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void Adafruit_RA8875::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
  drawLine(x, y, x+w, y, color);
}

/**************************************************************************/
/*!
      Draws a HW accelerated rectangle on the display

      @args x[in]     The 0-based x location of the top-right corner
      @args y[in]     The 0-based y location of the top-right corner
      @args w[in]     The rectangle width
      @args h[in]     The rectangle height
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  rectHelper(x, y, x+w, y+h, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled rectangle on the display

      @args x[in]     The 0-based x location of the top-right corner
      @args y[in]     The 0-based y location of the top-right corner
      @args w[in]     The rectangle width
      @args h[in]     The rectangle height
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  rectHelper(x, y, x+w, y+h, color, true);
}

/**************************************************************************/
/*!
      Fills the screen with the spefied RGB565 color

      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillScreen(uint16_t color)
{  
  rectHelper(0, 0, _width-1, _height-1, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated circle on the display

      @args x[in]     The 0-based x location of the center of the circle
      @args y[in]     The 0-based y location of the center of the circle
      @args w[in]     The circle's radius
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
  circleHelper(x0, y0, r, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled circle on the display

      @args x[in]     The 0-based x location of the center of the circle
      @args y[in]     The 0-based y location of the center of the circle
      @args w[in]     The circle's radius
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
  circleHelper(x0, y0, r, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated triangle on the display

      @args x0[in]    The 0-based x location of point 0 on the triangle
      @args y0[in]    The 0-based y location of point 0 on the triangle
      @args x1[in]    The 0-based x location of point 1 on the triangle
      @args y1[in]    The 0-based y location of point 1 on the triangle
      @args x2[in]    The 0-based x location of point 2 on the triangle
      @args y2[in]    The 0-based y location of point 2 on the triangle
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  triangleHelper(x0, y0, x1, y1, x2, y2, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled triangle on the display

      @args x0[in]    The 0-based x location of point 0 on the triangle
      @args y0[in]    The 0-based y location of point 0 on the triangle
      @args x1[in]    The 0-based x location of point 1 on the triangle
      @args y1[in]    The 0-based y location of point 1 on the triangle
      @args x2[in]    The 0-based x location of point 2 on the triangle
      @args y2[in]    The 0-based y location of point 2 on the triangle
      @args color[in] The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  triangleHelper(x0, y0, x1, y1, x2, y2, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated ellipse on the display

      @args xCenter[in]   The 0-based x location of the ellipse's center
      @args yCenter[in]   The 0-based y location of the ellipse's center
      @args longAxis[in]  The size in pixels of the ellipse's long axis
      @args shortAxis[in] The size in pixels of the ellipse's short axis
      @args color[in]     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color)
{
  ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled ellipse on the display

      @args xCenter[in]   The 0-based x location of the ellipse's center
      @args yCenter[in]   The 0-based y location of the ellipse's center
      @args longAxis[in]  The size in pixels of the ellipse's long axis
      @args shortAxis[in] The size in pixels of the ellipse's short axis
      @args color[in]     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color)
{
  ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated curve on the display

      @args xCenter[in]   The 0-based x location of the ellipse's center
      @args yCenter[in]   The 0-based y location of the ellipse's center
      @args longAxis[in]  The size in pixels of the ellipse's long axis
      @args shortAxis[in] The size in pixels of the ellipse's short axis
      @args curvePart[in] The corner to draw, where in clock-wise motion:
                            0 = 180-270°
                            1 = 270-0°
                            2 = 0-90°
                            3 = 90-180°
      @args color[in]     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color)
{
  curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled curve on the display

      @args xCenter[in]   The 0-based x location of the ellipse's center
      @args yCenter[in]   The 0-based y location of the ellipse's center
      @args longAxis[in]  The size in pixels of the ellipse's long axis
      @args shortAxis[in] The size in pixels of the ellipse's short axis
      @args curvePart[in] The corner to draw, where in clock-wise motion:
                            0 = 180-270°
                            1 = 270-0°
                            2 = 0-90°
                            3 = 90-180°
      @args color[in]     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color)
{
  curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, true);
}

/**************************************************************************/
/*!
      Helper function for higher level circle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875::circleHelper(int16_t x0, int16_t y0, int16_t r, uint16_t color, bool filled)
{
  /* Set X */
  writeCommand(0x99);
  writeData(x0);
  writeCommand(0x9a);
  writeData(x0 >> 8);
  
  /* Set Y */
  writeCommand(0x9b);
  writeData(y0); 
  writeCommand(0x9c);	   
  writeData(y0 >> 8);
  
  /* Set Radius */
  writeCommand(0x9d);
  writeData(r);  
  
  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));
  
  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled)
  {
    writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
  }
  else
  {
    writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_NOFILL);
  }
  
  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_CIRCLE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level rectangle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875::rectHelper(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, bool filled)
{
  /* Set X */
  writeCommand(0x91);
  writeData(x);
  writeCommand(0x92);
  writeData(x >> 8);
  
  /* Set Y */
  writeCommand(0x93);
  writeData(y); 
  writeCommand(0x94);	   
  writeData(y >> 8);
  
  /* Set X1 */
  writeCommand(0x95);
  writeData(w);
  writeCommand(0x96);
  writeData((w) >> 8);
  
  /* Set Y1 */
  writeCommand(0x97);
  writeData(h); 
  writeCommand(0x98);
  writeData((h) >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled)
  {
    writeData(0xB0);
  }
  else
  {
    writeData(0x90);
  }
  
  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level triangle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875::triangleHelper(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, bool filled)
{
  /* Set Point 0 */
  writeCommand(0x91);
  writeData(x0);
  writeCommand(0x92);
  writeData(x0 >> 8);
  writeCommand(0x93);
  writeData(y0); 
  writeCommand(0x94);
  writeData(y0 >> 8);

  /* Set Point 1 */
  writeCommand(0x95);
  writeData(x1);
  writeCommand(0x96);
  writeData(x1 >> 8);
  writeCommand(0x97);
  writeData(y1); 
  writeCommand(0x98);
  writeData(y1 >> 8);

  /* Set Point 2 */
  writeCommand(0xA9);
  writeData(x2);
  writeCommand(0xAA);
  writeData(x2 >> 8);
  writeCommand(0xAB);
  writeData(y2); 
  writeCommand(0xAC);
  writeData(y2 >> 8);
  
  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));
  
  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled)
  {
    writeData(0xA1);
  }
  else
  {
    writeData(0x81);
  }
  
  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level ellipse drawing code
*/
/**************************************************************************/
void Adafruit_RA8875::ellipseHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color, bool filled)
{
  /* Set Center Point */
  writeCommand(0xA5);
  writeData(xCenter);
  writeCommand(0xA6);
  writeData(xCenter >> 8);
  writeCommand(0xA7);
  writeData(yCenter); 
  writeCommand(0xA8);
  writeData(yCenter >> 8);

  /* Set Long and Short Axis */
  writeCommand(0xA1);
  writeData(longAxis);
  writeCommand(0xA2);
  writeData(longAxis >> 8);
  writeCommand(0xA3);
  writeData(shortAxis); 
  writeCommand(0xA4);
  writeData(shortAxis >> 8);
  
  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));
  
  /* Draw! */
  writeCommand(0xA0);
  if (filled)
  {
    writeData(0xC0);
  }
  else
  {
    writeData(0x80);
  }
  
  /* Wait for the command to finish */
  waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level curve drawing code
*/
/**************************************************************************/
void Adafruit_RA8875::curveHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color, bool filled)
{
  /* Set Center Point */
  writeCommand(0xA5);
  writeData(xCenter);
  writeCommand(0xA6);
  writeData(xCenter >> 8);
  writeCommand(0xA7);
  writeData(yCenter); 
  writeCommand(0xA8);
  writeData(yCenter >> 8);

  /* Set Long and Short Axis */
  writeCommand(0xA1);
  writeData(longAxis);
  writeCommand(0xA2);
  writeData(longAxis >> 8);
  writeCommand(0xA3);
  writeData(shortAxis); 
  writeCommand(0xA4);
  writeData(shortAxis >> 8);
  
  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(0xA0);
  if (filled)
  {
    writeData(0xD0 | (curvePart & 0x03));
  }
  else
  {
    writeData(0x90 | (curvePart & 0x03));
  }
  
  /* Wait for the command to finish */
  waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/************************* Mid Level ***********************************/

/**************************************************************************/
/*!

*/
/**************************************************************************/
void Adafruit_RA8875::GPIOX(bool on) {
  if (on)
    writeReg(RA8875_GPIOX, 1);
  else 
    writeReg(RA8875_GPIOX, 0);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void Adafruit_RA8875::PWM1out(uint8_t p) {
  writeReg(RA8875_P1DCR, p);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void Adafruit_RA8875::PWM2out(uint8_t p) {
  writeReg(RA8875_P2DCR, p);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void Adafruit_RA8875::PWM1config(bool on, uint8_t clock) {
  if (on) {
    writeReg(RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
  } else {
    writeReg(RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void Adafruit_RA8875::PWM2config(bool on, uint8_t clock) {
  if (on) {
    writeReg(RA8875_P2CR, RA8875_P2CR_ENABLE | (clock & 0xF));
  } else {
    writeReg(RA8875_P2CR, RA8875_P2CR_DISABLE | (clock & 0xF));
  }
}

/**************************************************************************/
/*!
      Enables or disables the on-chip touch screen controller
*/
/**************************************************************************/
void Adafruit_RA8875::touchEnable(bool on)
{
  uint8_t   adcClk = (uint8_t) RA8875_TPCR0_ADCCLK_DIV4;

  if ( _size == RA8875_800x480 ) //match up touch size with LCD size
    adcClk = (uint8_t) RA8875_TPCR0_ADCCLK_DIV16;

  if (on)
  {
    /* Enable Touch Panel (Reg 0x70) */
    writeReg(RA8875_TPCR0, RA8875_TPCR0_ENABLE        |
                           RA8875_TPCR0_WAIT_4096CLK  |
                           RA8875_TPCR0_WAKEENABLE   |
                           adcClk); // 10mhz max!
    /* Set Auto Mode      (Reg 0x71) */
    writeReg(RA8875_TPCR1, RA8875_TPCR1_AUTO    |
                           // RA8875_TPCR1_VREFEXT |
                           RA8875_TPCR1_DEBOUNCE);
    /* Enable TP INT */
    writeReg(RA8875_INTC1, readReg(RA8875_INTC1) | RA8875_INTC1_TP);
  }
  else
  {
    /* Disable TP INT */
    writeReg(RA8875_INTC1, readReg(RA8875_INTC1) & ~RA8875_INTC1_TP);
    /* Disable Touch Panel (Reg 0x70) */
    writeReg(RA8875_TPCR0, RA8875_TPCR0_DISABLE);
  }
}

/**************************************************************************/
/*!
      Checks if a touch event has occured
      
      @returns  True is a touch event has occured (reading it via
                touchRead() will clear the interrupt in memory)
*/
/**************************************************************************/
bool Adafruit_RA8875::touched(void)
{
  if (readReg(RA8875_INTC2) & RA8875_INTC2_TP) return true;
  return false;
}

/**************************************************************************/
/*!
      Reads the last touch event
      
      @args x[out]  Pointer to the uint16_t field to assign the raw X value
      @args y[out]  Pointer to the uint16_t field to assign the raw Y value
      
      @note Calling this function will clear the touch panel interrupt on
            the RA8875, resetting the flag used by the 'touched' function
*/
/**************************************************************************/
bool Adafruit_RA8875::touchRead(uint16_t *x, uint16_t *y)
{
  uint16_t tx, ty;
  uint8_t temp;
  
  tx = readReg(RA8875_TPXH);
  ty = readReg(RA8875_TPYH);
  temp = readReg(RA8875_TPXYL);
  tx <<= 2;
  ty <<= 2;
  tx |= temp & 0x03;        // get the bottom x bits
  ty |= (temp >> 2) & 0x03; // get the bottom y bits

  *x = tx;
  *y = ty;

  /* Clear TP INT Status */
  writeReg(RA8875_INTC2, RA8875_INTC2_TP);

  return true;
}

/**************************************************************************/
/*!
      Turns the display on or off
*/
/**************************************************************************/
void Adafruit_RA8875::displayOn(bool on)
{
 if (on) 
   writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
 else
   writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
}

/**************************************************************************/
/*!
    Puts the display in sleep mode, or disables sleep mode if enabled
*/
/**************************************************************************/
void Adafruit_RA8875::sleep(bool sleep)
{
 if (sleep) 
   writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF | RA8875_PWRR_SLEEP);
 else
   writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF);
}

/************************* Low Level ***********************************/

/**************************************************************************/
/*!

*/
/**************************************************************************/
void  Adafruit_RA8875::writeReg(uint8_t reg, uint8_t val) 
{
  writeCommand(reg);
  writeData(val);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t  Adafruit_RA8875::readReg(uint8_t reg) 
{
  writeCommand(reg);
  return readData();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void  Adafruit_RA8875::writeData(uint8_t d) 
{
  digitalWrite(LOW);
  spi_begin();
  //SPI.transfer(RA8875_DATAWRITE);
  //SPI.transfer(d);
  ssp1_exchange_byte(RA8875_DATAWRITE);
  ssp1_exchange_byte(d);
  spi_end();
  digitalWrite(HIGH);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t  Adafruit_RA8875::readData(void) 
{
  digitalWrite(LOW);
  spi_begin();
  //SPI.transfer(RA8875_DATAREAD);
  //uint8_t x = SPI.transfer(0x0);
  ssp1_exchange_byte(RA8875_DATAREAD);
  uint8_t x = ssp1_exchange_byte(0x0);
  spi_end();
  digitalWrite(HIGH);
  return x;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void  Adafruit_RA8875::writeCommand(uint8_t d) 
{
  digitalWrite(LOW);
  spi_begin();
  //SPI.transfer(RA8875_CMDWRITE);
  //SPI.transfer(d);
  ssp1_exchange_byte(RA8875_CMDWRITE);
  ssp1_exchange_byte(d);
  spi_end();
  digitalWrite(HIGH);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t  Adafruit_RA8875::readStatus(void) 
{
  digitalWrite(LOW);
  spi_begin();
  //SPI.transfer(RA8875_CMDREAD);
  //uint8_t x = SPI.transfer(0x0);
  ssp1_exchange_byte(RA8875_CMDREAD);
  uint8_t x = ssp1_exchange_byte(0x0);
  spi_end();
  digitalWrite(HIGH);
  return x;
}


/* Helper API's added for Snake and Ladder project*/
bool Adafruit_RA8875::init_display(RA8875sizes size, uint32_t pwm_clk)
{
	if(!begin(size))
	{
		printf("RA8875 Not Found!\n");
		return false;
	}

	displayOn(true);
	GPIOX(true);      // Enable TFT - display enable tied to GPIOX
	PWM1config(true, pwm_clk); // PWM output for backlight
	PWM1out(255);
	return true;
}


static  FRESULT read_wrapper(FIL file, void* pData, unsigned int bytesToRead, unsigned int offset)
{
    FRESULT status = FR_INT_ERR;
    unsigned int bytesRead = 0;

	if(offset) {
		f_lseek(&file, offset);
	}
	status = f_read(&file, pData, bytesToRead, &bytesRead);
    return status;
}

void Adafruit_RA8875::bmpDraw_8bit(char *filename, int x, int y)
{
	uint32_t  	bmpWidth, bmpHeight;   // W+H in pixels
	uint16_t  	bmpDepth;              // Bit depth (currently must be 24)
	uint32_t 	bmpImageoffset;        // Start of image data in file
	uint32_t 	rowSize;               // Not always = bmpWidth; may have padding
	uint8_t  	sdbuffer[BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
	uint8_t 	lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
	uint8_t  	buffidx = sizeof(sdbuffer); // Current position in sdbuffer
	bool  		goodBmp = false;       // Set to true on valid header parse
	bool  		flip    = true;        // BMP is stored bottom-to-top
	int      	w, h, row, col;
	uint8_t  	r, g, b;
	uint32_t 	pos = 0;
	uint8_t  	lcdidx = 0;
	bool  		first 	= true;

	if((x >= width()) || (y >= height())) return;

	//u0_dbg_printf("Loading image - %s\n", filename);
	FIL file = { 0 };
	FRESULT status;
	// Open Existing file
	if (FR_OK != (status = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ))) {
		u0_dbg_printf("##############File not found\n");
		return;
	}

	uint16_t data = 0;
	uint32_t offset = 0;
	// Parse BMP header
	read_wrapper(file,  &data, sizeof(data), offset);
	offset += sizeof(data);
	if(data == 0x4D42)
	{
		// BMP signature
		uint32_t data1 = 0;
		read_wrapper(file,  &data1, sizeof(data1), offset);
		offset += sizeof(data1);
		read_wrapper(file,  &data1, sizeof(data1), offset);
		offset += sizeof(data1);
		read_wrapper(file,  &bmpImageoffset, sizeof(bmpImageoffset), offset);
		offset += sizeof(bmpImageoffset);
		//u0_dbg_printf("Image Offset: %d\n",bmpImageoffset);

		// Read DIB header
		uint32_t biSize = 0;
		read_wrapper(file,  &biSize, sizeof(biSize), offset);
		offset += sizeof(biSize);
		read_wrapper(file,  &bmpWidth, sizeof(bmpWidth), offset);
		offset += sizeof(bmpWidth);
		read_wrapper(file,  &bmpHeight, sizeof(bmpHeight), offset);
		offset += sizeof(bmpHeight);
		//u0_dbg_printf("%d x %d\n", bmpWidth, bmpHeight);

		uint16_t biPlanes = 0;
		read_wrapper(file,  &biPlanes, sizeof(biPlanes), offset);
		offset += sizeof(biPlanes);

		if(biPlanes == 1)
		{
			// # planes -- must be '1'
			read_wrapper(file,  &bmpDepth, sizeof(bmpDepth), offset);  // bits per pixel
			offset += sizeof(bmpDepth);

			uint32_t biCompression = 0;
			read_wrapper(file,  &biCompression, sizeof(biCompression), offset);
			offset += sizeof(biCompression);
			u0_dbg_printf("...............bmpDepth = %d x biCompression  = %d\n", bmpDepth, biCompression);
			if(biCompression == 0)
			{
				// 0 = uncompressed
				goodBmp = true; // Supported BMP format -- proceed!

				// BMP rows are padded (if needed) to 4-byte boundary
				//rowSize = (bmpWidth * 3 + 3) & ~3;
				rowSize = (bmpWidth + 3) & ~3;

				// If bmpHeight is negative, image is in top-down order.
				// This is not canon but has been observed in the wild.
				if(bmpHeight < 0)
				{
					bmpHeight = -bmpHeight;
					flip      = false;
				}

				// Crop area to be loaded
				w = bmpWidth;
				h = bmpHeight;
				if((x+w-1) >= width())  w = width()  - x;
				if((y+h-1) >= height()) h = height() - y;

				// Set TFT address window to clipped image bounds

				for (row=0; row<h; row++)
				{
					// For each scanline...
					// Seek to start of scan line.  It might seem labor-
					// intensive to be doing this on every line, but this
					// method covers a lot of gritty details like cropping
					// and scanline padding.  Also, the seek only takes
					// place if the file position actually needs to change
					// (avoids a lot of cluster math in SD library).
					if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
						pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
					else     // Bitmap is stored top-to-bottom
						pos = bmpImageoffset + row * rowSize;
					if(offset != pos)
					{
						// Need seek?
						offset = pos;
						buffidx = sizeof(sdbuffer); // Force buffer reload
					}

					for (col=0; col<w; col++)
					{
						// For each column...
						// Time to read more pixel data?
						if (buffidx >= sizeof(sdbuffer))
						{
							// Indeed
							// Push LCD buffer to the display first
							if(lcdidx > 0)
							{
								drawPixel_8bit(col+x, row+y, lcdbuffer[lcdidx]);
								lcdidx = 0;
								first  = false;
							}
							read_wrapper(file,  sdbuffer, sizeof(sdbuffer), offset);
							offset += sizeof(sdbuffer);
							buffidx = 0; // Set index to beginning
						}
						// Convert pixel from BMP to TFT format
						lcdbuffer[lcdidx] = sdbuffer[buffidx++];
						drawPixel_8bit(col+x, row+y, lcdbuffer[lcdidx]);
					} // end pixel

				} // end scanline

				// Write any remaining data to LCD
				if(lcdidx > 0)
				{
					drawPixel_8bit(col+x, row+y, lcdbuffer[lcdidx]);
				}

			} // end goodBmp
		}
	}

	if(!goodBmp)
		u0_dbg_printf("BMP format not recognized.\n");

	f_close(&file);
}


void Adafruit_RA8875::bmpDraw(char *filename, int x, int y)
{
	uint32_t  	bmpWidth, bmpHeight;   // W+H in pixels
	uint16_t  	bmpDepth;              // Bit depth (currently must be 24)
	uint32_t 	bmpImageoffset;        // Start of image data in file
	uint32_t 	rowSize;               // Not always = bmpWidth; may have padding
	uint8_t  	sdbuffer[3*BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
	uint16_t 	lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
	uint8_t  	buffidx = sizeof(sdbuffer); // Current position in sdbuffer
	bool  		goodBmp = false;       // Set to true on valid header parse
	bool  		flip    = true;        // BMP is stored bottom-to-top
	int      	w, h, row, col;
	uint8_t  	r, g, b;
	uint32_t 	pos = 0;
	uint8_t  	lcdidx = 0;
	bool  		first 	= true;

	if((x >= width()) || (y >= height())) return;

	//u0_dbg_printf("Loading image - %s\n", filename);
	FIL file = { 0 };
	FRESULT status;
	// Open Existing file
	if (FR_OK != (status = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ))) {
		u0_dbg_printf("##############File not found\n");
		return;
	}

	uint16_t data = 0;
	uint32_t offset = 0;
	// Parse BMP header
	read_wrapper(file,  &data, sizeof(data), offset);
	offset += sizeof(data);
	if(data == 0x4D42)
	{
		// BMP signature
		uint32_t data1 = 0;
		read_wrapper(file,  &data1, sizeof(data1), offset);
		offset += sizeof(data1);
		read_wrapper(file,  &data1, sizeof(data1), offset);
		offset += sizeof(data1);
		read_wrapper(file,  &bmpImageoffset, sizeof(bmpImageoffset), offset);
		offset += sizeof(bmpImageoffset);
		//u0_dbg_printf("Image Offset: %d\n",bmpImageoffset);

		// Read DIB header
		uint32_t biSize = 0;
		read_wrapper(file,  &biSize, sizeof(biSize), offset);
		offset += sizeof(biSize);
		read_wrapper(file,  &bmpWidth, sizeof(bmpWidth), offset);
		offset += sizeof(bmpWidth);
		read_wrapper(file,  &bmpHeight, sizeof(bmpHeight), offset);
		offset += sizeof(bmpHeight);
		//u0_dbg_printf("%d x %d\n", bmpWidth, bmpHeight);

		uint16_t biPlanes = 0;
		read_wrapper(file,  &biPlanes, sizeof(biPlanes), offset);
		offset += sizeof(biPlanes);

		if(biPlanes == 1)
		{
			// # planes -- must be '1'
			read_wrapper(file,  &bmpDepth, sizeof(bmpDepth), offset);  // bits per pixel
			offset += sizeof(bmpDepth);

			uint32_t biCompression = 0;
			read_wrapper(file,  &biCompression, sizeof(biCompression), offset);
			offset += sizeof(biCompression);
			if((bmpDepth == 24) && (biCompression == 0))
			{
				// 0 = uncompressed
				goodBmp = true; // Supported BMP format -- proceed!

				// BMP rows are padded (if needed) to 4-byte boundary
				rowSize = (bmpWidth * 3 + 3) & ~3;

				// If bmpHeight is negative, image is in top-down order.
				// This is not canon but has been observed in the wild.
				if(bmpHeight < 0)
				{
					bmpHeight = -bmpHeight;
					flip      = false;
				}

				// Crop area to be loaded
				w = bmpWidth;
				h = bmpHeight;
				if((x+w-1) >= width())  w = width()  - x;
				if((y+h-1) >= height()) h = height() - y;

				// Set TFT address window to clipped image bounds

				for (row=0; row<h; row++)
				{
					// For each scanline...
					// Seek to start of scan line.  It might seem labor-
					// intensive to be doing this on every line, but this
					// method covers a lot of gritty details like cropping
					// and scanline padding.  Also, the seek only takes
					// place if the file position actually needs to change
					// (avoids a lot of cluster math in SD library).
					if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
						pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
					else     // Bitmap is stored top-to-bottom
						pos = bmpImageoffset + row * rowSize;
					if(offset != pos)
					{
						// Need seek?
						offset = pos;
						buffidx = sizeof(sdbuffer); // Force buffer reload
					}

					for (col=0; col<w; col++)
					{
						// For each column...
						// Time to read more pixel data?
						if (buffidx >= sizeof(sdbuffer))
						{
							// Indeed
							// Push LCD buffer to the display first
							if(lcdidx > 0)
							{
								drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
								lcdidx = 0;
								first  = false;
							}
							read_wrapper(file,  sdbuffer, sizeof(sdbuffer), offset);
							offset += sizeof(sdbuffer);
							buffidx = 0; // Set index to beginning
						}
						// Convert pixel from BMP to TFT format
						b = sdbuffer[buffidx++];
						g = sdbuffer[buffidx++];
						r = sdbuffer[buffidx++];
						lcdbuffer[lcdidx] = color565(r,g,b);
						drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
					} // end pixel

				} // end scanline

				// Write any remaining data to LCD
				if(lcdidx > 0)
				{
					drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
				}

			} // end goodBmp
		}
	}

	if(!goodBmp)
		u0_dbg_printf("BMP format not recognized.\n");

	f_close(&file);
}


void Adafruit_RA8875::bmpDrawFromHeader(char *filename, int x, int y)
{
	uint32_t  	bmpWidth, bmpHeight;   // W+H in pixels
	uint16_t  	bmpDepth;              // Bit depth (currently must be 24)
	uint32_t 	bmpImageoffset;        // Start of image data in file
	uint32_t 	rowSize;               // Not always = bmpWidth; may have padding
	uint8_t  	sdbuffer[3*BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
	uint16_t 	lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
	uint8_t  	buffidx = sizeof(sdbuffer); // Current position in sdbuffer
	bool  		goodBmp = false;       // Set to true on valid header parse
	bool  		flip    = true;        // BMP is stored bottom-to-top
	int      	w, h, row, col;
	uint8_t  	r, g, b;
	uint32_t 	pos = 0;
	uint8_t  	lcdidx = 0;
	bool  		first 	= true;
	w = 800;
	h = 480;

	if((x >= w) || (y >= 480)) return;

	uint16_t data = 0;
	uint32_t offset = 0;
	{
		{

			{
				// Set TFT address window to clipped image bounds
				for (row=0; row<h; row++)
				{
					// For each scanline...
					// Seek to start of scan line.  It might seem labor-
					// intensive to be doing this on every line, but this
					// method covers a lot of gritty details like cropping
					// and scanline padding.  Also, the seek only takes
					// place if the file position actually needs to change
					// (avoids a lot of cluster math in SD library).
					if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
						pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
					else     // Bitmap is stored top-to-bottom
						pos = bmpImageoffset + row * rowSize;
					if(offset != pos)
					{
						// Need seek?
						offset = pos;
						buffidx = sizeof(sdbuffer); // Force buffer reload
					}

					for (col=0; col<w; col++)
					{
						// For each column...
						// Time to read more pixel data?
						if (buffidx >= sizeof(sdbuffer))
						{
							// Indeed
							// Push LCD buffer to the display first
							if(lcdidx > 0)
							{
								drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
								lcdidx = 0;
								first  = false;
							}
//							read_wrapper(file,  sdbuffer, sizeof(sdbuffer), offset);
							memcpy(sdbuffer, MagickImage + offset, sizeof(sdbuffer));
							offset += sizeof(sdbuffer);
							buffidx = 0; // Set index to beginning
						}
						// Convert pixel from BMP to TFT format
						b = sdbuffer[buffidx++];
						g = sdbuffer[buffidx++];
						r = sdbuffer[buffidx++];
						lcdbuffer[lcdidx] = color565(r,g,b);
						drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
					} // end pixel

				} // end scanline

				// Write any remaining data to LCD
				if(lcdidx > 0)
				{
					drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
				}

			} // end goodBmp
		}
	}
}



uint16_t Adafruit_RA8875::color565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


/**************************************************************************/
/*!
      Set the display 'Color Space'
	  Parameters:
	  Bit per Pixel color (colors): 8 or 16 bit
	  NOTE:
	  For display over 272*480 give the ability to use
	  Layers since at 16 bit it's not possible.
*/
/**************************************************************************/
void Adafruit_RA8875::setColorBpp(uint8_t colors)
{
	if (colors != _color_bpp){//only if necessary
		if (colors < 16) {
			_color_bpp = 8;
			_colorIndex = 3;
			writeReg(RA8875_SYSR,0x00);
			if (_hasLayerLimits) _maxLayers = 2;
		} else if (colors > 8) {//65K
			_color_bpp = 16;
			_colorIndex = 0;
			writeReg(RA8875_SYSR,0x0C);
			if (_hasLayerLimits) _maxLayers = 1;
			_currentLayer = 0;
		}
	}
}



/**************************************************************************/
/*!
		Clear memory (different from fillWindow!)
	    Parameters:
		stop: stop clear memory operation
*/
/**************************************************************************/
void Adafruit_RA8875::clearMemory(bool stop)
{
	uint8_t temp;
	temp = readReg(RA8875_MCLR);
	stop == true ? temp &= ~(1 << 7) : temp |= (1 << 7);
	writeData(temp);
}

/**************************************************************************/


/**************************************************************************/
/*!
		Clear the active window
	    Parameters:
		full: false(clear current window), true clear full window
*/
/**************************************************************************/
void Adafruit_RA8875::clearActiveWindow(bool full)
{
	uint8_t temp;
	temp = readReg(RA8875_MCLR);
	full == true ? temp &= ~(1 << 6) : temp |= (1 << 6);
	writeData(temp);
}


/*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+								LAYER STUFF											 +
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

/**************************************************************************/
/*! This is the most important function to write on:
	LAYERS
	CGRAM
	PATTERN
	CURSOR
	Parameter:
	d (L1, L2, CGRAM, PATTERN, CURSOR)
	When writing on layers 0 or 1, if the layers are not enable it will enable automatically
	If the display doesn't support layers, it will automatically switch to 8bit color
	Remember that when layers are ON you need to disable manually, once that only Layer 1 will be visible

*/
/**************************************************************************/
void Adafruit_RA8875::setLayer(enum RA8875writes d)
{
	uint8_t temp = readReg(RA8875_MWCR1);
	switch(d)
	{
		case L1:
			temp &= ~((1<<3) | (1<<2));// Clear bits 3 and 2
			temp &= ~(1 << 0); //clear bit 0
			_currentLayer = 0;
			writeData(temp);
			if (!_useMultiLayers) useLayers(true);
		break;
		case L2:
			temp &= ~((1<<3) | (1<<2));// Clear bits 3 and 2
			temp |= (1 << 0); //bit set 0
			_currentLayer = 1;
			writeData(temp);
			if (!_useMultiLayers) useLayers(true);
		break;
		case CGRAM:
		case PATTERN:
		case CURSOR:
		default:
			return;
	}
}



/**************************************************************************/
/*!
		Instruct the RA8875 chip to use 2 layers
		If resolution bring to restrictions it will switch to 8 bit
		so you can always use layers.
		Parameters:
		on: true (enable multiple layers), false (disable)

*/
/**************************************************************************/
void Adafruit_RA8875::useLayers(bool on)
{
	if (_useMultiLayers == on) return; //no reason to do change that it's already as desidered.
	if (_hasLayerLimits && _color_bpp > 8)
	{
		setColorBpp(8);
		_maxLayers = 2;
	}
	if (on)
	{
		_useMultiLayers = true;
		_DPCR_Reg |= (1 << 7);
		clearActiveWindow(true);
	}
	else
	{
		_useMultiLayers = false;
		_DPCR_Reg &= ~(1 << 7);
		clearActiveWindow(false);
	}

	writeReg(RA8875_DPCR,_DPCR_Reg);
	if (!_useMultiLayers && _color_bpp < 16) setColorBpp(16);
}



/**************************************************************************/
/*!


*/
/**************************************************************************/
void Adafruit_RA8875::layerEffect(enum RA8875bool efx)
{
	uint8_t	reg = 0b00000000;
	//reg &= ~(0x07);//clear bit 2,1,0
	if (!_useMultiLayers) useLayers(true);//turn on multiple layers if it's off
	switch(efx){//                       bit 2,1,0 of LTPR0
		case LAYER1: //only layer 1 visible  [000]
			//do nothing
		break;
		case LAYER2: //only layer 2 visible  [001]
			reg |= (1 << 0);
		break;
		case TRANSPARENT: //transparent mode [011]
			reg |= (1 << 0); reg |= (1 << 1);
		break;
		case LIGHTEN: //lighten-overlay mode [010]
			reg |= (1 << 1);
		break;
		case OR: //bool OR mode           [100]
			reg |= (1 << 2);
		break;
		case AND: //bool AND mode         [101]
			reg |= (1 << 0); reg |= (1 << 2);
		break;
		case FLOATING: //floating windows    [110]
			reg |= (1 << 1); reg |= (1 << 2);
		break;
		default:
			//do nothing
		break;
	}
	writeReg(RA8875_LTPR0, reg);
}

/**************************************************************************/
/*!


*/
/**************************************************************************/
void Adafruit_RA8875::layerTransparency(uint8_t layer1, uint8_t layer2)
{
	if (layer1 > 8) layer1 = 8;
	if (layer2 > 8) layer2 = 8;
	if (!_useMultiLayers) useLayers(true);//turn on multiple layers if it's off
	//if (_useMultiLayers) _writeRegister(RA8875_LTPR1, ((layer2 & 0x0F) << 4) | (layer1 & 0x0F));
	//uint8_t res = 0b00000000;//RA8875_LTPR1
	//reg &= ~(0x07);//clear bit 2,1,0
	writeReg(RA8875_LTPR1, ((layer2 & 0xF) << 4) | (layer1 & 0xF));
}


/**************************************************************************/
/*! return the current drawing layer. If layers are OFF, return 255

*/
/**************************************************************************/
uint8_t Adafruit_RA8875::getCurrentLayer(void)
{
	if (!_useMultiLayers) return 255;
	return _currentLayer;
}
