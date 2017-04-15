/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */

#define BUFFPIXEL 20

#include <stdio.h>
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "Adafruit_RA8875-master/Adafruit_RA8875.h"
#include "eint.h"
#include "soft_timer.hpp"
#include "printf_lib.h"
#include "utilities.h"
#include "ff.h"
#include "storage.hpp"


SoftTimer debounceTimer;
SemaphoreHandle_t gButtonPressSemaphore = NULL;


void callback_func(void)
{
	long yield = 0;
    if (debounceTimer.expired())
    {
    	xSemaphoreGiveFromISR(gButtonPressSemaphore, &yield);
    	portYIELD_FROM_ISR(yield);
    	debounceTimer.reset(50);

    }
}


class switch_press_task : public scheduler_task
{
    public:
        switch_press_task(uint8_t priority) : scheduler_task("task", 2001, priority)
        {
            /* Nothing to init */
        }

        bool init(void) //Optional
        {
        	eint3_enable_port2(5, eint_rising_edge, callback_func);
        	debounceTimer.reset(50);
            return true;
        }

        bool run(void *p) //It is required
        {
        	while(1)
        	{
        		if(xSemaphoreTake(gButtonPressSemaphore, portMAX_DELAY))
        		{
        			u0_dbg_printf("Callback for P2.4 invoked\n");
        		}
        	}
            return true;
        }
};


class LCD_task : public scheduler_task
{
    public:
        LCD_task(uint8_t priority) : scheduler_task("task", 8000, priority)
        {
            /* Nothing to init */
        }

        bool init(void)
        {
        	return true;
        }

        void draw_shape()
        {
			Adafruit_RA8875 tft(3,4);
			if(!tft.begin(RA8875_800x480))
			{
				printf("RA8875 Not Found!\n");
				while(1);
			}

			tft.displayOn(true);
			tft.GPIOX(true);      // Enable TFT - display enable tied to GPIOX
			tft.PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
			tft.PWM1out(255);

			// With hardware accelleration this is instant
			tft.fillScreen(RA8875_WHITE);

			// Play with PWM
			for (uint8_t i=255; i!=0; i-=5 )
			{
				tft.PWM1out(i);
				delay_ms(10);
			}
			for (uint8_t i=0; i!=255; i+=5 )
			{
				tft.PWM1out(i);
				delay_ms(10);
			}
			tft.PWM1out(255);

			tft.fillScreen(RA8875_RED);
			delay_ms(500);
			tft.fillScreen(RA8875_YELLOW);
			delay_ms(500);
			tft.fillScreen(RA8875_GREEN);
			delay_ms(500);
			tft.fillScreen(RA8875_CYAN);
			delay_ms(500);
			tft.fillScreen(RA8875_MAGENTA);
			delay_ms(500);
			tft.fillScreen(RA8875_BLACK);

			// Try some GFX acceleration!
			tft.drawCircle(100, 100, 50, RA8875_BLACK);
			tft.fillCircle(100, 100, 49, RA8875_GREEN);

			tft.fillRect(11, 11, 398, 198, RA8875_BLUE);
			tft.drawRect(10, 10, 400, 200, RA8875_GREEN);
			tft.fillRoundRect(200, 10, 200, 100, 10, RA8875_RED);
			tft.drawPixel(10,10,RA8875_BLACK);
			tft.drawPixel(11,11,RA8875_BLACK);
			tft.drawLine(10, 10, 200, 100, RA8875_RED);
			tft.drawTriangle(200, 15, 250, 100, 150, 125, RA8875_BLACK);
			tft.fillTriangle(200, 16, 249, 99, 151, 124, RA8875_YELLOW);
			tft.drawEllipse(300, 100, 100, 40, RA8875_BLACK);
			tft.fillEllipse(300, 100, 98, 38, RA8875_GREEN);
			// Argument 5 (curvePart) is a 2-bit value to control each corner (select 0, 1, 2, or 3)
			tft.drawCurve(50, 100, 80, 40, 2, RA8875_BLACK);
			tft.fillCurve(50, 100, 78, 38, 2, RA8875_WHITE);

        }

        uint16_t color565(uint8_t r, uint8_t g, uint8_t b)
        {
        	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
        }

        void bmpDraw(Adafruit_RA8875 tft, char *filename, int x, int y)
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

        	if((x >= tft.width()) || (y >= tft.height())) return;

        	//u0_dbg_printf("Loading image - %s\n", filename);
        	FIL fatfs_file = { 0 };
        	FRESULT status;
        	// Open Existing file
        	if (FR_OK != (status = f_open(&fatfs_file, filename, FA_OPEN_EXISTING | FA_READ))) {
        		u0_dbg_printf("##############File not found\n");
        		return;
        	}



        	uint16_t data = 0;
        	uint32_t offset = 0;
//          u0_dbg_printf("............%d\n", Storage::read(filename,  &data, sizeof(data), offset));
//          u0_dbg_printf("...............%d\n", data);

        	// Parse BMP header
        	Storage::read(filename,  &data, sizeof(data), offset);
        	offset += sizeof(data);
        	if(data == 0x4D42)
        	{
        		// BMP signature
        		uint32_t data1 = 0;
        		Storage::read(filename,  &data1, sizeof(data1), offset);
        		offset += sizeof(data1);
        		Storage::read(filename,  &data1, sizeof(data1), offset);
        		offset += sizeof(data1);
        		Storage::read(filename,  &bmpImageoffset, sizeof(bmpImageoffset), offset);
        		offset += sizeof(bmpImageoffset);
        		//u0_dbg_printf("Image Offset: %d\n",bmpImageoffset);

        		// Read DIB header
        		uint32_t biSize = 0;
        		Storage::read(filename,  &biSize, sizeof(biSize), offset);
        		offset += sizeof(biSize);
        		Storage::read(filename,  &bmpWidth, sizeof(bmpWidth), offset);
        		offset += sizeof(bmpWidth);
        		Storage::read(filename,  &bmpHeight, sizeof(bmpHeight), offset);
        		offset += sizeof(bmpHeight);
        		//u0_dbg_printf("%d x %d\n", bmpWidth, bmpHeight);

        		uint16_t biPlanes = 0;
        		Storage::read(filename,  &biPlanes, sizeof(biPlanes), offset);
        		offset += sizeof(biPlanes);

        		if(biPlanes == 1)
        		{
        			// # planes -- must be '1'
        			Storage::read(filename,  &bmpDepth, sizeof(bmpDepth), offset);  // bits per pixel
        			offset += sizeof(bmpDepth);

        			uint32_t biCompression = 0;
        			Storage::read(filename,  &biCompression, sizeof(biCompression), offset);
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
        				if((x+w-1) >= tft.width())  w = tft.width()  - x;
        				if((y+h-1) >= tft.height()) h = tft.height() - y;

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
        								tft.drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
        								lcdidx = 0;
        								first  = false;
        							}
        							Storage::read(filename,  sdbuffer, sizeof(sdbuffer), offset);
        							offset += sizeof(sdbuffer);
        							buffidx = 0; // Set index to beginning
        						}
        						// Convert pixel from BMP to TFT format
        						b = sdbuffer[buffidx++];
        						g = sdbuffer[buffidx++];
        						r = sdbuffer[buffidx++];
        						lcdbuffer[lcdidx] = color565(r,g,b);
        						tft.drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
        					} // end pixel

        				} // end scanline

        				// Write any remaining data to LCD
        				if(lcdidx > 0)
        				{
        					tft.drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
        				}

        			} // end goodBmp
        		}
        	}

        	if(!goodBmp)
        		u0_dbg_printf("BMP format not recognized.\n");

        }



        void display_image()
        {
			Adafruit_RA8875 tft(3,4);
      	  /* Initialise the display using 'RA8875_480x272' or 'RA8875_800x480' */
			if(!tft.begin(RA8875_800x480))
			{
				printf("RA8875 Not Found!\n");
				while(1);
			}

        	  tft.displayOn(true);
        	  tft.GPIOX(true);      // Enable TFT - display enable tied to GPIOX
        	  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
        	  tft.PWM1out(255);

        	  //u0_dbg_printf("(");
        	  //u0_dbg_printf("%d, ",tft.width());
        	  //u0_dbg_printf("%d \n",tft.height());
        	  tft.graphicsMode();                 // go back to graphics mode
        	  tft.fillScreen(RA8875_BLACK);
        	  tft.graphicsMode();
        	  bmpDraw(tft, "1:board.bmp", 0, 0);
        }

        bool run(void *p) //It is required
        {
			//draw_shape();
        	display_image();


			while(1);
			return true;
        }
};



/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
int main(void)
{
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */
	scheduler_add_task(new terminalTask(PRIORITY_HIGH));

	gButtonPressSemaphore = xSemaphoreCreateBinary();
    scheduler_add_task(new LCD_task(PRIORITY_HIGH));
    scheduler_add_task(new switch_press_task(PRIORITY_HIGH));


    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    //scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    const bool run_1Khz = false;
    scheduler_add_task(new periodicSchedulerTask(run_1Khz));
    #endif

    /* The task for the IR receiver to "learn" IR codes */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
