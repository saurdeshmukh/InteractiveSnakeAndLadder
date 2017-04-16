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
			if(!tft.init_display(RA8875_800x480, RA8875_PWM_CLK_DIV1024))
			{
				printf("RA8875 Not Found!\n");
				while(1);
			}

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


        void display_image()
        {
        	Adafruit_RA8875 tft(3,4);
			if(!tft.init_display(RA8875_800x480, RA8875_PWM_CLK_DIV1024))
			{
				printf("RA8875 Not Found!\n");
				while(1);
			}
			tft.graphicsMode();                 // go back to graphics mode
			tft.fillScreen(RA8875_BLACK);
			tft.graphicsMode();
			tft.bmpDraw("1:board.bmp", 0, 0);

			for(int i = 0; i < 10; i++)
			{
				for(int j = 0; j < 10; j++)
				{
					tft.drawCircle(68 + (i*74), 437 - (j*44), 20, RA8875_BLACK);
					tft.fillCircle(68 + (i*74), 437 - (j*44), 19, RA8875_WHITE);

				}
			}

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
