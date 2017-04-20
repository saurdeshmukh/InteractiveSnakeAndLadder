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
#include "io.hpp"
#include<stdlib.h>
#include<time.h>
#define CountSnakeLadder 14

bool gameOver=false;

SoftTimer debounceTimer;
SemaphoreHandle_t gButtonPressSemaphore = NULL;
Adafruit_RA8875 tft(3,4);
time_t t;
typedef struct
{
	int Number;
	int nextLocation;
	int X;
	int Y;
}Cell;
Cell Game[100]={0};
uint8_t button1=0;
uint8_t button2=0;
uint8_t SnakeLadderMatrix[CountSnakeLadder][2]={{3,21},{8,30},{17,13},{28,84},{52,29},{57,40},{58,77},{62,22},{75,86},{80,100},{88,18},{90,91},{95,51},{97,79}};
Cell playerA={0,0,0,0};
Cell playerB={0,0,0,0};
void makeGame()
{
	uint8_t count=1;
	for(int j=0;j<10;j++)
	{
		if(j%2)
		{
          for(int i=9;i>=0;i--)
          {
        	  Game[count-1].Number=count;
        	  Game[count-1].X=68+(i*74);
        	  Game[count-1].Y=437-(j*44);
        	  for(int k=0;k<CountSnakeLadder;k++)
        	  {
        		  if(SnakeLadderMatrix[k][0]==count)
        			  Game[count-1].nextLocation=SnakeLadderMatrix[k][1];
        	  }
        	  count++;
          }

		}
		else
		{
		  for(int i=0;i<10;i++)
		  {
			  Game[count-1].Number=count;
			  Game[count-1].X=68+(i*74);
			  Game[count-1].Y=437-(j*44);
			  for(int k=0;k<CountSnakeLadder;k++)
			  {
				  if(SnakeLadderMatrix[k][0]==count)
					  Game[count-1].nextLocation=SnakeLadderMatrix[k][1];
			  }
			  count++;
		  }
		}
	}

}
void drawCircle(int drawA,int drawB)
{
	int i,j;
	//clear memory
	tft.clearMemory();
	delay_ms(100);
	//drawing circle
		i=Game[drawA].X;
	    j=Game[drawA].Y;
	    printf("\nPlayer A X-%d Y-%d",i,j);
	tft.drawCircle(i,j, 20, RA8875_BLUE);
    tft.fillCircle(i,j, 19, RA8875_BLUE);
    tft.layerEffect(OR);
    delay_ms(100);
        i=Game[drawB].X;
    	j=Game[drawB].Y;
    	printf("\nPlayer B X-%d Y-%d",i,j);
    tft.drawCircle(i,j, 20, RA8875_WHITE);
    tft.fillCircle(i,j, 19, RA8875_WHITE);
    tft.layerEffect(OR);
    delay_ms(100);


	button1=0;
	button2=0;

}
bool getNextLocation(int diceRollOut,int playerId)
{
	uint8_t location=0;
	int oldA=0,oldB=0;

  if(playerId==1 && gameOver==false)
  {
	  location=playerA.Number+diceRollOut;
	  oldA=playerA.Number-1;
	  if(location>100)
		  return false;
	  else if(location == 100 || playerA.Number ==100)
	  {
		   printf("\nPlayer 1 Winnnnnerrr");
		   playerA.Number=location;
		   button1=10;
		   gameOver=true;
	  }
	  else
	  {
	  if(Game[location-1].nextLocation!=0)
		  playerA.Number=Game[location-1].nextLocation;
	  else
		  playerA.Number=location;
	  }
	  for(int m=oldA;m<location;m++)
	  {
		  drawCircle(m,(playerB.Number-1));
		  delay_ms(100);
	  }

  }

  if(playerId==2 && gameOver==false)
   {
 	  location=playerB.Number+diceRollOut;
 	  oldB=playerB.Number-1;
 	  if(location>100)
 		  return false;
 	  else if(location==100 || playerB.Number ==100)
	  {
	  printf("\nPlayer 2 Winnnneerrrr");
	  playerB.Number=location;
	  button2=10;
	  gameOver=true;
	  }
 	 else
 	  {
	  if(Game[location-1].nextLocation!=0)
		  playerB.Number=Game[location-1].nextLocation;
	  else
		  playerB.Number=location;
 	  }
 	 for(int m=oldB;m<location;m++)
	  {
		  drawCircle((playerA.Number-1),m);
		  delay_ms(100);
	  }
   }
  printf("\n location X - %d location Y - %d",playerA.Number,playerB.Number);
  if(gameOver)
  {
		tft.fillScreen(RA8875_BLACK);
		tft.textMode();
	  /* Render some text! */
	  tft.textColor(RA8875_BLUE,RA8875_WHITE);
	  tft.textSetCursor(100, 150);
	  tft.textEnlarge(3);
	  tft.textWrite("Player 2 Won!!!!");
	  tft.layerEffect(OR);
  }
  else
  {
	  drawCircle((playerA.Number-1),(playerB.Number-1));
  }
  return gameOver;
}

int rollDice(int playerId)
{
	uint8_t diceRollout=(rand()%6)+1;
	LD.setNumber(diceRollout);
	printf("\n Player Played-%d RolledOut- %d",playerId,diceRollout);
	return diceRollout;
	//getNextLocation(diceRollout,playerId);
}
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
        	//draw_shape();
        	makeGame();
        	display_image();
        	srand((unsigned) time(&t));
        	return true;
        }

        void draw_shape()
        {

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
			tft.fillScreen(RA8875_RED);
			tft.graphicsMode();
			tft.setLayer(L1);
			tft.bmpDraw_8bit("1:board_8bit.bmp", 0, 0);
			//tft.bmpDraw("1:board.bmp", 0, 0);

			tft.setLayer(L2);
//			for(int i = 0; i < 10; i++)
//			{
//				for(int j = 0; j < 10; j++)
//				{
//					tft.drawCircle(68 + (i*74), 437 - (j*44), 20, RA8875_WHITE);
//					tft.fillCircle(68 + (i*74), 437 - (j*44), 19, RA8875_WHITE);
//					tft.layerEffect(OR);
//					delay_ms(100);
//					tft.clearMemory();
//					delay_ms(100);
//				}
//				tft.clearMemory();
//			}
        }

        bool run(void *p) //It is required
        {
          int roll=0;
          bool gameOver=false;
          if(!gameOver)
          {
            if(SW.getSwitch(1)&&(button1!=1))
            {
            	button1=1;

            }

            if(SW.getSwitch(2)&&(button2!=1))
            {
            	button2=1;
            }
            if(button1==1 && button1!=10)
            {
            	puts("\nSwitched 1");
            	roll=rollDice(1);
            	gameOver=getNextLocation(roll,1);

            }
            if(button2==1 && button2!=10)
            {
            	puts("\nSwitched 1");
				roll=rollDice(2);
				gameOver=getNextLocation(roll,2);
            }
          }
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

