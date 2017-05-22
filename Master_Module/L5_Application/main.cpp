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
#include<wireless.h>
#include<iostream>
#include<str.hpp>
#define CountSnakeLadder 12
#define max_hops 2
using namespace std;

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
uint8_t SnakeLadderMatrix[CountSnakeLadder][2]={{3,39},{10,12},{16,13},{27,53},{31,4},{47,25},{56,84},{61,99},{63,60},{66,52},{72,90},{97,75}};
typedef struct
{
 uint8_t count;
 uint8_t address;
}player_list;
static Uart3 &u3 = Uart3::getInstance();
typedef enum
{
	enRoll,
	gameRunning,
	gameOver
}State_t;
class Master_Task : public scheduler_task
{
    public:
	Master_Task(uint8_t priority) : scheduler_task("Master", 2048, priority)
        {
			current_State=enRoll;
			playerCount=0;
			winnerPlayerId=0;
        }

        bool init(void)
        {
			u3.init(9600);
			makeGame();
			display_image();
			srand((unsigned) time(&t));
			audioAlert("N2");
			return true;
        }
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

        void display_image()
		{
		//Adafruit_RA8875 tft(3,4);
		if(!tft.init_display(RA8875_800x480, RA8875_PWM_CLK_DIV1024))
		{
			printf("RA8875 Not Found!\n");
			while(1);
		}
		tft.fillScreen(RA8875_RED);
		tft.graphicsMode();
		tft.useLayers(true);
		tft.setLayer(L1);
		tft.bmpDraw_8bit("1:New_1.bmp", 0, 0);
		//tft.bmpDraw_8bit("1:board_8bit.bmp", 0, 0);
		//tft.bmpDraw("1:board.bmp", 0, 0);
		//tft.bmpDrawFromHeader("1:board.bmp", 0 , 0);

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
        void audioAlert(const char* message)
        {
        	u3.putline(message);
        }
        void showTextMessage(const char* message)
        {
			//tft.fillScreen(RA8875_BLACK);
			tft.textMode();
			tft.textColor(RA8875_BLUE,RA8875_RED);
			//tft.textColor(RA8875_BLUE,RA8875_WHITE);
			tft.textSetCursor(100, 150);
			tft.textEnlarge(3);
			tft.textWrite(message);
			tft.layerEffect(LAYER2);
			vTaskDelay(2000);
		}

        void pingAll()
        {
        	mesh_packet_t ackPkt;
        	//TODO Max Hops are 100 Now
			for(uint8_t i=101;i<=102;i++)
			{
			 if(wireless_send(i,mesh_pkt_ack,"enroll",strlen("enroll"),max_hops))
			 {
				puts("\nSent");
				if(wireless_get_ack_pkt(&ackPkt,100) && i==ackPkt.nwk.src)
				{
					puts("\nReceived ACK");
				}

			 }
			 else
				 puts("\n send failed");
			}
        }
        bool getEnrollment()
        {
        	bool outValue=false;
			mesh_packet_t rcPkt;
			printf("\n Packet waiting");
			while(wireless_get_rx_pkt(&rcPkt,8000))
			{
				str temp="";
				for(int i=0;i<rcPkt.info.data_len;i++)
				{
					temp+=(char)rcPkt.data[i];
				}
				printf("\n Packet Address : %d\n",rcPkt.nwk.src);
				puts(temp.c_str());
				 if(temp =="enrollMe")
				 {
					 players[playerCount].count=0;
					 players[playerCount].address=rcPkt.nwk.src;
					 playerCount++;
				 }
			}

			if(playerCount>0)
				outValue=true;
			for(int i=0;i<playerCount;i++)
			{
				printf("\n players address - %d count- %d Bool value - %d",players[i].address,playerCount,outValue);
			}
			return outValue;
        }
        bool getNextLocation(uint8_t diceRollOut,uint8_t playerId)
        {
        	uint8_t location=0;
        	uint8_t oldLocation=0;
        	uint8_t finalLocation=0;
        	bool returnVal=false;
        	  location=players[playerId].count+diceRollOut;
        	  printf("\nPlayerId-%d location - %d current Location-%d ",playerId,location,players[playerId].count);
        	  oldLocation=players[playerId].count;
        	  if(location>100)
        		  return returnVal;
        	  else if(location == 100)
        	  {
        		  players[playerId].count=location;
        		  returnVal=true;
        	  }
        	  else
        	  {
        	  if(Game[location-1].nextLocation!=0)
        	  {
        		  players[playerId].count=Game[location-1].nextLocation;
        		  if(Game[location-1].nextLocation > Game[location-1].Number)
        		  {
        			  puts("\n LADDDDDER");
        			  vTaskDelay(2000);
        			  audioAlert("SYou got Ladder");
        		  }
        		  else
        		  {
        			  vTaskDelay(2000);
        			  audioAlert("Sohhhhhhhhh You got Snake");
        			  puts("\n LADDDDDER");
        		  }
        		  printf("\nPlayerId-%d location - %d current Location-%d ",playerId,location,players[playerId].count);
        	  }
        	  else
        		  players[playerId].count=location;
        	  }
        	  finalLocation=players[playerId].count;
        	  printf("\n Final Count-%d",finalLocation);
        	  for(int m=oldLocation;m<=location;m++)
        	  {
        		  players[playerId].count=m;
        		  drawCircle();
        		  delay_ms(100);
        	  }
        	  players[playerId].count=finalLocation;
        	  drawCircle();
          return returnVal;
        }
        void drawCircle()
        {
        	int i,j;
        	//clear memory

        	tft.graphicsMode();
        	tft.clearMemory();
        	tft.layerEffect(OR);
        	//tft.setLayer(L2);
        	for(int k=0;k<playerCount;k++)
        	{
				delay_ms(250);
				//drawing circle
				i=Game[players[k].count-1].X;
				j=Game[players[k].count-1].Y;
				//add colors TODO
				tft.drawCircle(i,j, 20, RA8875_RED);
				tft.fillCircle(i,j, 19, RA8875_WHITE);
        	}
			//tft.layerEffect(OR);
        }
        void sendDataToPlayers(str message,uint8_t addr)
        {
        	mesh_packet_t ackPkt;
        	char buffer[100];
        	strcpy(buffer,message.c_str());
        	if(addr==0)
        	{
				for(int i=0;i<playerCount;i++)
				{
					if(wireless_send(players[i].address,mesh_pkt_ack,(char*)buffer,strlen(buffer),max_hops))
						puts("\nSent Stop");
				}
        	}
        	else
        	{
        		puts("\nSending GO");
        		bool ackFlag=false;
        		while(!ackFlag)
        		{
        			wireless_send(addr,mesh_pkt_ack,buffer,strlen(buffer),max_hops);
        			puts(message.c_str());
        			if(wireless_get_ack_pkt(&ackPkt,100) && addr==ackPkt.nwk.src)
					{
        				ackFlag=true;
						puts("\nReceived ACK");
					}

        		}
        	}
        }
        bool buttonPressed(uint8_t address)
        {
			mesh_packet_t rcPkt;
			puts("\nwaiting for button pressed");
			while(wireless_get_rx_pkt(&rcPkt,portMAX_DELAY))
			{
				str temp="";
				for(int i=0;i<rcPkt.info.data_len;i++)
				{
					temp+=(char)rcPkt.data[i];
				}
				puts(temp.c_str());
			 if(temp=="pushed" && rcPkt.nwk.src==address)
			 {
				 puts("\nButton Pressed by Player");
				 return true;
			 }

			}
			return false;
        }
        int rollDice()
        {
        	uint8_t diceRollout=(rand()%6)+1;
        	LD.setNumber(diceRollout);
        	return diceRollout;
        }
        bool run(void *p)
        {
        	uint8_t RollOut=0;
        	switch(current_State)
        	{
        	case enRoll:
        		tft.clearMemory();
        		vTaskDelay(2000);
        		//TODO Add Counting on Screen
				audioAlert("SEnroll for Game Guys");
				showTextMessage("Enroll For Game Now ");
				pingAll();
				while(!getEnrollment())
				{
					pingAll();
				}
				//tft.layerEffect(LAYER1);
				//tft.setLayer(L2);
				//tft.clearMemory();
				sendDataToPlayers("stop",0);
				//TODO Select First player Randomly and use Circular Queue
				current_State=gameRunning;
				break;
        	case gameRunning:
                 puts("\nReached in gameRunning State");
        		for(int i=0;i<playerCount;i++)
        		{
        			static int k=0;
        			audioAlert("SPress Button and Roll Dice");
        			vTaskDelay(2000);
        			sendDataToPlayers("go",players[i].address);
					if(buttonPressed(players[i].address))
					{
					 RollOut=rollDice();
					}
					printf("\n Rolled Out - %d %d",RollOut,playerCount);
					str temp1="dice";
					temp1+=(char)('0'+RollOut);  //check if working
					//TODO change Audio Message for Dice
					str temp2="Syou got";
					temp2+=(char)('0'+RollOut);
					audioAlert(temp2.c_str());
					puts("\n Sending Dice Rolled Out");
					puts(temp1.c_str());
					sendDataToPlayers(temp1,players[i].address);
					if(getNextLocation(RollOut,i))
					{
						puts("\n GameOver State");
						current_State=gameOver;
						winnerPlayerId=++i;
						break;
					}
					drawCircle();
                    vTaskDelay(2000);
        		}
        		break;
        	case gameOver:
        	{
        		tft.clearMemory();
        		vTaskDelay(2000);
        		str temp2="Player ";
        		temp2+=(char)('0'+winnerPlayerId);
        		temp2+=" is Winner";
        		puts(temp2.c_str());
        		showTextMessage(temp2.c_str());
        		temp2="S"+temp2;
        		audioAlert(temp2.c_str());
        		puts(temp2.c_str());
        		vTaskDelay(5000);
        		sendDataToPlayers("stop",0);

        		for(int i=0;i<playerCount;i++)
        		{
        			players[i].count=0;
        			players[i].address=0;
        		}
        		playerCount=0;
        		winnerPlayerId=0;

            	current_State=enRoll;
        	}
        		break;
        	default:
        		break;
        	}
        	return true;
        }
    private:
        State_t current_State;
        uint8_t playerCount;
        player_list players[100];
        uint8_t winnerPlayerId;
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
	/* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
	   scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));
	//scheduler_add_task(new terminalTask(PRIORITY_HIGH));
   scheduler_add_task(new Master_Task(PRIORITY_HIGH));




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

