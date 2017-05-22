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
 */

#include <stdio.h>
#include "utilities.h"
#include "io.hpp"
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "wireless/wireless.h"
#include "queue.h"
#include "string.h"
#include "LED_Display.hpp"
#include "soft_timer.hpp"


//Globals
#define CONSOL_ADDRESS 100

SemaphoreHandle_t buttonPressSemaphore = NULL;


typedef enum{
	ON = 0,
	OFF = 1
}LEDSTATUS_T;

typedef enum{
	// Remote status commands
	pushedBton,
	enrollMe,
	understood,
	// Master commands
	enroll,
	go,
	diceroll,
	stop,
}gConsol_command_t;

volatile gConsol_command_t stateOfRemote = stop;

// Message type struct
typedef struct {
	gConsol_command_t command;
	uint8_t diceCount;
} gConsole_message_t;

/// Shared queue
typedef enum {
	shared_TransmitQueueId,
	shared_ReceiveQueueId,
} sharedHandleId_t;

/**
 * Function prototypes
 */
void init_GPIO(void);
void set_LED(LEDSTATUS_T);
bool get_switch_status(void);


// Read interrupt handle task
class game_remote: public scheduler_task
{
    public:
	game_remote(uint8_t priority):scheduler_task("Game remote",1000,priority)
    {

    }

	bool init(void)
	{
		//Nothing to init.
		sevenSeg.setNumber(0);

		return true;
	}

	bool run(void *p)
	{
        /* We first get the queue handle the other task added using addSharedObject() */
		gConsole_message_t receivedMessage, sendMessage;
        QueueHandle_t recvQid = getSharedObject(shared_ReceiveQueueId);
        stateOfRemote = stop;

        //printf("game remote \n");

        /* Sleep the task forever until an item is available in the queue */
        if (xQueueReceive(recvQid, &receivedMessage, portMAX_DELAY))
        {
        	switch(receivedMessage.command){
        		case enroll:
        			printf ("Received enroll from master \n");
        			stateOfRemote = enroll;

        			if (xSemaphoreTake(buttonPressSemaphore, portMAX_DELAY)) {
        				printf("Got button press - send to Q \n");
        				sendMessage.command = enrollMe;

        				xQueueSend(getSharedObject(shared_TransmitQueueId), &sendMessage, portMAX_DELAY);
        				stateOfRemote = stop;
        			}
        			break;
        		case go:
        		{
        			// JJ: Debug prints-to be removed later
        			printf ("Received Go ahead from master \n");
        			stateOfRemote = go;

        			// Go (60) in 7 seg
        			sevenSeg.setNumber(60);

        			if (xSemaphoreTake(buttonPressSemaphore, portMAX_DELAY)) {
        				printf("Got button press - send to Q \n");
        				sendMessage.command = pushedBton;

        				xQueueSend(getSharedObject(shared_TransmitQueueId), &sendMessage, portMAX_DELAY);
        				stateOfRemote = stop;
        			}

        			sevenSeg.setNumber(88);

        			break;
        		}
        		case diceroll:
					printf ("Received dice value %c \n", receivedMessage.diceCount+'0');
					// Display the value in 7 seg
					sevenSeg.setNumber(receivedMessage.diceCount);
					sendMessage.command = understood;
					xQueueSend(getSharedObject(shared_TransmitQueueId), &sendMessage, portMAX_DELAY);
					stateOfRemote = stop;
        			break;
        		case stop:	//Fall throught to default
        			printf ("Stop received \n");
        			// Display 11 on 7 seg
					sevenSeg.setNumber(0);

					sendMessage.command = understood;
					xQueueSend(getSharedObject(shared_TransmitQueueId), &sendMessage, portMAX_DELAY);
					stateOfRemote = stop;
        			break;
        		//default:
        			// Display 11 on 7 seg
        			//sevenSeg.setNumber(11);
        	}
        }

        vTaskDelay(1000);
		return true;
	}

    private:
		LED_Display sevenSeg =  sevenSeg.getInstance();
};

// Wireless task
class remote_wireless: public scheduler_task
{
    public:
	remote_wireless(uint8_t priority) : scheduler_task("SampleWireless", 2048, priority)
        {
            /* We save the queue handle by using addSharedObject() */
            QueueHandle_t transmitQueue = xQueueCreate(1, sizeof(gConsole_message_t));
            QueueHandle_t receiveQueue = xQueueCreate(1, sizeof(gConsole_message_t));
            addSharedObject(shared_TransmitQueueId, transmitQueue);
            addSharedObject(shared_ReceiveQueueId, receiveQueue);
        }

        bool run(void *p)
        {
        	const char max_hops = 2;

        	mesh_packet_t rcvPkt, ackPkt;
        	gConsole_message_t messageToSend, messageReceived;
        	int timeout_ms = 900;
        	bool rx = false;
        	str messageString = "";

        	//printf("wireless \n");
        	// Wait for the game Master has send any message
        	//

        	while (wireless_get_rx_pkt(&rcvPkt, timeout_ms)) {
        		printf("Received data from %i\n", rcvPkt.nwk.src);
				for (int i = 0; i < rcvPkt.info.data_len; i++) {
					putchar(rcvPkt.data[i]);
				}
				printf("\n");
        		rx = true;
        	}


        	// Go in here if a packet was received from game console
        	if ((rx == true) && (CONSOL_ADDRESS == rcvPkt.nwk.src)){

                for (int i = 0; i < (rcvPkt.info.data_len-1); i++) {
                	messageString += (char)rcvPkt.data[i];
                }

                if (messageString == "dice"){
                	messageReceived.command = diceroll;
                	messageReceived.diceCount = rcvPkt.data[rcvPkt.info.data_len-1] - '0';
                }
                else{
                	messageString += (char)rcvPkt.data[rcvPkt.info.data_len-1];

                	if (messageString == "go"){
                		messageReceived.command = go;
                	}
                	else if (messageString == "stop"){
                		messageReceived.command = stop;
                	}
                	else if (messageString == "enroll"){
                		messageReceived.command = enroll;
                	}
                	else{
                		printf("Invalid command from console\n");
                		return true;
                	}
                }

                // Send the received command to the handling task
        		xQueueSend(getSharedObject(shared_ReceiveQueueId), &messageReceived, portMAX_DELAY);


        		// Wait for the response from the handling task
				if (xQueueReceive(getSharedObject(shared_TransmitQueueId), &messageToSend, portMAX_DELAY)){
					// ******* Form and add the send packet
					if (messageToSend.command == pushedBton){
						if (wireless_send(CONSOL_ADDRESS, mesh_pkt_ack, "pushed", strlen("pushed"), max_hops)){
							printf(" Packet send \n");

							if(wireless_get_ack_pkt(&ackPkt, 100) && CONSOL_ADDRESS == ackPkt.nwk.src){
								printf(" Received ACK \n");
							}
							wireless_flush_rx();
						}
						else{
							printf(" Something wrong - Packet not send !!! \n");
						}
					}
					else if (messageToSend.command == enrollMe){
						if (wireless_send(CONSOL_ADDRESS, mesh_pkt_ack, "enrollMe", strlen("enrollMe"), max_hops)){
							printf(" Packet send \n");

							if(wireless_get_ack_pkt(&ackPkt, 100) && CONSOL_ADDRESS == ackPkt.nwk.src){
								printf(" Received ACK \n");
							}
							wireless_flush_rx();
						}
						else{
							printf(" Something wrong - Packet not send !!! \n");
						}
					}
				}
			}
            vTaskDelay(1000);
            return true;
        }
    private:
};


// Wireless task
class handle_button : public scheduler_task
{
    public:
		handle_button(uint8_t priority) : scheduler_task("Button Handle", 2048, priority)
        {

        }

		bool init(void)
		{
			/* Initialize switches */
			init_GPIO();

			return true;
		}

        bool run(void *p)
        {
        	if (get_switch_status()){
        		// Port 0 interrupt triggered
        		if ((stateOfRemote == go) || (stateOfRemote == enroll)){
        			xSemaphoreGive(buttonPressSemaphore);
        		}
        		set_LED(ON);
        	}
        	else{
        		set_LED(OFF);
        	}

            vTaskDelay(100);
            return true;
        }
    private:

};




int main(void)
{
	buttonPressSemaphore = xSemaphoreCreateBinary();

	scheduler_add_task(new wirelessTask(PRIORITY_HIGH));
	scheduler_add_task(new terminalTask(PRIORITY_LOW));
	scheduler_add_task(new handle_button(PRIORITY_MEDIUM));
	scheduler_add_task(new remote_wireless(PRIORITY_MEDIUM));
	scheduler_add_task(new game_remote(PRIORITY_MEDIUM));

    scheduler_start(); ///< This shouldn't return

    return -1;
}

/**
 *  GPIO APIs
 */

void init_GPIO(void)
{
	/* Make direction of PORT1.15 (Switch 3) as input */
	LPC_GPIO1->FIODIR &= ~(1 << 15);

	/* Make direction of PORT1.8 (LED 3) as OUTPUT */
	LPC_GPIO1->FIODIR |= (1 << 8);
}

bool get_switch_status(void)
{
	/* Read the 32-bit FIOPIN registers, which corresponds to
	 * 32 physical pins of PORT1.  We use AND logic to test if JUST the
	 * pin number 9 is set. Returns true(1) if high, False(0) if low.
	 */
	return (LPC_GPIO1->FIOPIN & (1 << 15));
}

void set_LED(LEDSTATUS_T status)
{
	if (status == ON){
		/* Likewise, reset to 0 */
		LPC_GPIO1->FIOCLR = (1 << 8);
	}
	else{
		/* Faster, better way to set bit 3 (no OR logic needed) */
		LPC_GPIO1->FIOSET = (1 << 8);
	}
}
