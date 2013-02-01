/**
 *
 * \file
 *
 * \brief FreeRTOS USART driver echo test tasks
 *
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Atmel library includes. */
#include "freertos_uart_serial.h"

/* Demo includes. */
#include "demo-tasks.h"
#include <gpio.h>
#include "sam3n_ek.h"

#ifndef	RX_BUFFER_SIZE
#define RX_BUFFER_SIZE          (79)
#endif

/* The buffer provided to the USART driver to store incoming character in. */
#ifdef confINCLUDE_USART_ECHO_TASKS
static uint8_t receive_buffer[RX_BUFFER_SIZE] = {0};
#endif

#ifdef confINCLUDE_USART_UART_TUNNEL
static uint8_t usart_receive_buffer[RX_BUFFER_SIZE] = {0};
//static uint8_t uart_receive_buffer[RX_BUFFER_SIZE] = {0};
static xQueueHandle sim_pwr_commands_queue;
#endif

#if (defined confINCLUDE_USART_ECHO_TASKS) || (defined confINCLUDE_USART_UART_TUNNEL)

/* The size of the buffer used to receive characters from the USART driver.
 * This equals the length of the longest string used in this file. */
#define RX_BUFFER_SIZE          (79)

/* The baud rate to use. */
#define USART_BAUD_RATE         (115200)
/*-----------------------------------------------------------*/

#if defined confINCLUDE_USART_ECHO_TASKS
/*
 * Tasks used to develop the USART drivers.  One task sends out a series of
 * strings, the other task expects to receive the same series of strings.  An
 * error is latched if any characters are missing.  A loopback connector is
 * required to ensure the transmitted characters are also received.
 */
static void usart_echo_tx_task(void *pvParameters);
static void usart_echo_rx_task(void *pvParameters);
#endif


#if defined confINCLUDE_USART_UART_TUNNEL
/*
 * Tasks used to develop the USART drivers.  One task sends out a series of
 * strings, the other task expects to receive the same series of strings.  An
 * error is latched if any characters are missing.  A loopback connector is
 * required to ensure the transmitted characters are also received.
 */
static void usart_tunnel_rx_task(void *pvParameters);
static void uart_tunnel_rx_task(void *pvParameters);
static void turn_on_sim_task(void *pvParameters);

/* Counts the number of times the Rx task receives a string.  The count is used
to ensure the task is still executing. */
static uint32_t usart_rx_task_loops = 0UL;
static uint32_t uart_rx_task_loops = 0UL;

#endif
/*-----------------------------------------------------------*/

#if defined confINCLUDE_USART_ECHO_TASKS
/* Counts the number of times the Rx task receives a string.  The count is used
to ensure the task is still executing. */
static uint32_t rx_task_loops = 0UL;

/* The array of strings that are sent by the Tx task, and therefore received by
the Rx task. */
const uint8_t *echo_strings[] =
{
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "a",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "b",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "c",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "d",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "e",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "f",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "g",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "h",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "i",
	(uint8_t *) "j",
	(uint8_t *) "k",
	(uint8_t *) "l",
	(uint8_t *) "m",
	(uint8_t *) "n",
	(uint8_t *) "o",
	(uint8_t *) "p",
	(uint8_t *) "q",
	(uint8_t *) "r",
	(uint8_t *) "s",
	(uint8_t *) "t",
	(uint8_t *) "u",
	(uint8_t *) "v",
	(uint8_t *) "w",
	(uint8_t *) "x",
	(uint8_t *) "w",
	(uint8_t *) "z",
	(uint8_t *) "ab",
	(uint8_t *) "abc",
	(uint8_t *) "bcde",
	(uint8_t *) "bcdef",
	(uint8_t *) "bcdefg",
	(uint8_t *) "bcdefgh",
	(uint8_t *) "bcdefghi",
	(uint8_t *) "bcdefghij",
	(uint8_t *) "bcdefghijk",
	(uint8_t *) "bcdefghijkl",
	(uint8_t *) "bcdefghijklm",
	(uint8_t *) "bcdefghijklmn",
	(uint8_t *) "bcdefghijklmno",
	(uint8_t *) "bcdefghijklmnop",
	(uint8_t *) "bcdefghijklmnopq",
	(uint8_t *) "bcdefghijklmnopqr",
	(uint8_t *) "bcdefghijklmnopqrs",
	(uint8_t *) "bcdefghijklmnopqrst",
	(uint8_t *) "bcdefghijklmnopqrstu",
	(uint8_t *) "bcdefghijklmnopqrstuv",
	(uint8_t *) "bcdefghijklmnopqrstuvw",
	(uint8_t *) "bcdefghijklmnopqrstuvwx",
	(uint8_t *) "bcdefghijklmnopqrstuvwxy",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz12",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz123",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz12345",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz123456",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz12345678",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz123456789",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890A",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890AB",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABC",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCD",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDE",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEF",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFG",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGH",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHI",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJ",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJK",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKL",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLM",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMN",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNO",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOP",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQ",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQR",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRS",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRST",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTU",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUV",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVW",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWX",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXW",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ12",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ123",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ12345",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ123456",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ12345678",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ123456789",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[]",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=",
	(uint8_t *) "bcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "_cdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "__defghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "___efghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "____fghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
	(uint8_t *) "_____ghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXWZ1234567890[];'#=+",
};
#endif
#endif

#if defined confINCLUDE_USART_ECHO_TASKS
/*-----------------------------------------------------------*/
void create_usart_echo_test_tasks(Usart *usart_base,
		uint16_t stack_depth_words,
		unsigned portBASE_TYPE task_priority)
{
	freertos_usart_if freertos_usart;
	freertos_peripheral_options_t driver_options = {
		receive_buffer,								/* The buffer used internally by the USART driver to store incoming characters. */
		RX_BUFFER_SIZE,									/* The size of the buffer provided to the USART driver to store incoming characters. */
		configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,	/* The priority used by the USART interrupts. */
		USART_RS232,									/* Configure the USART for RS232 operation. */
		(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX)
	};

	const sam_usart_opt_t usart_settings = {
		USART_BAUD_RATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		0 /* Only used in IrDA mode. */
	}; /*_RB_ TODO This is not SAM specific, not a good thing. */

	/* Initialise the USART interface. */
	freertos_usart = freertos_usart_serial_init(usart_base,
			&usart_settings,
			&driver_options);
	configASSERT(freertos_usart);

	/* Create the two tasks as described above. */
	xTaskCreate(usart_echo_tx_task, (const signed char *const) "Tx",
			stack_depth_words, (void *) freertos_usart,
			task_priority, NULL);
	xTaskCreate(usart_echo_rx_task, (const signed char *const) "Rx",
			stack_depth_words, (void *) freertos_usart,
			task_priority + 1, NULL);
}
#endif

#if defined confINCLUDE_USART_UART_TUNNEL
/*-----------------------------------------------------------*/
void create_usart_uart_tunnel_tasks(Usart *usart_base,
		uint16_t usart_stack_depth_words,
		uint16_t uart_stack_depth_words,
		unsigned portBASE_TYPE task_priority)
{
	freertos_usart_if myUsart;
	/* Initialise the USART interface. */
	freertos_peripheral_options_t usart_driver_options = {
		usart_receive_buffer,							/* The buffer used internally by the USART driver to store incoming characters. */
		RX_BUFFER_SIZE,									/* The size of the buffer provided to the USART driver to store incoming characters. */
		configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,	/* The priority used by the USART interrupts. */
		USART_RS232,									/* Configure the USART for RS232 operation. */
		(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX)
	};
	const sam_usart_opt_t usart_settings = {
		USART_BAUD_RATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		0 /* Only used in IrDA mode. */
	}; ///*_RB_ TODO This is not SAM specific, not a good thing. */
	myUsart = freertos_usart_serial_init(usart_base,&usart_settings, &usart_driver_options);
	configASSERT(myUsart);
	
	sim_pwr_commands_queue = xQueueCreate(MAX_PWR_COMMANDS,COMMAND_SIZE);
	
	/* Success: Create the two tasks as described above. */
	/*xTaskCreate(uart_tunnel_tx_task, (const signed char *const) "UartTx",
	uart_stack_depth_words, (void *) myUart,
	task_priority, NULL);*/
	xTaskCreate(uart_tunnel_rx_task,					/* One of the tasks that implement the tunnel. */
				(const signed char *const) "UartRx",	/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
				uart_stack_depth_words,					/* The size of the stack allocated to the task. */
				(void *) myUsart,						/* The parameter is used to pass the already configured UART port into the task. */
				task_priority + 2,						/* The priority allocated to the task. */
				NULL);									/* Used to store the handle to the created task - in this case the handle is not required. */
	/*xTaskCreate(usart_tunnel_tx_task, (const signed char *const) "UsartTx",
	usart_stack_depth_words, (void *) freertos_usart,
	task_priority, NULL);*/
	xTaskCreate(usart_tunnel_rx_task,					/* One of the tasks that implement the tunnel. */
				(const signed char *const) "UsartRx",	/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
				usart_stack_depth_words,				/* The size of the stack allocated to the task. */
				(void *) myUsart,						/* The parameter is used to pass the already configured USART+UART ports into the task. */
				task_priority + 2,						/* The priority allocated to the task. */
				NULL);									/* Used to store the handle to the created task - in this case the handle is not required. */
	/* Task used to power on and power off Sim900 GPRS module */
	xTaskCreate(turn_on_sim_task,							/* One of the tasks that implement the tunnel. */
				(const signed char *const) "SimPowerOn",	/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
				configMINIMAL_STACK_SIZE,					/* The size of the stack allocated to the task. */
				(void *) &sim_pwr_commands_queue,			/* The parameter is used to pass the already configured USART+UART ports into the task. */
				tskIDLE_PRIORITY+2,							/* The priority allocated to the task. */
				NULL);										/* Used to store the handle to the created task - in this case the handle is not required. */
}
#endif

#if defined confINCLUDE_USART_ECHO_TASKS
/*-----------------------------------------------------------*/
static void usart_echo_tx_task(void *pvParameters)
{
	freertos_usart_if usart_port;
	static uint8_t local_buffer[RX_BUFFER_SIZE];
	const portTickType time_out_definition = (100UL / portTICK_RATE_MS),
			short_delay = (10UL / portTICK_RATE_MS);
	xSemaphoreHandle notification_semaphore;
	unsigned portBASE_TYPE string_index;
	status_code_t returned_status;

	/* Check the strings being sent fit in the buffers provided. */
	for(string_index = 0; string_index < sizeof(echo_strings) / sizeof(uint8_t *); string_index++)
	{
		configASSERT(strlen((char *) echo_strings[string_index]) <= RX_BUFFER_SIZE);
	}

	/* The (already open) USART port is passed in as the task parameter. */
	usart_port = (freertos_usart_if)pvParameters;

	/* Create the semaphore to be used to get notified of end of
	transmissions. */
	vSemaphoreCreateBinary(notification_semaphore);
	configASSERT(notification_semaphore);

	/* Start with the semaphore in the expected state - no data has been sent
	yet.  A block time of zero is used as the semaphore is guaranteed to be
	there as it has only just been created. */
	xSemaphoreTake(notification_semaphore, 0);

	string_index = 0;

	for (;;) {
		/* Data cannot be sent from Flash, so copy the string to RAM. */
		strcpy((char *) local_buffer,
				(const char *) echo_strings[string_index]);

		/* Start send. */
		returned_status = freertos_usart_write_packet_async(usart_port,
				local_buffer, strlen((char *) local_buffer),
				time_out_definition, notification_semaphore);
		configASSERT(returned_status == STATUS_OK);

		/* The async version of the write function is being used, so wait for
		the end of the transmission.  No CPU time is used while waiting for the
		semaphore.*/
		xSemaphoreTake(notification_semaphore, time_out_definition * 2);
		vTaskDelay(short_delay);

		/* Send the next string next time around. */
		string_index++;
		if (string_index >= (sizeof(echo_strings) / sizeof(uint8_t *))) {
			string_index = 0;
		}
	}
}
#endif

#if defined confINCLUDE_USART_ECHO_TASKS
/*-----------------------------------------------------------*/
static void usart_echo_rx_task(void *pvParameters)
{
	freertos_usart_if usart_port;
	static uint8_t rx_buffer[RX_BUFFER_SIZE];
	uint32_t received;
	unsigned portBASE_TYPE string_index;

	/* The (already open) USART port is passed in as the task parameter. */
	usart_port = (freertos_usart_if)pvParameters;

	string_index = 0;

	for (;;) {
		memset(rx_buffer, 0x00, sizeof(rx_buffer));

		received = freertos_usart_serial_read_packet(usart_port, rx_buffer,
				strlen((const char *) echo_strings[string_index]),
				portMAX_DELAY);

		/* Ensure the string received is that expected. */
		configASSERT(received == strlen((const char *) echo_strings[string_index]));
		configASSERT(strcmp((const char *) rx_buffer, (const char *) echo_strings[string_index]) == 0);

		/* Increment a loop counter as an indication that this task is still
		actually receiving strings. */
		rx_task_loops++;

		/* Expect the next string the next time around. */
		string_index++;
		if (string_index >= (sizeof(echo_strings) / sizeof(uint8_t *))) {
			string_index = 0;
		}
	}
}
#endif

#if defined confINCLUDE_USART_UART_TUNNEL
/*-----------------------------------------------------------*/
static void usart_tunnel_rx_task(void *pvParameters)
{
	freertos_usart_if		usart_port;
	static uint8_t			rx_buffer[RX_BUFFER_SIZE],i;
	uint32_t				received;
	const portTickType		time_out_definition = (100UL / portTICK_RATE_MS),
							short_delay = (10UL / portTICK_RATE_MS);

	/* The (already open) USART port is passed in as the task parameter. */
	usart_port = (freertos_usart_if)pvParameters;
	
	/* Clear memory buffer */
	memset(rx_buffer, 0x00, sizeof(rx_buffer));
	
	for (;;) {		
		//Recibe en la USART y forwardea por la UART
		received = freertos_usart_serial_read_packet(usart_port, rx_buffer,RX_BUFFER_SIZE,short_delay);
		if(received!=0)
		{
			/* Start send. */
			for (i=0;(i<received)&&(i<RX_BUFFER_SIZE);i++)
			{
				putchar(rx_buffer[i]);
			}
		} else {
			vTaskDelay(short_delay);
		}

		/* Increment a loop counter as an indication that this task is still
		actually receiving strings. */
		usart_rx_task_loops++;
	}
}
#endif

#if defined confINCLUDE_USART_UART_TUNNEL
/*-----------------------------------------------------------*/
static void uart_tunnel_rx_task(void *pvParameters)
{
	freertos_usart_if		usart_port;
	static uint8_t			rx_buffer[RX_BUFFER_SIZE],i=0;
	static uint8_t			rx_char;
	uint32_t				pwr_command;
	const portTickType		time_out_definition = (100UL / portTICK_RATE_MS),
							short_delay = (10UL / portTICK_RATE_MS);
	xSemaphoreHandle		notification_semaphore;
	status_code_t returned_status;
	
	/* The (already open) USART port is passed in as the task parameter. */
	usart_port = (freertos_usart_if *) pvParameters;

	/* Create the semaphore to be used to get notified of end of
	transmissions. */
	vSemaphoreCreateBinary(notification_semaphore);
	configASSERT(notification_semaphore);
	/* Clear memory buffer */
	memset(rx_buffer, 0x00, sizeof(rx_buffer));
	
	/* Start with the semaphore in the expected state - no data has been sent
	yet.  A block time of zero is used as the semaphore is guaranteed to be
	there as it has only just been created. */
	xSemaphoreTake(notification_semaphore, 0);
	
	for (;;) {
		//Recibe en la UART y forwardea por la USART
		if((rx_char=getchar())!=0)
		{
			rx_buffer[i]=rx_char;
			i=i+1;
			if(rx_char==10)										//Newline character received, therefore, must send!
			{
				if(rx_buffer[0]==COMMAND_HEADER) {
					//Terminal command received
					if((rx_buffer[1]=='O')&&(rx_buffer[2]=='N')) {
						pwr_command = ON_COMMAND;									//Pasar comando en queue
						xQueueSend(sim_pwr_commands_queue,&pwr_command,time_out_definition);
					} else if((rx_buffer[1]=='R')&&(rx_buffer[2]=='E')&&(rx_buffer[3]=='S')) {
						pwr_command = RES_COMMAND;									//Pasar comando en queue
						xQueueSend(sim_pwr_commands_queue,&pwr_command,time_out_definition);
					}
					i=0;
				} else {
					//Text received
					returned_status = freertos_usart_write_packet_async(usart_port,						/* Start send. */
					rx_buffer, i,time_out_definition, notification_semaphore);
					configASSERT(returned_status == STATUS_OK);
					/* The async version of the write function is being used, so wait for
					the end of the transmission.  No CPU time is used while waiting for the
					semaphore.*/
					xSemaphoreTake(notification_semaphore, time_out_definition * 2);
					i=0;
				}				
			}
		}
		
		/* Increment a loop counter as an indication that this task is still
		actually receiving strings. */
		uart_rx_task_loops++;
	}
}
#endif

/*-----------------------------------------------------------*/

#ifdef confINCLUDE_USART_ECHO_TASKS
portBASE_TYPE are_usart_echo_tasks_still_running(void)
{
	static uint32_t last_loop_count = 0;
	portBASE_TYPE return_value = pdPASS;

	/* Ensure the count of Rx loops is still incrementing. */
	if (last_loop_count == rx_task_loops) {
		/* The Rx task has somehow stalled, set the error LED. */
		return_value = pdFAIL;
	}

	last_loop_count = rx_task_loops;

	return return_value;
}
#endif

/*-----------------------------------------------------------*/

#ifdef confINCLUDE_USART_UART_TUNNEL
void turn_on_sim_task(void *pvParameters)
{
	uint32_t command;
	const portTickType		time_out_definition = (100UL / portTICK_RATE_MS),
							short_delay = (10UL / portTICK_RATE_MS);
	
	for(;;) {
		if(xQueueReceive(sim_pwr_commands_queue,&command,short_delay)==pdTRUE) {
			if(command==ON_COMMAND) {
				putchar('O'); putchar('n'); putchar('S'); putchar('i'); putchar('m'); putchar(13); putchar(10);	//OnSim message
				#if SIM_PWR_IDLE_LEVEL==0
					gpio_set_pin_high(SIM_PWR_GPIO);							//set pin high
				#else
					gpio_set_pin_low(SIM_PWR_GPIO);								//set pin low
					#error La ShangriBoard invierte la logica
				#endif
				vTaskDelay(SIM_PWR_SEQUENCE);									//corresponding bit bang time		
				#if SIM_PWR_IDLE_LEVEL==0
					gpio_set_pin_low(SIM_PWR_GPIO);								//set pin low
				#else
					gpio_set_pin_high(SIM_PWR_GPIO);							//set pin high
					#error La ShangriBoard invierte la logica
				#endif
			} else if (command==RES_COMMAND)
			{
				putchar('R'); putchar('e'); putchar('s'); putchar('S'); putchar('i'); putchar('m'); putchar(13); putchar(10);	//ResSim message
				#if SIM_NRST_IDLE_LEVEL==0
					gpio_set_pin_high(SIM_NRST_GPIO);							//set pin high
				#else
					gpio_set_pin_low(SIM_NRST_GPIO);							//set pin low
				#error La ShangriBoard invierte la logica
				#endif
				vTaskDelay(SIM_RES_SEQUENCE);									//corresponding bit bang time
				#if SIM_NRST_IDLE_LEVEL==0
					gpio_set_pin_low(SIM_NRST_GPIO);							//set pin low
				#else
					gpio_set_pin_high(SIM_NRST_GPIO);							//set pin high
				#error La ShangriBoard invierte la logica
				#endif
			}
		}
		else {
			vTaskDelay(time_out_definition);
		}
	}
}
#endif

/*-----------------------------------------------------------*/

#ifdef confINCLUDE_USART_UART_TUNNEL
portBASE_TYPE are_tunnel_tasks_still_running(void)
{
	portBASE_TYPE return_value = pdPASS;
	static uint32_t usart_rx_last_loop_count = 0;
	static uint32_t uart_rx_last_loop_count = 0;

	/* Ensure the count of Rx loops is still incrementing. */
	if (usart_rx_last_loop_count == usart_rx_task_loops || uart_rx_last_loop_count == uart_rx_task_loops) {
		/* The Rx task has somehow stalled, set the error LED. */
		return_value = pdFAIL;
	}

	usart_rx_last_loop_count = usart_rx_task_loops;
	uart_rx_last_loop_count = uart_rx_task_loops;

	return return_value;
}
#endif

/*-----------------------------------------------------------*/