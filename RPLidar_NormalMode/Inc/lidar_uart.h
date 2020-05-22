/*
 * lidar_uart.h
 *
 *  Created on: 10-Jun-2019
 *      Author: gautam
 */

#ifndef INC_LIDAR_UART_H_
#define INC_LIDAR_UART_H_



void MX_DMA_Init(void);
void MX_UART7_Init(void);

void lidar_uart_tx_byte(unsigned char data);
void lidar_uart_tx(unsigned char *data, unsigned char length);

unsigned char lidar_uart_rx();
void lidar_uart_rx_buffer(unsigned char *data,unsigned int length);



#endif /* INC_LIDAR_UART_H_ */
