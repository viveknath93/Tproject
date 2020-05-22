
#include "RPLidar.h"
#include "lidar_uart.h"
#include "string.h"
#include "stdio.h"
#include "stm32mp1xx_hal_uart.h"

#define false 0
#define true 1

boolean_t _bined_serialdev= false;

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;

#define DMA_RX_BUFFER_SIZE          2000//4000
 uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE] ={0};

/*#define UART_BUFFER_SIZE            1500
uint8_t UART_Buffer[UART_BUFFER_SIZE];*/


size_t Write;
volatile uint32_t Read=1;
size_t len, tocopy;
uint8_t* ptr;
uint8_t flag = 0;
volatile uint8_t dma_flag =0;

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart){

	//if(huart->RxState == HAL_UART_STATE_READY){
	dma_flag =1;
	//}
}

void HAL_UART_RxHalfCpltCallback (UART_HandleTypeDef *huart){

	dma_flag =1;
}


/*
 * Read the DMA buffer and fill the raw data
 * @param data OUT
 * @param length IN */
void ReadBuffer(uint8_t *data,size_t length)
{
size_t lengthcpy=0;

    lengthcpy = DMA_RX_BUFFER_SIZE - Read;

    if(length>10)
    {
    	length = 5;
    }

    if(length >lengthcpy ){
		memcpy(data, &DMA_RX_Buffer[Read], lengthcpy);
		Read =0;
		length =length - lengthcpy;
		memcpy((data+lengthcpy), &DMA_RX_Buffer[Read], length);
		Read = Read +length;
    }
    else
    {
    	memcpy(data, &DMA_RX_Buffer[Read], length);
    	Read = Read + length;
    }


}

// open the given serial interface and try to connect to the RPLIDAR
boolean_t begin_DMA()
{


	HAL_UART_Receive_DMA (&huart7, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE);


	return true;

}



// ask the RPLIDAR for its health info
u_result getHealth(rplidar_response_device_health_t *healthinfo, _u32 timeout)
{
    _u32 currentTs = HAL_GetTick();
    _u32 remainingtime;
  
    _u8 *infobuf = (_u8 *)healthinfo;
    _u8 recvPos = 0;

    rplidar_ans_header_t response_header;
    u_result  ans;

        if (IS_FAIL(ans = sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0))) {
            return ans;
        }

        if (IS_FAIL(ans = waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) {
            return RESULT_INVALID_DATA;
        }

        if ((response_header.size) < sizeof(rplidar_response_device_health_t)) {
            return RESULT_INVALID_DATA;
        }
        
        while ((remainingtime=HAL_GetTick() - currentTs) <= timeout) {
        	   int currentbyte = lidar_uart_rx();
          //  ReadBuffer(&currentbyte,1);
            if (currentbyte < 0) continue;
            
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_health_t)) {
                return RESULT_OK;
            }
        }

    return RESULT_OPERATION_TIMEOUT;
}

// ask the RPLIDAR for its device info like the serial number
u_result getDeviceInfo(rplidar_response_device_info_t *info, _u32 timeout )
{
    _u8  recvPos = 0;
  //  _u32 currentTs = HAL_GetTick();
   // _u32 remainingtime;
    _u8 *infobuf = (_u8*)info;
    rplidar_ans_header_t response_header;
    memset(&response_header,0,sizeof(rplidar_ans_header_t));
    u_result  ans;

    {
        if (IS_FAIL(ans = sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO,NULL,0))) {
            return ans;
        }

        if (IS_FAIL(ans = waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
            return RESULT_INVALID_DATA;
        }

        if (response_header.size < sizeof(rplidar_response_device_info_t)) {
            return RESULT_INVALID_DATA;
        }

        while (1) {
        	   int currentbyte = lidar_uart_rx();
           // ReadBuffer(&currentbyte,1);
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_info_t)) {
                return RESULT_OK;
            }
        }
    }
    
    return RESULT_OPERATION_TIMEOUT;
}

// stop the measurement operation
u_result stop()
{

    u_result ans = sendCommand(RPLIDAR_CMD_STOP,NULL,0);
    return ans;
}

/*
 * start the measurement operation
 * @param force IN
 * @param timeout value IN*/
u_result startScan(boolean_t force, _u32 timeout)
{
    u_result ans;

    

    stop(); //force the previous operation to stop

    {
        ans = sendCommand(force?RPLIDAR_CMD_FORCE_SCAN:RPLIDAR_CMD_SCAN, NULL, 0);
        if (IS_FAIL(ans)) return ans;

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

       begin_DMA();

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT) {
            return RESULT_INVALID_DATA;
        }

        if (response_header.size < sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_INVALID_DATA;
        }
    }
    return RESULT_OK;
}

/*
 * fill a complete node until from the raw data
 * @param node OUT
 * @param timeout IN
 * @retval result of execution
 * */
u_result waitNode(rplidar_response_measurement_node_t * node, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs =  HAL_GetTick();
    _u8  recvBuffer[sizeof(rplidar_response_measurement_node_t)];
    _u8 *nodeBuffer = (_u8*)node;
    _u32 waitTime;

   while ((waitTime= HAL_GetTick() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_response_measurement_node_t) - recvPos;

        ReadBuffer(recvBuffer, remainSize);

        for (size_t pos = 0; pos < remainSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0: // expect the sync bit and its reverse in this byte
                {
                    _u8 tmp = (currentByte>>1);
                    if ( (tmp ^ currentByte) & 0x1 ) {
                        // pass
                    } else {
                        continue;
                    }

                }
                break;
            case 1: // expect the highest bit to be 1
                {
                    if (currentByte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                        // pass
                    } else {
                        recvPos = 0;
                        continue;
                    }
                }
                break;
            }
            nodeBuffer[recvPos++] = currentByte;

            if (recvPos == sizeof(rplidar_response_measurement_node_t)) {

            	return RESULT_OK;
            }
        }
    }

    return RESULT_OPERATION_TIMEOUT;
}
/*
 * wait and fill the scan data of each node one by one
 * @param OUT nodebuffer
 * @param INOUT count
 * @param timeout IN
 * @retval result of execution
 * */
u_result waitScanData(rplidar_response_measurement_node_t *nodebuffer, size_t *count, _u32 timeout)
{
    size_t   recvNodeCount =  0;
    _u32     startTs = HAL_GetTick();
    _u32     waitTime;
    u_result ans;

    while ((waitTime = HAL_GetTick() - startTs) <= timeout && recvNodeCount < *count) {
        rplidar_response_measurement_node_t node;
        if (IS_FAIL(ans = waitNode(&node, timeout - waitTime))) {
            return ans;
        }

        nodebuffer[recvNodeCount++] = node;

        if (recvNodeCount == *count) return RESULT_OK;
    }
    *count = recvNodeCount;
    return RESULT_OPERATION_TIMEOUT;
}

/*
 * Driver function to send the command
 * @param IN cmd command with out payload and response based on force value
 * @param IN payload data
 * @param IN payload size
 * */
u_result sendCommand(_u8 cmd, const void * payload, size_t payloadsize)
{

    rplidar_cmd_packet_t pkt_header;
    rplidar_cmd_packet_t * header = &pkt_header;
    _u8 checksum = 0;

    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    // send header first
    lidar_uart_tx( (unsigned char *)header, 2);

    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // calc checksum
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((_u8 *)payload)[pos];
        }

        // send size
        _u8 sizebyte = payloadsize;
        lidar_uart_tx((uint8_t *)&sizebyte, 1);

        // send payload
        lidar_uart_tx((uint8_t *)&payload, sizebyte);

        // send checksum
        lidar_uart_tx((uint8_t *)&checksum, 1);

    }

    return RESULT_OK;
}

/*
 * Gets the response header
 * @param OUT header
 * @param IN timeout value
 * @retval result of execution
 * */
u_result waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout)
{
    _u8  recvPos = 0;
    _u32 currentTs =HAL_GetTick();
    _u32 remainingtime;
    _u8 *headerbuf = (_u8*)header;
    while ((remainingtime=HAL_GetTick() - currentTs) <= timeout) {
        
    	int currentbyte = lidar_uart_rx();
    	// ReadBuffer(&currentbyte,1);
        if (currentbyte<0) continue;
        switch (recvPos) {
        case 0:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE1) {
                continue;
            }
            break;
        case 1:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE2) {
                recvPos = 0;
                continue;
            }
            break;
        }
        headerbuf[recvPos++] = currentbyte;

        if (recvPos == sizeof(rplidar_ans_header_t)) {
            return RESULT_OK;
        }
  

    }

    return RESULT_OPERATION_TIMEOUT;
}
/*
 * wait until DMA is half full then start reading the buffer
 * check for correctness of data
 * @retval reslut of exection
 * */
u_result cacheScanData()
{
	size_t                                   count = 128;
    rplidar_response_measurement_node_t      local_buf[count];
    rplidar_response_measurement_node_hq_t   local_scan[400];
    size_t                                   scan_count = 0;
    u_result                                 ans;
    memset(local_scan, 0, sizeof(local_scan));

    while((DMA_RX_Buffer[1000] == 0) && (DMA_RX_Buffer[1001] == 0) && (DMA_RX_Buffer[1002] == 0) && (DMA_RX_Buffer[1003] == 0)){

    }

    while(1)
    {
        if (IS_FAIL(ans=waitScanData(local_buf, &count, RPLIDAR_DEFAULT_TIMEOUT))) {
            if (ans != RESULT_OPERATION_TIMEOUT) {
                return RESULT_OPERATION_FAIL;
            }
        }

        for (size_t pos = 0; pos < count; ++pos)
        {
            if (local_buf[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)
            {
                // only publish the data when it contains a full 360 degree scan

                if ((local_scan[0].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)) {

                    memcpy(_cached_scan_node_hq_buf, local_scan, (scan_count*sizeof(rplidar_response_measurement_node_hq_t)));
                    _cached_scan_node_hq_count = scan_count;

                    //Returning when the scan data is full circle
                    if(scan_count==360)
                    {
                    	return RESULT_OK;
                    }

                    memset(local_scan, 0, sizeof(local_scan));
                }
                scan_count = 0;
            }

            rplidar_response_measurement_node_hq_t nodeHq;
            convert(&local_buf[pos], &nodeHq);
            local_scan[scan_count++] = nodeHq;

        }



    }

    return RESULT_OK;
}

/*
 * convert the sensor data to specific format */
void convert(const rplidar_response_measurement_node_t *from, rplidar_response_measurement_node_hq_t *to)
{
    to->angle_z_q14 = (((from->angle_q6_checkbit) >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /*<< 8*/) /*/ 90*/;  //transfer to q14 Z-angle
    to->dist_mm_q2 = from->distance_q2;
    to->flag = (from->sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);  // trasfer syncbit to HQ flag field
    to->quality = (from->sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) /*<< RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT*/;  //remove the last two bits and then make quality from 0-63 to 0-255
}

