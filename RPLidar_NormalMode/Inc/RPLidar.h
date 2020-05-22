
#pragma once

#include "rptypes.h"
#include "rplidar_cmd.h"
#include "stm32mp1xx_hal.h"

#define RPLIDAR_DEFAULT_TIMEOUT  2000

typedef unsigned char boolean_t;
boolean_t _isScanning;

rplidar_response_measurement_node_hq_t   _cached_scan_node_hq_buf[400];
unsigned int                             _cached_scan_node_hq_count;

typedef struct
{
    float distance;
    float angle;
    uint8_t quality;
    boolean_t  startBit;
}RPLidarMeasurement;



    // open the given serial interface and try to connect to the RPLIDAR
	boolean_t begin_DMA();

    // ask the RPLIDAR for its health info
    u_result getHealth(rplidar_response_device_health_t * healthinfo, _u32 timeout ); //RPLIDAR_DEFAULT_TIMEOUT
    
    // ask the RPLIDAR for its device info like the serial number
    u_result getDeviceInfo(rplidar_response_device_info_t * info, _u32 timeout ); //RPLIDAR_DEFAULT_TIMEOUT

    // stop the measurement operation
    u_result stop();

    // start the measurement operation
    u_result startScan(boolean_t force, _u32 timeout); //force =0, RPLIDAR_DEFAULT_TIMEOUT*2

    // wait for one sample point to arrive
    u_result waitPoint(RPLidarMeasurement *_currentMeasurement, _u32 timeout);  //RPLIDAR_DEFAULT_TIMEOUT
    
    u_result waitNode(rplidar_response_measurement_node_t * node, _u32 timeout);

    u_result waitScanData(rplidar_response_measurement_node_t * nodebuffer, unsigned int * count, _u32 timeout);
    // retrieve currently received sample point
    
    u_result sendCommand(_u8 cmd, const void * payload, unsigned int payloadsize);

    u_result waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout);

    void convert(const rplidar_response_measurement_node_t *from, rplidar_response_measurement_node_hq_t *to);

    u_result cacheScanData();

    void ReadBuffer(uint8_t *data,size_t length);





