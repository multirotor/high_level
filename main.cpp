#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // for C++ usleep
#include "lidarLite.h"
#include "360lazer.h"
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;

    // lidarLite
    int fd, res, i, del;
    unsigned char st, ver;

    bool rpLidar = true;

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);


    if (!opt_com_path) {
        opt_com_path = "/dev/ttyUSB0";
    }

    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }


    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        //goto on_finished; // uncommented this part so we can at least continue
	rpLidar = false;
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        //goto on_finished; //uncommented this part so we can continue
        rpLidar = false;
    }

    if (rpLidar)
    {
        // start scan...
        drv->startScan();
    }

    // Main loop
    while (1) {

        // lidarLite
	fd = lidar_init(false);

        if (fd == -1) {
            printf("LIDARLITE: Reading error\n");
        }
        else {
            for (i=0;i<10;i++) {
                res = lidar_read(fd);
                st = lidar_status(fd);

                //ver = lidar_version(fd);

                printf("%3.0d cm \n", res);
                //lidar_status_print(st);

                //delay(del);
                }
        }

        // RPLidar
	if (rpLidar)
	{
            rplidar_response_measurement_node_t nodes[360*2];
            size_t   count = _countof(nodes);
            op_result = drv->grabScanData(nodes, count);

            if (IS_OK(op_result)) {

                drv->ascendScanData(nodes, count);

                for (int pos = 0; pos < (int)count ; ++pos) {
                    fprintf(stderr, "%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                        (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
                        (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                        nodes[pos].distance_q2/4.0f,
                        nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                }
            }
        }
	else
	{
		printf("360LAZER: Reading error..");
	}
    }
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}
