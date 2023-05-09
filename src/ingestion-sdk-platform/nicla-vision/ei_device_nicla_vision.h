/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef EI_DEVICE_NICLA_VISION
#define EI_DEVICE_NICLA_VISION

/* Include ----------------------------------------------------------------- */
#include "ei_device_info_lib.h"
#include "ei_camera.h"

/** Baud rates */
#define DEFAULT_BAUD 115200
#define MAX_BAUD     115200

/** Number of sensors used */
#define EI_MAX_FREQUENCIES                  5
#define EI_DEVICE_N_RESOLUTIONS				1
#define EI_DEVICE_N_RESIZE_RESOLUTIONS		3

/** C Callback types */
typedef int (*c_callback)(uint8_t out_buffer[32], size_t *out_size);
typedef bool (*c_callback_status)(void);

typedef struct {
	size_t width;
	size_t height;
} ei_device_resize_resolutions_t;

/**
 * @brief      Class description and implementation of device specific 
 * 			   characteristics
 */	
class EiDeviceNiclaVision : public EiDeviceInfo
{
private:

    static const int standalone_sensor_num = 1;
    ei_device_sensor_t standalone_sensor_list[standalone_sensor_num];

	static ei_device_snapshot_resolutions_t snapshot_resolutions[EI_DEVICE_N_RESOLUTIONS + EI_DEVICE_N_RESIZE_RESOLUTIONS];
	static ei_device_snapshot_resolutions_t resize_resolutions[EI_DEVICE_N_RESIZE_RESOLUTIONS];

    bool camera_present;
    EiCameraNiclaVision *cam;

    bool network_present;
    bool network_connected;   
    //EiNetworkDevice *net;


public:	
	EiDeviceNiclaVision(EiDeviceMemory* mem);
    ~EiDeviceNiclaVision();

    void init_device_id(void);
	bool get_wifi_connection_status(void);
	bool get_wifi_present_status();
	bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size);
	EiSnapshotProperties get_snapshot_list(void);
    void get_resize_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num);
	bool get_resize_list(const ei_device_resize_resolutions_t **resize_list,size_t *resize_list_size);
    bool start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms);
    bool stop_sample_thread(void);
    void set_default_data_output_baudrate(void) override;
    void set_max_data_output_baudrate(void) override;
    uint32_t get_data_output_baudrate(void) override;
	c_callback_status get_wifi_connection_status_function(void);
	c_callback_status get_wifi_present_status_function(void);

#if MULTI_FREQ_ENABLED == 1
	bool start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned) override;		
#endif
	
};

void print_buf(const uint8_t *buf, size_t len);

#endif