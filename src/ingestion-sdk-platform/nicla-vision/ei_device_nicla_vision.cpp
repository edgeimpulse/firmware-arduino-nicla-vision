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

/* Include ----------------------------------------------------------------- */

// Common headers
#include <stdio.h>
#include <stdarg.h>
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

// Platform headers
#include "Arduino.h"
#include "mbed.h"

// Target specific EI headers
#include "ei_device_nicla_vision.h"
#include "ei_flash_nicla_vision.h"
#include "ei_device_memory.h"

#include "ei_camera.h"
#include "ei_microphone.h"

using namespace rtos;
using namespace events;

/* Constants --------------------------------------------------------------- */

/** Memory location for the arduino device address */
#define DEVICE_ID_LSB_ADDR  ((uint32_t)0x100000A4)
#define DEVICE_ID_MSB_ADDR  ((uint32_t)0x100000A8)

/** Max size for device id array */
#define DEVICE_ID_MAX_SIZE  32

/** MBED thread */
Thread fusion_thread;
EventQueue fusion_queue;
mbed::Ticker fusion_sample_rate;

/* Private function declarations ------------------------------------------- */
static bool get_wifi_connection_status_c(void);
static bool get_wifi_present_status_c(void);

/* Public functions -------------------------------------------------------- */
EiDeviceNiclaVision::EiDeviceNiclaVision(EiDeviceMemory* mem)
{
    EiDeviceInfo::memory = mem;

    init_device_id();
    load_config();

    device_type = "ARDUINO_NICLA_VISION";

    cam = static_cast<EiCameraNiclaVision*>(EiCameraNiclaVision::get_camera());
    camera_present = cam->is_camera_present();

    // TODO
    //net = static_cast<EiWifiNiclaVision*>(EiWifiNiclaVision::get_network_device());
    // the absence of error on device init is success
    //network_present = !(net->init());

    // microphone is not handled by fusion system
    standalone_sensor_list[0].name = "Built-in microphone";
    standalone_sensor_list[0].start_sampling_cb = &ei_microphone_sample_start;
    standalone_sensor_list[0].max_sample_length_s = mem->get_available_sample_bytes() / (16000 * 2);
    standalone_sensor_list[0].frequencies[0] = 16000.0f;
    standalone_sensor_list[0].frequencies[1] = 8000.0f;
}

EiDeviceNiclaVision::~EiDeviceNiclaVision()
{
}

void EiDeviceNiclaVision::init_device_id(void)
{
    char buf[DEVICE_ID_MAX_SIZE];
    uint8_t id[12]; 
	uint32_t pdwUniqueID[3];
	pdwUniqueID[0] = HAL_GetUIDw0();
	pdwUniqueID[1] = HAL_GetUIDw1();
	pdwUniqueID[2] = HAL_GetUIDw2();
	for (int i = 0; i < 3; i++)
	{
		id[i * 4 + 0] = (uint8_t)(pdwUniqueID[i] >> 24);
		id[i * 4 + 1] = (uint8_t)(pdwUniqueID[i] >> 16);
		id[i * 4 + 2] = (uint8_t)(pdwUniqueID[i] >> 8);
		id[i * 4 + 3] = (uint8_t)(pdwUniqueID[i] >> 0);
	}

    /* Setup device ID */
    snprintf(&buf[0], DEVICE_ID_MAX_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X"
        ,id[6] & 0xFF
        ,id[7] & 0xFF
        ,id[8] & 0xFF
        ,id[9] & 0xFF
        ,id[10] & 0xFF
        ,id[11] & 0xFF
        );

    device_id = std::string(buf);
}


/**
 * @brief get_device is a static method of EiDeviceInfo class
 * It is used to implement singleton paradigm, so we are returning
 * here pointer always to the same object (dev)
 * 
 * @return EiDeviceInfo* 
 */
EiDeviceInfo* EiDeviceInfo::get_device(void)
{
#ifdef EI_DEVICE_USE_RAM
    static EiDeviceRAM<512, 96> memory(sizeof(EiConfig));
#else
    static EiFlashMemory memory(sizeof(EiConfig));
#endif
    static EiDeviceNiclaVision dev(&memory);

    return &dev;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceNiclaVision::get_wifi_connection_status(void)
{
    return false;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceNiclaVision::get_wifi_present_status(void)
{
    return false;
}

/**
 * @brief      Create sensor list with sensor specs
 *             The studio and daemon require this list
 * @param      sensor_list       Place pointer to sensor list
 * @param      sensor_list_size  Write number of sensors here
 *
 * @return     False if all went ok
 */
bool EiDeviceNiclaVision::get_sensor_list(
    const ei_device_sensor_t **sensor_list,
    size_t *sensor_list_size)
{

    *sensor_list = this->standalone_sensor_list;
    *sensor_list_size = this->standalone_sensor_num;

    return false;
}

/**
 * @brief      Create resolution list for snapshot setting
 *             The studio and daemon require this list
 * @param      snapshot_list       Place pointer to resolution list
 * @param      snapshot_list_size  Write number of resolutions here
 *
 * @return     False if all went ok
 */

EiSnapshotProperties EiDeviceNiclaVision::get_snapshot_list(void)
{
    ei_device_snapshot_resolutions_t *res = NULL;
    uint8_t res_num = 0;

    this->get_resize_resolutions(&res, &res_num);

    //TODO: move the getting of snapshot to camera device
    EiSnapshotProperties props = {
        .has_snapshot = false,
        .support_stream = false,
        .color_depth = "",
        .resolutions_num = res_num,
        .resolutions = res
    };

    if(this->cam->is_camera_present() == true) {
        this->cam->get_resolutions(&res, &res_num);
        props.has_snapshot = true;
        props.support_stream = true;
        props.color_depth = "RGB";
        props.resolutions_num = res_num;
        props.resolutions = res;
    }

    return props;
}

ei_device_snapshot_resolutions_t EiDeviceNiclaVision::resize_resolutions[] = {
        { .width = 64, .height = 64 },
        { .width = 96, .height = 96 },
        { .width = 160, .height = 120 },        
};

void EiDeviceNiclaVision::get_resize_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num) {

    *res = &EiDeviceNiclaVision::resize_resolutions[0];
    *res_num = sizeof(EiDeviceNiclaVision::resize_resolutions) / sizeof(ei_device_snapshot_resolutions_t);

}

/**
 * @brief Setup timer or thread with given interval and call cb function each period
 * @param sample_read_cb
 * @param sample_interval_ms
 * @return true
 */
bool EiDeviceNiclaVision::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    fusion_thread.start(callback(&fusion_queue, &EventQueue::dispatch_forever));
    fusion_sample_rate.attach(fusion_queue.event(sample_read_cb), (sample_interval_ms / 1000.f));
    return true;
}

/**
 * @brief Stop timer of thread
 * @return true
 */
bool EiDeviceNiclaVision::stop_sample_thread(void)
{
    fusion_sample_rate.detach();
    return true;
}

uint32_t EiDeviceNiclaVision::get_data_output_baudrate(void)
{
    return MAX_BAUD;
}

void EiDeviceNiclaVision::set_default_data_output_baudrate(void)
{
    Serial.flush(); // wait for last transmitted data to be sent 
    Serial.begin(DEFAULT_BAUD);
}

void EiDeviceNiclaVision::set_max_data_output_baudrate(void)
{
    Serial.flush(); // wait for last transmitted data to be sent 
    Serial.begin(MAX_BAUD);
}

/**
 * @brief      Get a C callback for the get_wifi_connection_status method
 *
 * @return     Pointer to c get function
 */
c_callback_status EiDeviceNiclaVision::get_wifi_connection_status_function(void)
{
    return &get_wifi_connection_status_c;
}

/**
 * @brief      Get a C callback for the wifi present method
 *
 * @return     The wifi present status function.
 */
c_callback_status EiDeviceNiclaVision::get_wifi_present_status_function(void)
{
    return &get_wifi_present_status_c;
}

/**
 * @brief      Printf function uses vsnprintf and output using Arduino Serial
 *
 * @param[in]  format     Variable argument list
 */
void ei_printf(const char *format, ...) {
    char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}

/**
 * @brief      Write serial data with length to Serial output
 *
 * @param      data    The data
 * @param[in]  length  The length
 */
void ei_write_string(char *data, int length) {
    Serial.write(data, length);
}

/**
 * @brief      Get Arduino serial object
 *
 * @return     pointer to Serial
 */
mbed::Stream* ei_get_serial() {
    return (mbed::Stream *)&Serial;
}

/**
 * @brief      Check if new serial data is available
 *
 * @return     Returns number of available bytes
 */
int ei_get_serial_available(void) {
    return Serial.available();
}

/**
 * @brief      Get next available byte
 *
 * @return     byte
 */
char ei_get_serial_byte(void) {
    return Serial.read();
}

/**
 * @brief      Write character to serial
 *
 * @param      cChar     Char addr to write
 */
void ei_putc(char cChar) {
    Serial.write(&cChar, 1);
}

/* Private functions ------------------------------------------------------- */

static bool get_wifi_connection_status_c(void)
{
    return false;
}

static bool get_wifi_present_status_c(void)
{
    return false;
}

char ei_getchar()
{
    char ch = 0;
    if (Serial.available() > 0) {
	    ch = Serial.read();
    }
    return ch;
}

void print_buf(const uint8_t *buf, size_t len)
{
    ei_printf("size of data: %d \n", len);
    for (size_t i = 0; i < len; ++i) {
        ei_printf("%02x", buf[i]);
        if (i % 16 == 15)
            ei_printf("\n");
        else
            ei_printf(" ");
    }
}