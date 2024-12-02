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
#include "model-parameters/model_metadata.h"
#include "ei_device_lib.h"

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "ei_camera.h"
#include "firmware-sdk/at_base64_lib.h"
#include "firmware-sdk/jpeg/encode_as_jpg.h"
#include "firmware-sdk/ei_device_interface.h"
#include "stdint.h"
#include "ei_device_nicla_vision.h"
#include "ei_run_impulse.h"

#define DWORD_ALIGN_PTR(a)   ((a & 0x3) ?(((uintptr_t)a + 0x4) & ~(uintptr_t)0x3) : a)
#define ALIGN_PTR(p,a)   ((p & (a-1)) ?(((uintptr_t)p + a) & ~(uintptr_t)(a-1)) : p)

typedef enum {
    INFERENCE_STOPPED,
    INFERENCE_WAITING,
    INFERENCE_SAMPLING,
    INFERENCE_DATA_READY
} inference_state_t;

static inference_state_t state = INFERENCE_STOPPED;
static uint64_t last_inference_ts = 0;

static bool debug_mode = false;
static bool continuous_mode = false;

static uint8_t *snapshot_buf = nullptr;
static uint32_t snapshot_buf_size;

static ei_device_snapshot_resolutions_t snapshot_resolution;
static ei_device_snapshot_resolutions_t fb_resolution;

static bool resize_required = false;
static uint32_t inference_delay;

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }

    // and done!
    return 0;
}

void ei_run_impulse(void)
{
    switch(state) {
        case INFERENCE_STOPPED:
            // nothing to do
            return;
        case INFERENCE_WAITING:
            if(ei_read_timer_ms() < (last_inference_ts + inference_delay)) {
                return;
            }
            state = INFERENCE_DATA_READY;
            break;
        case INFERENCE_SAMPLING:
        case INFERENCE_DATA_READY:
            if(continuous_mode == true) {
                state = INFERENCE_WAITING;
            }
            break;
        default:
            break;
    }

    EiCameraNiclaVision *camera = static_cast<EiCameraNiclaVision*>(EiCameraNiclaVision::get_camera());
    camera->get_fb_ptr(&snapshot_buf);
    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to get snapshot buffer!\n");
        return;
    }

    ei_printf("Taking photo...\n");

    bool isOK = camera->ei_camera_capture_rgb888_packed_big_endian(snapshot_buf, snapshot_buf_size);
    if (!isOK) {
        return;
    }

    if (resize_required) {
        ei::image::processing::crop_and_interpolate_rgb888(
            snapshot_buf,
            fb_resolution.width,
            fb_resolution.height,
            snapshot_buf,
            snapshot_resolution.width,
            snapshot_resolution.height);
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // Print framebuffer as JPG during debugging
    if(debug_mode) {
        ei_printf("Begin output\n");

        size_t jpeg_buffer_size = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT >= 128 * 128 ?
            8192 * 3 :
            4096 * 4;
        uint8_t *jpeg_buffer = NULL;
        jpeg_buffer = (uint8_t*)ei_malloc(jpeg_buffer_size);
        if (!jpeg_buffer) {
            ei_printf("ERR: Failed to allocate JPG buffer\r\n");
            return;
        }

        size_t out_size;
        int x = encode_rgb888_signal_as_jpg(&signal, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, jpeg_buffer, jpeg_buffer_size, &out_size);
        if (x != 0) {
            ei_printf("Failed to encode frame as JPEG (%d)\n", x);
            return;
        }

        ei_printf("Framebuffer: ");
        base64_encode((char*)jpeg_buffer, out_size, ei_putchar);
        ei_printf("\r\n");

        if (jpeg_buffer) {
            ei_free(jpeg_buffer);
        }
    }

    // run the impulse: DSP, neural network and the Anomaly algorithm
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &result, false);
    if (ei_error != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run impulse (%d)\n", ei_error);
        camera->deinit();
        return;
    }

    display_results(&ei_default_impulse, &result);

    if (debug_mode) {
        ei_printf("\r\n----------------------------------\r\n");
        ei_printf("End output\r\n");
    }

    if(continuous_mode == false) {
        ei_printf("Starting inferencing in %d seconds...\n", inference_delay / 1000);
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
    }
}

void ei_start_impulse(bool continuous, bool debug, bool use_max_uart_speed)
{
    snapshot_resolution.width = EI_CLASSIFIER_INPUT_WIDTH;
    snapshot_resolution.height = EI_CLASSIFIER_INPUT_HEIGHT;

    debug_mode = debug;
    continuous_mode = (debug) ? true : continuous;

    EiDeviceNiclaVision* dev = static_cast<EiDeviceNiclaVision*>(EiDeviceNiclaVision::get_device());
    EiCameraNiclaVision *camera = static_cast<EiCameraNiclaVision*>(EiCameraNiclaVision::get_camera());

    // check if minimum suitable sensor resolution is the same as
    // desired snapshot resolution
    // if not we need to resize later
    fb_resolution = camera->search_resolution(snapshot_resolution.width, snapshot_resolution.height);

    if (snapshot_resolution.width != fb_resolution.width || snapshot_resolution.height != fb_resolution.height) {
        resize_required = true;
    }

    if (!camera->init(snapshot_resolution.width, snapshot_resolution.height)) {
        ei_printf("Failed to init camera, check if camera is connected!\n");
        return;
    }

    snapshot_buf_size = fb_resolution.width * fb_resolution.height * ei::image::processing::RGB888_B_SIZE;

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tImage resolution: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    if(continuous_mode == true) {
        inference_delay = 0;
        state = INFERENCE_DATA_READY;
    }
    else {
        inference_delay = 2000;
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
        ei_printf("Starting inferencing in %d seconds...\n", inference_delay / 1000);
    }

    if (debug_mode) {
        ei_printf("OK\r\n");
        ei_sleep(100);
        dev->set_max_data_output_baudrate();
        ei_sleep(100);
    }

    while(!ei_user_invoke_stop_lib()) {
        ei_run_impulse();
        ei_sleep(1);
    }

    ei_stop_impulse();

    if (debug_mode) {
        ei_printf("\r\nOK\r\n");
        ei_sleep(100);
        dev->set_default_data_output_baudrate();
        ei_sleep(100);
    }

}

void ei_stop_impulse(void)
{
    EiCameraNiclaVision *camera = static_cast<EiCameraNiclaVision*>(EiCameraNiclaVision::get_camera());
    camera->deinit();
    state = INFERENCE_STOPPED;
}

bool is_inference_running(void)
{
    return (state != INFERENCE_STOPPED);
}

#endif /* defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA */

// AT+RUNIMPULSE