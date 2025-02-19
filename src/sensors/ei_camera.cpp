/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "firmware-sdk/ei_camera_interface.h"
#include "firmware-sdk/ei_device_interface.h"
#include "firmware-sdk/ei_image_lib.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "ei_camera.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gc2145.h"

#include <ea_malloc.h>

#define ALIGN_PTR(p,a)   ((p & (a-1)) ?(((uintptr_t)p + a) & ~(uintptr_t)(a-1)) : p)

GC2145 galaxyCore;
Camera cam(galaxyCore);

FrameBuffer fb;
ei_device_snapshot_resolutions_t frame_size;

static bool is_initialised = false;
static uint8_t *ei_camera_frame_mem = NULL;
static uint8_t *ei_camera_framebuffer = NULL;
static uint8_t *snapshot_buffer = NULL;

#include "mbed.h"
 static void print_memory_info2() {
     // allocate enough room for every thread's stack statistics
     int cnt = osThreadGetCount();
     mbed_stats_stack_t *stats = (mbed_stats_stack_t*) ei_malloc(cnt * sizeof(mbed_stats_stack_t));

     cnt = mbed_stats_stack_get_each(stats, cnt);
     for (int i = 0; i < cnt; i++) {
         ei_printf("Thread: 0x%lX, Stack size: %lu / %lu\r\n", stats[i].thread_id, stats[i].max_size, stats[i].reserved_size);
     }
     ei_free(stats);

     // Grab the heap statistics
     mbed_stats_heap_t heap_stats;
     mbed_stats_heap_get(&heap_stats);
     ei_printf("Heap size: %lu / %lu bytes (max: %lu)\r\n", heap_stats.current_size, heap_stats.reserved_size, heap_stats.max_size);
 }

ei_device_snapshot_resolutions_t EiCameraNiclaVision::resolutions[] = {
        { .width = 320, .height = 240 }
    };


bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, uint32_t src_len)
{
    uint8_t hb, lb;
    uint32_t pix_count = src_len / 2;

    for(uint32_t i = 0; i < pix_count; i ++) {
        hb = *src_buf++;
        lb = *src_buf++;

        *dst_buf++ = hb & 0xF8;
        *dst_buf++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3;
        *dst_buf++ = (lb & 0x1F) << 3;
    }

    return true;
}

EiCameraNiclaVision::EiCameraNiclaVision()
{
}


bool EiCameraNiclaVision::is_camera_present(void)
{
    return true;
}

ei_device_snapshot_resolutions_t EiCameraNiclaVision::get_min_resolution(void) {
    return resolutions[0];
}

void EiCameraNiclaVision::get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num) {

    *res = &EiCameraNiclaVision::resolutions[0];
    *res_num = sizeof(EiCameraNiclaVision::resolutions) / sizeof(ei_device_snapshot_resolutions_t);

}

bool EiCameraNiclaVision::set_resolution(const ei_device_snapshot_resolutions_t res) {
    // NOT IMPLEMENTED, SET TO 320x240 BY DEFAULT
    return true;
}

bool EiCameraNiclaVision::init(uint16_t width, uint16_t height)
{
    if (is_initialised) return true;

    if (!cam.begin(CAMERA_R320x240, CAMERA_RGB565, -1)) {
        ei_printf("ERR: Camera init failed\n");
        return false;
    }

    ei_sleep(100);

    // the camera framebuffer, in RGB565 format
    ei_camera_frame_mem = (uint8_t *)ei_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_RAW_FRAME_BYTE_SIZE + 32 /*alignment*/);
    if(ei_camera_frame_mem == NULL) {
        ei_printf("ERR: Failed to create ei_camera_frame_mem\r\n");
        return false;
    }
    ei_camera_framebuffer = (uint8_t *)ALIGN_PTR((uintptr_t)ei_camera_frame_mem, 32);
    fb.setBuffer(ei_camera_framebuffer);

    // the EI snapshot buffer, in RGB888 format
    snapshot_buffer = (uint8_t*)ea_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3 + 32);
    if(snapshot_buffer == NULL) {
        ei_printf("ERR: Failed to create snapshot_buf\r\n");
        return false;
    }
    snapshot_buffer = (uint8_t *)ALIGN_PTR((uintptr_t)snapshot_buffer, 32);

    is_initialised = true;

    return true;
}

bool EiCameraNiclaVision::deinit()
{
    delay(100);
    ei_free(ei_camera_frame_mem);
    ea_free(snapshot_buffer);
    ei_camera_framebuffer = NULL;
    snapshot_buffer = NULL;

    is_initialised = false;
    return true;
}

bool EiCameraNiclaVision::ei_camera_capture_rgb888_packed_big_endian(
    uint8_t *image,
    uint32_t image_size)
{
    if (cam.grabFrame(fb, 100) != 0) {
        ei_printf("ERR: Camera capture failed\n");
        deinit();
        return false;
    }

    bool converted = RBG565ToRGB888(ei_camera_framebuffer, image, cam.frameSize());

    if(!converted){
        ei_printf("ERR: Conversion failed\n");
        deinit();
        return false;
    }
    return true;
}

bool EiCameraNiclaVision::get_fb_ptr(uint8_t** fb_ptr)
{
    *fb_ptr = snapshot_buffer;
    return true;
}

bool EiCameraNiclaVision::ei_camera_capture_jpeg(uint8_t **image, uint32_t *image_size)
{
    // NOT IMPLEMENTED (JPEG OUTPUT NOT SUPPORTED BY CAMERA) PRESERVED FOR COMPATIBILITY
    return false;
}

bool EiCameraNiclaVision::ei_camera_jpeg_to_rgb888(uint8_t *jpeg_image, uint32_t jpeg_image_size,
                                             uint8_t *rgb88_image)
{
    // NOT IMPLEMENTED (JPEG OUTPUT NOT SUPPORTED BY CAMERA) PRESERVED FOR COMPATIBILITY
    return false;
}

EiCamera *EiCamera::get_camera()
{
    static EiCameraNiclaVision camera;
    return &camera;
}