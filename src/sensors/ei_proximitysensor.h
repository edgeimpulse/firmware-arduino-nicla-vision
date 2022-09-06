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

#ifndef EI_PROXIMITYSENSOR
#define EI_PROXIMITYSENSOR

/* Include ----------------------------------------------------------------- */
#include "ei_fusion.h"

/** Number of axis used and sample data format */
#define PROXIMITY_AXIS_SAMPLED 1

/* Function prototypes ----------------------------------------------------- */
bool ei_proximity_sensor_init(void);
float *ei_fusion_proximity_sensor_read_data(int n_samples);

static const ei_device_fusion_sensor_t proximity_sensor = {
    // name of sensor module to be displayed in fusion list
    "Proximity sensor",
    // number of sensor module axis
    PROXIMITY_AXIS_SAMPLED,
    // sampling frequencies
    { 50.0f, 25.0f, 12.5f, 5.0f},
    // axis name and units payload (must be same order as read in)
    {
        { "distance", "cm" },
    },
    // reference to read data function
    &ei_fusion_proximity_sensor_read_data
};

#endif
