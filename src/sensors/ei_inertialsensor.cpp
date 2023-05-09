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
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "ei_inertialsensor.h"
#include "LSM6DSOXSensor.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 9.80665f

LSM6DSOXSensor AccGyr(&SPI1, PIN_SPI_SS1);
uint8_t acceleroStatus;
uint8_t gyroStatus;

static float imu_data[INERTIAL_AXIS_SAMPLED];

bool ei_inertial_sensor_init(void)
{
    SPI1.begin();
    AccGyr.begin();

    if (AccGyr.Enable_X() != LSM6DSOX_OK || AccGyr.Enable_G() != LSM6DSOX_OK) {
        return false;
    }

    AccGyr.Set_X_ODR(208.0f);
    AccGyr.Set_X_FS(2);

    AccGyr.Set_G_ODR(208.0f);
    AccGyr.Set_G_FS(2000);

    ei_add_sensor_to_fusion_list(inertial_sensor);
    return true;
}

float *ei_fusion_inertial_sensor_read_data(int n_samples)
{    
    AccGyr.Get_X_DRDY_Status(&acceleroStatus);
    if (acceleroStatus == 1) {
        int32_t acceleration[3];
        AccGyr.Get_X_Axes(acceleration);

        imu_data[0] = (float)acceleration[0] * CONVERT_G_TO_MS2 / 1000.0f;
        imu_data[1] = (float)acceleration[1] * CONVERT_G_TO_MS2 / 1000.0f;
        imu_data[2] = (float)acceleration[2] * CONVERT_G_TO_MS2 / 1000.0f;
    }
    else {
        ei_printf("ERR accelerometer not ready!\n");
        imu_data[0] = 0.0f;
        imu_data[1] = 0.0f;
        imu_data[2] = 0.0f;
    }

    AccGyr.Get_G_DRDY_Status(&gyroStatus);
    if (n_samples > 3 && gyroStatus == 1) {
        int32_t rotation[3];
        AccGyr.Get_G_Axes(rotation);
        imu_data[3] = (float)rotation[0] / 32768.0;
        imu_data[4] = (float)rotation[1] / 32768.0;
        imu_data[5] = (float)rotation[2] / 32768.0;
    }
    else {
        ei_printf("ERR gyroscope not ready!\n");
        imu_data[3] = 0.0f;
        imu_data[4] = 0.0f;
        imu_data[5] = 0.0f;
    }

#ifdef EI_DEBUG
    for (int i = 0; i < INERTIAL_AXIS_SAMPLED; i++) {
        //imu_data[i] = 0.0;
        ei_printf("%f ", imu_data[i]);
    }
    ei_printf("\n");
#endif
    return imu_data;
}