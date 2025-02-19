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