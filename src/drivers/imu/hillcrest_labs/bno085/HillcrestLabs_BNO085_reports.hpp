/****************************************************************************
*
*   Copyright (c) 2022 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#pragma once

namespace HillcrestLabs_BNO085
{

enum shtp_channels
{
    CH0_COMMAND       = 0,
    CH1_EXECUTABLE    = 1,
    CH2_CONTROL       = 2,
    CH3_REPORTS       = 3,
    CH4_WAKE_REPORTS  = 4,
    CH5_GYRO_ROTATION = 5
};

// Sensor scaling
#define SCALE_Q(n) (1.0f / (1 << n))

// SHTP CH2 report IDs [from CEVA SH2-Reference Manual]
#define SHTP_REPORT_BASE_TIME            0xFB
#define SHTP_REPORT_GET_FEATURE_RESPONSE 0xFC
#define SHTP_REPORT_SET_FEATURE_COMMAND  0xFD

// SENSOR CH3  report IDs [from CEVA SH2-Reference Manual]
#define SENSOR_REPORTID_ACCELEROMETER  0x01
#define SENSOR_REPORTID_GYROSCOPE      0x02

struct Header {
    uint8_t length_lsb = 0;
    uint8_t length_msb = 0;
    uint8_t channel = 0;
    uint8_t ch_seq = 0;
};

struct FeatureControlCommand {
    uint8_t command_id = 0;
    uint8_t report_id_feature = 0;
    uint8_t flags = 0;
    uint8_t change_sensitivity_lsb = 0;
    uint8_t change_sensitivity_msb = 0;
    uint8_t report_interval_lsb = 0;
    uint8_t report_interval_1 = 0;
    uint8_t report_interval_2 = 0;
    uint8_t report_interval_msb = 0;
    uint8_t batch_interval_lsb = 0;
    uint8_t batch_interval_1 = 0;
    uint8_t batch_interval_2 = 0;
    uint8_t batch_interval_msb = 0;
    uint8_t specific_config_lsb = 0;
    uint8_t specific_config_1 = 0;
    uint8_t specific_config_2 = 0;
    uint8_t specific_config_msb = 0;
};

struct CommandPacket
{
    Header header{};
    FeatureControlCommand feature_control_payload{};
};

struct Ch3Payload {
    uint8_t timebase_id = 0;
    uint8_t delta_t_lsb = 0;
    uint8_t delta_t_1   = 0;
    uint8_t delta_t_2   = 0;
    uint8_t delta_t_msb = 0;
    uint8_t report_id   = 0;
    uint8_t seq         = 0;
    uint8_t status      = 0;
    uint8_t delay       = 0;
    uint8_t data_x_lsb  = 0;
    uint8_t data_x_msb  = 0;
    uint8_t data_y_lsb  = 0;
    uint8_t data_y_msb  = 0;
    uint8_t data_z_lsb  = 0;
    uint8_t data_z_msb  = 0;
};

struct Ch3Packet
{
    Header header{};
    Ch3Payload ch3_payload{};
};

} // namespace HillcrestLabs_BNO085
