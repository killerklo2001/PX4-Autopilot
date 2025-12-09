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

namespace hillcrest_labs::BNO085::reports
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

// SHTP CH2 report IDs [from CEVA SH2-Reference Manual]
#define SHTP_REPORT_BASE_TIMESTAMP       0xFB
#define SHTP_REPORT_GET_FEATURE_RESPONSE 0xFC
#define SHTP_REPORT_SET_FEATURE_COMMAND  0xFD

// SENSOR CH3  report IDs [from CEVA SH2-Reference Manual]
#define SENSOR_REPORTID_ACCELEROMETER  0x01
#define SENSOR_REPORTID_GYROSCOPE      0x02

enum header : uint8_t {
    LENGTH_LSB  = 0, // Byte 0:  packet length LSB
    LENGTH_MSB  = 1, // Byte 1:  packet length MSB
    CHANNEL     = 2, // Byte 2:  channel number
    CH_SEQ      = 3  // Byte 3:  channel sequence number
};

enum feature_control_payload : uint8_t {
    REPORT_ID              = 0,  // Byte 0:  set feature command or get feature response report id
    REPORT_ID_FEATURE      = 1,  // Byte 1:  feature report ID
    FLAGS                  = 2,  // Byte 2:  feature flags
    CHANGE_SENSITIVITY_LSB = 3,  // Byte 3:  change sensitivity [abs|rel] LSB
    CHANGE_SENSITIVITY_MSB = 4,  // Byte 4:  change sensitivity [abs|rel] MSB
    REPORT_INTERVAL_LSB    = 5,  // Byte 5:  report interval LSB
    REPORT_INTERVAL_1      = 6,  // Byte 6:  report interval byte 1
    REPORT_INTERVAL_2      = 7,  // Byte 7:  report interval byte 2
    REPORT_INTERVAL_MSB    = 8,  // Byte 8:  report interval MSB
    BATCH_INTERVAL_LSB     = 9,  // Byte 9:  batch interval LSB
    BATCH_INTERVAL_1       = 10, // Byte 10: batch interval byte 1
    BATCH_INTERVAL_2       = 11, // Byte 11: batch interval byte 2
    BATCH_INTERVAL_MSB     = 12, // Byte 12: batch interval MSB
    SPECIFIC_CONFIG_LSB    = 13, // Byte 13: sensor specific configuration word LSB
    SPECIFIC_CONFIG_1      = 14, // Byte 14: sensor specific configuration word byte 1
    SPECIFIC_CONFIG_2      = 15, // Byte 15: sensor specific configuration word byte 2
    SPECIFIC_CONFIG_MSB    = 16  // Byte 16: sensor specific configuration word MSB
};

enum ch3_payload : uint8_t {
    REPORT_ID_T = 0,  // Byte 0:  timebase report id
    DELTA_T_LSB = 1,  // Byte 1:  timebase LSB
    DELTA_T_1   = 2,  // Byte 2:  timebase byte 1
    DELTA_T_2   = 3,  // Byte 3:  timebase byte 2
    DELTA_T_MSB = 4,  // Byte 4:  timebase MSB
    REPORT_ID   = 5,  // Byte 5:  report ID
    SEQ         = 6,  // Byte 6: sequence number
    STATUS      = 7,  // Byte 7: status
    DELAY       = 8,  // Byte 8: delay
    DATA_X_LSB  = 9,  // Byte 9: x-axis LSB
    DATA_X_MSB  = 10, // Byte 10: x-axis MSB
    DATA_Y_LSB  = 11, // Byte 11: y-axis LSB
    DATA_Y_MSB  = 12, // Byte 12: y-axis MSB
    DATA_Z_LSB  = 13, // Byte 13: z-axis LSB
    DATA_Z_MSB  = 14  // Byte 14: z-axis MSB
};

}
