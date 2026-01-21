
#include "BNO085.hpp"

#include <pigpiod_if2.h>

using namespace time_literals;

BNO085::BNO085(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), ROTATION_ROLL_180_YAW_45),
	_px4_gyro(get_device_id(), ROTATION_ROLL_180_YAW_45),
	_px4_mag(get_device_id(), ROTATION_ROLL_180_YAW_45)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

}

BNO085::~BNO085()
{
	perf_free(_drdy_missed_perf);
}

int BNO085::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	_pi = pigpio_start(nullptr, nullptr);
	if (_pi < 0) {
		PX4_ERR("Cannot connect to pigpiod");
		return PX4_ERROR;
	}

	set_mode(_pi, CONFIG_BNO085_INT_PIN, PI_INPUT);
	set_pull_up_down(_pi, CONFIG_BNO085_INT_PIN, PI_PUD_UP);
	set_mode(_pi, CONFIG_BNO085_RESET_PIN, PI_OUTPUT);
	set_mode(_pi, CONFIG_BNO085_WAKEUP_PIN, PI_OUTPUT);

	return Reset() ? 0 : -1;
}

bool BNO085::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void BNO085::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
	pigpio_stop(_pi);
	_pi = -1;
}

void BNO085::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_drdy_missed_perf);
}

int BNO085::probe()
{
	//TODO: add real WHOAMI check
	return PX4_OK;
}


// main runner
void BNO085::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	// Watchdog
	if (_drdy_seen && hrt_elapsed_time(&_last_drdy) > 500_ms) {
		PX4_WARN("BNO085 DRDY timeout -> WakeUp()");
		_drdy_seen = false;
		WakeUp();
	}

	switch (_state) {
	case STATE::RESET:
	{
		// Reset sequence
		gpio_write(_pi, CONFIG_BNO085_WAKEUP_PIN, 1);
		px4_usleep(10 * 1000);

		gpio_write(_pi, CONFIG_BNO085_RESET_PIN, 0);
		px4_usleep(1 * 1000);
		gpio_write(_pi, CONFIG_BNO085_RESET_PIN, 1);

		if (DataReadyInterruptConfigure()) {
			_data_ready_interrupt_enabled = true;

			// backup schedule as a watchdog timeout
			ScheduleDelayed(100_ms);

		} else {
			_data_ready_interrupt_enabled = false;
		}

		_state = STATE::WAIT_FOR_REBOOT;
		break;
	}

	case STATE::WAIT_FOR_REBOOT:
	{
		// Boot handshake: wait for INT pin
		ScheduleDelayed(1_ms);

		_state = STATE::FLUSH_REBOOT_REPORTS;
		break;
	}

	case STATE::FLUSH_REBOOT_REPORTS:
	{
		// Flush initial boot reports
		uint8_t tx_dummy[24] {};
		uint8_t rx_dummy[24] {};
		for (int i = 0; i < 100; i++) {
			SPI::transfer(tx_dummy, rx_dummy, 20);
		}
		_reset_timestamp = now;
		_failure_count = 0;

		_state = STATE::CONFIGURE_PX4;
		break;
	}

	case STATE::CONFIGURE_PX4:
	{
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::SET_FEATURES;

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;
	}

	case STATE::SET_FEATURES:
	{
		if (_set_feature_tries > 50) {
			PX4_DEBUG("Configure failed after %d tries, resetting", _set_feature_tries++);
			_state = STATE::RESET;
			_accel_set = false;
			_gyro_set = false;
			_mag_set = false;
		}

		if (hrt_elapsed_time(&_last_set) >= 4_s) {

			if (!_accel_set) {
				if (_set_feature_tries == 0) {
					SetFeature(SENSOR_REPORTID_ACCELEROMETER, ACC_SAMPLE_PERIOD_US);
					_set_feature_tries++;
				}
				if (GetFeature(SENSOR_REPORTID_ACCELEROMETER, ACC_SAMPLE_PERIOD_US)) {
					_accel_set = true;
					_set_feature_tries = 0;
					_last_set = now;
					PX4_INFO("Feature 0x%02X set.", SENSOR_REPORTID_ACCELEROMETER);
				} else {
					_set_feature_tries++;
				}
			}
			else if (!_gyro_set) {
				if (_set_feature_tries == 0) {
					SetFeature(SENSOR_REPORTID_GYROSCOPE, GYRO_SAMPLE_PERIOD_US);
					_set_feature_tries++;
				}
				if (GetFeature(SENSOR_REPORTID_GYROSCOPE, GYRO_SAMPLE_PERIOD_US)) {
					_gyro_set = true;
					_set_feature_tries = 0;
					_last_set = now;
					PX4_INFO("Feature 0x%02X set.", SENSOR_REPORTID_GYROSCOPE);
				} else {
					_set_feature_tries++;
				}
			}
			else if (!_mag_set) {
				if (_set_feature_tries == 0) {
					SetFeature(SENSOR_REPORTID_MAGNETOMETER, MAG_SAMPLE_PERIOD_US);
					_set_feature_tries++;
				}
				if (GetFeature(SENSOR_REPORTID_MAGNETOMETER, MAG_SAMPLE_PERIOD_US)) {
					_mag_set = true;
					_set_feature_tries = 0;
					_last_set = now;
					PX4_INFO("Feature 0x%02X set.", SENSOR_REPORTID_MAGNETOMETER);
				} else {
					_set_feature_tries++;
				}
			}
			else {
				_state = STATE::READ_REPORTS;
			}

		}
		_drdy_seen = false;
		break;
	}

	case STATE::READ_REPORTS:
	{
		//hrt_abstime timestamp_sample = 0;

		bool success = false;

		if (ReadReport()) {
			success = true;

			if (_failure_count > 0) {
				_failure_count--;
			}
		}

		if (!success) {
			_failure_count++;

			// full reset if things are failing consistently
			if (_failure_count > 10) {
				_drdy_seen = false;
				return;
			}
		}
		_drdy_seen = false;
		break;
	}

	}
	
	// Watchdog
	ScheduleDelayed(200_ms);
}

void BNO085::WakeUp()
{
	gpio_write(_pi, CONFIG_BNO085_WAKEUP_PIN, 0);
    px4_usleep(10 * 1000); // 10 ms
    gpio_write(_pi, CONFIG_BNO085_WAKEUP_PIN, 1);
    px4_usleep(20 * 1000); // 10 ms
}

void BNO085::SetFeature(uint8_t feature_id, uint32_t report_interval_us)
{
	constexpr uint8_t CHANNEL_NUMBER = 2;

	CommandPacket tx_packet{};

	int ch_seq = _ch_seq;

	tx_packet.header.length_lsb = sizeof(CommandPacket) & 0xFF;
	tx_packet.header.length_msb = (sizeof(CommandPacket) >> 8) & 0xFF;
	tx_packet.header.channel = CHANNEL_NUMBER;
	tx_packet.header.ch_seq = ch_seq;

	_ch_seq = (_ch_seq + 1) & 0xFF;

	tx_packet.feature_control_payload.command_id = SHTP_REPORT_SET_FEATURE_COMMAND;
	tx_packet.feature_control_payload.report_id_feature = feature_id;
	tx_packet.feature_control_payload.report_interval_lsb = (report_interval_us >> 0) & 0xFF;
	tx_packet.feature_control_payload.report_interval_1   = (report_interval_us >> 8) & 0xFF;
	tx_packet.feature_control_payload.report_interval_2   = (report_interval_us >> 16) & 0xFF;
	tx_packet.feature_control_payload.report_interval_msb = (report_interval_us >> 24) & 0xFF;

	PX4_INFO("Sending 'SET FEATURE COMMAND' for Report ID: 0x%02X", feature_id);


	WakeUp();
	SPI::transfer(reinterpret_cast<uint8_t*>(&tx_packet), nullptr, sizeof(tx_packet));
}


bool BNO085::GetFeature(uint8_t feature_id, uint32_t report_interval_us)
{
	constexpr uint8_t CHANNEL_NUMBER = 2;
	CommandPacket dummy_tx_packet{};
	CommandPacket rx_packet{};

	SPI::transfer(reinterpret_cast<uint8_t*>(&dummy_tx_packet), reinterpret_cast<uint8_t*>(&rx_packet), sizeof(dummy_tx_packet));

	if (rx_packet.header.channel != CHANNEL_NUMBER) {
		return false;
	}

	if (rx_packet.feature_control_payload.command_id != SHTP_REPORT_GET_FEATURE_RESPONSE) {
		return false;
	}

	if (rx_packet.feature_control_payload.report_id_feature != feature_id) {
		return false;
	}

	return true;


}


bool BNO085::Configure()
{
	// scale with Q-Points from CEVA SH2-Reference Manual
	_px4_gyro.set_scale(SCALE_Q(9));
	_px4_gyro.set_range(math::radians(2000.f));

	_px4_accel.set_scale(SCALE_Q(8));
	_px4_accel.set_range(8.f * CONSTANTS_ONE_G);

	_px4_mag.set_scale(SCALE_Q(4) * 0.01f); // first BNO raw to uT, then to gauss


	return true;
}


void BNO085::DataReadyCallback(int pi, unsigned user_gpio, unsigned edge, uint32_t tick, void *userdata)

{
    if (edge == 0) {  // FALLING_EDGE
        auto *self = static_cast<BNO085 *>(userdata);
		if (self->_drdy_seen) {
			PX4_WARN("interrupt already set");
		}
        self->_last_drdy = hrt_absolute_time();
        self->_drdy_seen = true;
        self->ScheduleNow();
    }
}


bool BNO085::DataReadyInterruptConfigure()
{
    _pigpio_cb = callback_ex(
        _pi,
        CONFIG_BNO085_INT_PIN,
        FALLING_EDGE,
        &BNO085::DataReadyCallback,
        this
    );

    return _pigpio_cb >= 0;
}


bool BNO085::DataReadyInterruptDisable()
{
    if (_pigpio_cb >= 0) {
        callback_cancel(_pigpio_cb);
        _pigpio_cb = -1;
    }
    return true;
}



#include <iostream>
#include <fstream>
#include <iomanip> // für std::fixed, std::setprecision
#include <cstdint>

void saveCSV(const std::string &filename,
             uint64_t ts[][4], float data[][3], int rows)
{
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "Datei konnte nicht geoeffnet werden!\n";
        return;
    }

    file << std::fixed;

    for (int i = 0; i < rows; ++i) {
        // Timestamps / base_delta / delay
        for (int j = 0; j < 4; ++j) {
            file << ts[i][j] << ";";
        }
        // Sensorwerte
        for (int j = 0; j < 3; ++j) {
            file << data[i][j];
            if (j < 2) file << ";";
        }
        file << "\n";
    }

    file.close();
}



bool BNO085::ReadReport()
{
	constexpr uint8_t CHANNEL_NUMBER = 3;

	Ch3Packet tx_packet{};
	Ch3Packet rx_packet{};

	int ch_seq = _ch_seq;

	tx_packet.header.length_lsb = sizeof(Ch3Packet) & 0xFF;
	tx_packet.header.length_msb = (sizeof(Ch3Packet) >> 8) & 0xFF;
	tx_packet.header.channel = CHANNEL_NUMBER;
	tx_packet.header.ch_seq = ch_seq;

	_ch_seq = (_ch_seq + 1) & 0xFF;

	SPI::transfer(reinterpret_cast<uint8_t*>(&tx_packet), reinterpret_cast<uint8_t*>(&rx_packet), sizeof(tx_packet));

	// Host Timestamp
	hrt_abstime host_ts = _last_drdy;

	// check if timebase report
	if (rx_packet.ch3_payload.timebase_id != SHTP_REPORT_BASE_TIME) {
		PX4_WARN("Ignoring packet, not a Timebase report: 0x%02X", rx_packet.ch3_payload.timebase_id);
		return false;
	}


	// base_delta = signed 32-bit little endian (bytes delta_t_lsb .. delta_t_msb)
	int32_t base_delta_ticks =
		rx_packet.ch3_payload.delta_t_lsb |
		(rx_packet.ch3_payload.delta_t_1 << 8) |
		(rx_packet.ch3_payload.delta_t_2 << 16) |
		(rx_packet.ch3_payload.delta_t_msb << 24);

	// Datasheet: ticks are 100 us units
	int64_t base_delta_us = (int64_t)base_delta_ticks * 100LL;

	PX4_DEBUG("base delta: %lld | host: %llu", base_delta_us, host_ts);

	// catch if negative than host_ts
	if (base_delta_us < 0) {
		PX4_WARN("base_delta_us (%lld) < 0 (%llu), dropping packet.", base_delta_us, host_ts);
		return false;
	}

	// delay field from sensor report (1 byte), also in 100 us ticks
	uint8_t status = rx_packet.ch3_payload.status;
	uint8_t delay_low = rx_packet.ch3_payload.delay;
	uint32_t delay_high = (status >> 2) & 0x3F; // Masked Bits 7:2
	uint32_t delay_ticks = (delay_high << 8) | delay_low;
	int64_t delay_us = (int64_t)delay_ticks * 100LL;

	// compute actual sample timestamp according to datasheet
	hrt_abstime sample_ts = host_ts - base_delta_us + delay_us;


	int16_t data_x = (int16_t)((rx_packet.ch3_payload.data_x_msb << 8) |
								rx_packet.ch3_payload.data_x_lsb);
	int16_t data_y = (int16_t)((rx_packet.ch3_payload.data_y_msb << 8) |
								rx_packet.ch3_payload.data_y_lsb);
	int16_t data_z = (int16_t)((rx_packet.ch3_payload.data_z_msb << 8) |
								rx_packet.ch3_payload.data_z_lsb);

	switch (rx_packet.ch3_payload.report_id) {

		case SENSOR_REPORTID_ACCELEROMETER:
		{
			if (sample_ts <= _last_sample_ts_accel) {
				PX4_WARN("Lower or equal accel timestamp than before. Dropping packet...");
				break;
			}
			if (_debug_counter_acc < DEBUG_ROWS)
			{
				_debug_ts_acc[_debug_counter_acc][0] = sample_ts;
				_debug_ts_acc[_debug_counter_acc][1] = host_ts;
				_debug_ts_acc[_debug_counter_acc][2] = base_delta_us;
				_debug_ts_acc[_debug_counter_acc][3] = delay_us;

				_debug_data_acc[_debug_counter_acc][0] = (float)data_x * SCALE_Q(8);
				_debug_data_acc[_debug_counter_acc][1] = (float)data_y * SCALE_Q(8);
				_debug_data_acc[_debug_counter_acc][2] = (float)data_z * SCALE_Q(8);

				_debug_counter_acc++;
			} else if (!_saved_debug_acc)
			{
				saveCSV("/home/drone/px4/debugData/acc.csv", _debug_ts_acc, _debug_data_acc, _debug_counter_acc);
				PX4_INFO("SAVED ACC CSV!!!");
				_saved_debug_acc = true;	
			}

			_last_sample_ts_accel = sample_ts;
			_px4_accel.update(sample_ts, data_x, data_y, data_z);
			break;
		}

		case SENSOR_REPORTID_GYROSCOPE:
		{
			if (sample_ts <= _last_sample_ts_gyro) {
				PX4_WARN("Lower or equal gyro timestamp than before. Ignoring...");
				break;
			}
			if (_debug_counter_gyr < DEBUG_ROWS)
			{
				_debug_ts_gyr[_debug_counter_gyr][0] = sample_ts;
				_debug_ts_gyr[_debug_counter_gyr][1] = host_ts;
				_debug_ts_gyr[_debug_counter_gyr][2] = base_delta_us;
				_debug_ts_gyr[_debug_counter_gyr][3] = delay_us;

				_debug_data_gyr[_debug_counter_gyr][0] = (float)data_x * SCALE_Q(9);
				_debug_data_gyr[_debug_counter_gyr][1] = (float)data_y * SCALE_Q(9);
				_debug_data_gyr[_debug_counter_gyr][2] = (float)data_z * SCALE_Q(9);

				_debug_counter_gyr++;
			} else if (!_saved_debug_gyr)
			{
				saveCSV("/home/drone/px4/debugData/gyr.csv", _debug_ts_gyr, _debug_data_gyr, _debug_counter_gyr);
				PX4_INFO("SAVED GYR CSV!!!");
				_saved_debug_gyr = true;	
			}

			_last_sample_ts_gyro = sample_ts;
			_px4_gyro.update(sample_ts, data_x, data_y, data_z);
			break;
		}

		case SENSOR_REPORTID_MAGNETOMETER:
		{
			if (sample_ts <= _last_sample_ts_mag) {
				PX4_WARN("Lower or equal mag timestamp than before. Ignoring...");
				break;
			}
			if (_debug_counter_mag < DEBUG_ROWS)
			{
				_debug_ts_mag[_debug_counter_mag][0] = sample_ts;
				_debug_ts_mag[_debug_counter_mag][1] = host_ts;
				_debug_ts_mag[_debug_counter_mag][2] = base_delta_us;
				_debug_ts_mag[_debug_counter_mag][3] = delay_us;

				_debug_data_mag[_debug_counter_mag][0] = (float)data_x * SCALE_Q(4);
				_debug_data_mag[_debug_counter_mag][1] = (float)data_y * SCALE_Q(4);
				_debug_data_mag[_debug_counter_mag][2] = (float)data_z * SCALE_Q(4);

				_debug_counter_mag++;
			} else if (!_saved_debug_mag)
			{
				saveCSV("/home/drone/px4/debugData/mag.csv", _debug_ts_mag, _debug_data_mag, _debug_counter_mag);
				PX4_INFO("SAVED MAG CSV!!!");
				_saved_debug_mag = true;	
			}

			_last_sample_ts_mag = sample_ts;
			_px4_mag.update(sample_ts, data_x, data_y, data_z);
			break;
		}
	}

	return true;
}
