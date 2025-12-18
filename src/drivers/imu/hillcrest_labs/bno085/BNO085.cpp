
#include "BNO085.hpp"

#include <pigpiod_if2.h>

using namespace time_literals;

BNO085::BNO085(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
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
		PX4_INFO("IN RESET");
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
		PX4_INFO("IN WAIT_FOR_REBOOT");
		// Boot handshake: wait for INT pin
		ScheduleDelayed(1_ms);

		_state = STATE::FLUSH_REBOOT_REPORTS;
		break;
	}

	case STATE::FLUSH_REBOOT_REPORTS:
	{	
		PX4_INFO("IN FLUSH_REBOOT_REPORTS");
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
		PX4_INFO("IN CONFIGURE");
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
		PX4_INFO("IN SET_FEATURES");
        
		if (_set_feature_tries > 50) {
			PX4_DEBUG("Configure failed, resetting");
			_state = STATE::RESET;
		}

		if (!_accel_set) {
			if (_set_feature_tries == 0) {
				SetFeature(SENSOR_REPORTID_ACCELEROMETER, SENSOR_SAMPLE_PERIOD_US);
				_set_feature_tries++;
			}
			if (GetFeature(SENSOR_REPORTID_ACCELEROMETER, SENSOR_SAMPLE_PERIOD_US)) {
				_accel_set = true;
				_set_feature_tries = 0;
				ScheduleDelayed(4_s);
			} else {
				PX4_INFO("TRY %d FAILED!", _set_feature_tries);
				_set_feature_tries++;	
			}
		}
		else if (!_gyro_set) {
			if (_set_feature_tries == 0) {
				SetFeature(SENSOR_REPORTID_GYROSCOPE, SENSOR_SAMPLE_PERIOD_US);
				_set_feature_tries++;
			}
			if (GetFeature(SENSOR_REPORTID_GYROSCOPE, SENSOR_SAMPLE_PERIOD_US)) {
				_gyro_set = true;
				_set_feature_tries = 0;
				ScheduleDelayed(4_s);
			} else {
				PX4_INFO("TRY %d FAILED!", _set_feature_tries);
				_set_feature_tries++;
			}	
		}
		else {
			_state = STATE::READ_REPORTS;	
		}

		break;
	}

	case STATE::READ_REPORTS:
	{
		//PX4_INFO("IN READ_REPORTS");
		hrt_abstime timestamp_sample = 0;

		bool success = false;

		if (ReadReport((timestamp_sample == 0) ? now : timestamp_sample)) {
			success = true;

			if (_failure_count > 0) {
				_failure_count--;
			}
		}

		if (!success) {
			_failure_count++;

			// full reset if things are failing consistently
			if (_failure_count > 10) {
				return;
			}
		}

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
	

	//DEBUG
	const uint8_t *data = reinterpret_cast<const uint8_t *>(&tx_packet);
	const size_t len = sizeof(tx_packet);

	char line[256];
	int pos = 0;

	for (size_t i = 0; i < len && pos < (int)sizeof(line) - 3; i++) {
		pos += snprintf(&line[pos], sizeof(line) - pos, "%02X ", data[i]);
	}

	PX4_INFO("Packet: %s", line);
	//DEBUG END


	WakeUp();
	SPI::transfer(reinterpret_cast<uint8_t*>(&tx_packet), nullptr, sizeof(tx_packet));
}


bool BNO085::GetFeature(uint8_t feature_id, uint32_t report_interval_us)
{
	constexpr uint8_t CHANNEL_NUMBER = 2;
	CommandPacket dummy_tx_packet{};
	CommandPacket rx_packet{};

	SPI::transfer(reinterpret_cast<uint8_t*>(&dummy_tx_packet), reinterpret_cast<uint8_t*>(&rx_packet), sizeof(dummy_tx_packet));


	//DEBUG
	const uint8_t *data = reinterpret_cast<const uint8_t *>(&rx_packet);
	const size_t len = sizeof(dummy_tx_packet);

	char line[256];
	int pos = 0;

	for (size_t i = 0; i < len && pos < (int)sizeof(line) - 3; i++) {
		pos += snprintf(&line[pos], sizeof(line) - pos, "%02X ", data[i]);
	}

	PX4_INFO("RX-PACKET: %s", line);
	//DEBUG END

	PX4_INFO("Channel: 0x%02X", rx_packet.header.channel);

	if (rx_packet.header.channel != CHANNEL_NUMBER) {
		return false;
	} 

	PX4_INFO("Command: 0x%02X", rx_packet.feature_control_payload.command_id);

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
	_px4_gyro.set_scale(SCALE_Q(9));
    _px4_gyro.set_range(math::radians(2000.f));

	_px4_accel.set_scale(SCALE_Q(8));
	_px4_accel.set_range(8.f * CONSTANTS_ONE_G);

	return true;
}


void BNO085::DataReadyCallback(int pi, unsigned user_gpio, unsigned edge, uint32_t tick, void *userdata)

{
    if (edge == 0) {  // FALLING_EDGE
        auto *self = static_cast<BNO085 *>(userdata);
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


bool BNO085::ReadReport(const hrt_abstime &timestamp_sample)
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

	// check if timebase report
	if (rx_packet.ch3_payload.timebase_id != SHTP_REPORT_BASE_TIME) {
		PX4_WARN("Ignoring packet, not a Timebase report: 0x%02X", rx_packet.ch3_payload.timebase_id);
		return false;
	}

	// uint32_t delta_t_us = (rx_packet.ch3_payload.delta_t_msb << 24) |
    //                       (rx_packet.ch3_payload.delta_t_2   << 16) |
    //                       (rx_packet.ch3_payload.delta_t_1   << 8)  |
    //                       rx_packet.ch3_payload.delta_t_lsb;

	int16_t data_x = (int16_t)((rx_packet.ch3_payload.data_x_lsb << 8) |
                                rx_packet.ch3_payload.data_x_msb);
    int16_t data_y = (int16_t)((rx_packet.ch3_payload.data_y_lsb << 8) |
                                rx_packet.ch3_payload.data_y_msb);
    int16_t data_z = (int16_t)((rx_packet.ch3_payload.data_z_lsb << 8) |
                               	rx_packet.ch3_payload.data_z_msb);

	switch (rx_packet.ch3_payload.report_id) {
		
		case SENSOR_REPORTID_ACCELEROMETER:
		{
			_px4_accel.update(timestamp_sample, data_x, data_y, data_z);
			break;
		}

		case SENSOR_REPORTID_GYROSCOPE:
		{
			_px4_gyro.update(timestamp_sample, data_x, data_y, data_z);
			break;
		}

		// case SENSOR_REPORTID_ACCELEROMETER:
		// {
		// 	sensor_accel_fifo_s accel{};
		// 	accel.timestamp_sample = timestamp_sample;
		// 	accel.samples = 1;
		// 	accel.dt = SENSOR_SAMPLE_PERIOD_US  * 1e-6f;
		// 	accel.x[0] = data_x;
		// 	accel.y[0] = data_y;
		// 	accel.z[0] = data_z;
		// 	_px4_accel.update(accel);
		// 	break;
		// }

		// case SENSOR_REPORTID_GYROSCOPE:
		// {
		// 	sensor_gyro_fifo_s gyro{};
		// 	gyro.timestamp_sample = timestamp_sample;
		// 	gyro.samples = 1;
		// 	gyro.dt = SENSOR_SAMPLE_PERIOD_US  * 1e-6f;
		// 	gyro.x[0] = data_x;
		// 	gyro.y[0] = data_y;
		// 	gyro.z[0] = data_z;
		// 	_px4_gyro.update(gyro);
		// 	break;
		// }
	}


	return true;
}
