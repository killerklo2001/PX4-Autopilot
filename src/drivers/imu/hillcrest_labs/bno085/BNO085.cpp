
#include "BNO085.hpp"

using namespace time_literals;


BNO085::BNO085(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_pin_reset_gpio(29),
	_pin_wakeup_gpio(7),
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

	switch (_state) {
	case STATE::RESET:
	{
		PX4_INFO("BIN IM RESET");
		// Reset sequence
		px4_arch_gpiowrite(_pin_wakeup_gpio, true);
		px4_usleep(10 * 1000);

		px4_arch_gpiowrite(_pin_reset_gpio, false);
		px4_usleep(1 * 1000);
		px4_arch_gpiowrite(_pin_reset_gpio, true);

		break;
	}

	case STATE::WAIT_FOR_REBOOT:
	{
		// Boot handshake: wait for INT pin
		while (px4_arch_gpioread(_drdy_gpio) == true) { px4_usleep(1000); }
		while (px4_arch_gpioread(_drdy_gpio) == false) { px4_usleep(1000); }
		while (px4_arch_gpioread(_drdy_gpio) == true) { px4_usleep(1000); }
		while (px4_arch_gpioread(_drdy_gpio) == false) { px4_usleep(1000); }

		break;
	}

	case STATE::FLUSH_REBOOT_REPORTS:
	{
		// Flush initial boot reports
		uint8_t tx_dummy[24] {};  // nur Nullen
		uint8_t rx_dummy[24] {};
		for (int i = 0; i < 100; i++) {
			SPI::transfer(tx_dummy, rx_dummy, 20); // sendet 20 Bytes Dummy, liest 20 Bytes vom Sensor
		}
		DataReadyInterruptDisable();
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::CONFIGURE;
		ScheduleDelayed(30_ms);

		break;
	}

	case STATE::CONFIGURE:
	{
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::READ_REPORTS;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
			}

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

	case STATE::READ_REPORTS:
	{
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
}

void BNO085::WakeUp()
{
	px4_arch_gpiowrite(_pin_wakeup_gpio, false);
	px4_usleep(10 * 1000); // 10 ms
	px4_arch_gpiowrite(_pin_wakeup_gpio, true);
	px4_usleep(10 * 1000); // 10 ms
}

void BNO085::SetFeature(uint8_t feature_id, uint32_t report_interval_us)
{
	constexpr uint8_t CHANNEL_NUMBER = 2;

	CommandPacket tx_packet{};
	CommandPacket dummy_tx_packet{};
	CommandPacket rx_packet{};

	tx_packet.header.length_lsb = sizeof(Ch3Payload) & 0xFF;
	tx_packet.header.length_msb = (sizeof(Ch3Payload) >> 8) & 0xFF;
	tx_packet.header.channel = CHANNEL_NUMBER;
	tx_packet.header.ch_seq = (tx_packet.header.ch_seq + 1) & 0xFF;

	tx_packet.feature_control_payload.command_id = SHTP_REPORT_SET_FEATURE_COMMAND;
	tx_packet.feature_control_payload.report_id_feature = feature_id;
	tx_packet.feature_control_payload.report_interval_lsb = (report_interval_us >> 0) & 0xFF;
	tx_packet.feature_control_payload.report_interval_1   = (report_interval_us >> 8) & 0xFF;
	tx_packet.feature_control_payload.report_interval_2   = (report_interval_us >> 16) & 0xFF;
	tx_packet.feature_control_payload.report_interval_msb = (report_interval_us >> 24) & 0xFF;

	PX4_INFO("Sending 'SET FEATURE COMMAND' for Report ID: 0x%02X", feature_id);

	WakeUp();
	transfer(reinterpret_cast<uint8_t*>(&tx_packet), nullptr, sizeof(tx_packet));

	// wait for response
	uint16_t tries = 0;
	const uint16_t max_tries = 50;
	bool success = false;

	WakeUp();
	while (tries < max_tries && !success) {

		while (px4_arch_gpioread(_drdy_gpio)) {
			px4_usleep(1000);
		}

		transfer(reinterpret_cast<uint8_t*>(&dummy_tx_packet), reinterpret_cast<uint8_t*>(&rx_packet), sizeof(dummy_tx_packet));

		if (rx_packet.feature_control_payload.command_id == SHTP_REPORT_GET_FEATURE_RESPONSE) {
			PX4_INFO("Feature 0x%02X: got GET_FEATURE_RESPONSE", feature_id);
			success = true;
		}

		tries++;
	}

	if (!success) {
		PX4_WARN("BNO feature 0x%02X set failed after %u tries", feature_id, max_tries);
	}
}

bool BNO085::Configure()
{
	_px4_gyro.set_scale(SCALE_Q(9));
    	_px4_gyro.set_range(math::radians(2000.f));

	_px4_accel.set_scale(SCALE_Q(8));
	_px4_accel.set_range(8.f * CONSTANTS_ONE_G);

	SetFeature(SENSOR_REPORTID_ACCELEROMETER, SENSOR_SAMPLE_RATE);
	SetFeature(SENSOR_REPORTID_GYROSCOPE, SENSOR_SAMPLE_RATE);

	return true;
}

int BNO085::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<BNO085 *>(arg)->DataReady();
	return 0;
}

void BNO085::DataReady()
{
	ScheduleNow();
}

bool BNO085::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool BNO085::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool BNO085::ReadReport(const hrt_abstime &timestamp_sample)
{
	constexpr uint8_t CHANNEL_NUMBER = 3;

	Ch3Packet tx_packet{};
	Ch3Packet rx_packet{};

	tx_packet.header.length_lsb = sizeof(Ch3Payload) & 0xFF;
	tx_packet.header.length_msb = (sizeof(Ch3Payload) >> 8) & 0xFF;
	tx_packet.header.channel = CHANNEL_NUMBER;
	tx_packet.header.ch_seq = (tx_packet.header.ch_seq + 1) & 0xFF;

	transfer(reinterpret_cast<uint8_t*>(&tx_packet), reinterpret_cast<uint8_t*>(&rx_packet), sizeof(tx_packet));

	// check if timebase report
	if (rx_packet.ch3_payload.timebase_id != SHTP_REPORT_BASE_TIME) {
		PX4_WARN("Ignoring packet, not a Timebase report: 0x%02X", rx_packet.ch3_payload.timebase_id);
		return false;
	}

	int16_t data_x = (int16_t)((rx_packet.ch3_payload.data_x_lsb << 8) |
                                    rx_packet.ch3_payload.data_x_msb);
    	int16_t data_y = (int16_t)((rx_packet.ch3_payload.data_y_lsb << 8) |
                                    rx_packet.ch3_payload.data_y_msb);
    	int16_t data_z = (int16_t)((rx_packet.ch3_payload.data_z_lsb << 8) |
                               	    rx_packet.ch3_payload.data_z_msb);

	switch (rx_packet.ch3_payload.report_id) {

		case SENSOR_REPORTID_ACCELEROMETER:
		{
			sensor_accel_fifo_s accel{};
			accel.timestamp_sample = timestamp_sample;
			accel.samples = 1;
			accel.dt = SENSOR_SAMPLE_RATE;
			accel.x[0] = data_x;
			accel.y[0] = data_y;
			accel.z[0] = data_z;
			_px4_accel.updateFIFO(accel);
			break;
		}

		case SENSOR_REPORTID_GYROSCOPE:
		{
			sensor_gyro_fifo_s gyro{};
			gyro.timestamp_sample = timestamp_sample;
			gyro.samples = 1;
			gyro.dt = SENSOR_SAMPLE_RATE;
			gyro.x[0] = data_x;
			gyro.y[0] = data_y;
			gyro.z[0] = data_z;
			_px4_gyro.updateFIFO(gyro);
			break;
		}
	}

	return true;
}
