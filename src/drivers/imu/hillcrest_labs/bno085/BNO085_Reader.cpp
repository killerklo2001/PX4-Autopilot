
#include "BNO085_Reader.hpp"
#include <geo/geo.h>

using namespace time_literals;

namespace hillcrest_labs::BNO085::Reader
{

BNO085_Reader::BNO085_Reader(const I2CSPIDriverConfig &config) :
	BNO085(config),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (config.drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME"_gyro: DRDY missed");
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

void BNO085_Reader::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void BNO085_Reader::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// Reset sequence
		px4_arch_gpiowrite(_pin_wakeup_gpio, true);
		px4_usleep(10 * 1000);

		px4_arch_gpiowrite(_pin_reset_gpio, false);
		px4_usleep(1 * 1000);
		px4_arch_gpiowrite(_pin_reset_gpio, true);

		// Boot handshake: wait for INT pin
		hrt_abstime start = hrt_absolute_time();
		while (px4_arch_gpioread(_drdy_gpio) == true) { px4_usleep(1000); }
		while (px4_arch_gpioread(_drdy_gpio) == false) { px4_usleep(1000); }
		while (px4_arch_gpioread(_drdy_gpio) == true) { px4_usleep(1000); }
		while (px4_arch_gpioread(_drdy_gpio) == false) { px4_usleep(1000); }

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

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then reset the FIFO
			_state = STATE::CONFIGURATION_COMPLETE;
			ScheduleDelayed(10_ms);

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

	case STATE::CONFIGURATION_COMPLETE:
		// if configure succeeded then start reading from FIFO
		_state = STATE::READ;

		if (DataReadyInterruptConfigure()) {
			_data_ready_interrupt_enabled = true;

			// backup schedule as a watchdog timeout
			ScheduleDelayed(100_ms);

		} else {
			_data_ready_interrupt_enabled = false;
			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
		}

		break;

	case STATE::READ:
		hrt_abstime timestamp_sample = 0;

		if (_data_ready_interrupt_enabled) {
			// scheduled from interrupt if _drdy_timestamp_sample was set as expected
			const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

			if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
				timestamp_sample = drdy_timestamp_sample;

			} else {
				perf_count(_drdy_missed_perf);
			}

			// push backup schedule back
			ScheduleDelayed(_fifo_empty_interval_us * 2);
		}

		// always check current FIFO status/count
		bool success = false;

		if (Read((timestamp_sample == 0) ? now : timestamp_sample)) {
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

void BNO085_Reader::WakeUp()
{
	px4_arch_gpiowrite(_pin_wakeup_gpio, false);
	px4_usleep(10 * 1000); // 10 ms
	px4_arch_gpiowrite(_pin_wakeup_gpio, true);
	px4_usleep(10 * 1000); // 10 ms
}

void BNO085_Reader::SetFeature(uint8_t feature_id, uint32_t report_interval_us)
{
	constexpr uint8_t CHANNEL_NUMBER = 2;
	constexpr uint16_t PAYLOAD_LENGTH =
		static_cast<uint8_t>(feature_control_payload::SPECIFIC_CONFIG_MSB) + 1;

	constexpr size_t PACKET_LEN = 4 + PAYLOAD_LENGTH;

	uint8_t tx_packet[PACKET_LEN] {};
	uint8_t rx_packet[PACKET_LEN] {};

	// header
	tx_packet[(uint8_t)header::LENGTH_LSB] = PAYLOAD_LENGTH & 0xFF;
	tx_packet[(uint8_t)header::LENGTH_MSB] = (PAYLOAD_LENGTH >> 8) & 0xFF;
	tx_packet[(uint8_t)header::CHANNEL]    = CHANNEL_NUMBER;
	tx_packet[(uint8_t)header::CH_SEQ]     = seq++ & 0xFF;

	// feature command
	tx_packet[4 + (uint8_t)feature_control_payload::COMMAND_ID]        = SHTP_REPORT_SET_FEATURE_COMMAND;
	tx_packet[4 + (uint8_t)feature_control_payload::REPORT_ID_FEATURE] = feature_id;

	tx_packet[4 + (uint8_t)feature_control_payload::FLAGS] = 0;

	tx_packet[4 + (uint8_t)feature_control_payload::CHANGE_SENSITIVITY_LSB] = 0;
	tx_packet[4 + (uint8_t)feature_control_payload::CHANGE_SENSITIVITY_MSB] = 0;

	tx_packet[4 + (uint8_t)feature_control_payload::REPORT_INTERVAL_LSB] = (report_interval_us >> 0) & 0xFF;
	tx_packet[4 + (uint8_t)feature_control_payload::REPORT_INTERVAL_1]   = (report_interval_us >> 8) & 0xFF;
	tx_packet[4 + (uint8_t)feature_control_payload::REPORT_INTERVAL_2]   = (report_interval_us >> 16) & 0xFF;
	tx_packet[4 + (uint8_t)feature_control_payload::REPORT_INTERVAL_MSB] = (report_interval_us >> 24) & 0xFF;

	PX4_INFO("Sending 'SET FEATURE COMMAND' for Report ID: 0x%02X", feature_id);

	WakeUp();
	transfer(tx_packet, rx_packet, PACKET_LEN);

	uint16_t tries = 0;
	const uint16_t max_tries = 50;
	bool success = false;

	WakeUp();
	while (tries < max_tries && !success) {

		while (px4_arch_gpioread(_drdy_gpio)) {
			px4_usleep(1000);
		}

		uint8_t response[PACKET_LEN] {};
		transfer(nullptr, response, PACKET_LEN);

		const uint8_t cmd_id =
			response[4 + (uint8_t)feature_control_payload::COMMAND_ID];

		if (cmd_id == SHTP_REPORT_GET_FEATURE_RESPONSE) {
			PX4_INFO("Feature 0x%02X: got GET_FEATURE_RESPONSE", feature_id);
			success = true;
		}

		tries++;
	}

	if (!success) {
		PX4_WARN("BNO feature 0x%02X set failed after %u tries", feature_id, max_tries);
	}
}


void BNO085_Reader::Configure()
{
	_px4_gyro.set_scale(SCALE_Q(9));
    	_px4_gyro.set_range(math::radians(2000.f));

	_px4_accel.set_scale(SCALE_Q(8));
	_px4_accel.set_range(8.f * CONSTANTS_ONE_G);

	SetFeature(SENSOR_REPORTID_ACCELEROMETER, SAMPLE_DT);
	SetFeature(SENSOR_REPORTID_GYROSCOPE, SAMPLE_DT);
}

int BNO085_Reader::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<BNO085_Reader *>(arg)->DataReady();
	return 0;
}

void BNO085_Reader::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool BNO085_Reader::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool BNO085_Reader::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool BNO085_Reader::Read(const hrt_abstime &timestamp_sample)
{
	constexpr uint8_t CHANNEL_NUMBER = 2;
	constexpr uint16_t PAYLOAD_LENGTH = static_cast<uint8_t>(ch3_payload::SPECIFIC_CONFIG_MSB) + 1;

	// build and send dummy packet to read
	uint8_t tx_packet[4 + PAYLOAD_LENGTH] = {0};

	tx_packet[(uint8_t)header::LENGTH_LSB] = PAYLOAD_LENGTH & 0xFF;
	tx_packet[(uint8_t)header::LENGTH_MSB] = (PAYLOAD_LENGTH >> 8) & 0xFF;
	tx_packet[(uint8_t)header::CHANNEL]    = CHANNEL_NUMBER;
	tx_packet[(uint8_t)header::CH_SEQ]     = seq++ & 0xFF;
	tx_packet[4 + (uint8_t)ch3_payload::REPORT_ID] = 0;

	transfer(tx_packet, rx_packet, sizeof(tx_packet));
	const uint8_t* rx_payload = rx_packet + 4;

	// check if timebase report
	if (rx_payload[(uint8_t)ch3_payload::REPORT_ID_T] != SHTP_REPORT_BASE_TIMESTAMP) {
		PX4_WARN("Ignoring packet, not a Timebase report: 0x%02X", rx_payload[(uint8_t)ch3_payload::REPORT_ID_T]);
		return false;
	}

	int16_t data_x = (int16_t)((rx_payload[(uint8_t)ch3_payload::DATA_X_LSB] << 8) |
                               rx_payload[(uint8_t)ch3_payload::DATA_X_MSB]);
    	int16_t data_y = (int16_t)((rx_payload[(uint8_t)ch3_payload::DATA_Y_LSB] << 8) |
                               rx_payload[(uint8_t)ch3_payload::DATA_Y_MSB]);
    	int16_t data_z = (int16_t)((rx_payload[(uint8_t)ch3_payload::DATA_Z_LSB] << 8) |
                               rx_payload[(uint8_t)ch3_payload::DATA_Z_MSB]);

	switch (rx_payload[(uint8_t)ch3_payload::REPORT_ID]) {

		case SENSOR_REPORTID_ACCELEROMETER: {
			sensor_accel_fifo_s accel{};
			accel.timestamp_sample = timestamp_sample;
			accel.samples = 1;
			accel.dt = SAMPLE_DT;
			accel.x[0] = data_x;
			accel.y[0] = data_y;
			accel.z[0] = data_z;
			_px4_accel.updateFIFO(accel);
			break;
		}

		case SENSOR_REPORTID_GYROSCOPE: {
			sensor_gyro_fifo_s gyro{};
			gyro.timestamp_sample = timestamp_sample;
			gyro.samples = 1;
			gyro.dt = SAMPLE_DT;
			gyro.x[0] = data_x;
			gyro.y[0] = data_y;
			gyro.z[0] = data_z;
			_px4_gyro.updateFIFO(gyro);
			break;
		}
	}

	return true;
}

} // namespace hillcrest_labs::BNO085::Reader
