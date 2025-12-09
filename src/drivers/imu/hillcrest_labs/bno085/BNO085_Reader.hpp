
#pragma once

#include "BNO085.hpp"
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include "hillcrest_labs_BNO085_Gyroscope_Reports.hpp"

namespace hillcrest_labs::BNO085::Gyroscope
{

class BNO085_Reader : public BNO085
{
public:
	BNO085_Reader(const I2CSPIDriverConfig &config);
	~BNO085_Reader() override = default;

	void RunImpl() override;
	void print_status() override {}

private:
	void exit_and_cleanup() override;

	// Sensor configuration
	static constexpr uint32_t RATE{2000};     // 2000 Hz
	static constexpr float SAMPLE_DT{1e6f / RATE};

	int probe() override { return PX4_OK; }

	bool Configure();
	void ConfigureGyro();
	void ConfigureSampleRate(int sample_rate = 0) {}
	void ConfigureFIFOWatermark(uint8_t samples) {}

	// Feature configuration
	void SetFeature(uint8_t feature_id, uint32_t report_interval_us);

	// DRDY interrupt
	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	// Wake pin toggling
	void WakeUp();

	// SPI read
	bool Read(const hrt_abstime &timestamp_sample);

	PX4Gyroscope _px4_gyro;

	perf_counter_t _drdy_missed_perf{nullptr};

	// Reading buffer
	uint8_t rx_packet[64] {};

	// state machine
	enum class STATE : uint8_t {
		RESET,
		CONFIGURE,
		CONFIGURATION_COMPLETE,
		READ,
	};

	STATE _state{STATE::RESET};

	hrt_abstime _reset_timestamp{0};
	uint8_t _failure_count{0};

	uint8_t seq{0};
};

} // namespace hillcrest_labs::BNO085::Gyroscope
