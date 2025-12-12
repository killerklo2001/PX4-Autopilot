
#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include "HillcrestLabs_BNO085_reports.hpp"

using namespace HillcrestLabs_BNO085;

class BNO085 : public device::SPI, public I2CSPIDriver<BNO085>
{
public:
	BNO085(const I2CSPIDriverConfig &config);
	~BNO085() override;

	static void print_usage();
	void RunImpl();

	int init() override;
	void print_status() override;

private:

	void exit_and_cleanup() override;

	// Sensor configuration
	static constexpr float SENSOR_SAMPLE_RATE{1e6f / 200.0f}; // 200 Hz -> 5000 us

	int probe() override;

	void WakeUp();

	bool Reset();

	void SetFeature(uint8_t feature_id, uint32_t report_interval_us);

	bool Configure();

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	bool ReadReport(const hrt_abstime &timestamp_sample);

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	int32_t _drdy_count{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_REBOOT,
		FLUSH_REBOOT_REPORTS,
		CONFIGURE,
		READ_REPORTS,
	} _state{STATE::RESET};

};
