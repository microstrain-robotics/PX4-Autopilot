/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <lib/drivers/device/Device.hpp>

#include "MicroStrain.hpp"

ModalIoSerial device_uart;

MicroStrain::MicroStrain(const char *uart_port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	// Store port name
	memset(_port, '\0', sizeof(_port));
	const size_t max_len = math::min(sizeof(_port), strnlen(uart_port, sizeof(_port)));
	strncpy(_port, uart_port, max_len);

	// Enforce null termination
	_port[max_len] = '\0';

	// The scheduling rate (in microseconds) is set higher than the highest sensor data rate (in Hz)
	const int max_param_rate = math::max(_param_ms_imu_rate_hz.get(), _param_ms_mag_rate_hz.get(),
					     _param_ms_baro_rate_hz.get());

	// We always want the schedule rate faster than the highest sensor data rate
	_ms_schedule_rate_us = (1e6) / (2 * max_param_rate);

	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_INS_DEVTYPE_MICROSTRAIN;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.bus = 2;
	_dev_id = device_id.devid;

	_px4_accel.set_device_id(_dev_id);
	_px4_gyro.set_device_id(_dev_id);
	_px4_mag.set_device_id(_dev_id);

	_sensor_baro.device_id = _dev_id;
	_sensor_baro.pressure = 0;
	_sensor_baro.temperature = 0;
	_sensor_baro.error_count = 0;

	_ms_mode = _param_ms_mode.get();

	// MS_MODE 1 full INS
	// if (_ms_mode == 1) {
	// 	int32_t m = 0;

	// 	// EKF2_EN 0 (disabled)
	// 	m = 0;
	// 	param_set(param_find("EKF2_EN"), &m);

	// 	// SENS_IMU_MODE (VN handles sensor selection)
	// 	m = 0;
	// 	param_set(param_find("SENS_IMU_MODE"), &m);

	// 	// SENS_MAG_MODE (VN handles sensor selection)
	// 	m = 0;
	// 	param_set(param_find("SENS_MAG_MODE"), &m);
	// }
}

MicroStrain::~MicroStrain()
{
	if (device_uart.isOpen()) {
		device_uart.uartClose();
	}

	PX4_DEBUG("Destructor");
	_sensor_baro_pub.unadvertise();

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool mipInterfaceUserRecvFromDevice(mip_interface *device, uint8_t *buffer, size_t max_length,
				    timeout_type wait_time,
				    size_t *out_length, timestamp_type *timestamp_out)
{
	(void)device;

	*timestamp_out = hrt_absolute_time();

	int res = device_uart.uartRead(buffer, max_length);

	if (res == -1 && errno != EAGAIN) {
		PX4_ERR("MicroStrain driver failed to read(%d): %s", errno, strerror(errno));
		*out_length = 0;
		return false;
	}

	if (res >= 0) {
		*out_length = res;
		PX4_DEBUG("Number of bytes read %d(%d)", *out_length, max_length);
	}

	return true;
}

bool mipInterfaceUserSendToDevice(mip_interface *device, const uint8_t *data, size_t length)
{

	int res = device_uart.uartWrite(const_cast<uint8_t *>(data), length);

	if (res >= 0) {
		return true;
	}

	PX4_ERR("MicroStrain driver failed to write(%d): %s", errno, strerror(errno));
	return false;

}

mip::CmdResult MicroStrain::forceIdle()
{
	// Setting to idle may fail the first couple times, so call it a few times in case the device is streaming too much data
	mip::CmdResult result;
	uint8_t set_to_idle_tries = 0;

	while (set_to_idle_tries++ < 3) {
		if (!!(result = mip_base_set_idle(&_device))) {
			break;

		} else {
			usleep(1_s);
		}
	}

	return result;
}

int MicroStrain::connectAtBaud(int32_t baud)
{
	if (device_uart.isOpen()) {
		if (device_uart.uartSetBaud(baud) == PX4_ERROR) {
			PX4_INFO(" - Failed to set UART %" PRIu32 " baud", baud);
		}

	} else if (device_uart.uartOpen(_port, baud) == PX4_ERROR) {
		PX4_INFO(" - Failed to open UART");
		PX4_ERR("ERROR: Could not open device port!");
		return PX4_ERROR;
	}

	PX4_INFO("Serial Port %s with baud of %" PRIu32 " baud", (device_uart.isOpen() ? "CONNECTED" : "NOT CONNECTED"), baud);

	// Re-init the interface with the correct timeouts
	mip_interface_init(&_device, _parse_buffer, sizeof(_parse_buffer), mip_timeout_from_baudrate(baud) * 1_ms, 250_ms,
			   &mipInterfaceUserSendToDevice, &mipInterfaceUserRecvFromDevice, &mip_interface_default_update, NULL);

	if (!(forceIdle())) {
		return PX4_ERROR;
	}

	PX4_INFO("Successfully opened and pinged");
	return PX4_OK;
}

mip_cmd_result MicroStrain::getSupportedDescriptors()
{
	const size_t descriptors_max_size = sizeof(_supported_descriptors) / sizeof(_supported_descriptors[0]);
	uint8_t descriptors_count, extended_descriptors_count;

	// Pull the descriptors and the extended descriptors from the device
	mip_cmd_result res = mip_base_get_device_descriptors(&_device, _supported_descriptors, descriptors_max_size,
			     &descriptors_count);
	mip_cmd_result res_extended = mip_base_get_extended_descriptors(&_device, &(_supported_descriptors[descriptors_count]),
				      descriptors_max_size - descriptors_count, &extended_descriptors_count);

	if (res != MIP_ACK_OK) {
		return res;
	}

	if (res_extended != MIP_ACK_OK) {
		PX4_DEBUG(node_, "Device does not appear to support the extended descriptors command.");
	}

	_supported_desc_len = descriptors_count + extended_descriptors_count;

	// Get the supported descriptor sets from the obtained descriptors
	for (uint16_t i = 0; i < _supported_desc_len; i++) {
		auto descriptor_set = static_cast<uint8_t>((_supported_descriptors[i] & 0xFF00) >> 8);
		bool unique = true;

		uint16_t j;

		for (j = 0; j < _supported_desc_set_len; j++) {
			if (_supported_descriptor_sets[j] == descriptor_set) {
				unique = false;
				break;

			} else if (_supported_descriptor_sets[j] == 0) {
				break;
			}
		}

		if (unique) {_supported_descriptor_sets[j] = descriptor_set; _supported_desc_set_len++;}
	}

	return res;
}

bool MicroStrain::supportsDescriptorSet(uint8_t descriptor_set)
{
	for (uint16_t i = 0; i < _supported_desc_set_len; i++) {
		if (_supported_descriptor_sets[i] == descriptor_set) {
			return true;
		}
	}

	return false;
}

bool MicroStrain::supportsDescriptor(uint8_t descriptor_set, uint8_t field_descriptor)
{
	// Check if the descriptor set is supported
	if (!supportsDescriptorSet(descriptor_set)) {return false;}

	// Check if the field descriptor is supported
	const uint16_t full_descriptor = (descriptor_set << 8) | field_descriptor;

	for (uint16_t i = 0; i < _supported_desc_len; i++) {
		if (_supported_descriptors[i] == full_descriptor) {
			return true;
		}
	}

	return false;
}

mip_cmd_result MicroStrain::getBaseRate(uint8_t descriptor_set, uint16_t *base_rate)
{
	// If the device supports the mip_3dm_get_base_rate command, use that one, otherwise use the specific function
	if (supportsDescriptor(MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GET_BASE_RATE)) {
		return mip_3dm_get_base_rate(&_device, descriptor_set, base_rate);

	} else {
		switch (descriptor_set) {
		case MIP_SENSOR_DATA_DESC_SET:
			return mip_3dm_imu_get_base_rate(&_device, base_rate);

		case MIP_GNSS_DATA_DESC_SET:
			return mip_3dm_gps_get_base_rate(&_device, base_rate);

		case MIP_FILTER_DATA_DESC_SET:
			return mip_3dm_filter_get_base_rate(&_device, base_rate);

		default:
			return MIP_NACK_INVALID_PARAM;
		}
	}

}

mip_cmd_result MicroStrain::writeMessageFormat(uint8_t descriptor_set, uint8_t num_descriptors,
		const mip::DescriptorRate *descriptors)
{
	if (supportsDescriptor(MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT)) {
		return mip_3dm_write_message_format(&_device, descriptor_set, num_descriptors,
						    descriptors);

	} else {
		switch (descriptor_set) {
		case MIP_SENSOR_DATA_DESC_SET:
			return mip_3dm_write_imu_message_format(&_device, num_descriptors, descriptors);

		case MIP_GNSS_DATA_DESC_SET:
			return mip_3dm_write_gps_message_format(&_device, num_descriptors, descriptors);

		case MIP_FILTER_DATA_DESC_SET:
			return mip_3dm_write_filter_message_format(&_device, num_descriptors, descriptors);

		default:
			return MIP_NACK_INVALID_PARAM;
		}
	}
}

mip_cmd_result MicroStrain::configureImuMessageFormat()
{
	uint8_t num_imu_descriptors = 0;
	mip_descriptor_rate imu_descriptors[4];
	PX4_INFO("Start2 %d", sizeof(imu_descriptors));

	// Get the base rate
	uint16_t base_rate;
	mip_cmd_result res = getBaseRate(MIP_SENSOR_DATA_DESC_SET, &base_rate);

	PX4_INFO("The sensor base rate is %d", base_rate);

	if (res != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not get the base rate");
		return res;
	}

	// Configure the Message Format depending on if the device supports the descriptor
	uint16_t imu_decimation = base_rate / (uint16_t)_param_ms_imu_rate_hz.get();
	uint16_t mag_decimation = base_rate / (uint16_t)_param_ms_mag_rate_hz.get();
	uint16_t baro_decimation = base_rate / (uint16_t)_param_ms_baro_rate_hz.get();

	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_ACCEL_SCALED)
	    && _param_ms_imu_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_ACCEL_SCALED, imu_decimation};
	}

	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_GYRO_SCALED)
	    && _param_ms_imu_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_GYRO_SCALED, imu_decimation};
	}

	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_MAG_SCALED)
	    && _param_ms_mag_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_MAG_SCALED, mag_decimation};
	}

	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_PRESSURE_SCALED)
	    && _param_ms_baro_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_PRESSURE_SCALED, baro_decimation};
	}

	PX4_INFO("End2 %d", sizeof(imu_descriptors));

	// Write the settings
	res = writeMessageFormat(MIP_SENSOR_DATA_DESC_SET, num_imu_descriptors,
				 imu_descriptors);

	return res;

}

mip_cmd_result MicroStrain::configureFilterMessageFormat()
{
	uint8_t num_filter_descriptors = 0;
	mip_descriptor_rate filter_descriptors[11];

	PX4_INFO("Start %d", sizeof(filter_descriptors));

	// Get the base rate
	uint16_t base_rate;
	mip_cmd_result res = getBaseRate(MIP_FILTER_DATA_DESC_SET, &base_rate);

	PX4_INFO("The filter base rate is %d", base_rate);

	if (res != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not get the base rate");
		return res;
	}

	// Configure the Message Format depending on if the device supports the descriptor
	uint16_t filter_decimation1 = base_rate / (uint16_t)_param_ms_filter_rate1_hz.get();
	uint16_t filter_decimation2 = base_rate / (uint16_t)_param_ms_filter_rate2_hz.get();
	uint16_t filter_decimation3 = base_rate / (uint16_t)_param_ms_filter_rate3_hz.get();

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_POS_LLH)
	    && _param_ms_filter_rate1_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_POS_LLH, filter_decimation1};
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_ATT_QUATERNION)
	    && _param_ms_filter_rate1_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_ATT_QUATERNION, filter_decimation1};
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_VEL_NED)
	    && _param_ms_filter_rate1_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_VEL_NED, filter_decimation1};
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_REL_POS_NED)
	    && _param_ms_filter_rate1_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_REL_POS_NED, filter_decimation1};
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION)
	    && _param_ms_filter_rate2_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION, filter_decimation2};
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_POS_UNCERTAINTY)
	    && _param_ms_filter_rate1_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_POS_UNCERTAINTY, filter_decimation1};
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY)
	    && _param_ms_filter_rate1_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY, filter_decimation1};
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER)
	    && _param_ms_filter_rate1_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER, filter_decimation1};
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE)
	    && _param_ms_filter_rate3_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE, filter_decimation3};
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_FILTER_STATUS)
	    && _param_ms_filter_rate3_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_FILTER_STATUS, filter_decimation3};
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_AID_MEAS_SUMMARY)
	    && _param_ms_filter_rate3_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_AID_MEAS_SUMMARY, filter_decimation3};
	}

	PX4_INFO("Hi: %d, %d, %d, %d, %d", num_filter_descriptors, filter_decimation1, filter_decimation2, filter_decimation3,
		 sizeof(filter_descriptors));

	// Write the settings
	res = writeMessageFormat(MIP_FILTER_DATA_DESC_SET, num_filter_descriptors,
				 filter_descriptors);

	return res;

}

mip_cmd_result MicroStrain::writeBaudRate(uint32_t baudrate, uint8_t port)
{
	if (supportsDescriptor(MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED)) {
		return mip_base_write_comm_speed(&_device, port, baudrate);

	}

	return mip_3dm_write_uart_baudrate(&_device, baudrate);

}

bool MicroStrain::initializeIns()
{
	const uint32_t DESIRED_BAUDRATE = 921600;

	static constexpr uint32_t BAUDRATES[] {115200, 921600, 460800, 230400, 128000, 38400, 19200, 57600, 9600};
	bool is_connected = false;

	for (auto &baudrate : BAUDRATES) {
		if (connectAtBaud(baudrate) == PX4_OK) {
			PX4_INFO("found baudrate %" PRIu32, baudrate);
			is_connected = true;
			break;
		}
	}

	if (!is_connected) {
		PX4_ERR("Could not connect to the device, exiting");
		return false;
	}

	// Setting the device baudrate to the desired value
	PX4_INFO("Setting the baud to desired baud rate");

	if (writeBaudRate(DESIRED_BAUDRATE, 1) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set the baudrate!");
		return false;
	}

	tcflush(device_uart.uartGetFd(), TCIOFLUSH);

	// Connecting using the desired baudrate
	if (connectAtBaud(DESIRED_BAUDRATE) != PX4_OK) {
		PX4_INFO("ERROR: Could not Connect at %lu", DESIRED_BAUDRATE);
		return false;
	}

	// Get the supported descriptors for the device in use
	if (getSupportedDescriptors() != MIP_ACK_OK) {
		PX4_INFO("ERROR: Could not get descriptors");
		return false;
	}

	// Configure the IMU message formt based on what descriptors are supported
	if (configureImuMessageFormat() != MIP_ACK_OK) {
		PX4_INFO("ERROR: Could not write message format");
		return false;
	}

	// Register data callbacks
	mip_interface_register_packet_callback(&_device, &_sensor_data_handler, MIP_SENSOR_DATA_DESC_SET, false,
					       &sensorCallback,
					       this);

	if (_ms_mode == 1) {

		mip_aiding_frame_config_command_rotation rotation{0};
		uint8_t gnss_antenna_sensor_id = 2;

		for (uint8_t i = 0; i < 3; i++) {
			rotation.euler[i] = 0.0;
		}

		float translation[3] = { 0.0F, 0.0F, 0.0F };
		mip_aiding_write_frame_config(&_device, gnss_antenna_sensor_id, MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER, false,
					      translation, &rotation);

		if (configureFilterMessageFormat() != MIP_ACK_OK) {
			PX4_INFO("ERROR: Could not write message format");
			return false;
		}

		// Register data callbacks
		mip_interface_register_packet_callback(&_device, &_filter_data_handler, MIP_FILTER_DATA_DESC_SET, false,
						       &filterCallback,
						       this);

		// Selectively turn on the mag aiding source
		if (_param_cv7_int_mag_en.get() == 1) {
			if (mip_filter_write_aiding_measurement_enable(&_device,
					MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER, true) != MIP_ACK_OK) {
				PX4_ERR("ERROR: Could not set filter aiding measurement enable!");
			}
		}

		else {
			if (mip_filter_write_aiding_measurement_enable(&_device,
					MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER, false) == MIP_ACK_OK) {
				PX4_INFO("Mag Disabled");
			}
		}

		if (mip_filter_write_aiding_measurement_enable(&_device,
				MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL, true) != MIP_ACK_OK) {
			PX4_ERR("ERROR: Could not set filter aiding measurement enable!");
		}

		// Reset the filter, then set the initial conditions
		if (mip_filter_reset(&_device) != MIP_ACK_OK) {
			PX4_ERR("ERROR: Could not reset the filter!");
		}


		float filter_init_pos[3] = {0};
		float filter_init_vel[3] = {0};
		uint8_t initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_MAGNETOMETER;

		if (_param_cv7_alignment.get() == 1) {
			initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_KINEMATIC;
		}

		if (_param_cv7_alignment.get() == 2) {
			initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_EXTERNAL;
		}

		if (_param_cv7_alignment.get() == 3) {
			initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_DUAL_ANTENNA;
		}

		// Config for Position, Velocity and Attitude, use kinematic alignment for initialization
		if (mip_filter_write_initialization_configuration(&_device, 0,
				MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL_ATT,
				initial_alignment,
				0.0, 0.0, 0.0, filter_init_pos, filter_init_vel, MIP_FILTER_REFERENCE_FRAME_LLH) != MIP_ACK_OK) {
			PX4_ERR("ERROR: Could not set filter initialization configuration!");
		}


		//
		// Setup the rotation based on PX4 standard rotation sets
		//

		if (mip_3dm_write_sensor_2_vehicle_transform_euler(&_device, 0, 0, 0) != MIP_ACK_OK) {
			PX4_ERR("ERROR: Could not set sensor-to-vehicle transformation!");
			return false;
		}
	}

	if (mip_3dm_write_datastream_control(&_device, MIP_3DM_DATASTREAM_CONTROL_COMMAND_ALL_STREAMS, true) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not enable the data stream");
		return false;
	}

	// Resume the device
	if (mip_base_resume(&_device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not resume the device!");
		return false;
	}

	return true;

}

void MicroStrain::sensorCallback(void *user, const mip_packet *packet, mip::Timestamp timestamp)
{
	MicroStrain *ref = static_cast<MicroStrain *>(user);

	assert(mip_packet_descriptor_set(packet) == MIP_SENSOR_DATA_DESC_SET);

	mip_sensor_scaled_accel_data accel;
	mip_sensor_scaled_gyro_data gyro;
	mip_sensor_scaled_mag_data mag;
	mip_sensor_scaled_pressure_data baro;

	bool accel_valid = false;
	bool gyro_valid = false;
	bool mag_valid = false;
	bool baro_valid = false;

	// Iterate through the packet and extract based on the descriptor present
	auto t = hrt_absolute_time();

	for (mip_field field = mip_field_first_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field)) {
		switch (mip_field_field_descriptor(&field)) {

		case MIP_DATA_DESC_SENSOR_ACCEL_SCALED:
			extract_mip_sensor_scaled_accel_data_from_field(&field, &accel);
			accel_valid = true;
			break;


		case MIP_DATA_DESC_SENSOR_GYRO_SCALED:
			extract_mip_sensor_scaled_gyro_data_from_field(&field, &gyro);
			gyro_valid = true;
			break;


		case MIP_DATA_DESC_SENSOR_MAG_SCALED:
			extract_mip_sensor_scaled_mag_data_from_field(&field, &mag);
			mag_valid = true;
			break;


		case MIP_DATA_DESC_SENSOR_PRESSURE_SCALED:
			extract_mip_sensor_scaled_pressure_data_from_field(&field, &baro);
			baro_valid = true;
			break;


		default:
			break;


		}
	}

	// Publish only if the corresponding data was extracted from the packet
	if (accel_valid) {
		ref->_px4_accel.update(t, accel.scaled_accel[0]*CONSTANTS_ONE_G,
				       accel.scaled_accel[1]*CONSTANTS_ONE_G,
				       accel.scaled_accel[2]*CONSTANTS_ONE_G);

	}

	if (gyro_valid) {
		ref->_px4_gyro.update(t, gyro.scaled_gyro[0],
				      gyro.scaled_gyro[1],
				      gyro.scaled_gyro[2]);
	}

	if (mag_valid) {
		ref->_px4_mag.update(t, mag.scaled_mag[0],
				     mag.scaled_mag[1],
				     mag.scaled_mag[2]);
	}

	if (baro_valid) {
		ref->_sensor_baro.timestamp = timestamp;
		ref->_sensor_baro.timestamp_sample = t;
		ref->_sensor_baro.pressure = baro.scaled_pressure * 100.f; // convert [Pa] to [mBar]
		ref->_sensor_baro_pub.publish(ref->_sensor_baro);

	}

}

void MicroStrain::filterCallback(void *user, const mip_packet *packet, mip::Timestamp timestamp)
{
	MicroStrain *ref = static_cast<MicroStrain *>(user);

	assert(mip_packet_descriptor_set(packet) == MIP_FILTER_DATA_DESC_SET);

	mip_filter_position_llh_data pos_llh;
	mip_filter_attitude_quaternion_data att_quat;
	mip_filter_comp_angular_rate_data ang_rate;
	mip_filter_rel_pos_ned_data rel_pos;
	mip_filter_velocity_ned_data vel_ned;
	mip_filter_status_data stat;
	mip_filter_position_llh_uncertainty_data llh_uncert;
	mip_filter_velocity_ned_uncertainty_data vel_uncert;
	mip_filter_euler_angles_uncertainty_data att_euler_uncert;
	mip_filter_linear_accel_data lin_accel;
	mip_filter_aiding_measurement_summary_data aid_sum;

	bool updated_data[11] = {false};
	auto t = hrt_absolute_time();
	float yaw = NAN;

	for (mip_field field = mip_field_first_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field)) {
		switch (mip_field_field_descriptor(&field)) {
		case MIP_DATA_DESC_FILTER_POS_LLH: extract_mip_filter_position_llh_data_from_field(&field, &pos_llh);
			updated_data[0] = true; break;

		case MIP_DATA_DESC_FILTER_ATT_QUATERNION: extract_mip_filter_attitude_quaternion_data_from_field(&field, &att_quat);
			updated_data[1] = true; break;

		case MIP_DATA_DESC_FILTER_VEL_NED: extract_mip_filter_velocity_ned_data_from_field(&field, &vel_ned);
			updated_data[2] = true ; break;

		case MIP_DATA_DESC_FILTER_REL_POS_NED: extract_mip_filter_rel_pos_ned_data_from_field(&field, &rel_pos);
			updated_data[3] = true; break;

		case MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION: extract_mip_filter_linear_accel_data_from_field(&field, &lin_accel);
			updated_data[4] = true; break;

		case MIP_DATA_DESC_FILTER_POS_UNCERTAINTY: extract_mip_filter_position_llh_uncertainty_data_from_field(&field,
					&llh_uncert); updated_data[5] = true; break;

		case MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY: extract_mip_filter_velocity_ned_uncertainty_data_from_field(&field,
					&vel_uncert); updated_data[6] = true; break;

		case MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER: extract_mip_filter_euler_angles_uncertainty_data_from_field(&field,
					&att_euler_uncert); updated_data[7] = true; break;

		case MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE: extract_mip_filter_comp_angular_rate_data_from_field(&field,
					&ang_rate); updated_data[8] = true; break;

		case MIP_DATA_DESC_FILTER_FILTER_STATUS: extract_mip_filter_status_data_from_field(&field, &stat);
			updated_data[9] = true; break;

		case MIP_DATA_DESC_FILTER_AID_MEAS_SUMMARY: extract_mip_filter_aiding_measurement_summary_data_from_field(&field,
					&aid_sum); updated_data[10] = true; break;

		default: break;
		}
	}

	bool vgp_valid = updated_data[0];
	bool va_valid = updated_data[1];
	bool vlp_valid = va_valid && updated_data[2] && updated_data[3] && updated_data[4] && updated_data[5]
			 && updated_data[6];
	bool vo_valid = updated_data[2] && updated_data[3] && updated_data[5] && updated_data[6] && updated_data[7];
	bool vav_valid = updated_data[8];
	bool debug_valid = updated_data[9] && updated_data[10];

	if (vgp_valid) {
		vehicle_global_position_s gp{0};
		gp.timestamp = t;
		gp.timestamp_sample = t;
		gp.lat = pos_llh.latitude;
		gp.lon = pos_llh.longitude;
		gp.alt_ellipsoid = pos_llh.ellipsoid_height;
		gp.alt = pos_llh.ellipsoid_height;
		gp.eph = 0.1f;
		gp.epv = 0.1f;
		ref->_global_position_pub.publish(gp);
	}

	if (va_valid) {
		vehicle_attitude_s att_data{0};
		att_data.timestamp = t;
		att_data.timestamp_sample = t;
		att_data.q[0] = att_quat.q[0];
		att_data.q[1] = att_quat.q[1];
		att_data.q[2] = att_quat.q[2];
		att_data.q[3] = att_quat.q[3];
		att_data.quat_reset_counter = 0;
		ref->_vehicle_attitude_pub.publish(att_data);
		// convert to YAW
		matrix::Eulerf euler_attitude(matrix::Quatf(att_data.q));
		yaw = euler_attitude.psi();
	}

	if (vlp_valid) {
		vehicle_local_position_s vp{0};
		vp.timestamp = t;
		vp.timestamp_sample = t;
		vp.x = rel_pos.relative_position[0];
		vp.y = rel_pos.relative_position[1];
		vp.z = rel_pos.relative_position[2];
		vp.vx = vel_ned.north;
		vp.vy = vel_ned.east;
		vp.vz = vel_ned.down;
		vp.ax = lin_accel.accel[0];
		vp.ay = lin_accel.accel[1];
		vp.az = lin_accel.accel[2];
		vp.eph = sqrt(llh_uncert.north * llh_uncert.north + llh_uncert.east * llh_uncert.east);
		vp.epv = llh_uncert.down;
		vp.evh = sqrt(vel_uncert.north * vel_uncert.north + vel_uncert.east * vel_uncert.east);
		vp.evv = vel_uncert.down;
		vp.delta_xy[0] = 0.f;
		vp.delta_xy[1] = 0.f;
		vp.xy_reset_counter = 0;
		vp.z_valid = true;
		vp.xy_valid = true;
		vp.v_xy_valid = true;
		vp.v_z_valid = true;
		vp.z_deriv = vp.vz;
		vp.z_global = true;
		vp.xy_global = true;
		// Set heading as valid if we have any yaw
		vp.heading_good_for_control = (yaw != NAN);
		vp.heading = yaw; // This is extracted from vehicle attitude
		vp.vxy_max = INFINITY;
		vp.vz_max = INFINITY;
		vp.hagl_min = INFINITY;
		vp.hagl_max = INFINITY;
		ref->_vehicle_local_position_pub.publish(vp);
	}

	if (vo_valid) {
		vehicle_odometry_s vo{0};
		vo.timestamp = t;
		vo.timestamp_sample = t;
		vo.position[0] = rel_pos.relative_position[0];
		vo.position[1] = rel_pos.relative_position[1];
		vo.position[2] = rel_pos.relative_position[2];
		vo.velocity[0] = vel_ned.north;
		vo.velocity[1] = vel_ned.east;
		vo.velocity[2] = vel_ned.down;
		vo.position_variance[0] = llh_uncert.north;
		vo.position_variance[1] = llh_uncert.east;
		vo.position_variance[2] = llh_uncert.down;
		vo.velocity_variance[0] = vel_uncert.north;
		vo.velocity_variance[1] = vel_uncert.east;
		vo.velocity_variance[2] = vel_uncert.down;
		vo.orientation_variance[0] = att_euler_uncert.roll;
		vo.orientation_variance[1] = att_euler_uncert.pitch;
		vo.orientation_variance[2] = att_euler_uncert.yaw;
		ref->_vehicle_odometry_pub.publish(vo);
	}

	if (vav_valid) {
		vehicle_angular_velocity_s av{0};
		av.timestamp = t;
		av.timestamp_sample = t;
		av.xyz[0] = ang_rate.gyro[0];
		av.xyz[1] = ang_rate.gyro[1];
		av.xyz[2] = ang_rate.gyro[2];
		// xyz_derivative ??
		ref->_vehicle_angular_velocity_pub.publish(av);
	}

	if (debug_valid) {
		debug_array_s dbg{0};
		dbg.id = 0x01;
		dbg.timestamp = t;
		strcpy(dbg.name, "CV7");
		dbg.data[0] = stat.filter_state * 1.0f;
		dbg.data[1] = stat.dynamics_mode * 1.0f;
		dbg.data[2] = stat.status_flags * 1.0f;
		dbg.data[3] = aid_sum.indicator * 1.0f;
		dbg.data[4] = aid_sum.time_of_week * 1.0f;
		dbg.data[5] = aid_sum.source * 1.0f;
		dbg.data[6] = aid_sum.type * 1.0f;
		ref->_debug_array_pub.publish(dbg);
	}

	//check if condition if correct - Joel
	//Keep status and llh_uncert at same rate? - Joel
	if (true) {
		estimator_status_s status{0};
		status.timestamp = t;
		status.timestamp_sample = t;
		// Note: These flags require insight into the inner operations of the filter.
		//       Below is a minimal mapping of error flags from CV7 to the PX4 health flags
		// Filter State Mapping, if the filter is in 4, set merging GPS and Height being fused flags
		// Only the gps flag is inspected in the checks
		status.control_mode_flags = stat.filter_state == 4 ? (0x1 << estimator_status_s::CS_GPS) |
					    (0x1 << estimator_status_s::CS_GPS_HGT) : 0x00;
		// If there is a general filter condition, set a fault flag to trigger an issue
		status.filter_fault_flags = (stat.status_flags & MIP_FILTER_STATUS_FLAGS_GQ7_FILTER_CONDITION) ? 0x1 : 0x00;
		status.gps_check_fail_flags = 0;
		status.vel_test_ratio = 0.1f;
		status.pos_test_ratio = 0.1f;
		status.hgt_test_ratio = 0.1f;
		status.tas_test_ratio = 0.1f;
		status.hagl_test_ratio = 0.1f;
		status.beta_test_ratio = 0.1f;
		status.pos_horiz_accuracy = sqrt(llh_uncert.north * llh_uncert.north + llh_uncert.east * llh_uncert.east);
		status.pos_vert_accuracy = llh_uncert.down;
		status.solution_status_flags = 0;
		status.time_slip = 0;
		status.pre_flt_fail_innov_heading = false;
		status.pre_flt_fail_innov_vel_horiz = false;
		status.pre_flt_fail_innov_vel_vert = false;
		status.pre_flt_fail_innov_height = false;
		status.pre_flt_fail_mag_field_disturbed = false;
		status.accel_device_id = ref->_dev_id;
		status.gyro_device_id = ref->_dev_id;
		status.mag_device_id = ref->_dev_id;
		status.baro_device_id = ref->_dev_id;
		ref->_estimator_status_pub.publish(status);
		sensor_selection_s sensor_selection{};
		sensor_selection.accel_device_id = ref->_dev_id;
		sensor_selection.gyro_device_id = ref->_dev_id;
		sensor_selection.timestamp = t;
		ref->_sensor_selection_pub.publish(sensor_selection);
	}
}


bool MicroStrain::init()
{
	// Run on fixed interval
	ScheduleOnInterval(_ms_schedule_rate_us);

	return true;
}

void MicroStrain::service_cv7()
{
	sensor_gps_s gps{0};

	// No new data
	if (!_sensor_gps_sub.update(&gps)) {
		return;
	}

	// Fix isn't 3D or RTK or RTCM
	if ((gps.fix_type < 3) || (gps.fix_type > 6)) {
		return;
	}

#define deg_conv(x) (double((x*1.0) / 10000000.0))

	// If the timestamp has not been set, then don't send any data
	// into the cv7 filter
	if (gps.time_utc_usec == 0) {
		return;
	}

	mip_time t;
	t.timebase = MIP_TIME_TIMEBASE_TIME_OF_ARRIVAL;
	t.reserved = 0x01;
	t.nanoseconds = 0; // No offset


	// float llh_uncertainty[3] = {gps.eph,gps.eph,gps.eph}; // What is the uncertainty?
	float llh_uncertainty[3] = {1.0, 1.0, 1.0};
	mip_aiding_llh_pos(&_device, &t, MIP_FILTER_REFERENCE_FRAME_LLH, deg_conv(gps.latitude_deg),
			   deg_conv(gps.longitude_deg),
			   ((gps.altitude_ellipsoid_m * 1.0) / 1000.0), llh_uncertainty, MIP_AIDING_LLH_POS_COMMAND_VALID_FLAGS_ALL);

	if (gps.vel_ned_valid) {
		float ned_v[3] = {gps.vel_n_m_s, gps.vel_e_m_s, gps.vel_d_m_s};
		// float ned_velocity_uncertainty[3] = {gps.s_variance_m_s,gps.s_variance_m_s,gps.s_variance_m_s}; // What is the uncertainty of NED velocity?
		float ned_velocity_uncertainty[3] = {0.1, 0.1, 0.1};
		mip_aiding_ned_vel(&_device, &t, MIP_FILTER_REFERENCE_FRAME_LLH, ned_v, ned_velocity_uncertainty,
				   MIP_AIDING_NED_VEL_COMMAND_VALID_FLAGS_ALL);
	}

	if (PX4_ISFINITE(gps.heading)) {
		// float heading = PX4_ISFINITE(gps.heading_offset) ? gps.heading + gps.heading_offset : gps.heading;
		float heading = gps.heading;
		// There are no pre-defined flags for the heading (that I can find), setting everything to 1 for now
		mip_aiding_true_heading(&_device, &t, MIP_FILTER_REFERENCE_FRAME_LLH, heading, gps.heading_accuracy, 0xff);
	}

}

void MicroStrain::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (!_is_initialized) {
		_is_initialized = initializeIns();
	}

	// Initialization failed, stop the module
	if (!_is_initialized) {
		request_stop();
		perf_end(_loop_perf);
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)

	}

	mip_interface_update(&_device, false);

	if (_ms_mode == 1) {service_cv7();}

	perf_end(_loop_perf);
}

int MicroStrain::task_spawn(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	const char *dev = "/dev/ttyS4";

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			dev = myoptarg;
			break;

		default:
			PX4_WARN("Unrecognized option, Using defaults");
			break;
		}


	}

	if (dev == nullptr || strlen(dev) == 0) {
		print_usage("no device specified");
		_object.store(nullptr);
		_task_id = -1;

		return PX4_ERROR;
	}

	PX4_INFO("Opening device port %s", dev);
	MicroStrain *instance = new MicroStrain(dev);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MicroStrain::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int MicroStrain::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MicroStrain::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MicroStrain by HBK Inertial Sensor Driver.
Currently supports the CV7-AR and CV7-AHRS

Communicates over serial port using submodule MIP_SDK.

### Examples
Attempt to start driver on a specified serial device.
The driver will choose /dev/ttyS4 by default if no port is specified
$ microstrain start -d /dev/ttyS1
Stop driver
$ microstrain stop

)DESCR_STR");



	PRINT_MODULE_USAGE_NAME("MicroStrain", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("ins");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS4", "<file:dev>", "Port", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");

	return PX4_OK;
}

extern "C" __EXPORT int microstrain_main(int argc, char *argv[])
{
	return MicroStrain::main(argc, argv);
}
