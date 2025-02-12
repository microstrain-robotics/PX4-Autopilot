/****************************************************************************
*
*   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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

#include "input.h"


namespace gimbal
{

InputBase::InputBase(Parameters &parameters) :
	_parameters(parameters)
{}

void InputBase::control_data_set_lon_lat(ControlData &control_data, double lon, double lat, float altitude,
		uint64_t timestamp)
{
	control_data.timestamp_last_update = timestamp;
	control_data.type = ControlData::Type::LonLat;
	control_data.type_data.lonlat.lon = lon;
	control_data.type_data.lonlat.lat = lat;
	control_data.type_data.lonlat.altitude = altitude;
	control_data.type_data.lonlat.roll_offset = NAN;
	control_data.type_data.lonlat.pitch_fixed_angle = NAN;
	control_data.type_data.lonlat.pitch_offset = 0.f;
	control_data.type_data.lonlat.yaw_offset = 0.f;
}


} /* namespace gimbal */
