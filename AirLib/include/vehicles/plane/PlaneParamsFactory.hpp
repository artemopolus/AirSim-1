#pragma once
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_PlaneParamsFactory_hpp
#define msr_airlib_vehicles_PlaneParamsFactory_hpp

#include "vehicles/plane/firmwares/mavlink/MavLinkPlaneApi.hpp"
#include "vehicles/plane/firmwares/mavlink/Px4PlaneParams.hpp"
//#include "vehicles/multirotor/firmwares/simple_flight/SimpleFlightQuadXParams.hpp"
//#include "vehicles/multirotor/firmwares/mavlink/ArduCopterSoloParams.hpp"
//#include "vehicles/multirotor/firmwares/arducopter/ArduCopterParams.hpp"


namespace msr {
	namespace airlib {

		class PlaneParamsFactory {
		public:
			static std::unique_ptr<PlaneParams> createConfig(const AirSimSettings::VehicleSetting* vehicle_setting,
				std::shared_ptr<const SensorFactory> sensor_factory)
			{
				std::unique_ptr<PlaneParams> config;

				if (vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypePlane) {
					config.reset(new Px4PlaneParams(*static_cast<const AirSimSettings::MavLinkVehicleSetting*>(vehicle_setting),
						sensor_factory));
				}
				//else if (vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypeArduCopterSolo) {
				//	config.reset(new ArduCopterSoloParams(*static_cast<const AirSimSettings::MavLinkVehicleSetting*>(vehicle_setting), sensor_factory));
				//}
				//else if (vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypeArduCopter) {
				//	config.reset(new ArduCopterParams(*static_cast<const AirSimSettings::MavLinkVehicleSetting*>(vehicle_setting), sensor_factory));
				//}
				//else if (vehicle_setting->vehicle_type == "" || //default config
				//	vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypeSimpleFlight) {
				//	config.reset(new SimpleFlightQuadXParams(vehicle_setting, sensor_factory));
				//}
				else
					throw std::runtime_error(Utils::stringf(
						"Cannot create vehicle config because vehicle name '%s' is not recognized",
						vehicle_setting->vehicle_name.c_str()));

				config->initialize(vehicle_setting);

				return config;
			}
		};

	}
} //namespace
#endif
