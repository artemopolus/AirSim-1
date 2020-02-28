#pragma once

#ifndef msr_AirLibUnitTests_PlaneTest_hpp
#define msr_AirLibUnitTests_PlaneTest_hpp

#include "vehicles/plane/PlaneParamsFactory.hpp"
#include "TestBase.hpp"

namespace msr {
	namespace airlib {

		class PlaneTest : public TestBase {
		public:
			virtual void run() override
			{
				//Test PX4 based drones
				auto pixhawk = PlaneParamsFactory::createConfig(AirSimSettings::singleton().getVehicleSetting("Pixhawk"),
					std::make_shared<SensorFactory>());
				auto api = pixhawk->createPlaneApi();
				testAssert(api != nullptr, "Couldn't get pixhawk controller");

				try {
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
				catch (std::domain_error& ex) {
					std::cout << ex.what() << std::endl;
				}
			}
		};


	}
}
#endif