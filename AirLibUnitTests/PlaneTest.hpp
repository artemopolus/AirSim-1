#pragma once

#ifndef msr_AirLibUnitTests_PlaneTest_hpp
#define msr_AirLibUnitTests_PlaneTest_hpp

#include "vehicles/plane/PlaneParamsFactory.hpp"
#include "vehicles/plane/Plane.hpp"
#include "TestBase.hpp"
#include "common/AirSimSettings.hpp"

namespace msr {
	namespace airlib {

		class PlaneTest : public TestBase {
		public:
			PlaneTest()
			{
				std::cout << "Plane test" << std::endl;
			}
			std::string getSimMode() const
			{
				return "Plane";
			}
			virtual void run() override
			{
				std::cout << "Start plane test" << std::endl;

				std::string settingsPath = msr::airlib::Settings::Settings::getUserDirectoryFullPath("settings.json").c_str();
				
				std::string settingsText = "", str;

				std::ifstream input(settingsPath);
				while (getline(input, str))
					settingsText += str;
				AirSimSettings::singleton().initializeSettings(settingsText);


				std::unique_ptr<msr::airlib::Kinematics> kinematics;
				std::unique_ptr<msr::airlib::Environment> environment;

				Kinematics::State initial_kinematic_state = Kinematics::State::zero();;
				kinematics.reset(new Kinematics(initial_kinematic_state));

				Environment::State initial_environment;
				environment.reset(new Environment(initial_environment));

				std::vector<std::unique_ptr<msr::airlib::Plane>> planes;

				std::vector<std::string> vehicle_names;

				std::map<std::string, std::unique_ptr<msr::airlib::AirSimSettings::VehicleSetting>> vehicles;

				msr::airlib::AirSimSettings::singleton().load(std::bind(&PlaneTest::getSimMode,this));


				for (auto const& vehicle_setting_pair : AirSimSettings::singleton().vehicles)
				{
					const auto& vehicle_setting = *vehicle_setting_pair.second;
					// some settings loading
					std::string name = vehicle_setting.vehicle_name.c_str();
					vehicle_names.emplace_back(name);
					std::cout << "name:" << name << std::endl;
				}
				//Test PX4 based drones
				for (auto name : vehicle_names)
				{
					auto vehicle_params_ = PlaneParamsFactory::createConfig(AirSimSettings::singleton().getVehicleSetting(name),
						std::make_shared<SensorFactory>());
					auto vehicle_api_ = vehicle_params_->createPlaneApi();
					testAssert(vehicle_api_ != nullptr, "Couldn't get pixhawk controller"); 
					std::unique_ptr<Plane> phys_vehicle = std::unique_ptr<Plane>(new Plane(vehicle_params_.get(), vehicle_api_.get(),
						kinematics.get(), environment.get(), name));
					planes.push_back(std::move(phys_vehicle));
				}
				/*auto pixhawk = PlaneParamsFactory::createConfig(AirSimSettings::singleton().getVehicleSetting("Pixhawk"),
					std::make_shared<SensorFactory>());
				auto api = pixhawk->createPlaneApi();
				testAssert(api != nullptr, "Couldn't get pixhawk controller");*/

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