#pragma once

#ifndef msr_AirLibUnitTests_PlaneTest_hpp
#define msr_AirLibUnitTests_PlaneTest_hpp

#include "vehicles/plane/PlaneParamsFactory.hpp"
#include "vehicles/plane/Plane.hpp"
#include "TestBase.hpp"
#include "common/AirSimSettings.hpp"
#include "physics/ExPhysicsEngine.hpp"

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

				Kinematics::State initial_kinematic_state = Kinematics::State::zero();
				kinematics.reset(new Kinematics(initial_kinematic_state));

				Environment::State initial_environment;
				initial_environment.air_density = EarthUtils::getAirDensity(0.0f);
				environment.reset(new Environment(initial_environment));

				std::vector<std::unique_ptr<msr::airlib::Plane>> planes;

				std::vector<std::string> vehicle_names;

				std::map<std::string, std::unique_ptr<msr::airlib::AirSimSettings::VehicleSetting>> vehicles;

				msr::airlib::AirSimSettings::singleton().load(std::bind(&PlaneTest::getSimMode, this));


				for (auto const& vehicle_setting_pair : AirSimSettings::singleton().vehicles)
				{
					const auto& vehicle_setting = *vehicle_setting_pair.second;
					// some settings loading
					std::string name = vehicle_setting.vehicle_name.c_str();
					vehicle_names.emplace_back(name);
					std::cout << "name:" << name << std::endl;
				}
				//Test PX4 based drones
				std::cout << "Load vehicles" << std::endl;
				const float angle_attack = 15.0f;
				const float velocity_forward = 25.0f;
				Quaternionr angle(AngleAxisr((angle_attack/180.0f * 3.14f), Vector3r(0.f, 1.f, 0.f)));
				Vector3r linear_velocity(1.f, 0.f, 0.f);
				linear_velocity *= velocity_forward;
				linear_velocity = VectorMath::rotateVector(linear_velocity, angle, true);
				Vector3r angular_velocity(0.f, 0.f, 0.f);
				LogFileWriter log;
				std::string filename = Settings::getUserDirectoryFullPath(std::string("unittest_PhysicsEngine.txt"));
				log.open(filename, true);
				std::vector<float> inputs = {0.4f, 0.4f, 0.0f, 0.2f};

				log.write("linear velocity:");
				log.write(linear_velocity);
				log.write("angular velocity:");
				log.write(angular_velocity);

				log.endl();

				for (auto name : vehicle_names)
				{
					auto vehicle_params_ = PlaneParamsFactory::createConfig(AirSimSettings::singleton().getVehicleSetting(name),
						std::make_shared<SensorFactory>());
					auto vehicle_api_ = vehicle_params_->createPlaneApi();
					testAssert(vehicle_api_ != nullptr, "Couldn't get pixhawk controller"); 
					std::unique_ptr<Plane> phys_vehicle = std::unique_ptr<Plane>(new Plane(vehicle_params_.get(), vehicle_api_.get(),
						kinematics.get(), environment.get(), name));

					phys_vehicle.get()->reset();
					phys_vehicle.get()->inputForKinematicsForce(inputs, linear_velocity, angular_velocity);

					//phys_vehicle.get()->update();

					//phys_vehicle.get()->forceReset();

					phys_vehicle.get()->forceUpdate();

					PhysicsBody * body = static_cast<PhysicsBody *>(phys_vehicle.get());
					Wrench result = ExPhysicsEngine::getBodyWrench(*body, angle, log);

					planes.push_back(std::move(phys_vehicle));
				}
				/*auto pixhawk = PlaneParamsFactory::createConfig(AirSimSettings::singleton().getVehicleSetting("Pixhawk"),
					std::make_shared<SensorFactory>());
				auto api = pixhawk->createPlaneApi();
				testAssert(api != nullptr, "Couldn't get pixhawk controller");*/


				for (auto & plane : planes)
				{
					PhysicsBody * body = static_cast<PhysicsBody *>(plane.get());
					Wrench result = ExPhysicsEngine::getBodyWrench(*body, angle, log);
				}
				std::cout << "Done getting wrench" << std::endl;
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