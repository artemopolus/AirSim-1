#pragma once

#ifndef msr_AirLibUnitTests_PlaneTest_hpp
#define msr_AirLibUnitTests_PlaneTest_hpp

#include "vehicles/plane/PlaneParamsFactory.hpp"
#include "vehicles/plane/Plane.hpp"
#include "TestBase.hpp"
#include "common/AirSimSettings.hpp"
#include "physics/ExPhysicsEngine.hpp"

#define LOG_WRITE_TEST log.write

namespace msr {
	namespace airlib {

		class PlaneTest : public TestBase {
		public:
			PlaneTest()
			{
				std::cout << "Plane test" << std::endl;
			}
			void evaluate(Plane * plane, std::vector<float> inputs, const Kinematics::State & current, Kinematics::State & next, 
				TTimeDelta dt, CollisionInfo& collision_info, std::string pointer_name, 
				LogFileWriter & log)
			{
				LOG_WRITE_TEST("\n========================>\n");
				LOG_WRITE_TEST("\n========================>\n");
				LOG_WRITE_TEST("\n========================>\n");
				std::string header = "\n=====>[FREE AIR]";
				LOG_WRITE_TEST(header + pointer_name + "\n");
				plane->setPose(current.pose);
				plane->setTwist(current.twist);
				plane->inputForKinematicsForce(inputs);
				plane->forceUpdate();
				PhysicsBody * body = static_cast<PhysicsBody *>(plane);
				Kinematics::State observation = current;
				Wrench result_wrench = ExPhysicsEngine::calcDebugGetBodyWrench(*body, observation.pose.orientation, log);
				LOG_WRITE_TEST(header + "[1]initial state");
				ExPhysicsEngine::stateReport2log(current, log);
				LOG_WRITE_TEST("Forces output\n");
				for (uint i = 0; i < plane->getForcesCount(); i++)
				{
					auto output = plane->getForceOutput(i);
					LOG_WRITE_TEST("index:");
					LOG_WRITE_TEST(i);
					LOG_WRITE_TEST("vel_in:");
					LOG_WRITE_TEST(output.vel_input);
					LOG_WRITE_TEST("resist:");
					LOG_WRITE_TEST(output.resistance);
					LOG_WRITE_TEST("thrust:");
					LOG_WRITE_TEST(output.thrust);
					LOG_WRITE_TEST("vx^2:");
					LOG_WRITE_TEST(output.airspeed_x_2);
					LOG_WRITE_TEST("vz^2:");
					LOG_WRITE_TEST(output.airspeed_z_2);
					LOG_WRITE_TEST("t_r:");
					LOG_WRITE_TEST(output.torq_resist);
					LOG_WRITE_TEST("c_1:");
					LOG_WRITE_TEST(output.c_1);
					LOG_WRITE_TEST("c_2:");
					LOG_WRITE_TEST(output.c_2);
					LOG_WRITE_TEST("angle:");
					LOG_WRITE_TEST(output.angle);
					LOG_WRITE_TEST("\n");
				}

				body->setGrounded(false);
				ExPhysicsEngine::calcDebugBodyKinematicsNoCollisions(dt, *body, current, next, result_wrench, log);
				LOG_WRITE_TEST(header + "\n=====>[1]REPORT:" + pointer_name);
				ExPhysicsEngine::stateReport2log(next, log);
				observation.accelerations.linear = next.accelerations.linear;
				observation.accelerations.angular = next.accelerations.angular;
				LOG_WRITE_TEST(header + "[2]initial state");
				ExPhysicsEngine::stateReport2log(current, log);
				ExPhysicsEngine::calcDebugBodyKinematicsNoCollisions(dt, *body, observation, next, result_wrench, log);
				LOG_WRITE_TEST(header + "\n=====>[2]REPORT:" + pointer_name);
				ExPhysicsEngine::stateReport2log(next, log);
				/*current.accelerations.linear = Vector3r::Zero();
				current.accelerations.angular = Vector3r::Zero();*/

				header = "\n=====>[ON COLLISION]";
				plane->setPose(current.pose);
				plane->setTwist(current.twist);
				plane->inputForKinematicsForce(inputs);
				plane->forceUpdate();
				result_wrench = ExPhysicsEngine::calcDebugGetBodyWrench(*body, observation.pose.orientation, log);
				LOG_WRITE_TEST(header + "[1]initial state");
				ExPhysicsEngine::stateReport2log(current, log);
				body->setGrounded(true);
				ExPhysicsEngine::calcDebugBodyKinematicsOnCollisions(dt, collision_info, *body, current, next, result_wrench, true, log);
				LOG_WRITE_TEST(header + "\n=====>[1]REPORT:" + pointer_name);
				ExPhysicsEngine::stateReport2log(next, log);
				observation.accelerations.linear = next.accelerations.linear;
				observation.accelerations.angular = next.accelerations.angular;
				LOG_WRITE_TEST(header + "[2]initial state");
				ExPhysicsEngine::stateReport2log(current, log);
				ExPhysicsEngine::calcDebugBodyKinematicsOnCollisions(dt, collision_info, *body, observation, next, result_wrench, true, log);
				LOG_WRITE_TEST(header + "\n=====>[2]REPORT:" + pointer_name);
				ExPhysicsEngine::stateReport2log(next, log);
				/*current.accelerations.linear = Vector3r::Zero();
				current.accelerations.angular = Vector3r::Zero();*/
				LOG_WRITE_TEST("\n===============================================\n");

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
				initial_environment.gravity = Vector3r(0, 0, 1) * EarthUtils::getGravity(0.0f);
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
				const float angular_rate = 60.0f;
				const float velocity_forward = 25.0f;
				Quaternionr start_angle(AngleAxisr((0.0f), Vector3r(0.f, 1.f, 0.f)));
				Quaternionr middle_angle(AngleAxisr((angle_attack / 2.0f / 180.0f * 3.14f), Vector3r(0.f, 1.f, 0.f)));
				Quaternionr attack_angle(AngleAxisr((angle_attack/180.0f * 3.14f), Vector3r(0.f, 1.f, 0.f)));
				Quaternionr middle_angle_invert(AngleAxisr((- angle_attack / 2.0f / 180.0f * 3.14f), Vector3r(0.f, 1.f, 0.f)));
				Quaternionr attack_angle_invert(AngleAxisr((- angle_attack/180.0f * 3.14f), Vector3r(0.f, 1.f, 0.f)));


				Vector3r linear_velocity(1.f, 0.f, 0.f);
				linear_velocity *= velocity_forward;

				Vector3r middle_linear_velocity = VectorMath::rotateVector(linear_velocity, middle_angle, true);
				Vector3r attack_linear_velocity = VectorMath::rotateVector(linear_velocity, attack_angle, true);

				Vector3r middle_linear_velocity_invert = VectorMath::rotateVector(linear_velocity, middle_angle_invert, true);
				Vector3r attack_linear_velocity_invert = VectorMath::rotateVector(linear_velocity, attack_angle_invert, true);

				Vector3r middle_angular_velocity = Vector3r(0.0f, (angular_rate / 2.0f * 3.14f / 180.0f), 0.0f);
				Vector3r attack_angular_velocity = Vector3r(0.0f, (angular_rate / 1.0f * 3.14f / 180.0f), 0.0f);

				Vector3r middle_angular_velocity_invert = Vector3r(0.0f, (- angular_rate / 2.0f * 3.14f / 180.0f), 0.0f);
				Vector3r attack_angular_velocity_invert = Vector3r(0.0f, (- angular_rate / 1.0f * 3.14f / 180.0f), 0.0f);
	
				Vector3r angular_velocity(0.f, 0.f, 0.f);
				LogFileWriter log;
				std::string filename = Settings::getUserDirectoryFullPath(std::string("unittest_PhysicsEngine.txt"));
				log.open(filename, true);
				std::vector<float> inputs = {0.68f, 0.28f, 0.0f, 0.5f};
				std::vector<float> inputs_zero = { 0.5f, 0.5f, 0.0f, 0.2f };
				std::vector<float> inputs_rvrsRoll = {0.28f, 0.68f, 0.0f, 0.5f};
				std::vector<float> inputs_rvrsPitch = {0.72f, 0.32f, 0.0f, 0.5f};

				std::vector<float> inputs_fullroll_1 = {1.0f, 0.0f, 0.0f, 0.5f};
				std::vector<float> inputs_fullroll_2 = {0.0f, 1.0f, 0.0f, 0.5f};
				std::vector<float> inputs_fullpitch_1 = {0.0f, 0.0f, 0.0f, 0.5f};
				std::vector<float> inputs_fullpitch_2 = {1.0f, 1.0f, 0.0f, 0.5f};


				Kinematics::State current = Kinematics::State::zero();
				Kinematics::State next = Kinematics::State::zero();


				TTimeDelta dt = 0.003;

				CollisionInfo collision_info;
				collision_info.has_collided = true;
				collision_info.collision_count = 1;
				collision_info.impact_point = Vector3r::Zero();
				collision_info.normal = Vector3r(0, 0, -1);
				collision_info.object_id = 1;
				collision_info.object_name = "test_ground";
				collision_info.penetration_depth = 0;
				collision_info.position = Vector3r::Zero();
				collision_info.time_stamp = 0;

				log.write("linear velocity:");
				log.write(linear_velocity);
				log.write("angular velocity:");
				log.write(angular_velocity);
					LOG_WRITE_TEST("\nDelta time:");
					LOG_WRITE_TEST(dt);

				log.endl();

				for (auto name : vehicle_names)
				{
					auto vehicle_params_ = PlaneParamsFactory::createConfig(AirSimSettings::singleton().getVehicleSetting(name),
						std::make_shared<SensorFactory>());
					auto vehicle_api_ = vehicle_params_->createPlaneApi();
					testAssert(vehicle_api_ != nullptr, "Couldn't get pixhawk controller"); 
					std::unique_ptr<Plane> phys_vehicle = std::unique_ptr<Plane>(new Plane(vehicle_params_.get(), vehicle_api_.get(),
						kinematics.get(), environment.get(), name));

					current.pose.position = Vector3r::Zero();
					current.twist.linear = linear_velocity;
					current.twist.angular = angular_velocity;
					phys_vehicle.get()->reset();
					LOG_WRITE_TEST("Gravity:");
					LOG_WRITE_TEST(phys_vehicle.get()->getEnvironment().getState().gravity);
					LOG_WRITE_TEST("\n========================");
					LOG_WRITE_TEST("\n=========NEW============");
					LOG_WRITE_TEST("\n========================");
					std::string plane_tested_mode = "";
					std::vector<float> data_input;
					plane_tested_mode = "analysis position";
					current.pose.orientation = Quaternionr(0.00310554f,	0.503667f, - 0.0119108f, - 0.86381f);
					current.twist.linear = Vector3r(-3.54498f, - 0.00353718f,	0.0f);
					current.twist.angular = Vector3r(0.0258447f,	11.1272f, - 0.00258371f);
					current.accelerations.linear = Vector3r(-0.229462f, - 0.0157405f,	0.0f);
					current.accelerations.angular = Vector3r(0.129511f,	409.993f, - 0.10382f);
					data_input = { 0.0f,	0.652944f,	0.0f,	0.2f };
					evaluate(phys_vehicle.get(), data_input, current, next, dt, collision_info, "initial" + plane_tested_mode, log);
				/*	current.pose.orientation = middle_angle;
					evaluate(phys_vehicle.get(), data_input, current, next, dt, collision_info, "middle normal angle" + plane_tested_mode, log);
					current.pose.orientation = middle_angle_invert;
					evaluate(phys_vehicle.get(), data_input, current, next, dt, collision_info, "middle invert angle" + plane_tested_mode, log);
					current.pose.orientation = attack_angle;
					evaluate(phys_vehicle.get(), data_input, current, next, dt, collision_info, "attack normal angle" + plane_tested_mode, log);
					current.pose.orientation = attack_angle_invert;
					evaluate(phys_vehicle.get(), data_input, current, next, dt, collision_info, "attack invert angle" + plane_tested_mode, log);
*/
					/*current.twist.linear = linear_velocity;
					current.twist.angular = angular_velocity;
					current.pose.orientation = start_angle;
					evaluate(phys_vehicle.get(), inputs, current, next, dt, collision_info, "[FROM ZERO positive]", log);
					evaluate(phys_vehicle.get(), inputs_rvrsRoll, current, next, dt, collision_info, "[FROM ZERO negative roll]", log);
					evaluate(phys_vehicle.get(), inputs_rvrsPitch, current, next, dt, collision_info, "[FROM ZERO negative pitch]", log);
					evaluate(phys_vehicle.get(), inputs_fullroll_1, current, next, dt, collision_info, "[FROM ZERO full roll positive]", log);
					evaluate(phys_vehicle.get(), inputs_fullpitch_1, current, next, dt, collision_info, "[FROM ZERO full pitch positive]", log);

					LOG_WRITE_TEST("\n========================");
					LOG_WRITE_TEST("\n=========NEW============");
					LOG_WRITE_TEST("\n========================");
					current.twist.linear = linear_velocity;
					current.twist.angular = middle_angular_velocity;
					current.pose.orientation = middle_angle;
					std::string name_try;
					name_try = "[Middle angle]";
					evaluate(phys_vehicle.get(), inputs, current, next, dt, collision_info, name_try + "positive]", log);
					evaluate(phys_vehicle.get(), inputs_zero, current, next, dt, collision_info, name_try + "zero", log);
					evaluate(phys_vehicle.get(), inputs_rvrsRoll, current, next, dt, collision_info, name_try + "negative roll]", log);
					evaluate(phys_vehicle.get(), inputs_rvrsPitch, current, next, dt, collision_info,name_try + " negative pitch]", log);
					evaluate(phys_vehicle.get(), inputs_fullroll_1, current, next, dt, collision_info,name_try + " full roll pos", log);
					evaluate(phys_vehicle.get(), inputs_fullroll_2, current, next, dt, collision_info,name_try + " full roll neg", log);
					evaluate(phys_vehicle.get(), inputs_fullpitch_1, current, next, dt, collision_info,name_try + " full pitch pos", log);
					evaluate(phys_vehicle.get(), inputs_fullpitch_2, current, next, dt, collision_info,name_try + " full pitch neg", log);

					LOG_WRITE_TEST("\n========================");
					LOG_WRITE_TEST("\n=========NEW============");
					LOG_WRITE_TEST("\n========================");
					current.twist.linear = linear_velocity;
					current.twist.angular = middle_angular_velocity_invert;
					current.pose.orientation = middle_angle_invert;
					
					name_try = "[Middle angle_invert]";
					evaluate(phys_vehicle.get(), inputs, current, next, dt, collision_info, name_try + "positive]", log);
					evaluate(phys_vehicle.get(), inputs_zero, current, next, dt, collision_info, name_try + "zero", log);
					evaluate(phys_vehicle.get(), inputs_rvrsRoll, current, next, dt, collision_info, name_try + "negative roll]", log);
					evaluate(phys_vehicle.get(), inputs_rvrsPitch, current, next, dt, collision_info,name_try + " negative pitch]", log);
					evaluate(phys_vehicle.get(), inputs_fullroll_1, current, next, dt, collision_info,name_try + " full roll positive]", log);
					evaluate(phys_vehicle.get(), inputs_fullpitch_1, current, next, dt, collision_info,name_try + " full pitch positive]", log);

					LOG_WRITE_TEST("\n========================");
					LOG_WRITE_TEST("\n=========NEW============");
					LOG_WRITE_TEST("\n========================");
					current.twist.linear = linear_velocity;
					current.twist.angular = attack_angular_velocity;
					current.pose.orientation = attack_angle;
					name_try = "[attack angle]";
					evaluate(phys_vehicle.get(), inputs, current, next, dt, collision_info, name_try + "positive]", log);
					evaluate(phys_vehicle.get(), inputs_zero, current, next, dt, collision_info, name_try + "zero", log);
					evaluate(phys_vehicle.get(), inputs_rvrsRoll, current, next, dt, collision_info, name_try + "negative roll]", log);
					evaluate(phys_vehicle.get(), inputs_rvrsPitch, current, next, dt, collision_info,name_try + " negative pitch]", log);
					evaluate(phys_vehicle.get(), inputs_fullroll_1, current, next, dt, collision_info,name_try + " full roll positive]", log);
					evaluate(phys_vehicle.get(), inputs_fullpitch_1, current, next, dt, collision_info,name_try + " full pitch positive]", log);

					LOG_WRITE_TEST("\n========================");
					LOG_WRITE_TEST("\n=========NEW============");
					LOG_WRITE_TEST("\n========================");
					current.twist.linear = linear_velocity;
					current.twist.angular = attack_angular_velocity_invert;
					current.pose.orientation = attack_angle_invert;
					name_try = "[attack angle_invert]";
					evaluate(phys_vehicle.get(), inputs, current, next, dt, collision_info, name_try + "positive]", log);
					evaluate(phys_vehicle.get(), inputs_zero, current, next, dt, collision_info, name_try + "zero", log);
					evaluate(phys_vehicle.get(), inputs_rvrsRoll, current, next, dt, collision_info, name_try + "negative roll]", log);
					evaluate(phys_vehicle.get(), inputs_rvrsPitch, current, next, dt, collision_info,name_try + " negative pitch]", log);
					evaluate(phys_vehicle.get(), inputs_fullroll_1, current, next, dt, collision_info,name_try + " full roll positive]", log);
					evaluate(phys_vehicle.get(), inputs_fullpitch_1, current, next, dt, collision_info,name_try + " full pitch positive]", log);*/


					planes.push_back(std::move(phys_vehicle));
				}
				/*auto pixhawk = PlaneParamsFactory::createConfig(AirSimSettings::singleton().getVehicleSetting("Pixhawk"),
					std::make_shared<SensorFactory>());
				auto api = pixhawk->createPlaneApi();
				testAssert(api != nullptr, "Couldn't get pixhawk controller");*/


				for (auto & plane : planes)
				{
					PhysicsBody * body = static_cast<PhysicsBody *>(plane.get());
					Wrench result = ExPhysicsEngine::calcDebugGetBodyWrench(*body, current.pose.orientation, log);
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