#pragma once
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_plane_hpp
#define msr_airlib_plane_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "vehicles/multirotor/Rotor.hpp"
#include "Rudder.hpp"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "PlaneParams.hpp"
#include <vector>
#include "physics/PhysicsBody.hpp"
/* наше все! */
#include "common/LogFileWriter.hpp"
#include "UniForce.hpp"
#include "UFRotor.hpp"
#include "UFRudder.hpp"


namespace msr {
	namespace airlib {



		class Plane : public PhysicsBody {
		public:
			Plane(PlaneParams* params, VehicleApiBase* vehicle_api,
				Kinematics* kinematics, Environment* environment, std::string name)
				: params_(params), vehicle_api_(vehicle_api), plane_name_(name)
			{
				initialize(kinematics, environment);
				std::string filename = std::string("J:/Unreal/LogAirSim/") + plane_name_ + std::string("_PlaneData.txt");
				this->Logger_.open(filename, true);
				setObjType(UpdatableObject::typeUpdObj::plane);
				setObjName(plane_name_);
			}

			//*** Start: UpdatableState implementation ***//
			virtual void resetImplementation() override
			{
				//reset rotors, kinematics and environment
				PhysicsBody::resetImplementation();

				//reset sensors last after their ground truth has been reset
				resetSensors();
			}

			virtual void update() override
			{
				//update forces on vertices that we will use next
				PhysicsBody::update();

				//Note that controller gets updated after kinematics gets updated in updateKinematics
				//otherwise sensors will have values from previous cycle causing lags which will appear
				//as crazy jerks whenever commands like velocity is issued
			}
			virtual void reportState(StateReporter& reporter) override
			{
				//call base
				PhysicsBody::reportState(reporter);

				reportSensors(*params_, reporter);

				//report rotors
				for (uint rotor_index = 0; rotor_index < uniforces_.size(); ++rotor_index) {
					reporter.startHeading("", 1);
					reporter.writeValue("Force", rotor_index);
					reporter.endHeading(false, 1);
					//rotors_.at(rotor_index).reportState(reporter);
					uniforces_.at(rotor_index)->reportState(reporter);
				}
				//TODO: report rudders

			}
			//*** End: UpdatableState implementation ***//


			//Physics engine calls this method to set next kinematics
			virtual void updateKinematics(const Kinematics::State& kinematics) override
			{
				PhysicsBody::updateKinematics(kinematics);

				updateSensors(*params_, getKinematics(), getEnvironment());

				//update controller which will update actuator control signal
				vehicle_api_->update();

				_air_speed = getEnvironment().getState().air_wind.getValue() + getKinematics().twist.linear;

				//transfer new input values from controller to rotors
				if (isFullLogging_)
				{
					Logger_.write("all");
					for (uint rotor_index = 0; rotor_index < 8; ++rotor_index) {
						auto val = vehicle_api_->getActuation(rotor_index);
						Logger_.write(val);
					}
					Logger_.endl();
				}
				//Logger_.write("input");
				{
					//map 
					uniforces_[0]->setControlSignal( vehicle_api_->getActuation(3));
					uniforces_[1]->setControlSignal(vehicle_api_->getActuation(0));
					uniforces_[2]->setControlSignal(vehicle_api_->getActuation(1));

					uniforces_[1]->setAirSpeed(_air_speed);
					uniforces_[2]->setAirSpeed(_air_speed);
				}
				//for (uint rotor_index = 0; rotor_index < uniforces_.size(); ++rotor_index) {
				//	//rotors_.at(rotor_index).setControlSignal(vehicle_api_->getActuation(rotor_index));
				//	uniforces_.at(rotor_index)->setAirSpeed(_air_speed);
				//	auto val = vehicle_api_->getActuation(rotor_index);
				//	Logger_.write(val);
				//	uniforces_.at(rotor_index)->setControlSignal(val);
				//}
				//Logger_.endl();
				if (isFullLogging_)
				{
					Logger_.write("sig input:");
					for (uint rotor_index = 0; rotor_index < uniforces_.size(); ++rotor_index) {
						auto val = uniforces_.at(rotor_index)->getOutput().control_signal_input;
						Logger_.write(val);
					}
					Logger_.endl();
					Logger_.write("filtered:");
					for (uint rotor_index = 0; rotor_index < uniforces_.size(); ++rotor_index) {
						auto val1 = uniforces_.at(rotor_index)->getOutput().control_signal_filtered;
						Logger_.write(val1);
						auto val2 = uniforces_.at(rotor_index)->getOutput().thrust;
						Logger_.write(val2);
						auto val3 = uniforces_.at(rotor_index)->getOutput().torque_scaler;
						Logger_.write(val3);
					}
					Logger_.endl();
					Logger_.write("___________________________");
					Logger_.endl();
				}
			}
			/* Здесь обновляем */

			//sensor getter
			const SensorCollection& getSensors() const
			{
				return params_->getSensors();
			}

			//physics body interface
			virtual uint wrenchVertexCount() const  override
			{
				return (params_->getParams().rotor_count + 
						params_->getParams().rudder_count + 
						params_->getParams().wing_count);
			}
			virtual PhysicsBodyVertex& getWrenchVertex(uint index)  override
			{
				//return rotors_.at(index);
				/*PhysicsBodyVertex & ret = *uniforces_.at(index);
				return ret;*/
				return *uniforces_.at(index);
			}
			virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const override
			{
				//return rotors_.at(index);
				return *uniforces_.at(index);
			}

			virtual uint dragVertexCount() const override
			{
				return static_cast<uint>(drag_vertices_.size());
			}
			virtual PhysicsBodyVertex& getDragVertex(uint index)  override
			{
				return drag_vertices_.at(index);
			}
			virtual const PhysicsBodyVertex& getDragVertex(uint index) const override
			{
				return drag_vertices_.at(index);
			}

			virtual real_T getRestitution() const override
			{
				return params_->getParams().restitution;
			}
			virtual real_T getFriction()  const override
			{
				return params_->getParams().friction;
			}

			//Rotor::Output getRotorOutput(uint rotor_index) const
			//{
			//	return rotors_.at(rotor_index).getOutput();
			//}
			UniForce::Output getForceOutput(uint rotor_index) const
			{
				return uniforces_.at(rotor_index)->getOutput();
			}

			virtual ~Plane() = default;

		private: //methods
			void initialize(Kinematics* kinematics, Environment* environment)
			{
				PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, kinematics, environment);

				//createRotors(*params_, rotors_, environment
				_rotors_count = params_->getParams().rotor_count;
				createRuRoWs(*params_, uniforces_, environment);
				
				createDragVertices();

				initSensors(*params_, getKinematics(), getEnvironment());
			}

			//static void createRotors(const PlaneParams& params, vector<Rotor>& rotors, const Environment* environment)
			//{
			//	rotors.clear();
			//	//for each rotor pose
			//	for (uint rotor_index = 0; rotor_index < (params.getParams().rotor_count + params.getParams().rudder_count ); ++rotor_index) {
			//		const PlaneParams::RotorPose& rotor_pose = params.getParams().rotor_poses.at(rotor_index);
			//		rotors.emplace_back(rotor_pose.position, rotor_pose.normal, rotor_pose.direction, params.getParams().rotor_params, environment, rotor_index);
			//	}
			//}

			static void createRuRoWs(const PlaneParams& params, vector<UniForce*>& uniforces, const Environment* environment)
			{
				uniforces.clear();
				uint force_count = params.getParams().rotor_count + 
									params.getParams().rudder_count + 
									params.getParams().wing_count;
				
				
				for (uint i = 0; i < force_count; ++i)
				{
					const PlaneParams::RotorPose& rotor_pose = params.getParams().rotor_poses.at(i);
					//UniForce force;
					if (i < params.getParams().rotor_count)
					{
						/*force = new UFRotor(rotor_pose.position, rotor_pose.normal, rotor_pose.direction, 
									(UFRotorParams*)params.getParams().force_params, environment, i);*/
						uniforces.emplace_back(new UFRotor(
							rotor_pose.position,
							rotor_pose.normal,
							(UFRotorParams::UniForceDirection) rotor_pose.direction,
							params.getParams().ufrotor_params, environment, i));
					}
					else if (i < (params.getParams().rotor_count + params.getParams().rudder_count)
									&&(params.getParams().rudder_count))
					{ 
					//TODO
						uniforces.emplace_back(new UFRudder(
							rotor_pose.position,
							rotor_pose.normal,
							(UFRudderParams::UniForceDirection) rotor_pose.direction,
							params.getParams().ufrudder_params, environment, i));
					}
					else if (params.getParams().wing_count)
					{ }
				}
			}

			void reportSensors(PlaneParams& params, StateReporter& reporter)
			{
				params.getSensors().reportState(reporter);
			}

			void updateSensors(PlaneParams& params, const Kinematics::State& state, const Environment& environment)
			{
				unused(state);
				unused(environment);
				params.getSensors().update();
			}

			void initSensors(PlaneParams& params, const Kinematics::State& state, const Environment& environment)
			{
				params.getSensors().initialize(&state, &environment);
			}

			void resetSensors()
			{
				params_->getSensors().reset();
			}

			void createDragVertices()
			{
				const auto& params = params_->getParams();

				//Drone is seen as central body that is connected to propellers via arm. We approximate central body as box of size x, y, z.
				//The drag depends on area exposed so we also add area of propellers to approximate drag they may introduce due to their area.
				//while moving along any axis, we find area that will be exposed in that direction
				real_T propeller_area = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_diameter;
				real_T propeller_xsection = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_height;

				real_T top_bottom_area = params.body_box.x() * params.body_box.y();
				real_T left_right_area = params.body_box.x() * params.body_box.z();
				real_T front_back_area = params.body_box.y() * params.body_box.z();
				Vector3r drag_factor_unit = Vector3r(
					/*front_back_area + rotors_.size() * propeller_xsection,
					left_right_area + rotors_.size() * propeller_xsection,
					top_bottom_area + rotors_.size() * propeller_area)*/
					front_back_area + _rotors_count * propeller_xsection,
					left_right_area + _rotors_count * propeller_xsection,
					top_bottom_area + _rotors_count * propeller_area)
					* params.linear_drag_coefficient / 2;

				//add six drag vertices representing 6 sides
				drag_vertices_.clear();
				drag_vertices_.emplace_back(Vector3r(0, 0, -params.body_box.z()), Vector3r(0, 0, -1), drag_factor_unit.z());
				drag_vertices_.emplace_back(Vector3r(0, 0, params.body_box.z()), Vector3r(0, 0, 1), drag_factor_unit.z());
				drag_vertices_.emplace_back(Vector3r(0, -params.body_box.y(), 0), Vector3r(0, -1, 0), drag_factor_unit.y());
				drag_vertices_.emplace_back(Vector3r(0, params.body_box.y(), 0), Vector3r(0, 1, 0), drag_factor_unit.y());
				drag_vertices_.emplace_back(Vector3r(-params.body_box.x(), 0, 0), Vector3r(-1, 0, 0), drag_factor_unit.x());
				drag_vertices_.emplace_back(Vector3r(params.body_box.x(), 0, 0), Vector3r(1, 0, 0), drag_factor_unit.x());

			}
			virtual Vector3r getReaction( Vector3r & hitposition, Vector3r & hitnormal, const float mu, Vector3r & gravity) override
			{

				Vector3r result = Vector3r::Zero();
				float g_value = std::abs(gravity.z());
				if ((hitnormal.z() >= 0.9f) && (hitnormal.z() <= 1.0f)
											&& (std::abs(hitposition.x()) < 0.05f)
											&& (std::abs(hitposition.y()) < 0.05f)
											&& (std::abs(hitposition.z()) < 0.05f))
				{
					float mass = this->getMass();
					float radius = 0.5f;
					float x, y, z;
					x = y = mass * radius * g_value;
					z = mass * radius * mu * g_value;
					result = Vector3r(x, y, z);
				}
				return result;
			}

		private: //fields
			PlaneParams* params_;
			uint _rotors_count;
			//let us be the owner of rotors object
			//vector<Rotor> rotors_;   // Помимо роторов здесь доступны и рули

			std::vector<UniForce*> uniforces_;

			vector<PhysicsBodyVertex> drag_vertices_;
			
			std::unique_ptr<Environment> environment_;
			VehicleApiBase* vehicle_api_;

			Vector3r _air_speed;

			std::string plane_name_;
			msr::airlib::LogFileWriter Logger_;
			uint isFullLogging_ = 1;
		};

	}
} //namespace
#endif
