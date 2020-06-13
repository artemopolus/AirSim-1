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
#include "UFWing.hpp"
#include "common/Settings.hpp"
#include "common/AirSimSettings.hpp"

namespace msr {
	namespace airlib {



		class Plane : public PhysicsBody {
		public:
			Plane(PlaneParams* params, VehicleApiBase* vehicle_api,
				Kinematics* kinematics, Environment* environment, std::string name)
				: params_(params), vehicle_api_(vehicle_api), plane_name_(name)
			{
				std::string filename = Settings::getUserDirectoryFullPath(plane_name_ + std::string("_PlaneData.txt"));
				this->Logger_.open(filename, true);
				initialize(kinematics, environment);
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
			void forceReset()
			{
				for (auto force : uniforces_)
				{
					force->reset();
				}
			}
			void forceUpdate()
			{
				for (auto & force : uniforces_)
				{
					force->debugSetAirDensityRatio();
					force->debugSetOutput();
					force->forceSetWrench();
				}
					Logger_.write("[DEBUG_OUTPUT]");
					for (uint rotor_index = 0; rotor_index < uniforces_.size(); ++rotor_index) {
						const auto force_info = uniforces_.at(rotor_index)->getObjType();
						writeType2logger(force_info);
						auto val1 = uniforces_.at(rotor_index)->getOutput().control_signal_filtered;
						Logger_.write("\nctrl");
						Logger_.write(val1);
						auto val2 = uniforces_.at(rotor_index)->getOutput().thrust;
						Logger_.write("\nthrust");
						Logger_.write(val2);
						auto val3 = uniforces_.at(rotor_index)->getOutput().torque_scaler;
						Logger_.write("\ntorq_scl");
						Logger_.write(val3);
						auto val4 = uniforces_.at(rotor_index)->getOutput().resistance;
						Logger_.write("\nresistance");
						Logger_.write(val4);
						Logger_.write("\nangle");
						Logger_.write(uniforces_.at(rotor_index)->getOutput().angle);
						Logger_.write("\nair_speed");
						Logger_.write(uniforces_.at(rotor_index)->getAirSpeed());
						Logger_.write("\nforce");
						Logger_.write(uniforces_.at(rotor_index)->getWrench().force);
						Logger_.write("\nnormal");
						Logger_.write(uniforces_.at(rotor_index)->getNormal());
						Logger_.write("\nair_density_ratio");
						Logger_.write(uniforces_.at(rotor_index)->getAirDensityRatio());
						Logger_.write("\nair_speed_x");
						Logger_.write(uniforces_.at(rotor_index)->getOutput().airspeed_x_2);
						Logger_.write("\nair_speed_z");
						Logger_.write(uniforces_.at(rotor_index)->getOutput().airspeed_z_2);
						Logger_.write("\ncoef_1");
						Logger_.write(uniforces_.at(rotor_index)->getOutput().c_1);
						Logger_.write("\ncoef_2");
						Logger_.write(uniforces_.at(rotor_index)->getOutput().c_2);
						Logger_.write("\nw");
						Logger_.write(uniforces_.at(rotor_index)->getRotation());
						Logger_.write("\nt_res");
						Logger_.write(uniforces_.at(rotor_index)->getOutput().torq_resist);
						Logger_.endl();

					}
			}
			virtual void inputForKinematicsForce(std::vector<float> inputs) override
			{
				const Quaternionr orientation_w = getKinematics().pose.orientation;
				const Vector3r lin_velocity_w = getKinematics().twist.linear;
				const Vector3r ang_velocity_w = getKinematics().twist.angular;
				const Vector3r lin_velocity_b = VectorMath::transformToBodyFrame(lin_velocity_w, orientation_w);
				const Vector3r ang_velocity_b = VectorMath::transformToBodyFrame(ang_velocity_w, orientation_w);
				Logger_.write("\nForce input for Kinemtics");
				Logger_.write("orientation_w");
				Logger_.write(orientation_w);
				Logger_.write("lin_velocity_w");
				Logger_.write(lin_velocity_w);
				Logger_.write("lin_velocity_b");
				Logger_.write(lin_velocity_b);
				Logger_.write("ang_velocity_w");
				Logger_.write(ang_velocity_w);
				Logger_.write("ang_velocity_b");
				Logger_.write(ang_velocity_b);
				Logger_.write("\n[MAP]");


				for (uint i = 0; i < uniforces_.size(); i++)
				{
					const auto type = uniforces_[i]->getObjType();
					writeType2logger(type);
					if (type != UpdatableObject::typeUpdObj::wing)
					{
						uint trg_id = uniforces_[i]->getActID();
						Logger_.write(trg_id);
						uniforces_[i]->debugCtrlSignal(inputs[trg_id]);
					}
					if ((type == UpdatableObject::typeUpdObj::rudder)||(type == UpdatableObject::typeUpdObj::wing))
					{
						Vector3r pos = uniforces_[i]->getPosition();
						Vector3r relative_speed = pos.cross(ang_velocity_b);
						uniforces_[i]->setAirSpeed(lin_velocity_b + relative_speed);
						uniforces_[i]->setRotation(ang_velocity_b);
					}
					else
						uniforces_[i]->setAirSpeed(lin_velocity_b);

				}
				Logger_.endl();


			}


			//Physics engine calls this method to set next kinematics
			virtual void updateKinematics(const Kinematics::State& kinematics) override
			{
				PhysicsBody::updateKinematics(kinematics);

				updateSensors(*params_, getKinematics(), getEnvironment());

				//update controller which will update actuator control signal
				vehicle_api_->update();
				orientation_ = getKinematics().pose.orientation;
				Vector3r wind_tmp = getEnvironment().getState().air_wind;
				Vector3r velocity_tmp = getKinematics().twist.linear;
				const Vector3r velocity_angular_w = getKinematics().twist.angular;
				Vector3r air_speed_tmp = wind_tmp + velocity_tmp;
				air_speed_ = VectorMath::transformToBodyFrame(air_speed_tmp, orientation_);
				//const Vector3r velocity_angular = VectorMath::transformToBodyFrame(velocity_angular_w, orientation_);
				const Vector3r velocity_angular = velocity_angular_w;

				if (isFullLogging_) {
					Logger_.write("[air_speed]\tw");
					Logger_.write(wind_tmp);
					Logger_.write("air_w");
					Logger_.write(air_speed_tmp);
					Logger_.write("air_b");
					Logger_.write(air_speed_);
					Logger_.write("q_b");
					Logger_.write(orientation_);
					Logger_.write("w");
					Logger_.write(velocity_angular);
					Logger_.endl();
				}

				//transfer new input values from controller to rotors
				if (isFullLogging_)
				{
					Logger_.write("[INPUT_SIG]");
					for (uint rotor_index = 0; rotor_index < 8; ++rotor_index) {
						auto val = vehicle_api_->getActuation(rotor_index);
						Logger_.write(val);
					}
					Logger_.endl();
				}
				//Logger_.write("input");
				{
					//map 
					Logger_.write("[MAP]");
					for (uint i = 0; i < uniforces_.size(); i++)
					{
						const auto type = uniforces_[i]->getObjType();
						writeType2logger(type);
						if (type != UpdatableObject::typeUpdObj::wing)
						{
							uint trg_id = uniforces_[i]->getActID();
							Logger_.write(trg_id);
							uniforces_[i]->setControlSignal(vehicle_api_->getActuation(trg_id));
						}
						if ((type == UpdatableObject::typeUpdObj::rudder)||(type == UpdatableObject::typeUpdObj::wing))
						{
							Vector3r pos = uniforces_[i]->getPosition();
							Vector3r relative_speed = pos.cross(velocity_angular);
							uniforces_[i]->setAirSpeed(air_speed_ + relative_speed);
							uniforces_[i]->setRotation(velocity_angular);
						}
						else
							uniforces_[i]->setAirSpeed(air_speed_);

					}
					Logger_.endl();
				
				}
				if (isFullLogging_)
				{
					Logger_.write("[CTRL_SIG_INP]");
					for (uint rotor_index = 0; rotor_index < uniforces_.size(); ++rotor_index) {
						auto val = uniforces_.at(rotor_index)->getOutput().control_signal_input;
						Logger_.write(val);
					}
					Logger_.endl();
					Logger_.write("[FRC_OUTPUT]");
					for (uint rotor_index = 0; rotor_index < uniforces_.size(); ++rotor_index) {
						const auto force_info = uniforces_.at(rotor_index)->getObjType();
						writeType2logger(force_info);
						auto val1 = uniforces_.at(rotor_index)->getOutput().control_signal_filtered;
						Logger_.write("ctrl");
						Logger_.write(val1);
						auto val2 = uniforces_.at(rotor_index)->getOutput().thrust;
						Logger_.write("thrust");
						Logger_.write(val2);
						auto val3 = uniforces_.at(rotor_index)->getOutput().torque_scaler;
						Logger_.write("torq_scl");
						Logger_.write(val3);
						auto val4 = uniforces_.at(rotor_index)->getOutput().resistance;
						Logger_.write("resistance");
						Logger_.write(uniforces_.at(rotor_index)->getOutput().airspeed_x_2);
						Logger_.write(val4);
						Logger_.write(uniforces_.at(rotor_index)->getAirSpeed().x());
						Logger_.write(uniforces_.at(rotor_index)->getOutput().c_1);
						Logger_.write("angle");
						Logger_.write(uniforces_.at(rotor_index)->getOutput().angle);
	
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
				auto & one_force = uniforces_.at(index);
				Logger_.write("[OUT_WRENCH]");
				writeType2logger(one_force->getObjType());
				Logger_.write("F");
				Logger_.write(one_force->getWrench().force);
				Logger_.write("M");
				Logger_.write(one_force->getWrench().torque);
				Logger_.write("air_speed");
				Logger_.write(one_force->getAirSpeed());
				Logger_.write("ang_velocity");
				Logger_.write(one_force->getRotation());
				Logger_.endl();
				return *uniforces_.at(index);
			}
			virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const override
			{
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
			uint getForcesCount() const
			{
				return (uint)uniforces_.size();
			}

			virtual ~Plane() = default;

		private: //methods
			void initialize(Kinematics* kinematics, Environment* environment)
			{
				// TODO: new inertia computation
				Matrix3x3r inertia = Matrix3x3r::Zero();
				params_->calculatePlaneInertiaMatrix(inertia, Logger_);
				PhysicsBody::initialize(params_->getParams().mass, inertia, kinematics, environment);

				//createRotors(*params_, rotors_, environment
				_rotors_count = params_->getParams().rotor_count;
				createRuRoWs(*params_, uniforces_, environment, Logger_);
				
				createDragVertices();

				initSensors(*params_, getKinematics(), getEnvironment());
			}
			void writeType2logger(UpdatableObject::typeUpdObj type)
			{
				if (type == UpdatableObject::typeUpdObj::rotor) {
					Logger_.write("Rotor");
				}
				else if (type == UpdatableObject::typeUpdObj::rudder) {
					Logger_.write("Rudder");
				}
				else if (type == UpdatableObject::typeUpdObj::wing) {
					Logger_.write("Wing");
				}
				else {
					Logger_.write("UnknownType");
				}
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
			static void createRuRoWs(const PlaneParams& params, vector<UniForce*>& uniforces, const Environment* environment, LogFileWriter & logger)
			{
				uniforces.clear();
				uint counter = 0;
				for (const  UFRotorParams & rotor_param : params.getParams().rotor_list)
				{
					UFRotorParams * trg_param = new UFRotorParams(rotor_param);
					uniforces.emplace_back(new UFRotor(
						rotor_param.getPosition(), rotor_param.getNormal(),
						UFRotorParams::UniForceDirection::UniForceDirectionFront,
						trg_param, environment, counter
					));
					counter++;
				}
				for (const  UFRudderParams & rudder_param : params.getParams().rudder_list)
				{
					UFRudderParams * trg_param = new UFRudderParams(rudder_param);
					uniforces.emplace_back(new UFRudder(
						rudder_param.getPosition(), rudder_param.getNormal(),
						UFRotorParams::UniForceDirection::UniForceDirectionFront,
						trg_param, environment, counter
					));
					counter++;
				}
				for (const  UFWingParams & wing_param : params.getParams().wing_list)
				{
					UFWingParams * trg_param = new UFWingParams(wing_param);
					uniforces.emplace_back(new UFWing(
						wing_param.getPosition(), wing_param.getNormal(),
						UFRotorParams::UniForceDirection::UniForceDirectionFront,
						trg_param, environment, counter
					));
					counter++;
				}

				for (auto & one_force : uniforces)
				{
					if (one_force->getObjType() == UpdatableObject::typeUpdObj::rotor)
					{
						UFRotor * base =  (static_cast<UFRotor*>(one_force));
						UFRotorParams * base_params = base->getCurrentParams();
						logger.write("Rotor [normal]");
						logger.write(base_params->getNormal());
						logger.write("[position]");
						logger.write(base_params->getPosition());
						logger.write("[act id]");
						logger.write(base_params->getActID());
						logger.endl();
					}
					if (one_force->getObjType() == UpdatableObject::typeUpdObj::rudder)
					{
						UFRudder * base = static_cast<UFRudder*>(one_force);
						UFRudderParams * base_params = base->getCurrentParams();
						logger.write("Rudder");
						logger.write(base_params->getNormal());
						logger.write(base_params->getPosition());
						logger.write("[act id]");
						logger.write(base_params->getActID());
						logger.write("max_thrust");
						logger.write(base_params->getMaxThrust());
						logger.write("angle");
						logger.write(base_params->getMaxAngle());
						logger.write("resist");
						logger.write(base_params->getResistance());
						logger.endl();
					}
					if (one_force->getObjType() == UpdatableObject::typeUpdObj::wing)
					{
						UFWing * base = static_cast<UFWing*>(one_force);
						UFWingParams * base_params = base->getCurrentParams();
						logger.write("Wing");
						logger.write(base_params->getNormal());
						logger.write(base_params->getPosition());
						logger.write("[act id]");
						logger.write(base_params->getActID());
						logger.write("torq_resist");
						logger.write(base_params->getVelocityResistence());
						logger.write("mass");
						logger.write(base_params->getMass());
						logger.write("torq_resist");
						logger.write(base_params->getVelocityResistence());

						logger.endl();
					}
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
			bool vectorLessValue(Vector3r trg, const float value)
			{
				bool res = ((std::abs(trg.x()) < std::abs(value))
						&&  (std::abs(trg.y()) < std::abs(value))
						&&  (std::abs(trg.z()) < std::abs(value))) ? true : false;
				return res;
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

			Vector3r air_speed_;
			Quaternionr orientation_;

			std::string plane_name_;
			msr::airlib::LogFileWriter Logger_;
			uint isFullLogging_ = 1;
		};

	}
} //namespace
#endif
