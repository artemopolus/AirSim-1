// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.



#ifndef airsimcore_uniforce_hpp
#define airsimcore_uniforce_hpp

#include <limits>
#include "common/Common.hpp"
#include "physics/Environment.hpp"
#include "common/FirstOrderFilter.hpp"
#include "physics/PhysicsBodyVertex.hpp"
#include "UniForceParams.hpp"

namespace msr {
	namespace airlib {

		//Rotor gets control signal as input (PWM or voltage represented from 0 to 1) which causes 
		//change in rotation speed and turning direction and ultimately produces force and thrust as
		//output
		class UniForce : public PhysicsBodyVertex {
		public: //types
			enum class UniForceType : uint {
				Rotor = 1,
				Rudder = 2,
				Wing = 3
			};
			struct Output {
				real_T thrust;
				real_T resistance;
				real_T torque_scaler;
				real_T speed;
				real_T angle;
				UniForceParams::UniForceDirection turning_direction;
				real_T control_signal_filtered;
				real_T control_signal_input;
				UniForceType type;
				Vector3r vel_input;
				Vector3r torq_resist;

				real_T airspeed_x_2 = 0;
				real_T airspeed_z_2 = 0;
				real_T c_1 = 0;
				real_T c_2 = 0;
				real_T c_3 = 0;
				real_T c_4 = 0;
			};
			
		public: //methods
			
			void initialize(const Vector3r& position, const Vector3r& normal, UniForceParams::UniForceDirection turning_direction,
				 const Environment* environment, uint id = -1)
			{
				id_ = id;
				turning_direction_ = turning_direction;
				environment_ = environment;
				air_density_sea_level_ = EarthUtils::getAirDensity(0.0f);
				real_T csf_tc = this->getCtrlSigFltTC();
				control_signal_filter_.initialize(csf_tc, 0, 0);
				air_speed_ = Vector3r::Zero();
				rotation_ = Vector3r::Zero();

				PhysicsBodyVertex::initialize(position, normal);   //call base initializer
			}
			virtual uint getActID() const 
			{
				return 0;
			}
			void setType(UniForceType type)
			{
				output_.type = type;
			}
			UniForceType getType() const
			{
				return output_.type;
			}
			void setAirSpeed(Vector3r air_speed)
			{
				air_speed_ = air_speed;
			}
			Vector3r getAirSpeed() const
			{
				return air_speed_;
			}
			void setRotation(Vector3r rot)
			{
				rotation_ = rot;
			}
			Vector3r getRotation() const
			{
				return rotation_;
			}
			void setForwardProjection(Vector3r norm)
			{
				unused( norm);
			}
			//0 to 1 - will be scaled to 0 to max_speed
			virtual void setControlSignal(real_T control_signal)
			{
				//control_signal_filter_.setInput(Utils::clip(control_signal, 0.0f, 1.0f));
				control_signal_filter_.setInput(control_signal);
			}
			void debugCtrlSignal(real_T control_signal)
			{
				control_signal_filter_.setDebugOutput(control_signal);
			}
			void debugSetOutput()
			{
				setOutput(output_, control_signal_filter_);
			}
			void debugSetAirDensityRatio()
			{
				air_density_ratio_ = 1.0f;
			}

			Output getOutput() const
			{
				return output_;
			}
			real_T getAirDensityRatio()
			{
				return air_density_ratio_;
			}
			UniForceParams::UniForceDirection getTurningDirection()
			{
				return turning_direction_;
			}

			
			//*** Start: UpdatableState implementation ***//
			virtual void resetImplementation() override
			{
				PhysicsBodyVertex::resetImplementation();

				//update environmental factors before we call base
				updateEnvironmentalFactors();

				control_signal_filter_.reset();

				setOutput(output_, control_signal_filter_);
			}

			virtual void update() override
			{
				//update environmental factors before we call base
				updateEnvironmentalFactors();

				//this will in turn call setWrench
				PhysicsBodyVertex::update();

				//update our state
				setOutput(output_, control_signal_filter_);

				//update filter - this should be after so that first output is same as initial
				control_signal_filter_.update();
			}


			//*** End: UpdatableState implementation ***//

		private: //methods
			//static void setOutput(Output& output, const UniForceParams& params, const FirstOrderFilter<real_T>& control_signal_filter, UniForceParams::UniForceDirection turning_direction)
			//{
			//	output.control_signal_input = control_signal_filter.getInput();
			//	output.control_signal_filtered = control_signal_filter.getOutput();
			//	//see relationship of rotation speed with thrust: http://physics.stackexchange.com/a/32013/14061
			//	output.speed = sqrt(output.control_signal_filtered * params.max_speed_square);
			//	output.thrust = output.control_signal_filtered * params.max_thrust;
			//	output.torque_scaler = output.control_signal_input * params.max_torque * static_cast<int>(turning_direction);
			//	output.turning_direction = turning_direction;
			//}
			virtual void setOutput(Output& output, const FirstOrderFilter<real_T>& control_signal_filter)
			{
				unused(output);
				unused(control_signal_filter);
			}

			void updateEnvironmentalFactors()
			{
				//update air density ration - this will affect generated force and torques by rotors
				air_density_ratio_ = environment_->getState().air_density / air_density_sea_level_;

			}
			virtual UniForceParams& getParams() const = 0;
			virtual real_T getCtrlSigFltTC() const = 0;
			
		private: //fields
			uint id_; //only used for debug messages
			UniForceParams::UniForceDirection turning_direction_;
			
			FirstOrderFilter<real_T> control_signal_filter_;
			const Environment* environment_ = nullptr;
			real_T air_density_sea_level_, air_density_ratio_;
			Output output_;
			

			Vector3r air_speed_;
			Vector3r rotation_;
		};


	}
} //namespace
#endif
