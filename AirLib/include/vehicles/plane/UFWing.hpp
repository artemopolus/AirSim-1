#pragma once
#ifndef airsimcore_ufwing_hpp
#define airsimcore_ufwing_hpp

#include "UniForce.hpp"
#include "UFWingParams.hpp"

namespace msr {
	namespace airlib {
		class UFWing : public UniForce
		{
		public:
			UFWing(const Vector3r& position, const Vector3r& normal, UFWingParams::UniForceDirection turning_direction,
				UFWingParams * params, const Environment* environment, uint id = -1)
				: params_(params)
			{
				initialize(position, normal, turning_direction, environment, id);
				setType(UniForceType::Wing);
				setObjType(UpdatableObject::typeUpdObj::wing);
				setWrench2Zero();
				//params_->calculateMaxThrust();
			}
			UFWingParams * getCurrentParams() const
			{
				return params_;
			}
			void reportState(StateReporter& reporter) override
			{
				reporter.writeValue("Dir", static_cast<int>(getTurningDirection()));
				reporter.writeValue("Ctrl-in", getOutput().control_signal_input);
				reporter.writeValue("Ctrl-fl", getOutput().control_signal_filtered);
				reporter.writeValue("speed", getOutput().speed);
				reporter.writeValue("thrust", getOutput().thrust);
				reporter.writeValue("torque", getOutput().torque_scaler);
			}
			
			void setControlSignal(real_T control_signal) override
			{
				unused(control_signal);
			}
		protected:
			void setWrench(Wrench& wrench) override
			{
				Vector3r normal = getNormal();
				//forces and torques are proportional to air density: http://physics.stackexchange.com/a/32013/14061
				wrench.force = normal * (getOutput().thrust + getOutput().resistance)* getAirDensityRatio();
				wrench.torque = getOutput().torq_resist + normal * getOutput().torque_scaler * getAirDensityRatio(); //TODO: try using filtered control here
			}
		private:
			void setOutput(Output& output, const FirstOrderFilter<real_T>& control_signal_filter) override
			{
				unused(control_signal_filter);
				output.control_signal_input = 0.0f;
				output.control_signal_filtered = 0.0f;
				//see relationship of rotation speed with thrust: http://physics.stackexchange.com/a/32013/14061
				output.speed = 0.0f;
				real_T pitch, roll, yaw;
				Quaternionr angle_attack = Quaternionr::FromTwoVectors( Vector3r(getAirSpeed().x(), 0, getAirSpeed().z()),Vector3r(1, 0, 0));
				VectorMath::toEulerianAngle(angle_attack, pitch, roll, yaw);
				float unit = 180.0f / (float)M_PI;
				pitch *= unit;
				real_T c_lift = params_->getClift(pitch);
				real_T c_drag = params_->getCdrag(pitch);
				float a_x_2 = getAirSpeed().x() * getAirSpeed().x();
				float a_z_2 = std::abs(getAirSpeed().z()) * getAirSpeed().z();
				output.thrust =  a_x_2 * c_lift * params_->getMlift();
				output.torque_scaler = 0.0f;
				output.turning_direction = getTurningDirection();
				output.resistance = a_z_2 * c_drag * params_->getMresi();
				output.angle = pitch;

				output.airspeed_x_2 = a_x_2;
				output.airspeed_z_2 = a_z_2;
				output.c_1 = c_lift;
				output.c_2 = c_drag;

				Vector3r torq = getRotation();
				Vector3r resist = params_->getVelocityResistence();

				float r_x = - std::abs(torq.x()) * torq.x() * resist.x();
				float r_y = - std::abs(torq.y()) * torq.y() * resist.y();
				float r_z = - std::abs(torq.z()) * torq.z() * resist.z();
				output.torq_resist = Vector3r(r_x, r_y, r_z);
			}
			UniForceParams& getParams() const override
			{
				return (UniForceParams &)params_;
			}
			real_T getCtrlSigFltTC() const override
			{
				return params_->control_signal_filter_tc;
			}
		private:
			UFWingParams * params_;
		};
	}
}

#endif
