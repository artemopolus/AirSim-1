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

			}
		protected:
			void setWrench(Wrench& wrench) override
			{
				Vector3r normal = getNormal();
				//forces and torques are proportional to air density: http://physics.stackexchange.com/a/32013/14061
				wrench.force = normal * (getOutput().thrust + getOutput().resistance)* getAirDensityRatio();
				wrench.torque = normal * getOutput().torque_scaler * getAirDensityRatio(); //TODO: try using filtered control here
			}
		private:
			void setOutput(Output& output, const FirstOrderFilter<real_T>& control_signal_filter) override
			{
				output.control_signal_input = 0.0f;
				output.control_signal_filtered = 0.0f;
				//see relationship of rotation speed with thrust: http://physics.stackexchange.com/a/32013/14061
				output.speed = 0.0f;
				output.thrust = getAirSpeed().x() * getAirSpeed().x() * params_->getMlift() * static_cast<int>(getTurningDirection());
				output.torque_scaler = 0.0f;
				output.turning_direction = getTurningDirection();
				output.resistance = std::abs(getAirSpeed().z()) * getAirSpeed().z() * params_->getMresi();
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
