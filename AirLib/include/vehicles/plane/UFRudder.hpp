#ifndef airsimcore_ufrudder_hpp
#define airsimcore_ufrudder_hpp

#include "UniForce.hpp"
#include "UFRudderParams.hpp"

namespace msr {
	namespace airlib {
		class UFRudder : public UniForce
		{
		public:
			UFRudder(const Vector3r& position, const Vector3r& normal, UFRudderParams::UniForceDirection turning_direction,
				UFRudderParams * params, const Environment* environment, uint id = -1)
				: params_(params)
			{
				initialize(position, normal, turning_direction, environment, id);
				setType(UniForceType::Rudder);
			}
			virtual void reportState(StateReporter& reporter) override
			{
				reporter.writeValue("Dir", static_cast<int>(getTurningDirection()));
				reporter.writeValue("Ctrl-in", getOutput().control_signal_input);
				reporter.writeValue("Ctrl-fl", getOutput().control_signal_filtered);
				reporter.writeValue("speed", getOutput().speed);
				reporter.writeValue("thrust", getOutput().thrust);
				reporter.writeValue("torque", getOutput().torque_scaler);
			}
		protected:
			virtual void setWrench(Wrench& wrench) override
			{
				//Vector3r normal = getNormal();
				////forces and torques are proportional to air density: http://physics.stackexchange.com/a/32013/14061
				//wrench.force = normal * getOutput().thrust * getAirDensityRatio();
				//wrench.torque = normal * getOutput().torque_scaler * getAirDensityRatio(); //TODO: try using filtered control here
			}
		private:
			virtual void setOutput(Output& output, const FirstOrderFilter<real_T>& control_signal_filter)
			{
				output.control_signal_input = control_signal_filter.getInput();
				output.control_signal_filtered = control_signal_filter.getOutput();
				//see relationship of rotation speed with thrust: http://physics.stackexchange.com/a/32013/14061
				output.speed = sqrt(output.control_signal_filtered * params_->max_speed_square);
				output.thrust = output.control_signal_filtered * params_->max_thrust;
				output.torque_scaler = output.control_signal_input * params_->max_torque * static_cast<int>(getTurningDirection());
				output.turning_direction = getTurningDirection();
			}
			virtual UniForceParams& getParams()
			{
				return (UniForceParams &)params_;
			}
		private:
			UFRudderParams * params_;
		};
	}
}

#endif