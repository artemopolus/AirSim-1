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
			virtual void setAirSpeed(Vector3r air_speed)
			{
				_air_speed = air_speed;
			}
			virtual void setControlSignal(real_T control_signal) override
			{
				real_T ctrl = control_signal;
				UniForce::setControlSignal(ctrl);
			}
		
		protected:
			virtual void setWrench(Wrench& wrench) override
			{
				Vector3r normal = getNormal();
				wrench.force = normal * getOutput().thrust * getAirDensityRatio();
				wrench.torque = Vector3r::Zero();
			}
		private:
			virtual void setOutput(Output& output, const FirstOrderFilter<real_T>& control_signal_filter)
			{
				output.control_signal_input = control_signal_filter.getInput();
				output.control_signal_filtered = control_signal_filter.getOutput();
				output.angle = output.control_signal_filtered * params_->getMaxAngle();
				//output.speed = sqrt(output.control_signal_filtered * params_->max_speed_square);
				output.thrust = output.control_signal_filtered * params_->getMaxThrust() * static_cast<int>(getTurningDirection());
				output.torque_scaler = 0;
				output.turning_direction = getTurningDirection();
			}
			virtual UniForceParams& getParams()
			{
				return (UniForceParams &)params_;
			}
		private:
			UFRudderParams * params_;
			Vector3r _air_speed;
		};
	}
}

#endif