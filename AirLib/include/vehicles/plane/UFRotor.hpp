#ifndef airsimcore_ufrotor_hpp
#define airsimcore_ufrotor_hpp

#include "UniForce.hpp"
#include "UFRotorParams.hpp"

namespace msr { namespace airlib {
		class UFRotor : public UniForce
		{
		public:
			UFRotor(const Vector3r& position, const Vector3r& normal, UFRotorParams::UniForceDirection turning_direction,
				UFRotorParams * params, const Environment* environment, uint id = -1)
				: params_(params)
			{
				initialize(position, normal, turning_direction, environment, id);
				setType(UniForceType::Rotor);
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
			virtual void setControlSignal(real_T control_signal) override
			{
				//control_signal_filter_.setInput(Utils::clip(control_signal, 0.0f, 1.0f));
				real_T ctrl = Utils::clip(control_signal, 0.0f, 1.0f);
				UniForce::setControlSignal(ctrl);
			}
		protected:
			virtual void setWrench(Wrench& wrench) override
			{
				Vector3r normal = getNormal();
				//forces and torques are proportional to air density: http://physics.stackexchange.com/a/32013/14061
				wrench.force = normal * getOutput().thrust * getAirDensityRatio();
				wrench.torque = normal * getOutput().torque_scaler * getAirDensityRatio(); //TODO: try using filtered control here
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
			UFRotorParams * params_;
		};
}}

#endif