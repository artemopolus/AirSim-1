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
				setWrench2Zero();
				setObjType(UpdatableObject::typeUpdObj::rudder);
				//params_->calculateMaxThrust();
			}
			UFRudderParams * getCurrentParams() const
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
			
			uint getActID() const override
			{
				return params_->getActID();
			}
			void setControlSignal(real_T control_signal) override
			{
				real_T ctrl = Utils::clip(control_signal, 0.0f, 1.0f);
				UniForce::setControlSignal(ctrl);
			}
		
		protected:
			void setWrench(Wrench& wrench) override
			{
				Vector3r normal = getNormal();
				Vector3r forward = Vector3r(1, 0, 0);
				wrench.force = (normal * getOutput().thrust - forward*getOutput().resistance )* getAirDensityRatio();
				wrench.torque = Vector3r(0,0,0);
			}
		private:
			void setOutput(Output& output, const FirstOrderFilter<real_T>& control_signal_filter) override
			{
				output.control_signal_input = control_signal_filter.getInput();
				output.control_signal_filtered = control_signal_filter.getOutput();
				output.angle =  (output.control_signal_filtered - 0.5f) * params_->getMaxAngle() * static_cast<int>(getTurningDirection());
				float x = getAirSpeed().x();
				x *= x;
				output.thrust =  x * (output.control_signal_filtered - 0.5f) * params_->getMaxThrust() * static_cast<int>(getTurningDirection());
				output.resistance =  x * params_->getResistance();
				output.torque_scaler = 0.0f;
				output.turning_direction = getTurningDirection();
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
			UFRudderParams * params_;
		};
	}
}

#endif