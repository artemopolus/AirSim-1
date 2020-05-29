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
				real_T ctrl = Utils::clip(control_signal, -1.0f, 1.0f);
				UniForce::setControlSignal(ctrl);
			}
		
		protected:
			void setWrench(Wrench& wrench) override
			{
				Vector3r normal = getNormal();
				wrench.force = normal *( getOutput().thrust + getOutput().resistance )* getAirDensityRatio();
				wrench.torque = Vector3r(0,0,0);
			}
		private:
			void setOutput(Output& output, const FirstOrderFilter<real_T>& control_signal_filter) override
			{
				output.control_signal_input = control_signal_filter.getInput();
				output.control_signal_filtered = control_signal_filter.getOutput();
				auto ctrl_signal = output.control_signal_filtered - 0.5f;
				output.angle =  ctrl_signal * params_->getMaxAngle() ;

				//resistance drag
				Vector3r unit_z(0, 1, 0);  //NED frame
				Quaternionr angle_plane(AngleAxisr( output.angle, unit_z)); 
				Vector3r force2plane = VectorMath::rotateVector(Vector3r(getAirSpeed().x(), 0, getAirSpeed().z()), angle_plane, true);
				const float velocity_input = std::abs(force2plane.z()) * force2plane.z();
				output.resistance =  velocity_input * params_->getResistance();
				
				//airflow correction
				const float velocity_forward = getAirSpeed().x() * getAirSpeed().x();
				output.thrust = velocity_forward * ctrl_signal * params_->getMaxThrust() ;
				
				output.torque_scaler = 0.0f;
				output.turning_direction = getTurningDirection();
				output.vel_input = force2plane;
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