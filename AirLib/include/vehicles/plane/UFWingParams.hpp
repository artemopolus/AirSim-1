#pragma once
#ifndef msr_airlib_UFWingParams_hpp
#define msr_airlib_UFWingParams_hpp

#include "UniForceParams.hpp"
namespace msr {
	namespace airlib {
		class UFWingParams : public UniForceParams
		{
		public:
			UFWingParams()
			{
				initialize();
			}
			virtual void calculateMaxThrust() {
				multLift = C_lift * S;
				multResi = C_resi * S;
			}
			virtual void calculateMaxThrust(real_T value) {
				S = value;
				calculateMaxThrust();
			}
			virtual void calculateMaxThrust(real_T rudder_lengths[]) {
				S = rudder_lengths[0];
				C_lift = rudder_lengths[1];
				C_resi = rudder_lengths[2];
				calculateMaxThrust();
			}
			virtual void calculateMaxThrust( std::vector<float>( datamass) )
			{
				if (datamass.size() != 4)
					return;
				wing_length = datamass[0];
				wing_width = datamass[1];
				S = wing_length * wing_width;
				C_lift = datamass[2];
				C_resi = datamass[3];
				calculateMaxThrust();
			}
			real_T getMlift() const
			{
				return multLift;
			}
			real_T getMresi() const
			{
				return multResi;
			}
		private:
			void initialize() override
			{
				/*

			»так, основные моменты, касающиес€ рул€:
			ћаксимальный поворот в одну сторону
			ћаксимальный поворот в другую сторону
			ѕодъемна€ сила на единицу длины?

			*/
				wing_length = 1.0f; // rad
				wing_width = 1.0f; // rad
				C_lift = 0.01f; // Newton per meter
				C_resi = 0.00005f;
				S = wing_length * wing_width;

				max_torque = 0.000005f; //есть ли какое-то вли€ние вращени€ от рул€???? или проще просто ноль ввести
				control_signal_filter_tc = 0.005f;    //time constant for low pass filter
				// какие-то данные рассчитанные после
				calculateMaxThrust();
			}
		private:
			real_T S;
			real_T C_lift;
			real_T C_resi;
			real_T wing_length;
			real_T wing_width;

			real_T multLift;
			real_T multResi;
		};
	}
}
#endif
