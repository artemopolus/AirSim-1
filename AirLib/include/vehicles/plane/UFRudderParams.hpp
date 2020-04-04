#ifndef msr_airlib_UFRudderParams_hpp
#define msr_airlib_UFRudderParams_hpp

#include "UniForceParams.hpp"
namespace msr {
	namespace airlib {
		class UFRudderParams : public UniForceParams
		{
		public:
			UFRudderParams()
			{
				initialize();
			}
			virtual void calculateMaxThrust() {
				real_T value = 1.0f;
				rudder_length_1 = value;
				rudder_length_2 = value;
			}
			virtual void calculateMaxThrust(real_T value) {
				rudder_length_1 = value;
				rudder_length_2 = value;
				calculateMaxThrust();
			}
			virtual void calculateMaxThrust(real_T rudder_lengths[]) {
				rudder_length_1 = rudder_lengths[0];
				rudder_length_2 = rudder_lengths[1];
				calculateMaxThrust();
			}
			real_T getMaxAngle() const
			{
				return max_angle_1;
			}
			real_T getMaxThrust() const
			{
				return max_angle_1;
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
				max_angle_1 = 1.05f; // rad
				max_angle_2 = 1.05f; // rad
				thrust_per_lenght = 1.0f; // Newton per meter

				
				max_torque = 0.000005f; //есть ли какое-то вли€ние вращени€ от рул€???? или проще просто ноль ввести
				control_signal_filter_tc = 0.005f;    //time constant for low pass filter
				// какие-то данные рассчитанные после
			}
		private:
			real_T max_angle_1;
			real_T max_angle_2;
			real_T thrust_per_lenght;
			real_T max_thrust_angle_1;
			real_T max_thrust_angle_2;
			real_T rudder_length_1;
			real_T rudder_length_2;
		};
	}
}
#endif