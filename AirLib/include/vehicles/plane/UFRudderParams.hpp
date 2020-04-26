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
				max_angle_1 = 1.05f;
				max_thrust_angle_1 = 0.1f;
				C_res = 0.5;
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
			virtual void calculateMaxThrust( std::vector<float>( datamass) )
			{
				if (datamass.size() != 3)
					return;
				max_angle_1 = datamass[0];
				max_thrust_angle_1 = datamass[1];
				C_res = datamass[2];
				//calculateMaxThrust();
			}
			real_T getMaxAngle() const
			{
				return max_angle_1;
			}
			real_T getMaxThrust() const
			{
				return max_thrust_angle_1;
			}
			real_T getResistance() const
			{
				return C_res;
			}
		private:
			void initialize() override
			{
				/*

			����, �������� �������, ���������� ����:
			������������ ������� � ���� �������
			������������ ������� � ������ �������
			��������� ���� �� ������� �����?

			*/
				max_angle_1 = 1.05f; // rad
				max_angle_2 = 1.05f; // rad
				thrust_per_lenght = 1.0f; // Newton per meter

				
				max_torque = 0.000005f; //���� �� �����-�� ������� �������� �� ����???? ��� ����� ������ ���� ������
				control_signal_filter_tc = 0.005f;    //time constant for low pass filter
				// �����-�� ������ ������������ �����
			}
		private:
			real_T max_angle_1;
			real_T max_angle_2;
			real_T thrust_per_lenght;
			real_T max_thrust_angle_1;
			real_T max_thrust_angle_2;
			real_T rudder_length_1;
			real_T rudder_length_2;
			real_T C_res;
		};
	}
}
#endif