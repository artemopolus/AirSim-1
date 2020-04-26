#ifndef msr_airlib_UFRotorParams_hpp
#define msr_airlib_UFRotorParams_hpp

#include "UniForceParams.hpp"
namespace msr {
	namespace airlib {
		class UFRotorParams : public UniForceParams
		{
		public:
			UFRotorParams()
			{
				initialize();
			}
			virtual void calculateMaxThrust() {
				revolutions_per_second = max_rpm / 60;
				max_speed = revolutions_per_second * 2 * M_PIf;  // radians / sec
				max_speed_square = pow(max_speed, 2.0f);

				real_T nsquared = revolutions_per_second * revolutions_per_second;
				max_thrust = C_T * air_density * nsquared * pow(propeller_diameter, 4);
				max_torque = C_P * air_density * nsquared * pow(propeller_diameter, 5) / (2 * M_PIf);
			}
			virtual void calculateMaxThrust( real_T value) {
				max_rpm = value;
				calculateMaxThrust();
			}
			virtual void calculateMaxThrust( real_T datamass[] ) {
				max_rpm = datamass[0];
				air_density = datamass[2];
				calculateMaxThrust();
			}
			virtual void calculateMaxThrust( std::vector<float>( datamass) )
			{
				if (datamass.size() != 5)
					return;
				max_rpm = datamass[0];
				propeller_diameter = datamass[1];
				air_density = datamass[2];
				C_T = datamass[3];
				C_P = datamass[4];
				calculateMaxThrust();
			}
			real_T getMultiResistance() const
			{
				return multiR;
			}
		private:
			void initialize() override
			{
				/*
				Ref: http://physics.stackexchange.com/a/32013/14061
				force in Newton = C_T * \rho * n^2 * D^4
				torque in N.m = C_P * \rho * n^2 * D^5 / (2*pi)
				where,
				\rho = air density (1.225 kg/m^3)
				n = radians per sec
				D = propeller diameter in meters
				C_T, C_P = dimensionless constants available at
				propeller performance database http://m-selig.ae.illinois.edu/props/propDB.html

				We use values for GWS 9X5 propeller for which,
				C_T = 0.109919, C_P = 0.040164 @ 6396.667 RPM
				*/
				C_T = 0.109919f; // the thrust co-efficient @ 6396.667 RPM, measured by UIUC.
				C_P = 0.040164f; // the torque co-efficient at @ 6396.667 RPM, measured by UIUC.
				air_density = 1.225f; //  kg/m^3
				max_rpm = 6396.667f; // revolutions per minute
				propeller_diameter = 0.2286f;   //diameter in meters, default is for DJI Phantom 2
				propeller_height = 1 / 100.0f;   //height of cylindrical area when propeller rotates, 1 cm
				control_signal_filter_tc = 0.005f;    //time constant for low pass filter

				max_thrust = 4.179446268f; //computed from above formula for the given constants
				max_torque = 0.055562f; //computed from above formula
				calculateMaxThrust();
				multiR = 0.000005f;
			}
		private:
			real_T multiR;
		};
	}
}
#endif