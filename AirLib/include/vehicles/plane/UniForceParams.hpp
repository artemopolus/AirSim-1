#ifndef msr_airlib_UniForceParams_hpp
#define msr_airlib_UniForceParams_hpp


#include "common/Common.hpp"
namespace msr {
	namespace airlib {
		
		class UniForceParams
		{
		public:
			enum class UniForceDirection :int {
				UniForceDirectionBack = -1,
				UniForceDirectionFront = 1
			};
			real_T C_T;
			real_T C_P;
			real_T air_density;
			real_T max_rpm;
			real_T propeller_diameter;
			real_T propeller_height;
			real_T control_signal_filter_tc;

			real_T revolutions_per_second;
			real_T max_speed;
			real_T max_speed_square;
			real_T max_thrust;
			real_T max_torque;
		public:
			UniForceParams()
			{
				initialize();
			}
			virtual ~UniForceParams()
			{

			}
			virtual void calculateMaxThrust() {
			}
			virtual void calculateMaxThrust( real_T value) {
				unused(value);
				 throw std::out_of_range("no physics vertex are available");
			}
			virtual void calculateMaxThrust( real_T datamass[] ) {
				unused(datamass);
				 throw std::out_of_range("no physics vertex are available");
			}
		private:
			virtual void initialize()
			{
				// do not do anything
			}
		};

	}
}

#endif
