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
			void setNormal(Vector3r norm)
			{
				normal_ = norm;
			}
			Vector3r getNormal() const
			{
				return normal_;
			}
			void setPosition(Vector3r pos)
			{
				position_ = pos;
			}
			Vector3r getPosition() const
			{
				return position_;
			}
			void setActID(uint id)
			{
				act_id_ = id;
			}
			uint getActID() const
			{
				return act_id_;
			}
			virtual ~UniForceParams()
			{

			}
			virtual void calculateMaxThrust() {
				throw std::out_of_range("no max thrust are available");
			}
			virtual void calculateMaxThrust( real_T value) {
				unused(value);
				 throw std::out_of_range("no max thrust are available");
			}
			virtual void calculateMaxThrust( real_T datamass[] ) {
				unused(datamass);
				 throw std::out_of_range("no max thrust are available");
			}
			virtual void calculateMaxThrust( std::vector<float>( datamass) ) {
				unused(datamass);
				 throw std::out_of_range("no max thrust are available");
			}
		private:
			virtual void initialize()
			{

			}
		private:
			Vector3r normal_;
			Vector3r position_;
			uint act_id_ = 0;
		};

	}
}

#endif
