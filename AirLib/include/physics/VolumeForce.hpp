#ifndef airsim_core_volumeforce_hpp
#define airsim_core_volumeforce_hpp

#include "common/Common.hpp"
#include "common/UpdatableObject.hpp"
#include "common/CommonStructs.hpp"
#include "common/EarthUtils.hpp"

namespace msr { namespace airlib {
		class VolumeForce
		{
		public:
			 VolumeForce()
			{
				initialize();
			}
			virtual ~VolumeForce()
			{			}
			virtual Vector3r getValue() const
			{	
				return Vector3r::Zero();
			}
			virtual Vector3r getValue(Vector3r position) const
			{		
				unused(position);
				return Vector3r::Zero();
			}
			virtual void setShift(Vector3r value)
			{
				unused(value);
			}
			virtual void update()
			{			}
			virtual void reset()
			{			}
		private:
			virtual void initialize()
			{

			}
		};
}}
#endif