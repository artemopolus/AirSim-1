#ifndef airsim_core_wind_hpp
#define airsim_core_wind_hpp

#include "VolumeForce.hpp"

namespace msr { namespace airlib {
	class Wind : public VolumeForce
	{
	public:


		virtual Vector3r getValue() const
		{
			return _wind_force;
		}
		virtual Vector3r getValue(Vector3r position) const
		{
			// TODO: fix this for future volume objects
			unused(position);
			return _wind_force;
		}
		virtual void setShift(Vector3r value)
		{
			_wind_force += value;
		}
		virtual void update()
		{
			//nothing
		}
		virtual void reset()
		{
			initialize();
		}
	private:
		virtual void initialize()
		{
			_wind_force = Vector3r::Zero();
		}
	private:
		/* Здесь сейчас только статическое значение */
		Vector3r _wind_force;
	};
}}
#endif