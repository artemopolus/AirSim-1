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
				if (datamass.size() != 5)
					return;
				wing_length = datamass[0];
				wing_width = datamass[1];
				wing_height = datamass[2];
				S = wing_length * wing_width;
				C_lift = datamass[3];
				C_resi = datamass[4];
				calculateMaxThrust();
			}
			void setCoefAngAttMass( std::vector<float> angle, std::vector<float> c_lift, std::vector<float> c_drag)
			{
				if ((angle.size() != c_lift.size()) && (c_lift.size() != c_drag.size()))
					return;
				attack_angle_mass.clear();
				C_lift_mass.clear();
				C_drag_mass.clear();
				attack_angle_mass = angle;
				C_lift_mass = c_lift;
				C_drag_mass = c_drag;
			}
			real_T getClift(const float angle) const 
			{
				if ((angle < attack_angle_mass.front()) || (angle > attack_angle_mass.back()))
					return 0;
				for (uint i = 0; i < (attack_angle_mass.size() - 1); i++)
				{
					if ((angle >= attack_angle_mass[i]) && (angle <= attack_angle_mass[i + 1]))
					{
						return C_lift_mass[i] + (C_lift_mass[i + 1] - C_lift_mass[i]) * 
							(angle - attack_angle_mass[i])/
							(attack_angle_mass[i+1] - attack_angle_mass[i]);
					}
				}
				return 0;
			}
			real_T getCdrag(const float angle) const 
			{
				if ((angle < attack_angle_mass.front()) || (angle > attack_angle_mass.back()))
					return 0;
				for (uint i = 0; i < (attack_angle_mass.size() - 1); i++)
				{
					if ((angle >= attack_angle_mass[i]) && (angle <= attack_angle_mass[i + 1]))
					{
						return C_drag_mass[i] + (C_drag_mass[i + 1] - C_drag_mass[i]) * 
							(angle - attack_angle_mass[i])/
							(attack_angle_mass[i+1] - attack_angle_mass[i]);
					}
				}
				return 0;
			}
			real_T getMlift() const
			{
				return multLift;
			}
			real_T getMresi() const
			{
				return multResi;
			}
			real_T getWidth() const
			{
				return wing_width;
			}
			real_T getLength() const
			{
				return wing_length;
			}
			real_T getHeight() const
			{
				return wing_height;
			}
			void setVelocityResistence(Vector3r value)
			{
				VelocityResistence_ = value;
			}
			Vector3r getVelocityResistence() const
			{
				return VelocityResistence_;
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
			real_T wing_height;

			real_T multLift;
			real_T multResi;
			std::vector<real_T> attack_angle_mass = { 0,0,0 };
			std::vector<real_T> C_lift_mass = { 0,0,0 };
			std::vector<real_T> C_drag_mass = { 0,0,0 };
			Vector3r VelocityResistence_;
		};
	}
}
#endif
