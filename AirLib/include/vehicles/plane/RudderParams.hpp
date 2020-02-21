#pragma once
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_RudderParams_hpp
#define msr_airlib_RudderParams_hpp


#include "common/Common.hpp"

namespace msr {
	namespace airlib {


		//In NED system, +ve torque would generate clockwise rotation
		// � ����� ������� �������������� ����, ����� ��?
		enum class RudderTurningDirection :int {
			RudderTurningDirectionCCW = -1,
			RudderTurningDirectionCW = 1
		};

		struct RudderParams {
			/*
			
			����, �������� �������, ���������� ����:
			������������ ������� � ���� �������
			������������ ������� � ������ �������
			��������� ���� �� ������� �����?

			*/
			real_T max_angle_1 = 1.05f; // rad
			real_T max_angle_2 = 1.05f; // rad
			real_T thrust_per_lenght = 1.0f; // Newton per meter

			real_T max_thrust_angle_1;
			real_T max_thrust_angle_2;
			real_T max_torque = 0.000005f; //���� �� �����-�� ������� �������� �� ����???? ��� ����� ������ ���� ������
			real_T control_signal_filter_tc = 0.005f;    //time constant for low pass filter
			// �����-�� ������ ������������ �����
			void calculateMaxThrust(
				real_T rudder_lengths[] /*  */
			)
			{
				max_thrust_angle_1 = thrust_per_lenght * sin(max_angle_1) * rudder_lengths[0];
				max_thrust_angle_2 = thrust_per_lenght * sin(max_angle_2) * rudder_lengths[1];
			}

		};


	}
} //namespace
#endif
