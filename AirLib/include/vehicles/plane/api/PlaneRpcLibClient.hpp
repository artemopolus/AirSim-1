// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_PlaneRpcLibClient_hpp
#define air_PlaneRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/plane/api/PlaneApiBase.hpp"
#include "api/RpcLibClientBase.hpp"
#include "vehicles/plane/api/PlaneCommon.hpp"


namespace msr { namespace airlib {

class PlaneRpcLibClient : public RpcLibClientBase {
public:
    PlaneRpcLibClient(const string& ip_address = "localhost", uint16_t port = RpcLibPort, float timeout_sec = 60);

    PlaneRpcLibClient* takeoffAsync(float timeout_sec = 20, const std::string& vehicle_name = "");
    PlaneRpcLibClient* landAsync(float timeout_sec = 60, const std::string& vehicle_name = "");
    PlaneRpcLibClient* goHomeAsync(float timeout_sec = Utils::max<float>(), const std::string& vehicle_name = "");

    PlaneRpcLibClient* moveByAngleZAsync(float pitch, float roll, float z, float yaw, float duration, const std::string& vehicle_name = "");
    PlaneRpcLibClient* moveByAngleThrottleAsync(float pitch, float roll, float throttle, float yaw_rate, float duration, const std::string& vehicle_name = "");
    PlaneRpcLibClient* moveByVelocityAsync(float vx, float vy, float vz, float duration,
        PlaneDrivetrainType drivetrain = PlaneDrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), const std::string& vehicle_name = "");
    PlaneRpcLibClient* moveByVelocityZAsync(float vx, float vy, float z, float duration,
        PlaneDrivetrainType drivetrain = PlaneDrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), const std::string& vehicle_name = "");
    PlaneRpcLibClient* moveOnPathAsync(const vector<Vector3r>& path, float velocity, float timeout_sec = Utils::max<float>(),
        PlaneDrivetrainType drivetrain = PlaneDrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), 
        float lookahead = -1, float adaptive_lookahead = 1, const std::string& vehicle_name = "");
    PlaneRpcLibClient* moveToPositionAsync(float x, float y, float z, float velocity, float timeout_sec = Utils::max<float>(),
        PlaneDrivetrainType drivetrain = PlaneDrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), 
        float lookahead = -1, float adaptive_lookahead = 1, const std::string& vehicle_name = "");
    PlaneRpcLibClient* moveToZAsync(float z, float velocity, float timeout_sec = Utils::max<float>(),
        const YawMode& yaw_mode = YawMode(), float lookahead = -1, float adaptive_lookahead = 1, const std::string& vehicle_name = "");
    PlaneRpcLibClient* moveByManualAsync(float vx_max, float vy_max, float z_min, float duration,
        PlaneDrivetrainType drivetrain = PlaneDrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), const std::string& vehicle_name = "");
    PlaneRpcLibClient* rotateToYawAsync(float yaw, float timeout_sec = Utils::max<float>(), float margin = 5, const std::string& vehicle_name = "");
    PlaneRpcLibClient* rotateByYawRateAsync(float yaw_rate, float duration, const std::string& vehicle_name = "");
    PlaneRpcLibClient* hoverAsync(const std::string& vehicle_name = "");

    void moveByRC(const RCData& rc_data, const std::string& vehicle_name = "");


    PlaneState getPlaneState(const std::string& vehicle_name = "");

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z, const std::string& vehicle_name = "");

    virtual PlaneRpcLibClient* waitOnLastTask(bool* task_result = nullptr, float timeout_sec = Utils::nan<float>()) override;

    virtual ~PlaneRpcLibClient();    //required for pimpl

private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif
