#pragma once

#include "CoreMinimal.h"

#include "PawnSimApi.h"
#include "vehicles/plane/Plane.hpp"
#include "vehicles/plane/PlaneParams.hpp"
#include "physics/Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "common/common_utils/UniqueValueMap.hpp" 
#include "PlanePawnEvents.h"
#include <future>


class PlanePawnSimApi : public PawnSimApi
{
public:
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::Plane Plane;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::Pose Pose;

    typedef PlanePawnEvents::RotorInfo RotorInfo;

public:
    virtual void initialize() override;

    virtual ~PlanePawnSimApi() = default;

    //VehicleSimApiBase interface
    //implements game interface to update pawn
    PlanePawnSimApi(const Params& params);
    virtual void updateRenderedState(float dt) override;
    virtual void updateRendering(float dt) override;

    //PhysicsBody interface
    //this just wrapped around Plane physics body
    virtual void resetImplementation() override;
    virtual void update() override;
    virtual void reportState(StateReporter& reporter) override;
    virtual UpdatableObject* getPhysicsBody() override;

    virtual void setPose(const Pose& pose, bool ignore_collision) override;
    virtual void pawnTick(float dt) override;

    msr::airlib::PlaneApiBase* getVehicleApi() const
    {
        return vehicle_api_.get();
    }

    virtual msr::airlib::VehicleApiBase* getVehicleApiBase() const override
    {
        return vehicle_api_.get();
    }

private:
    std::unique_ptr<msr::airlib::PlaneApiBase> vehicle_api_;
    std::unique_ptr<msr::airlib::PlaneParams> vehicle_params_;

    std::unique_ptr<Plane> phys_vehicle_;
    unsigned int rotor_count_;
    std::vector<RotorInfo> rotor_info_;

    //show info on collision response from physics engine
    CollisionResponse collision_response;

    PlanePawnEvents* pawn_events_;

    //when pose needs to set from non-physics thread, we set it as pending
    bool pending_pose_collisions_;
    enum class PendingPoseStatus {
        NonePending, RenderStatePending, RenderPending
    } pending_pose_status_;
    Pose pending_phys_pose_; //force new pose through API

    //reset must happen while World is locked so its async task initiated from API thread
    bool reset_pending_;
    bool did_reset_;
    std::packaged_task<void()> reset_task_;

    Pose last_phys_pose_; //for trace lines showing vehicle path
    std::vector<std::string> vehicle_api_messages_;
};
