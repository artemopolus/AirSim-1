#include "PlaneFlyingPawn.h"
#include "Components/StaticMeshComponent.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "common/Common.hpp"
#include "common/Settings.hpp"
APlaneFlyingPawn::APlaneFlyingPawn()
{
    pawn_events_.getActuatorSignal().connect_member(this, &APlaneFlyingPawn::setRotorSpeed);
    //logfilename_ = std::string("J:/Unreal/LogAirSim/") + std::string(TCHAR_TO_UTF8(*GetName())) + std::string("_PawnAct.txt");;
    std::string name = std::string(TCHAR_TO_UTF8(*GetName())) + std::string("_PawnAct.txt");
    logfilename_ = Settings::getUserDirectoryFullPath(name);

    Logger_.open(logfilename_, true);
}

void APlaneFlyingPawn::BeginPlay()
{
    Super::BeginPlay();

    for (auto i = 0; i < rotor_count; ++i) {
        rotating_movements_[i] = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Rotation") + FString::FromInt(i));
    }
    for (auto i = 0; i < rudder_count; ++i) {
        rudder_orientation_[i] = UAirBlueprintLib::GetActorComponent<USceneComponent>(this, TEXT("RudderHandle") + FString::FromInt(i));
    }
}

void APlaneFlyingPawn::initializeForBeginPlay()
{    
    //get references of existing camera
    camera_front_right_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontRightCamera")))->GetChildActor());
    camera_front_left_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontLeftCamera")))->GetChildActor());
    camera_front_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontCenterCamera")))->GetChildActor());
    camera_back_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BackCenterCamera")))->GetChildActor());
    camera_bottom_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BottomCenterCamera")))->GetChildActor());
}

void APlaneFlyingPawn::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    pawn_events_.getPawnTickSignal().emit(DeltaSeconds);
}


void APlaneFlyingPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    camera_front_right_ = nullptr;
    camera_front_left_ = nullptr;
    camera_front_center_ = nullptr;
    camera_back_center_ = nullptr;
    camera_bottom_center_ = nullptr;

    Super::EndPlay(EndPlayReason);
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> APlaneFlyingPawn::getCameras() const
{
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
    cameras.insert_or_assign("front_center", camera_front_center_);
    cameras.insert_or_assign("front_right", camera_front_right_);
    cameras.insert_or_assign("front_left", camera_front_left_);
    cameras.insert_or_assign("bottom_center", camera_bottom_center_);
    cameras.insert_or_assign("back_center", camera_back_center_);

    cameras.insert_or_assign("0", camera_front_center_);
    cameras.insert_or_assign("1", camera_front_right_);
    cameras.insert_or_assign("2", camera_front_left_);
    cameras.insert_or_assign("3", camera_bottom_center_);
    cameras.insert_or_assign("4", camera_back_center_);

    cameras.insert_or_assign("", camera_front_center_);
    cameras.insert_or_assign("fpv", camera_front_center_);

    return cameras;
}

void APlaneFlyingPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, 
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
        HitNormal, NormalImpulse, Hit);
}

void APlaneFlyingPawn::setRotorSpeed(const std::vector<PlanePawnEvents::RotorInfo>& rotor_infos)
{
    auto comp = rotating_movements_[0];
    float rotating = rotor_infos[0].rotor_speed;
    uint isEndl = 0;
    if (comp != nullptr)
    {
        Logger_.write("throtle:");
        Logger_.write(rotating);
        rotating *=  rotor_infos[0].rotor_direction * 180.0f / M_PIf * RotatorFactor;
        Logger_.write(rotating);
        comp->RotationRate.Yaw = rotating;
        isEndl = 1;
    }

    // for (auto rotor_index = 0; rotor_index < rotor_count ; ++rotor_index) {
    //     auto comp = rotating_movements_[rotor_index];
    //     if (comp != nullptr) {
    //         comp->RotationRate.Yaw = 
    //         rotor_infos.at(rotor_index).rotor_speed * rotor_infos.at(rotor_index).rotor_direction *
    //             180.0f / M_PIf * RotatorFactor;
    //     }
    // }
    for (auto rudder_index = 0; rudder_index < rudder_count ; ++rudder_index) {
        auto rot = rudder_orientation_[rudder_index];
        if (rot != nullptr) {
            float angle = rotor_infos[rotor_count + rudder_index].rotor_angle;
            Logger_.write("ang:");
            Logger_.write(angle);
            rot->SetRelativeRotation(FRotator(angle,0,0));
            isEndl = 1;
        }
    }
    if(isEndl)
        Logger_.endl();
}

