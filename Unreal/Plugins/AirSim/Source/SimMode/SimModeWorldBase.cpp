#include "SimModeWorldBase.h"
#include <exception>
#include "AirBlueprintLib.h"


void ASimModeWorldBase::BeginPlay()
{
    Super::BeginPlay();

    manual_pose_controller = NewObject<UManualPoseController>();
    setupInputBindings();
}

void ASimModeWorldBase::initializeForPlay()
{
    physics_world_.reset(new msr::airlib::PhysicsWorld(
        createPhysicsEngine(), toUpdatableObjects(vehicles_), 
        getPhysicsLoopPeriod()));

    if (getSettings().usage_scenario == kUsageScenarioComputerVision) {
        manual_pose_controller->initializeForPlay();
        manual_pose_controller->setActor(getVehicleSimApi()->getPawn());
    }
}

void ASimModeWorldBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //remove everything that we created in BeginPlay
    physics_world_.reset();
    physics_engine_.reset();
    manual_pose_controller = nullptr;

    Super::EndPlay(EndPlayReason);
}

void ASimModeWorldBase::startAsyncUpdator()
{
    physics_world_->startAsyncUpdator();
}
void ASimModeWorldBase::stopAsyncUpdator()
{
    physics_world_->stopAsyncUpdator();
}


std::vector<ASimModeWorldBase::UpdatableObject*> ASimModeWorldBase::toUpdatableObjects(
    const std::vector<ASimModeWorldBase::VehiclePtr>& vehicles)
{
    std::vector<UpdatableObject*> bodies;
    for (const VehiclePtr& body : vehicles)
        bodies.push_back(body.get());

    return bodies;
}


ASimModeWorldBase::PhysicsEngineBase* ASimModeWorldBase::createPhysicsEngine()
{
    std::string physics_engine_name = getSettings().physics_engine_name;
    if (physics_engine_name == "" || getSettings().usage_scenario == kUsageScenarioComputerVision)
        physics_engine_.reset(); //no physics engine
    else if (physics_engine_name == "FastPhysicsEngine") {
        msr::airlib::Settings fast_phys_settings;
        if (msr::airlib::Settings::singleton().getChild("FastPhysicsEngine", fast_phys_settings)) {
            physics_engine_.reset(
                new msr::airlib::FastPhysicsEngine(fast_phys_settings.getBool("EnableGroundLock", true))
            );
        }
        else {
            physics_engine_.reset(
                new msr::airlib::FastPhysicsEngine()
            );
        }
    }
    else {
        physics_engine_.reset();
        UAirBlueprintLib::LogMessageString("Unrecognized physics engine name: ",  physics_engine_name, LogDebugLevel::Failure);
    }

    return physics_engine_.get();
}

bool ASimModeWorldBase::isPaused() const
{
    return physics_world_->isPaused();
}

void ASimModeWorldBase::pause(bool is_paused)
{
    physics_world_->pause(is_paused);
}

void ASimModeWorldBase::continueForTime(double seconds)
{
    physics_world_->continueForTime(seconds);

}

void ASimModeWorldBase::Tick(float DeltaSeconds)
{
    { //keep this lock as short as possible
        physics_world_->lock();

        physics_world_->enableStateReport(EnableReport);
        physics_world_->updateStateReport();

        for (auto& pair : getApiProvider()->getVehicleSimApis())
            pair.second->updateRenderedState(DeltaSeconds);

        physics_world_->unlock();
    }

    //perform any expensive rendering update outside of lock region
    for (auto& pair : getApiProvider()->getVehicleSimApis())
        pair.second->updateRendering(DeltaSeconds);

    Super::Tick(DeltaSeconds);
}

void ASimModeWorldBase::reset()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        physics_world_->reset();
    }, true);
    
    Super::reset();
}

std::string ASimModeWorldBase::getDebugReport()
{
    return physics_world_->getDebugReport();
}


