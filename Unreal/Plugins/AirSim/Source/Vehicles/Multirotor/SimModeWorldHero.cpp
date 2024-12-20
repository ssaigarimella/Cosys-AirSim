#include "SimModeWorldHero.h"
#include "UObject/ConstructorHelpers.h"
#include "Logging/MessageLog.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"

#include "AirBlueprintLib.h"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "Vehicles/SkidSteer/SkidVehiclePawnSimApi.h"
#include "MultirotorPawnSimApi.h"
#include "physics/PhysicsBody.hpp"
#include "common/ClockFactory.hpp"
#include <memory>
#include "vehicles/car/api/CarRpcLibServer.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"
#include "common/SteppableClock.hpp"

void ASimModeWorldHero::BeginPlay()
{
    Super::BeginPlay();

    // let base class setup physics world
    initializeForPlay();
}

void ASimModeWorldHero::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // stop physics thread before we dismantle
    stopAsyncUpdator();

    Super::EndPlay(EndPlayReason);
}

void ASimModeWorldHero::setupClockSpeed()
{
    typedef msr::airlib::ClockFactory ClockFactory;

    float clock_speed = getSettings().clock_speed;

    // setup clock in ClockFactory
    std::string clock_type = getSettings().clock_type;

    if (clock_type == "ScalableClock")
    {
        // scalable clock returns interval same as wall clock but multiplied by a scale factor
        ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
    }
    else if (clock_type == "SteppableClock")
    {
        // steppable clock returns interval that is a constant number irrespective of wall clock
        // we can either multiply this fixed interval by scale factor to speed up/down the clock
        // but that would cause vehicles like quadrotors to become unstable
        // so alternative we use here is instead to scale control loop frequency. The downside is that
        // depending on compute power available, we will max out control loop frequency and therefore can no longer
        // get increase in clock speed

        // Approach 1: scale clock period, no longer used now due to quadrotor instability
        // ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
        // static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));

        // Approach 2: scale control loop frequency if clock is speeded up
        if (clock_speed >= 1)
        {
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9))); // no clock_speed multiplier

            setPhysicsLoopPeriod(getPhysicsLoopPeriod() / static_cast<long long>(clock_speed));
        }
        else
        {
            // for slowing down, this don't generate instability
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));
        }
    }
    else
        throw std::invalid_argument(common_utils::Utils::stringf(
            "clock_type %s is not recognized", clock_type.c_str()));
}


//-------------------------------- overrides -----------------------------------------------//
std::vector<std::unique_ptr<msr::airlib::ApiServerBase>> ASimModeWorldHero::createApiServer() const
{
    std::vector<std::unique_ptr<msr::airlib::ApiServerBase>> api_servers;
#ifdef AIRLIB_NO_RPC
    api_servers.push_back(ASimModeBase::createApiServer());
    return api_servers;
#else
    uint16_t port_drone = 41451;
    api_servers.push_back(std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::MultirotorRpcLibServer(
        getApiProvider(), getSettings().api_server_address, port_drone)));

    uint16_t port_skidvehicle = 41452;
    api_servers.push_back(std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::CarRpcLibServer(
        getApiProvider(), getSettings().api_server_address, port_skidvehicle)));
    return api_servers;
#endif
}


void ASimModeWorldHero::getExistingVehiclePawns(TArray<AActor *> &pawns) const
{
    TArray<AActor *> FlyingPawns;
    UAirBlueprintLib::FindAllActor<TFlyingPawn>(this, FlyingPawns);
    for (AActor *fpawn : FlyingPawns)
    {
        pawns.Add(fpawn);
        if (getSettings().simmode_name == "HERO")
        {
            APawn *vehicle_pawn = static_cast<APawn *>(fpawn);
            addPawnToMap(vehicle_pawn, AirSimSettings::kVehicleTypeSimpleFlight);
        }
    }

    TArray<AActor *> SkidVehiclePawns;
    UAirBlueprintLib::FindAllActor<TSkidVehiclePawn>(this, SkidVehiclePawns);
    for (AActor *svpawn : SkidVehiclePawns)
    {
        pawns.Add(svpawn);
        if (getSettings().simmode_name == "HERO")
        {
            APawn *vehicle_pawn = static_cast<APawn *>(svpawn);
            addPawnToMap(vehicle_pawn, AirSimSettings::kVehicleTypeCPHusky); // this will specifically add the CPHusky, need to change if Jackal needed
        }
    }
}

bool ASimModeWorldHero::isVehicleTypeSupported(const std::string &vehicle_type) const
{
    return ((vehicle_type == AirSimSettings::kVehicleTypeSimpleFlight) ||
            (vehicle_type == AirSimSettings::kVehicleTypePhysXCar) ||
            (vehicle_type == AirSimSettings::kVehicleTypePX4) ||
            (vehicle_type == AirSimSettings::kVehicleTypeArduCopterSolo) ||
            (vehicle_type == AirSimSettings::kVehicleTypeCPHusky) || 
            (vehicle_type == AirSimSettings:: kVehicleTypeArduRover) || 
            (vehicle_type == AirSimSettings:: kVehicleTypePioneer) || 
            (vehicle_type == AirSimSettings:: kVehicleTypeBoxCar));
}

std::string ASimModeWorldHero::getVehiclePawnPathName(const AirSimSettings::VehicleSetting &vehicle_setting) const
{
    // decide which derived BP to use
    std::string pawn_path = vehicle_setting.pawn_path;
    if (pawn_path == "")
    {
        if (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypeCPHusky)
        {
            pawn_path = "DefaultSkidVehicle";
        }
        else
        {
            pawn_path = "DefaultQuadrotor";
        }
    }
    return pawn_path;
}

PawnEvents *ASimModeWorldHero::getVehiclePawnEvents(APawn *pawn) const
{
    std::string vehicle_type = getVehicleType(pawn);
    if (vehicle_type == AirSimSettings::kVehicleTypeCPHusky)
    {
        return static_cast<TSkidVehiclePawn *>(pawn)->getPawnEvents(); // may need to change this to TVehiclePawn
    }
    else
    {
        return static_cast<TFlyingPawn *>(pawn)->getPawnEvents();
    }
}

const common_utils::UniqueValueMap<std::string, APIPCamera *> ASimModeWorldHero::getVehiclePawnCameras(
    APawn *pawn) const
{
    std::string vehicle_type = getVehicleType(pawn);
    if (vehicle_type == AirSimSettings::kVehicleTypeCPHusky)
    {
        return (static_cast<const TSkidVehiclePawn *>(pawn))->getCameras();
    }
    else
    {
        return (static_cast<const TFlyingPawn *>(pawn))->getCameras();
    }
}

void ASimModeWorldHero::initializeVehiclePawn(APawn *pawn)
{
    std::string vehicle_type = getVehicleType(pawn);
    if (vehicle_type == AirSimSettings::kVehicleTypeCPHusky)
    {
        static_cast<TSkidVehiclePawn *>(pawn)->initializeForBeginPlay(getSettings().engine_sound);
    }
    else
    {
        static_cast<TFlyingPawn *>(pawn)->initializeForBeginPlay();
    }
}

std::unique_ptr<PawnSimApi> ASimModeWorldHero::createVehicleSimApi(
    const PawnSimApi::Params &pawn_sim_api_params) const
{
    APawn *pawn = pawn_sim_api_params.pawn;
    std::string vehicle_type = getVehicleType(pawn);
    if (vehicle_type == AirSimSettings::kVehicleTypeCPHusky)
    {
        auto vehicle_pawn = static_cast<TSkidVehiclePawn *>(pawn_sim_api_params.pawn);
        auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new SkidVehiclePawnSimApi(pawn_sim_api_params, vehicle_pawn->getKeyBoardControls()));
        vehicle_sim_api->initialize();
        // vehicle_sim_api->reset();
        return vehicle_sim_api;
    }
    else
    {
        auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new MultirotorPawnSimApi(pawn_sim_api_params));
        vehicle_sim_api->initialize();
        // For multirotors the vehicle_sim_api are in PhysicsWorld container and then get reseted when world gets reseted
        return vehicle_sim_api;
    }
    // vehicle_sim_api->reset();
}

msr::airlib::VehicleApiBase *ASimModeWorldHero::getVehicleApi(const PawnSimApi::Params &pawn_sim_api_params,
                                                              const PawnSimApi *sim_api) const
{
    APawn *pawn = pawn_sim_api_params.pawn;
    std::string vehicle_type = getVehicleType(pawn);
    if (vehicle_type == AirSimSettings::kVehicleTypeCPHusky)
    {
        const auto skidvehicle_sim_api = static_cast<const SkidVehiclePawnSimApi *>(sim_api);
        return skidvehicle_sim_api->getVehicleApi();
    }
    else
    {
        const auto multirotor_sim_api = static_cast<const MultirotorPawnSimApi *>(sim_api);
        return multirotor_sim_api->getVehicleApi();
    }
}