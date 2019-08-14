// Fill out your copyright notice in the Description page of Project Settings.

#include "SimModeCPHusky.h"
#include "ConstructorHelpers.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "CPHuskyPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "vehicles/car/api/CarRpcLibServer.hpp"


void ASimModeCPHusky::BeginPlay()
{
	Super::BeginPlay();

	initializePauseState();
}

void ASimModeCPHusky::initializePauseState()
{
	pause_period_ = 0;
	pause_period_start_ = 0;
	pause(false);
}

bool ASimModeCPHusky::isPaused() const
{
	return current_clockspeed_ == 0;
}

void ASimModeCPHusky::pause(bool is_paused)
{
	if (is_paused)
		current_clockspeed_ = 0;
	else
		current_clockspeed_ = getSettings().clock_speed;

	UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
}

void ASimModeCPHusky::continueForTime(double seconds)
{
	pause_period_start_ = ClockFactory::get()->nowNanos();
	pause_period_ = seconds;
	pause(false);
}

void ASimModeCPHusky::setupClockSpeed()
{
	current_clockspeed_ = getSettings().clock_speed;

	//setup clock in PhysX
	UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
	UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(current_clockspeed_), LogDebugLevel::Informational);
}

void ASimModeCPHusky::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (pause_period_start_ > 0) {
		if (ClockFactory::get()->elapsedSince(pause_period_start_) >= pause_period_) {
			if (!isPaused())
				pause(true);

			pause_period_start_ = 0;
		}
	}
}

//-------------------------------- overrides -----------------------------------------------//

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeCPHusky::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
	return ASimModeBase::createApiServer();
#else
	return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::CarRpcLibServer(
		getApiProvider(), getSettings().api_server_address));
#endif
}

void ASimModeCPHusky::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
	UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);
}

bool ASimModeCPHusky::isVehicleTypeSupported(const std::string& vehicle_type) const
{
	return vehicle_type == AirSimSettings::kVehicleTypeCPHusky;
}

std::string ASimModeCPHusky::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
	//decide which derived BP to use
	std::string pawn_path = vehicle_setting.pawn_path;
	if (pawn_path == "")
		pawn_path = "CPHusky";

	return pawn_path;
}

PawnEvents* ASimModeCPHusky::getVehiclePawnEvents(APawn* pawn) const
{
	return static_cast<TVehiclePawn*>(pawn)->getPawnEvents();
}
const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeCPHusky::getVehiclePawnCameras(
	APawn* pawn) const
{
	return (static_cast<const TVehiclePawn*>(pawn))->getCameras();
}
void ASimModeCPHusky::initializeVehiclePawn(APawn* pawn)
{
	auto vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
	vehicle_pawn->initializeForBeginPlay(getSettings().engine_sound);
}
std::unique_ptr<PawnSimApi> ASimModeCPHusky::createVehicleSimApi(
	const PawnSimApi::Params& pawn_sim_api_params) const
{
	auto vehicle_pawn = static_cast<TVehiclePawn*>(pawn_sim_api_params.pawn);
	auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new CPHuskyPawnSimApi(pawn_sim_api_params,
		vehicle_pawn->getKeyBoardControls(), vehicle_pawn->getVehicleMovementComponent()));
	vehicle_sim_api->initialize();
	vehicle_sim_api->reset();
	return vehicle_sim_api;
}
msr::airlib::VehicleApiBase* ASimModeCPHusky::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
	const PawnSimApi* sim_api) const
{
	const auto car_sim_api = static_cast<const CPHuskyPawnSimApi*>(sim_api);
	return car_sim_api->getVehicleApi();
}