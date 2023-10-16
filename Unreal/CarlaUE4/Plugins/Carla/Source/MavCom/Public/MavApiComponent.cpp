#include "MavApiComponent.h"
#include <iostream>
#include <future>
#include <string>
#include <memory>
#include <thread>

#include "mavsdk.h"
#include "Engine/Engine.h"



// Sets default values for this component's properties
UMavApiComponent::UMavApiComponent()
{
    // Set this component to be initialized when the game starts, and to be ticked every frame.
    // You can turn these features off to improve performance if you don't need them.
    PrimaryComponentTick.bCanEverTick = true;
    ConnectToPX4();

    // Initialize any other variables or settings here
}

// Called when the game starts
void UMavApiComponent::BeginPlay()
{
    Super::BeginPlay();

    // Implement any logic that should happen when the game starts here
}

// Called every frame
void UMavApiComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // Implement any logic that should happen every frame here
}

// Implement the logic to connect to PX4 using MavSDK
void UMavApiComponent::ConnectToPX4()
{

     mavsdk::Mavsdk me; // Create an instance of Mavsdk
    std::string goonV = me.version();
    

    // Convert std::string to FString
    FString unrealgoonV(goonV.c_str());

    // Log the FString using UE_LOG
    UE_LOG(LogTemp, Warning, TEXT("%s"), *unrealgoonV);
    UE_LOG(LogTemp, Warning, TEXT("-----------------------mavlog:"));
    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, unrealgoonV);
    }
  
    //mavsdk::ConnectionResult connection_result= me.add_any_connection("udp://127.0.0.1:4560");

   //int k= add(2,3);
    // Add your connection logic here using MavSDK
}

// Implement the logic to send commands to PX4 using MavSDK
void UMavApiComponent::SendCommandToPX4()
{
  

    // Add your command-sending logic here using MavSDK
}

// Implement the logic to receive telemetry from PX4 using MavSDK
void UMavApiComponent::ReceiveTelemetryFromPX4()
{
    // Add your telemetry-receiving logic here using MavSDK
}

