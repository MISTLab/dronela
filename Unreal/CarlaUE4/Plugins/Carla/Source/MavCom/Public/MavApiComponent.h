#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include <optional>
#include "mavsdk.h"


#include "MavApiComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class MAVCOM_API UMavApiComponent : public UActorComponent
{
    GENERATED_BODY()

public:    
    // Sets default values for this component's properties
    UMavApiComponent();

    // Called when the game starts
    virtual void BeginPlay() override;

    // Called every frame
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    // Public functions for PX4 communication
    UFUNCTION(BlueprintCallable, Category = "MavSDK")
    void ConnectToPX4();

    UFUNCTION(BlueprintCallable, Category = "MavSDK")
    void SendCommandToPX4();

    UFUNCTION(BlueprintCallable, Category = "MavSDK")
    void ReceiveTelemetryFromPX4();

};
