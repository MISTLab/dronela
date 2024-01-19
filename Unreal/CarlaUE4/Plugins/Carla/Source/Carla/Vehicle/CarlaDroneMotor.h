#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include "CarlaDroneMotor.generated.h"

UCLASS(Blueprintable)
class CARLA_API UDroneMotor : public UObject
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, Category = "DroneMotor")
    FVector LocalLocation;

    UPROPERTY(EditAnywhere, Category = "DroneMotor")
    FVector MotorTorqueAxis;

    UPROPERTY(EditAnywhere, Category = "DroneMotor")
    FVector MotorForceAxis;
    

    UPROPERTY(EditAnywhere, Category = "DroneMotor")
    float ForceFactor;

    UPROPERTY(EditAnywhere, Category = "DroneMotor")
    float TorqueFactor;

    UPROPERTY(EditAnywhere, Category = "DroneMotor")
    bool bClockwise;

    UPROPERTY()
    float ArmLength;

private:
    

public:
    UDroneMotor(); // Constructor

    // Function to apply force
    UFUNCTION(BlueprintCallable, Category = "DroneMotor")
    void ApplyForceTorque(UPrimitiveComponent* Body, float Speed);
};
