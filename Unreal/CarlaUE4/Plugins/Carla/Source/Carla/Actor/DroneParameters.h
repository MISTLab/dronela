#pragma once

#include "Carla/Vehicle/CarlaDrone.h"

#include "DroneParameters.generated.h"

USTRUCT(BlueprintType)
struct CARLA_API FDroneParameters
{
  GENERATED_BODY()

  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  FString Make;

  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  FString Model;

  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  TSubclassOf<ADrone> Class;
  

  
};
