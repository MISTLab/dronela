#pragma once

#include "CoreMinimal.h"
#include"GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/BoxComponent.h"
#include "Components/SphereComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "CarlaDroneMotor.h"


#include "CarlaDrone.generated.h"
UCLASS()
class CARLA_API ADrone : public APawn
{
    GENERATED_BODY()

public:
    ADrone();
  
    virtual void Tick(float DeltaTime) override;
    
    // Declare a blueprint callable function
    UFUNCTION(BlueprintCallable, Category = "Drone_Functions")
    void Stringer();
    UFUNCTION(Category = "Drone_Functions", BlueprintCallable)
    void ApplyDroneControl();

    UFUNCTION(Category = "Drone_Functions", BlueprintCallable)
    void ApplyDroneMotorSpeed(float m1,float m2,float m3,float m4);

    UPROPERTY(BlueprintReadWrite,EditAnywhere)
    USkeletalMeshComponent* SkeletalMeshComponent;
    UPROPERTY(BlueprintReadWrite,EditAnywhere)
    UBoxComponent* RootCollisionBox;
    UPROPERTY(BlueprintReadWrite,EditAnywhere)
    USphereComponent* EngineComponent;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Animation")
    float propellerSpeed= 100.0F;

    UPROPERTY()
    UDroneMotor* Motor_Front_Left; // Declare a member variable to hold UMyUClass instance

    UPROPERTY()
    UDroneMotor* Motor_Front_Right;

    UPROPERTY()
    UDroneMotor* Motor_Rear_Left;

    UPROPERTY()
    UDroneMotor* Motor_Rear_Right;

  

private:
  
    
   
   

protected:
 
  virtual void BeginPlay() override;

};



