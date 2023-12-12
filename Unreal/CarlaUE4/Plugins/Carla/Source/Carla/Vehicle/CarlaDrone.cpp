#include "CarlaDrone.h"
#include "Engine/Engine.h"
#include "Components/TimelineComponent.h"


#include "Components/SceneComponent.h"


ADrone::ADrone()
{ 


  auto RootComponent = Cast<UPrimitiveComponent>(this->GetRootComponent());
  
  this->SetActorEnableCollision(true);


  RootCollisionBox = CreateDefaultSubobject<UBoxComponent>(TEXT("RootCollisionBox"));
  SetRootComponent(RootCollisionBox);
  RootCollisionBox->SetMobility(EComponentMobility::Movable);
  RootCollisionBox->SetSimulatePhysics(true);
  RootCollisionBox->SetCollisionProfileName(TEXT("Vehicle"));
  RootCollisionBox->SetEnableGravity(true);
  // Set the mass of the physics component
  RootCollisionBox->SetMassOverrideInKg(NAME_None, 10.0f);

  SkeletalMeshComponent = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("SkeletalMeshComponent"));
  SkeletalMeshComponent->SetupAttachment(RootCollisionBox);
  SkeletalMeshComponent->SetMobility(EComponentMobility::Movable);
  SkeletalMeshComponent->SetSimulatePhysics(true);
  SkeletalMeshComponent->SetCollisionProfileName(TEXT("NoCollision"));
  SkeletalMeshComponent->SetEnableGravity(true);



  EngineComponent = CreateDefaultSubobject<USphereComponent>(TEXT("CylinderComponent"));
  EngineComponent->SetupAttachment(RootCollisionBox); // Attach to the collision box
  EngineComponent->SetMobility(EComponentMobility::Movable);
  EngineComponent->SetSimulatePhysics(true);
  SkeletalMeshComponent->SetCollisionProfileName(TEXT("NoCollision"));
  SkeletalMeshComponent->SetEnableGravity(true);

  Motor_Front_Right = CreateDefaultSubobject<UDroneMotor>(TEXT("Motor_Front_Right"));
  Motor_Front_Left = CreateDefaultSubobject<UDroneMotor>(TEXT("Motor_Front_Left"));
  Motor_Rear_Right = CreateDefaultSubobject<UDroneMotor>(TEXT("Motor_Rear_Right"));
  Motor_Rear_Left = CreateDefaultSubobject<UDroneMotor>(TEXT("Motor_Rear_Left"));


  Motor_Front_Left->LocalLocation = FVector(1.0f, -1.0f, 0);
  Motor_Front_Left->ForceFactor = 2.0f;
  Motor_Front_Left->TorqueFactor = 1.0f;
  //Motor_Front_Left->bClockwise = false;
  Motor_Front_Left->MotorTorqueAxis= FVector(0.0f,0.0f,1.0f);
  Motor_Front_Left->MotorForceAxis= FVector(0.0f,0.0f,1.0f);
  Motor_Front_Left->ArmLength = 0.05f;


  Motor_Front_Right->LocalLocation = FVector(1.0f, 1.0f, 0);
  Motor_Front_Right->ForceFactor = 2.0f;
  Motor_Front_Right->TorqueFactor = 1.0f;
  //Motor_Front_Right->bClockwise = false;
  Motor_Front_Right->MotorTorqueAxis= FVector(0.0f,0.0f,-1.0f);
  Motor_Front_Right->MotorForceAxis= FVector(0.0f,0.0f,1.0f);
  Motor_Front_Right->ArmLength = 0.05f;
  
  Motor_Rear_Left->LocalLocation = FVector(-1.0f, -1.0, 0);
  Motor_Rear_Left->ForceFactor = 2.0f;
  Motor_Rear_Left->TorqueFactor = 1.0f;
  //Motor_Rear_Left->bClockwise = false;
  Motor_Rear_Left->MotorTorqueAxis= FVector(0.0f,0.0f,-1.0f);
  Motor_Rear_Left->MotorForceAxis= FVector(0.0f,0.0f,1.0f);
  Motor_Rear_Left->ArmLength = 0.05f;


  Motor_Rear_Right->LocalLocation = FVector(-1.0f, 1.0f, 0);
  Motor_Rear_Right->ForceFactor = 2.0f;
  Motor_Rear_Right->TorqueFactor = 1.0f;
  //Motor_Rear_Right->bClockwise = false;
  Motor_Rear_Right->MotorTorqueAxis= FVector(0.0f,0.0f,1.0f);
  Motor_Rear_Right->MotorForceAxis= FVector(0.0f,0.0f,1.0f);
  Motor_Rear_Right->ArmLength = 0.05f; 



  
}


  
void ADrone::Stringer()
{
    GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, TEXT("hahahahahaha"));
    UE_LOG(LogTemp, Warning, TEXT("hahahahahaha"));
}

void ADrone::ApplyDroneMotorSpeed(float m1,float m2,float m3,float m4)
{
  //MavConnection->connect();


  auto RootComponent = Cast<UPrimitiveComponent>(this->GetRootComponent());
  FVector ForceToApply = FVector(0.0f, 0.0f, 2800.0f);
  FVector LocalLocation = FVector(0.5f, 0.5f, 0.0f);
  Motor_Front_Left->ApplyForceTorque(RootComponent, m1);
  Motor_Front_Right->ApplyForceTorque(RootComponent, m2);
  Motor_Rear_Left->ApplyForceTorque(RootComponent, m3); 
  Motor_Rear_Right->ApplyForceTorque(RootComponent, m4);
  
}

void ADrone::ApplyDroneControl()
{
  //MavConnection->connect();


  auto RootComponent = Cast<UPrimitiveComponent>(this->GetRootComponent());
  FVector ForceToApply = FVector(0.0f, 0.0f, 2800.0f);
  FVector LocalLocation = FVector(0.5f, 0.5f, 0.0f);
  Motor_Front_Right->ApplyForceTorque(RootComponent, 1500.0);
  Motor_Front_Left->ApplyForceTorque(RootComponent, 1500.0);
  Motor_Rear_Right->ApplyForceTorque(RootComponent, 1500.0);
  Motor_Rear_Left->ApplyForceTorque(RootComponent, 1500.0); 
  
  /* RootComponent->AddForceAtLocationLocal(ForceToApply,LocalLocation, "None");
   LocalLocation = FVector(0.5f, -0.5f, 0.0f);
  RootComponent->AddForceAtLocationLocal(ForceToApply,LocalLocation, "None");
   LocalLocation = FVector(-0.5f, 0.5f, 0.0f);
  RootComponent->AddForceAtLocationLocal(ForceToApply,LocalLocation, "None");
   LocalLocation = FVector(-0.5f, -0.5f, 0.0f);
  RootComponent->AddForceAtLocationLocal(ForceToApply,LocalLocation, "None");
 
  
  FVector ForceToApplyF = FVector(100000.0f, -100000.0f, 0.0f);
  FVector ForceToApplyC = FVector(-100000.0f, 100000.0f, 0.0f);
  FVector LocalLocationF = FVector(0.55f, 0.55f, 0.0f);
  FVector LocalLocationC = FVector(0.45f, 0.45f, 0.0f);
  RootComponent->AddForceAtLocationLocal(ForceToApplyF,LocalLocationF, "None");
  RootComponent->AddForceAtLocationLocal(ForceToApplyC,LocalLocationC, "None");
 */
  }

void ADrone::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
  
    
}
  



void ADrone::BeginPlay()
{
  Super::BeginPlay();
  
}

