#include "CarlaDroneMotor.h"

UDroneMotor::UDroneMotor()
{
    LocalLocation = FVector::ZeroVector;
    MotorTorqueAxis= FVector(0.0f,0.0f,1.0f);
    MotorForceAxis= FVector(0.0f,0.0f,1.0f);
    ForceFactor = 1.0f;
    TorqueFactor = 1000.0f;
    //bClockwise = true;
    ArmLength = 0.1f; // Set your default value for ArmLength here
}

void UDroneMotor::ApplyForceTorque(UPrimitiveComponent* Body, float Speed)
{

    FVector N = FVector::CrossProduct(MotorTorqueAxis, LocalLocation);

    // Calculate the magnitude of Location
    float LocationMagnitude = LocalLocation.Size();

    
    FVector NormalizedN = N / LocationMagnitude;


    FVector ForceToApplyClose = (Speed * TorqueFactor / ArmLength) * NormalizedN;
    FVector ForceToApplyFar = -(Speed * TorqueFactor / ArmLength) * NormalizedN;
    FVector LocationClose = (1.0f - ArmLength) * LocalLocation;
    FVector LocationFar = (1.0f + ArmLength) * LocalLocation;

    Body->AddForceAtLocationLocal(ForceToApplyClose, LocationClose,  FName("None"));
    Body->AddForceAtLocationLocal(ForceToApplyFar, LocationFar, FName("None"));

    // FVector ForceToApplyClose;
    // FVector ForceToApplyFar;
    // if (bClockwise)
    // {
    //      ForceToApplyClose = (Speed * TorqueFactor / ArmLength) * FVector(LocalLocation.X, -LocalLocation.Y, 0.0f);
    //      ForceToApplyFar = (Speed * TorqueFactor / ArmLength) * FVector(-LocalLocation.X, LocalLocation.Y, 0.0f);
    // }
    // else
    // {
    //     ForceToApplyClose = (Speed * TorqueFactor / ArmLength) * FVector(-LocalLocation.X, LocalLocation.Y, 0.0f);
    //     ForceToApplyFar = (Speed * TorqueFactor / ArmLength) * FVector(LocalLocation.X, -LocalLocation.Y, 0.0f);
    // }

    // FVector LocationClose = (1.0f - ArmLength) * LocalLocation;
    // FVector LocationFar = (1.0f + ArmLength) * LocalLocation;

    // Body->AddForceAtLocationLocal(ForceToApplyClose, LocationClose,  FName("None"));
    // Body->AddForceAtLocationLocal(ForceToApplyFar, LocationFar, FName("None"));
    
    FVector ForceToApply = Speed*ForceFactor*MotorForceAxis; //FVector(0.0f, 0.0f, Speed * ForceFactor);
    Body->AddForceAtLocationLocal(ForceToApply, LocalLocation, FName("None"));
    
}
