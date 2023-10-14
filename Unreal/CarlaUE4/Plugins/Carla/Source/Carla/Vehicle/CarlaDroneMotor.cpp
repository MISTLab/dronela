#include "CarlaDroneMotor.h"

UDroneMotor::UDroneMotor()
{
    LocalLocation = FVector::ZeroVector;
    ForceFactor = 1.0f;
    TorqueFactor = 1.0f;
    bClockwise = true;
    ArmLength = 1.0f; // Set your default value for ArmLength here
}

void UDroneMotor::ApplyForceTorque(UPrimitiveComponent* Body, float Speed)
{
    FVector ForceToApply = FVector(0.0f, 0.0f, Speed * ForceFactor);
    Body->AddForceAtLocationLocal(ForceToApply, LocalLocation, FName("None"));
    FVector ForceToApplyClose;
    FVector ForceToApplyFar;
    if (bClockwise)
    {
         ForceToApplyClose = (Speed * TorqueFactor / ArmLength) * FVector(LocalLocation.X, -LocalLocation.Y, 0.0f);
         ForceToApplyFar = (Speed * TorqueFactor / ArmLength) * FVector(-LocalLocation.X, LocalLocation.Y, 0.0f);
    }
    else
    {
        ForceToApplyClose = (Speed * TorqueFactor / ArmLength) * FVector(-LocalLocation.X, LocalLocation.Y, 0.0f);
        ForceToApplyFar = (Speed * TorqueFactor / ArmLength) * FVector(LocalLocation.X, -LocalLocation.Y, 0.0f);
    }

    FVector LocationClose = (1.0f - ArmLength) * LocalLocation;
    FVector LocationFar = (1.0f + ArmLength) * LocalLocation;

    Body->AddForceAtLocationLocal(ForceToApplyClose, LocationClose, "None");
    Body->AddForceAtLocationLocal(ForceToApplyFar, LocationFar, "None");
}
