#include "carla/client/Drone.h"

#include "carla/client/ActorList.h"
#include "carla/client/detail/Simulator.h"
#include "carla/Memory.h"


namespace carla {

namespace client {

  Drone::Drone(ActorInitializer init)
    : Actor(std::move(init)) {}

  
  void Drone::ApplyMotorSpeed(float m1,float m2,float m3,float m4) {
    GetEpisode().Lock()->ApplyMotorSpeedToDrone(*this, m1, m2, m3, m4 );
    }
  }
}
