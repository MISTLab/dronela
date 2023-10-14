#include "carla/client/Drone.h"

#include "carla/client/ActorList.h"
#include "carla/client/detail/Simulator.h"
#include "carla/Memory.h"


namespace carla {

namespace client {

  Drone::Drone(ActorInitializer init)
    : Actor(std::move(init)) {}

  void Drone::ApplyControl() {
    GetEpisode().Lock()->ApplyControlToDrone(*this);
    }
  }
}
