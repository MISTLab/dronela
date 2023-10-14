#pragma once

#include "carla/client/Actor.h"


namespace carla {
namespace client {


  class Drone : public Actor {
  public:
    explicit Drone(ActorInitializer init);
    void ApplyControl();
  };

}
}
