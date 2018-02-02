#ifndef BuoyancyForce_H
#define BuoyancyForce_H

#include <chrono/physics/ChLoadContainer.h>
#include "chrono/physics/ChBodyEasy.h"

#include "params.h"

class BuoyancyForce {
private:
  std::shared_ptr<ChLoadBodyForce> buoyancyForce;
public:
  BuoyancyForce(params p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<ChBodyEasyCylinder> monopile);

};

#endif
