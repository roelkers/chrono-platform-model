#ifndef BuoyancyForce_H
#define BuoyancyForce_H

#include <chrono/physics/ChLoadContainer.h>
#include "chrono/physics/ChBodyEasy.h"

#include "params.h"

class BuoyancyForce {
private:
  params p;
  double gravityCenter;
  std::shared_ptr<ChBodyEasyCylinder> mmonopile;
  std::shared_ptr<ChLoadBodyForce> mbuoyancyForce;
  std::shared_ptr<ChLoadContainer> mloadContainer;

  double mbuoyancyCenter; //z-coordinate of buyoancy center of monopile from xy-plane
public:
  BuoyancyForce(params p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<ChBodyEasyCylinder> monopile);
  void update();
  double computeBuoyancyForce();
  void computeBuoyancyCenter();
};

#endif
