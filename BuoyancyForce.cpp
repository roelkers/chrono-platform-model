#include <chrono/physics/ChLoadsBody.h>
#include "chrono/physics/ChBodyEasy.h"
#include <chrono/physics/ChLoadContainer.h>

#include "BuoyancyForce.h"
#include "params.h"

BuoyancyForce::BuoyancyForce(params p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<ChBodyEasyCylinder> monopile){
  buoyancyForce = std::make_shared<ChLoadBodyForce> (
    monopile, //body
    ChVector<>(50000,-50000,50000), //force
    false, //local_force
    ChVector<>(0,0,0), //point
    true //local point
  );
  loadContainer->Add(buoyancyForce);
}
