#include <chrono/physics/ChLoadsBody.h>
#include "chrono/physics/ChBodyEasy.h"
#include <chrono/physics/ChLoadContainer.h>
#include <chrono/core/ChVector.h>
#include <chrono/core/ChLog.h>

#include "BuoyancyForce.h"
#include "params.h"

BuoyancyForce::BuoyancyForce(params p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<ChBodyEasyCylinder> monopile){
  ChVector<> pos = monopile->GetPos();
  double gravityCenter = pos.z();
  GetLog() << "gravityCenter: " << gravityCenter << "\n";

  double buoyancyCenter = -0.5*(p.towerHeight - gravityCenter);
  GetLog() << "buoyancyCenter: " << buoyancyCenter << "\n";

  if(buoyancyCenter<0){

    double submergedVolumeMonopile = (M_PI*pow(p.towerRadius,2))*2*abs(buoyancyCenter);

    double force = submergedVolumeMonopile*p.rhoWater*p.g;

    GetLog() << "Buoyancy Force: " << force << "\n";

    GetLog() << "submergedVolumeMonopile: " << submergedVolumeMonopile << "\n";

      buoyancyForce = std::make_shared<ChLoadBodyForce> (
        monopile, //body
        ChVector<>(0,0,force), //force
        false, //local_force
        ChVector<>(0,0,0), //local Gravity Center, therefore just set to 0
        true //local point
      );
      loadContainer->Add(buoyancyForce);
  }
}
