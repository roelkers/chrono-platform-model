#include <chrono/physics/ChLoadsBody.h>
#include "chrono/physics/ChBodyEasy.h"
#include <chrono/physics/ChLoadContainer.h>
#include <chrono/core/ChVector.h>
#include <chrono/core/ChLog.h>

#include "BuoyancyForce.h"
#include "params.h"

BuoyancyForce::BuoyancyForce(params p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<ChBodyEasyCylinder> monopile){

  p = p;
  mmonopile = monopile;
  mloadContainer = loadContainer;

  ChVector<> pos = mmonopile->GetPos();

  //Init Buoyancy force
  update();

  //Add load to container
  mloadContainer->Add(mbuoyancyForce);
}

void BuoyancyForce::update(){

  computeBuoyancyCenter();

  double force = computeBuoyancyForce();
  GetLog() << "Buoyancy Force: " << force << "\n\n";

  //update buoyancy force
  mbuoyancyForce = std::make_shared<ChLoadBodyForce> (
    mmonopile, //body
    ChVector<>(0,0,force), //force in positive z direction
    false, //local_force
    ChVector<>(0,0,0), //local Gravity Center, therefore just set to 0
    true //local point
  );

}

void BuoyancyForce::computeBuoyancyCenter(){

  ChVector<> pos = mmonopile->GetPos();
  gravityCenter = pos.z();
  GetLog() << "gravityCenter: " << gravityCenter << "\n";

  double z_iw = 0.5*p.towerHeight + gravityCenter;

  mbuoyancyCenter = -0.5*(p.towerHeight - z_iw);
  GetLog() << "buoyancyCenter: " << mbuoyancyCenter << "\n";
  GetLog() << "Monopile Mass: " << mmonopile->GetMass() << "\n";
  GetLog() << "Gravity Force: " << mmonopile->GetMass()*p.towerDensity*p.g << "\n";

}

double BuoyancyForce::computeBuoyancyForce(){

  //check if monopile is actually submerged
  if(mbuoyancyCenter<0){
    double submergedVolumeMonopile;
    GetLog() << "0.5*towerHeight: " << 0.5*p.towerHeight << "\n";

    if(gravityCenter<-0.5*p.towerHeight){
      //tower completely submerged return volume of complete water cube
      submergedVolumeMonopile =  M_PI*pow(p.towerRadius,2)*p.towerHeight;
    }
    else{
      //tower partly submerged, return volume
      submergedVolumeMonopile = abs((M_PI*pow(p.towerRadius,2))*2*mbuoyancyCenter);
    }

    double force = submergedVolumeMonopile*p.rhoWater*p.g;

    GetLog() << "submergedVolumeMonopile: " << submergedVolumeMonopile << "\n";

    return force;
  }
  return 0; //if monopile is not submerged set force to 0
}
