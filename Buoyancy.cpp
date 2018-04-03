#include <chrono/physics/ChLoadsBody.h>
#include "chrono/physics/ChBodyEasy.h"
#include <chrono/physics/ChLoadContainer.h>
#include "chrono/physics/ChMarker.h"
#include <chrono/core/ChVector.h>
#include <chrono/core/ChLog.h>
#include <chrono/core/ChCoordsys.h>
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChSystem.h"
#include <chrono_fea/ChNodeFEAxyz.h>


#include "Buoyancy.h"
#include "params.h"

using namespace chrono;
using namespace chrono::fea;

Buoyancy::Buoyancy(params p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<ChBodyEasyCylinder> monopile, std::shared_ptr<ChMesh> mesh, ChSystem& system)

:p(p),
monopile(monopile),
loadContainer(loadContainer)
{
  ChVector<> pos = monopile->GetPos();

  ChVector<> towerPos = monopile->GetPos();
  ChFrameMoving<> frame = monopile->GetFrame_COG_to_abs();
  //Get rotation of frame as a quaternion
  ChQuaternion<> quaternion = frame.GetRot();
  //Get unity vector in z direction
  ChVector<> zUnityVector = ChVector<>(0,0,1);
  //Get vector in direction of tower axis by rotating vector around quaternion
  ChVector<> towerAxis = quaternion.Rotate(zUnityVector);

  markerBottom = std::make_shared<ChMarker>();
  //Set Marker Position relative to local coordinate system
  ChCoordsys<> bottomCoordsys = ChCoordsys<>(-0.5*p.towerHeight*towerAxis);
  //Set marker parameters
  markerBottom->SetBody(monopile.get());
  markerBottom->Impose_Abs_Coord(bottomCoordsys);

  //markerBottom->SetPos(ChVector<>(0,0,-0.5*p.towerHeight));

  markerTop = std::make_shared<ChMarker>();
  //Set Marker Position relative to local coordinate system
  ChCoordsys<> topCoordsys = ChCoordsys<>(0.5*p.towerHeight*towerAxis);
  //Set marker parameters
  markerTop->SetBody(monopile.get());
  markerTop->Impose_Abs_Coord(topCoordsys);

  buoyancyCenterVizNode = std::make_shared<ChNodeFEAxyz>(ChVector<>(0,0,0));

  mesh->AddNode(buoyancyCenterVizNode);

  //markerTop->SetPos(ChVector<>(0,0,0.5*p.towerHeight));

  ChVector<> vecE = markerBottom->GetAbsCoord().pos;
  ChVector<> vecI = markerTop->GetAbsCoord().pos;

  //Init Buoyancy force with null vectors
  buoyancyForce = std::make_shared<ChLoadBodyForce> (
    monopile, //body
    ChVector<>(0,0,0), //force in positive z direction
    false, //local_force
    ChVector<>(0,0,0), //local Gravity Center
    true //local point
  );

  //Update Buoyancy Force
  update();

  //Add load to container
  loadContainer->Add(buoyancyForce);

}

void Buoyancy::update(){

  markerBottom->UpdateState();
  markerTop->UpdateState();

  ChVector<> seaLevelVector = ChVector<>(0,0,p.seaLevel);
  ChVector<> towerPos = monopile->GetPos();
  ChFrameMoving<> frame = monopile->GetFrame_COG_to_abs();
  //Get rotation of frame as a quaternion
  ChQuaternion<> quaternion = frame.GetRot();
  //Get unity vector in z direction
  ChVector<> zUnityVector = ChVector<>(0,0,1);
  //Get vector in direction of tower axis by rotating vector around quaternion
  ChVector<> towerAxis = quaternion.Rotate(zUnityVector);

  GetLog() << "towerAxis: " << towerAxis << "\n";

  ChVector<> intersectionPoint;
  ChVector<> vecE;
  ChVector<> vecI;
  //Check if the sea level plane is parallel to the tower axis,
  //this is the edge case. then the intersection point will be undefined.
  //Scalar Product:
  if(towerAxis^zUnityVector==0){
    GetLog() << "Edge Case for Buoyancy" << "\n";
    //intersection point is exactly on a line parallel to the z axis below the gravity
    //center. Point E and I are at the radius of the tower
    intersectionPoint = ChVector<>(towerPos.x(),towerPos.y(),p.seaLevel);

    vecE = ChVector<>(towerPos.x(),towerPos.y(),towerPos.z()-p.towerRadius);
    vecI = ChVector<>(towerPos.y(),towerPos.y(),towerPos.z()+p.towerRadius);
  }
  else{
    //Point E and I are at the top and bottom of the tower
    //Calculate Intersection point of sea level plane with algebraic equation
    const double rConstant = ((seaLevelVector-towerPos)^zUnityVector)/(towerAxis^zUnityVector);
    //Intersection point using straight line equation
    intersectionPoint = towerPos + towerAxis*rConstant;
    //Get Position of the Top and bottom via the body markers
    vecE = markerBottom->GetAbsCoord().pos;
    vecI = markerTop->GetAbsCoord().pos;

    GetLog() << "r-Konstante:" << rConstant << "\n";
  }

  GetLog() << "vecE:" << vecE << "\n";
  GetLog() << "vecI:" << vecI << "\n";
  GetLog() << "intersectionPoint:" << intersectionPoint << "\n";

  computeBuoyancyCenter(vecE,vecI,  intersectionPoint);

  computeBuoyancyForce(vecE, vecI, intersectionPoint);

  GetLog() << "gravityCenter: X: " << towerPos.x() << "\n";
  GetLog() << "gravityCenter: Y: " << towerPos.y() << "\n";
  GetLog() << "gravityCenter: Z: " << towerPos.z() << "\n";

  GetLog() << "buoyancyCenter: X: " << buoyancyCenter.x() << "\n";
  GetLog() << "buoyancyCenter: Y: " << buoyancyCenter.y() << "\n";
  GetLog() << "buoyancyCenter: Z: " << buoyancyCenter.z() << "\n";

  //GetLog() << "Monopile Mass: " << monopile->GetMass() << "\n";
  //GetLog() << "Gravity Force: " << monopile->GetMass()*p.towerDensity*p.g << "\n";

}

void Buoyancy::computeBuoyancyCenter(ChVector<> vecE, ChVector<> vecI, ChVector<> intersectionPoint){
  // Skizze:
  // G: Center of gravity
  // S: intersection point
  // B: buoayancy Center
  // E: bottom
  // I: top
  // -----------I------------
  //            |
  //            |
  //            |
  //            |
  //            G
  //            |
  //~~~~~~~~~~~~S~~~~~~~~~~~~
  //            |
  //            B
  //            |
  //------------E-------------

  ChVector<> towerPos = monopile->GetPos();

  ChVector<> vecEI = vecI - vecE;

  GetLog() << "Length EI:" << vecEI.Length() << "\n";

  ChVector<> vecSG = towerPos - intersectionPoint;

  ChVector<> vecGI = vecI - towerPos;

  ChVector<> vecSI = vecSG + vecGI;

  ChVector<> vecES = vecEI - vecSI;

  buoyancyCenter = intersectionPoint - 0.5*vecES;

  buoyancyCenterVizNode->SetPos(buoyancyCenter);

  buoyancyForce->SetApplicationPoint(buoyancyCenter,false);
}

void Buoyancy::computeBuoyancyForce(ChVector<> vecE, ChVector<> vecI, ChVector<> intersectionPoint){

  double force = 0;
  double submergedVolumeMonopile = 0;
  //check if monopile is actually submerged
  if(buoyancyCenter.z()<p.seaLevel){
    ChVector<> towerPos = monopile->GetPos();

    ChVector<> vecEG = towerPos - vecE;
    ChVector<> vecGS = intersectionPoint - towerPos;
    ChVector<> vecGI = vecI - towerPos;

    //check if distance to sea level is larger than distance to top of tower
    if(vecGS.Length() >= vecGI.Length()){
      //tower completely submerged, return volume of complete water cube
      submergedVolumeMonopile =  M_PI*pow(p.towerRadius,2)*p.towerHeight;
    }
    else{
      //tower partly submerged
      //submerged length is half the tower + vector of G to S (sea level)
      ChVector<> submergedVec = vecEG + vecGS;
      submergedVolumeMonopile = abs((M_PI*pow(p.towerRadius,2))*2*submergedVec.Length());
    }

    force = submergedVolumeMonopile*p.rhoWater*p.g;

  }
  GetLog() << "submergedVolumeMonopile: " << submergedVolumeMonopile << "\n";
  GetLog() << "buoyancyForce:" << force << "\n";

  //update buoyancy force
  buoyancyForce->SetForce(ChVector<>(0,0,force),false);
}
