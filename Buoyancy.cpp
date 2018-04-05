#include <chrono/physics/ChLoadsBody.h>
#include "chrono/physics/ChBodyEasy.h"
#include <chrono/physics/ChLoadContainer.h>
#include "chrono/physics/ChMarker.h"
#include <chrono/core/ChVector.h>
#include <chrono/core/ChLog.h>
#include <chrono/core/ChCoordsys.h>
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChSystem.h"
#include <chrono/assets/ChTexture.h>


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
  ChQuaternion<> qmonopile = frame.GetRot();
  //Get unity vector in z direction
  ChVector<> zUnityVector = ChVector<>(0,0,1);
  //Rotate Coordinate system back
  ChQuaternion<> qcorrection = Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X);

  ChQuaternion<> qcombined = qmonopile* qcorrection;
  //Get vector in direction of tower axis by rotating vector around quaternion
  ChVector<> towerAxis = qcombined.Rotate(zUnityVector);

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

  //
  //
  //buoyancy visualized
  //buoyancyCenterVizNode = std::make_shared<ChNodeFEAxyz>(ChVector<>(0,0,0));
  //mesh->AddNode(buoyancyCenterVizNode);
  // Create a sphere that will indicate buoyancy center position
  buoyancyCenterVizSphere = std::make_shared<ChBodyEasySphere>(8,      // radius
                                                       8000,   // density
                                                       false,   // collide enable?
                                                       true);  // visualization?
  buoyancyCenterVizSphere->SetPos(ChVector<>(0, 0, 0));
  buoyancyCenterVizSphere->SetBodyFixed(true);
  system.Add(buoyancyCenterVizSphere);
  // optional, attach a texture for better visualization
  auto mtextureball1 = std::make_shared<ChTexture>();
  mtextureball1->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
  buoyancyCenterVizSphere->AddAsset(mtextureball1);

  ipCenterVizSphere = std::make_shared<ChBodyEasySphere>(8,      // radius
                                                       8000,   // density
                                                       false,   // collide enable?
                                                       true);  // visualization?
  ipCenterVizSphere->SetPos(ChVector<>(0, 0, 0));
  ipCenterVizSphere->SetBodyFixed(true);
  system.Add(ipCenterVizSphere);
  // optional, attach a texture for better visualization
  auto mtextureball2 = std::make_shared<ChTexture>();
  mtextureball2->SetTextureFilename(GetChronoDataFile("blu.png"));
  ipCenterVizSphere->AddAsset(mtextureball2);

  topMarkerVizSphere = std::make_shared<ChBodyEasySphere>(8,      // radius
                                                       8000,   // density
                                                       false,   // collide enable?
                                                       true);  // visualization?
  topMarkerVizSphere->SetPos(ChVector<>(0, 0, 0));
  topMarkerVizSphere->SetBodyFixed(true);
  system.Add(topMarkerVizSphere);
  // optional, attach a texture for better visualization
  topMarkerVizSphere->AddAsset(mtextureball2);

  bottomMarkerVizSphere = std::make_shared<ChBodyEasySphere>(8,      // radius
                                                       8000,   // density
                                                       false,   // collide enable?
                                                       true);  // visualization?
  bottomMarkerVizSphere->SetPos(ChVector<>(0, 0, 0));
  bottomMarkerVizSphere->SetBodyFixed(true);
  system.Add(bottomMarkerVizSphere);
  // optional, attach a texture for better visualization
  bottomMarkerVizSphere->AddAsset(mtextureball2);

  //Update Buoyancy Force
  update();

  //Add load to container
  loadContainer->Add(buoyancyForce);
}

void Buoyancy::update(){

  markerBottom->UpdateState();
  markerTop->UpdateState();

  bottomMarkerVizSphere->SetPos(markerBottom->GetAbsCoord().pos);
  topMarkerVizSphere->SetPos(markerTop->GetAbsCoord().pos);

  ChVector<> seaLevelVector = ChVector<>(0,0,p.seaLevel);
  ChVector<> towerPos = monopile->GetPos();
  ChFrameMoving<> frame = monopile->GetFrame_COG_to_abs();
  //Get rotation of frame as a quaternion
  ChQuaternion<> qmonopile = frame.GetRot();
  //Rotate Coordinate system back
  ChQuaternion<> qcorrection = Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X);

  ChQuaternion<> qcombined = qmonopile* qcorrection;
  GetLog() << "qcombined:" << qcombined << "\n";
  //Get unity vector in z direction
  ChVector<> zUnityVector = ChVector<>(0,0,1);
  //Get vector in direction of tower axis by rotating vector around quaternion
  ChVector<> towerAxis = qcombined.Rotate(zUnityVector);

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

  ipCenterVizSphere->SetPos(intersectionPoint);

  computeBuoyancy(vecE, vecI, intersectionPoint);

  GetLog() << "gravityCenter:" << towerPos << "\n";
}

void Buoyancy::computeBuoyancy(ChVector<> vecE, ChVector<> vecI, ChVector<> intersectionPoint){
  // Skizze:
  // G: Center of gravity
  // S: intersection point at water surface
  // B: buoyancy Center
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

  ChVector<> vecES = intersectionPoint- vecE;
  ChVector<> vecIS = intersectionPoint- vecE;
  ChVector<> vecSE = vecE - intersectionPoint;
  ChVector<> vecSI = vecI - intersectionPoint;

  ChVector<> vecGE = vecE - towerPos;
  ChVector<> vecGS = intersectionPoint - towerPos;
  ChVector<> vecGI = vecI - towerPos;

  GetLog() << "vecSE:" << vecSE << "\n";

  double force = 0;
  //check if distance to sea level is larger than distance to top of tower
  if(vecGS.Length() >= vecGI.Length() && vecGS.Length() >= vecGE.Length()){
    //sanity check if I and E are both below Sea level
    if(vecE.z() < p.seaLevel && vecI.z() < p.seaLevel){
    //tower completely submerged, buoyancy center is same as gravity center
    buoyancyCenter = towerPos;
    force = computeMaximumBuoyancyForce();
    }
    else{
    // in this case the tower is "flying", and we should not apply any buoyancy force
    GetLog() << "flying tower\n";
    force = 0;
    }
  }
  else{
    //Check which end of the tower is submerged, and which is above the sea
    if(vecE.z() > p.seaLevel){
      GetLog() << "partly submerged, E above sea level\n";
      //construct buoyancy volume from S to I
      buoyancyCenter = intersectionPoint + 0.5*vecSI;
      ChVector<> submergedVector = vecSI*2;
      force = computeBuoyancyForce(submergedVector.Length());
    }
    else if(vecI.z() > p.seaLevel){
      GetLog() << "partly submerged, I above sea level\n";
      //construct buoyancy volume from S to E
      buoyancyCenter = intersectionPoint + 0.5*vecSE;
      ChVector<> submergedVector = vecSE*2;
      force = computeBuoyancyForce(submergedVector.Length());
    }
    else GetLog() << "both E and I below sea level.This MUST not happen.\n";
  }

  GetLog() << "buoyancy center" << buoyancyCenter << "\n";
  GetLog() << "buoyancyForce:" << force << "\n";

  buoyancyCenterVizSphere->SetPos(buoyancyCenter);

  buoyancyForce->SetApplicationPoint(buoyancyCenter,false);

  buoyancyForce->SetForce(ChVector<>(0,0,force),false);
}

double Buoyancy::computeMaximumBuoyancyForce(){
        //tower completely submerged, return volume of complete water cylinder
    double submergedVolumeMonopile =  M_PI*pow(p.towerRadius,2)*p.towerHeight;
    return submergedVolumeMonopile*p.rhoWater*p.g;
}

double Buoyancy::computeBuoyancyForce(double submergedLength){
    double submergedVolumeMonopile = M_PI*pow(p.towerRadius,2)*submergedLength;
    return submergedVolumeMonopile*p.rhoWater*p.g;
}
