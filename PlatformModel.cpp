#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChElementCableANCF.h"

#include "PlatformModel.h"
#include "params.h"
#include "Buoyancy.h"

using namespace chrono;
using namespace chrono::fea;

PlatformModel::PlatformModel(ChSystem& system, std::shared_ptr<ChMesh> mesh, params p) {

      //Monopile
      monopileInitNode = std::make_shared<ChNodeFEAxyzD>(p.towerInitPos, p.towerInitDir);
      mesh->AddNode(monopileInitNode);

      monopile = std::make_shared<ChBodyEasyCylinder>(p.towerRadius,p.towerHeight,p.towerDensity);
      monopile->SetPos(monopileInitNode->GetPos());

      //Setup location of monopile for construction of mooring lines
      ChQuaternion<> qSetup = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X);

      monopile->SetRot(qSetup);

      system.Add(monopile);

      //Constraints

      //Add Buoyancy force

      //Init Load container
      auto loadcontainer = std::make_shared<ChLoadContainer>();
      system.Add(loadcontainer);

      buoyancy = std::make_shared<Buoyancy>(p, loadcontainer, monopile, mesh, system);

      //Fix Monopile in space
      /*auto constraint_pos = std::make_shared<ChLinkPointFrame>();
      monopileInitNode->SetFixed(true);
      constraint_pos->Initialize(monopileInitNode,monopile);
      system.Add(constraint_pos);

      auto constraint_dir = std::make_shared<ChLinkDirFrame>();
      constraint_dir->Initialize(monopileInitNode,monopile);
      constraint_dir->SetDirectionInAbsoluteCoords(ChVector<>(0, 0, 1));
      system.Add(constraint_dir);
      */

      //Add Gravity
      system.Set_G_acc(ChVector<>(0,0,-p.g));
      //system.Set_G_acc(ChVector<>(0,0,0));

      //Angular increment of Mooring Line on Monopile
      double thetaInc = 360/p.mooringLineNr;
      //Angle on Monopile of mooring line
      double theta = 0;
      //Construct Mooring Lines
      for(int i = 0; i < p.mooringLineNr; i++){
        theta = theta + thetaInc;

        GetLog() << "Mooring Line Angular position: " << theta << "deg\n";
        GetLog() << "Constructing mooring line " << i << "\n";
        MooringLine mLine(system, mesh, p, theta, monopile);
        mooringLines[i] = mLine;

      }
      //Initial rotation of the monopile
      //Rotate around x axis and y axis
      ChQuaternion<> qRotationX = Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_X);
      ChQuaternion<> qRotationZ= Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_Z);
      //Translate to initial Position
      //ChVector<> initPos = ChVector<>(0,0,0);
      ChVector<> initPos = ChVector<>(20,20,0);

      //Define initial displacement
      ChCoordsys<> initCoords =ChCoordsys<>(initPos,qRotationX*qRotationZ);
      monopile->Move(initCoords);

      GetLog() << "monopile initial position:" << monopile->GetPos() << "\n";
      GetLog() << "monopile initial rotation:" << monopile->GetRot() << "\n";
      GetLog() << "Rest Length: " << p.mooringRestLength << "\n";
}

void PlatformModel::update(){
  buoyancy->update();
}
