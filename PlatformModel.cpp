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

using namespace chrono;
using namespace chrono::fea;

PlatformModel::PlatformModel(ChSystem& system, std::shared_ptr<ChMesh> mesh, params p) {

      //Monopile
      monopileInitNode = std::make_shared<ChNodeFEAxyzD>(p.towerInitPos, p.towerInitDir);
      mesh->AddNode(monopileInitNode);

      monopile = std::make_shared<ChBodyEasyCylinder>(p.towerRadius,p.towerHeight,p.towerDensity);
      monopile->SetPos(monopileInitNode->GetPos());

      //Rotate around x axis to align tower with z axis
      ChQuaternion<> q = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X);
      monopile->SetRot(q);

      system.Add(monopile);

      //Constraints

      GetLog() << "monopile initial position:" << monopile->GetPos() << "\n";
      GetLog() << "Rest Length" << p.mooringRestLength << "\n";

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
      system.Set_G_acc(ChVector<>(0,0,-9.8));

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
}
