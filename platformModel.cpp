#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChElementCableANCF.h"

#include "platformModel.h"
#include "params.h"

using namespace chrono;
using namespace chrono::fea;

void platformModel(ChSystem& system, std::shared_ptr<ChMesh> mesh, params p) {

      auto node1 = std::make_shared<ChNodeFEAxyzD>(ChVector<>(0, 0, -0.2), ChVector<>(1, 0, 0));
      mesh->AddNode(node1);

      auto monopile = std::make_shared<ChBodyEasyCylinder>(p.towerRadius,p.towerHeight,p.towerDensity);
      monopile->SetPos(node1->GetPos());
      system.Add(monopile);

      //Constraints

      node1->SetFixed(true);

      GetLog() << "monopile position:" << monopile->GetPos() << "\n";

      auto constraint_pos = std::make_shared<ChLinkPointFrame>();
      constraint_pos->Initialize(node1,monopile);
      system.Add(constraint_pos);

      auto constraint_dir = std::make_shared<ChLinkDirFrame>();
      constraint_dir->Initialize(node1,monopile);
      constraint_dir->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
      system.Add(constraint_dir);

}

void createMooringLines(ChSystem& system, std::shared_ptr<ChMesh> mesh, params p){

  

}
