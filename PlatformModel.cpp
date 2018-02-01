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
      auto node1 = std::make_shared<ChNodeFEAxyzD>(p.towerInitPos, p.towerInitDir);
      mesh->AddNode(node1);

      monopile = std::make_shared<ChBodyEasyCylinder>(p.towerRadius,p.towerHeight,p.towerDensity);
      monopile->SetPos(node1->GetPos());

      //Rotate around x axis to align tower with z axis
      ChQuaternion<> q = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X);
      monopile->SetRot(q);

      system.Add(monopile);

      //Constraints
      node1->SetFixed(true);

      GetLog() << "monopile initial position:" << monopile->GetPos() << "\n";

      auto constraint_pos = std::make_shared<ChLinkPointFrame>();
      constraint_pos->Initialize(node1,monopile);
      //system.Add(constraint_pos);

      auto constraint_dir = std::make_shared<ChLinkDirFrame>();
      constraint_dir->Initialize(node1,monopile);
      constraint_dir->SetDirectionInAbsoluteCoords(ChVector<>(0, 0, 1));
      system.Add(constraint_dir);

      //Add Gravity
      system.Set_G_acc(ChVector<>(0,0,-9.8));

      // Mooring Lines
      sectionCable = std::make_shared<ChBeamSectionCable>();
      sectionCable->SetDiameter(p.mooringDiameter);
      sectionCable->SetYoungModulus(p.mooringYoungModulus);
      sectionCable->SetBeamRaleyghDamping(p.mooringRaleyghDamping);

      //for(int i = 0; i < 2; i++){
        // Shortcut!
        // This ChBuilderBeamANCF helper object is very useful because it will
        // subdivide 'beams' into sequences of finite elements of beam type, ex.
        // one 'beam' could be made of 5 FEM elements of ChElementBeamANCF class.
        // If new nodes are needed, it will create them for you.
        ChBuilderBeamANCF builder;

        // Now, simply use BuildBeam to create a beam from a point to another:
        builder.BuildBeam(mesh,                       // the mesh where to put the created nodes and elements
                          sectionCable,            // the ChBeamSectionCable to use for the ChElementBeamANCF elements
                          p.mooringNrElements,      // the number of ChElementBeamANCF to create
                          ChVector<>(0, 0, p.mooringPosZ),     // the 'A' point in space (beginning of beam)
                          ChVector<>(0, p.mooringL,p.mooringPosZ));  // the 'B' point in space (end of beam)

        // After having used BuildBeam(), you can retrieve the nodes used for the beam,
        // For example say you want to fix both pos and dir of A end and apply a force to the B end:
        // builder.GetLastBeamNodes().back()->SetFixed(true);
        // builder.GetLastBeamNodes().front()->SetForce(ChVector<>(0, -0.2, 0));

        // For instance, now retrieve the A end and add a constraint to
        // block the position only of that node:
        auto mtruss = std::make_shared<ChBody>();
        mtruss->SetBodyFixed(true);

        //fairleads constraint
        auto constraint_pos2 = std::make_shared<ChLinkPointFrame>();
        constraint_pos2->Initialize(builder.GetLastBeamNodes().front(), monopile);
        system.Add(constraint_pos2);

        auto constraint_dir2 = std::make_shared<ChLinkDirFrame>();
        constraint_dir2->Initialize(builder.GetLastBeamNodes().front(), monopile);
        constraint_dir2->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
        system.Add(constraint_dir2);

        //anchor constraint
        auto constraint_hinge = std::make_shared<ChLinkPointFrame>();
        constraint_hinge->Initialize(builder.GetLastBeamNodes().back(), mtruss);
        system.Add(constraint_hinge);

}
