#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include <chrono_irrlicht/ChIrrTools.h>
#include <chrono_irrlicht/ChIrrCamera.h>
#include <chrono/physics/ChLoadsBody.h>
#include <chrono/physics/ChLoadContainer.h>

#include "chrono_fea/ChElementSpring.h"
#include "chrono_fea/ChElementBar.h"
#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChElementTetra_10.h"
#include "chrono_fea/ChElementHexa_8.h"
#include "chrono_fea/ChElementHexa_20.h"

#include "params.h"
#include "defineParameters.h"
#include "PlatformModel.h"
#include "Buoyancy.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Cables FEM", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(100.f, 0.6f, -1.f)); //camera position

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    //define parameters
    params p = defineParameters();

    PlatformModel model(my_system, my_mesh, p);
    //get reference to monopile
    //std::shared_ptr<Buoyancy> buoyancy = model.getBuoyancy();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!


    auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);


    auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS); // E_GLYPH_NODE_CSYS
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);


    //test visualization
    /*
    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(mvisualizemeshref);

    std::shared_ptr<ChNodeFEAxyz> hnode1_lower = std::make_shared<ChNodeFEAxyz>(ChVector<>(0,0,0));
    my_mesh->AddNode(hnode1_lower);

    auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(1.006);
    my_mesh->AddAsset(mvisualizemeshC);
    */
    //Set "UP"-direction of ChCamera

    auto device = application.GetDevice();
    RTSCamera* camera = new RTSCamera(device, device->getSceneManager()->getRootSceneNode(), device->getSceneManager(),
                                      -1, -160.0f, 1.0f, 0.003f);

    camera->setPosition(irr::core::vector3df(100.f, 0.6f, -1.f));
    camera->setTarget(irr::core::vector3df(0.f, 0.f, 0.f));
    camera->setUpVector(irr::core::vector3df(0.f, 0.f, 100.f));
    camera->setNearValue(0.1f);
    camera->setMinZoom(0.6f);

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!
    application.AssetUpdateAll();

    // Mark completion of system construction
    my_system.SetupInitial();

    //auto forcedNode = std::make_shared<ChNodeFEAxyz>(ChVector<>(0,0,0));
    //my_mesh->AddNode(forcedNode);
    //auto std::make_shared<ChLoadBodyForce> loadBodyForce;

    // Change solver settings
    my_system.SetSolverType(ChSolver::Type::MINRES);
    my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetMaxItersSolverSpeed(200);
    my_system.SetMaxItersSolverStab(200);
    my_system.SetTolForce(1e-13);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

    // Change type of integrator:
    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    // my_system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);  // precise,slower, might iterate each step

    // if later you want to change integrator settings:
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxiters(2);
        mystepper->SetAbsTolerances(1e-6);
    }

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    application.SetTimestep(0.01);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();

        //Update BuoyancyForce
        //GetLog() << buoyancy->getMonopile() << "\n";
        //GetLog() << "before update\nu
        model.update();

        //ChIrrAppTools
        ChIrrTools::drawAllCOGs(my_system,application.GetVideoDriver(),100);
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
