#ifndef PLATFORMMODEL_H
#define PLATFORMMODEL_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChElementCableANCF.h"
#include "params.h"

using namespace chrono;
using namespace chrono::fea;

class PlatformModel{

  private:
    std::shared_ptr<ChBodyEasyCylinder> monopile;
    std::shared_ptr<ChBeamSectionCable> sectionCable;

  public:
    PlatformModel(ChSystem& system, std::shared_ptr<ChMesh> mesh, params p);

};

#endif
