#ifndef BuoyancyForce_H
#define BuoyancyForce_H

#include "chrono/physics/ChMarker.h"
#include <chrono/physics/ChLoadsBody.h>
#include <chrono/physics/ChLoadContainer.h>
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChSystem.h"
#include <chrono_fea/ChNodeFEAxyz.h>

#include "params.h"

using namespace chrono;
using namespace chrono::fea;

class Buoyancy {
private:
  params p;
  std::shared_ptr<ChBodyEasyCylinder> monopile;
  std::shared_ptr<ChLoadBodyForce> buoyancyForce;
  std::shared_ptr<ChLoadContainer> loadContainer;
  std::shared_ptr<ChMarker> markerBottom;
  std::shared_ptr<ChMarker> markerTop;
  ChVector<> buoyancyCenter; //z-coordinate of buyoancy center of monopile from xy-plane
  std::shared_ptr<ChNodeFEAxyz> buoyancyCenterVizNode;
public:
  Buoyancy(params p, std::shared_ptr<ChLoadContainer> loadContainer, std::shared_ptr<ChBodyEasyCylinder> monopile, std::shared_ptr<ChMesh> mesh, ChSystem& system);
  void update();
  void computeBuoyancyForce(ChVector<> vecE, ChVector<> vecI, ChVector<> intersectionPoint);
  void computeBuoyancyCenter(ChVector<> vecE, ChVector<> vecI, ChVector<> intersectionPoint);

};

#endif
