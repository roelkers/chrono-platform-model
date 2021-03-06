#ifndef MOORINGLINE_H
#define MOORINGLINE_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_fea/ChMesh.h"

#include "params.h"

using namespace chrono;
using namespace chrono::fea;

class MooringLine{
private:

public:
  MooringLine(ChSystem& system, std::shared_ptr<ChMesh> mesh, params p, double theta, std::shared_ptr<ChBodyEasyCylinder> monopile);

};

#endif
