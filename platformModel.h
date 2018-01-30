#ifndef PLATFORMMODEL_H
#define PLATFORMMODEL_H

#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChMesh.h"
#include "params.h"

using namespace chrono;
using namespace chrono::fea;

void platformModel(ChSystem& system, std::shared_ptr<ChMesh> mesh, params p);
void createMooringLines(ChSystem& system, std::shared_ptr<ChMesh> mesh, params p);

#endif
