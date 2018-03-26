#include "params.h"
#include "defineParameters.h"



params defineParameters(){

  params p;
  p.towerHeight=100;
  p.towerRadius=5;
  p.towerDensity=1000;
  p.towerInitPos = ChVector<>(0, 0, 0);
  p.towerInitDir = ChVector<>(0, 0, 1);

  p.mooringLineNr = 3;
  p.mooringDiameter = 0.15;
  p.mooringYoungModulus = 200e9;
  p.mooringRaleyghDamping = 0.000;
  p.mooringNrElements = 10;
  p.mooringL = 100;
  p.mooringPosFairleadZ = 25;
  p.mooringPosBottomZ = 10;

  return p;
}
