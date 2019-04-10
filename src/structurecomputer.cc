#include "structurecomputer.h"

void StructureComputer::clear() {
  point_.rXIHat.fill(0);
  point_.Re.fill(0);
  bundleVec_.clear();
}

void StructureComputer::push(std::shared_ptr<const CameraBundle> bundle) {
  bundleVec_.push_back(bundle);
}

Point StructureComputer::computeStructure() {

  return point_;
}
