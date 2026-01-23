#include "DrakeSim.hpp"

DrakeSim::DrakeSim(const std::string& /* urdf_path */) {}

DrakeSim::~DrakeSim() {}

void DrakeSim::SetMecanumParameters(const std::vector<double>& /* params */) {}

void DrakeSim::SetMotorParameters(const std::vector<double>& /* params */) {}

void DrakeSim::Step(double /* dt */, const std::vector<double>& /* inputs */) {}

std::vector<double> DrakeSim::GetState() {
    return {};
}

void DrakeSim::SpawnBall(double /* x */, double /* y */, double /* z */) {}

void DrakeSim::SpawnBallWithVelocity(double /* x */, double /* y */, double /* z */,
                                      double /* vx */, double /* vy */, double /* vz */) {}

void DrakeSim::RemoveBall(int /* index */) {}

void DrakeSim::SetPosition(double /* x */, double /* y */, double /* yaw */) {}
