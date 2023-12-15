// samporbit.h
//
// Copyright (C) 2002-2007, Chris Laurel <claurel@gmail.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#pragma once

#include <memory>
#include <Eigen/Core>

#include <celcompat/filesystem.h>

namespace celestia::ephem
{

class Orbit;

enum class TrajectoryInterpolation
{
    Linear,
    Cubic,
};

enum class TrajectoryPrecision
{
    Single,
    Double
};

std::unique_ptr<Orbit> LoadSampledTrajectoryDoublePrec(const fs::path& filename, const fs::path& colorFilename, TrajectoryInterpolation interpolation);
std::unique_ptr<Orbit> LoadSampledTrajectorySinglePrec(const fs::path& filename, const fs::path& colorFilename, TrajectoryInterpolation interpolation);
std::unique_ptr<Orbit> LoadXYZVTrajectoryDoublePrec(const fs::path& filename, TrajectoryInterpolation interpolation);
std::unique_ptr<Orbit> LoadXYZVTrajectorySinglePrec(const fs::path& filename, TrajectoryInterpolation interpolation);
std::unique_ptr<Orbit> LoadXYZVBinarySinglePrec(const fs::path& filename, TrajectoryInterpolation interpolation);
std::unique_ptr<Orbit> LoadXYZVBinaryDoublePrec(const fs::path& filename, TrajectoryInterpolation interpolation);


//Eigen::Vector3d cubicInterpolate(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
//                                 const Eigen::Vector3d& p1, const Eigen::Vector3d& v1,
//                                 double t);
//
//
//Eigen::Vector3d cubicInterpolateVelocity(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
//                                         const Eigen::Vector3d& p1, const Eigen::Vector3d& v1,
//                                         double t);

}
