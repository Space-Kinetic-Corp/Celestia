// samporient.h
//
// Copyright (C) 2006, Chris Laurel <claurel@shatters.net>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#pragma once

#include <celcompat/filesystem.h>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "rotation.h"

namespace celestia::ephem
{
struct OrientationSample
{
    Eigen::Quaternionf q;
    double             t;
};

using OrientationSampleVector = std::vector<OrientationSample>;


/*! SampledOrientation is a rotation model that interpolates a sequence
 *  of quaternion keyframes. Typically, an instance of SampledRotation will
 *  be created from a file with LoadSampledOrientation().
 */
class SampledOrientation : public RotationModel
{
public:
    SampledOrientation()           = default;
    ~SampledOrientation() override = default;

    /*! Add another quaternion key to the sampled orientation. The keys
     *  should have monotonically increasing time values.
     */
    void addSample(double tjd, const Eigen::Quaternionf &q);

    /*! The orientation of a sampled rotation model is entirely due
     *  to spin (i.e. there's no notion of an equatorial frame.)
     */
    Eigen::Quaterniond spin(double tjd) const override;

    bool   isPeriodic() const override;
    double getPeriod() const override;

    void getValidRange(double &begin, double &end) const override;

private:
    Eigen::Quaternionf getOrientation(double tjd) const;

private:
    OrientationSampleVector samples;
    mutable int             lastSample{ 0 };

    enum InterpolationType
    {
        Linear = 0,
        Cubic  = 1,
    };

    InterpolationType interpolation{ Linear };
};

std::unique_ptr<RotationModel> LoadSampledOrientation(const fs::path &filename);

} // namespace celestia::ephem
