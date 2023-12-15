// torusreferencemark.h
//
// Copyright (C) 2007-2009, Celestia Development Team
// Original version by Chris Laurel <claurel@gmail.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#ifndef _CELENGINE_TORUSREFERENCEMARK_H_
#define _CELENGINE_TORUSREFERENCEMARK_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <celengine/referencemark.h>
#include <celengine/selection.h>
#include <celmath/vecmath.h>
#include <celutil/color.h>

class Body;

// Torus params sample
template<typename T>
struct TorusSample
{
    double t;
    T      x, y;
};

template<typename T>
bool
operator<(const TorusSample<T> &a, const TorusSample<T> &b)
{
    return a.t < b.t;
}

class TorusReferenceMark : public ReferenceMark
{
public:
    TorusReferenceMark(const Body &_body);
    ~TorusReferenceMark() override;

    void setColor(Color _color);
    void setScale(float _scale);
    void setRadii(double innerRadius, double outerRadius);
    void loadSemiPrincipalFromFile(const std::string &filename);

    void render(
        Renderer              *renderer,
        const Eigen::Vector3f &position,
        float                  discSize,
        double                 tdb,
        const Matrices& m) const override;
    float boundingSphereRadius() const override;

    bool isOpaque() const override
    {
        return m_color.alpha() == 1.0f;
    }

private:
    Eigen::Quaterniond getOrientation(double tdb) const;
    Eigen::Vector2d    getRadii(double tdb) const;

    void     loadSampledData(const std::string &filename);
    Eigen::Vector2d computeSample(double jd) const;

private:
    const Body &m_body;
    Color       m_color;

    enum class ParamSource
    {
        FIXED,
        FILE
    };

    ParamSource m_paramSource;

    double m_fixedMajorRadius;
    double m_fixedMinorRadius;

    std::vector<TorusSample<double>> m_samples;
    mutable int                 m_lastSample;

    mutable double m_radius;
    float          m_scale;
    mutable double m_lastTime;
};

#endif // _CELENGINE_TORUSREFERENCEMARK_H_