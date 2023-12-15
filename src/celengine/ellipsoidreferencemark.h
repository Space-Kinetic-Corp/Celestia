// ellipsoidreferencemark.h
//
// Copyright (C) 2007-2009, Celestia Development Team
// Original version by Chris Laurel <claurel@gmail.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#ifndef _CELENGINE_ELLIPSOIDREFERENCEMARK_H_
#define _CELENGINE_ELLIPSOIDREFERENCEMARK_H_

#include <celutil/color.h>
#include <celengine/referencemark.h>
#include <celengine/selection.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <celmath/vecmath.h>
#include <celmath/frustum.h>
#include "render.h"


class Body;


// Position-only sample
template <typename T> struct Sample
{
    double t;
    T x, y, z;
};


template <typename T> bool operator<(const Sample<T>& a, const Sample<T>& b)
{
    return a.t < b.t;
}


class EllipsoidReferenceMark : public ReferenceMark
{
public:
    EllipsoidReferenceMark(const Body& _body);
    ~EllipsoidReferenceMark() override ;

    void setColor(Color _color);
    void setScale(float _scale);
    void setSemiPrincipalAxis(const Vec3d& axis);
    void loadSemiPrincipalFromFile(const std::string& filename);


    void render(Renderer* renderer,
                 const Eigen::Vector3f& position,
                 float discSize,
                 double tdb,
                 const Matrices& m) const override;
    float boundingSphereRadius() const override;

    bool isOpaque() const override
    {
        return m_color.alpha() == 1.0f;
    }

private :
    Eigen::Quaterniond getOrientation(double tdb) const;
    Eigen::Vector3d getSemiPrincipalAxis(double tdb) const ;

    void loadSampledData(const std::string& filename);
    Eigen::Vector3d computeSample(double jd) const;

private:
    const Body& m_body;
    Color m_color;

    enum class AxisSource {
        FIXED,
        FILE
    };

    AxisSource m_axisSource;
    Eigen::Vector3d m_fixedSemiAxis;

    std::vector<Sample<double> > m_samples;
    mutable int m_lastSample;

    mutable double m_radius;
    float m_scale;
    mutable double m_lastTime;
//    GLUquadricObj *m_quadratic;
};


#endif // _CELENGINE_ELLIPSOIDREFERENCEMARK_H_
