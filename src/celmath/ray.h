// ray.h
//
// Copyright (C) 2002, Chris Laurel <claurel@shatters.net>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "vecmath.h"

namespace celmath
{
// Eigen 3.4 will support a transform method on ParametrizedLine, which may enable
// removing at least one of the below overloads.

template<typename T>
Eigen::ParametrizedLine<T, 3>
transformRay(const Eigen::ParametrizedLine<T, 3> &line, const Eigen::Matrix<T, 3, 3> &m)
{
    return Eigen::ParametrizedLine<T, 3>(m * line.origin(), m * line.direction());
}

template<typename T>
Eigen::ParametrizedLine<T, 3>
transformRay(const Eigen::ParametrizedLine<T, 3> &line, const Eigen::Matrix<T, 4, 4> &m)
{
    Eigen::Matrix<T, 4, 1> o(Eigen::Matrix<T, 4, 1>::Ones());
    o.head(3) = line.origin();
    Eigen::Matrix<T, 4, 1> d(Eigen::Matrix<T, 4, 1>::Zero());
    d.head(3) = line.direction();
    return Eigen::ParametrizedLine<T, 3>((m * o).head(3), (m * d).head(3));
}

template<class T>
class Ray3
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Ray3();
    Ray3(const Eigen::Matrix<T, 3, 1> &origin, const Eigen::Matrix<T, 3, 1> &direction);
    // Compatibility
    Ray3(const Point3<T> &, const Eigen::Vector3<T> &);

    Eigen::Matrix<T, 3, 1> point(T) const;

    Ray3<T> transform(const Eigen::Matrix<T, 3, 3> &m) const
    {
        return Ray3<T>(m * origin, m * direction);
    }

public:
    Eigen::Matrix<T, 3, 1> origin;
    Eigen::Matrix<T, 3, 1> direction;
};

typedef Ray3<float>  Ray3f;
typedef Ray3<double> Ray3d;

template<class T>
Ray3<T>::Ray3() : origin(0, 0, 0), direction(0, 0, -1)
{
}

template<class T>
Ray3<T>::Ray3(const Eigen::Matrix<T, 3, 1> &_origin, const Eigen::Matrix<T, 3, 1> &_direction) :
    origin(_origin), direction(_direction)
{
}

// Compatibility
template<class T>
Ray3<T>::Ray3(const Point3<T> &_origin, const Eigen::Vector3<T> &_direction) :
    origin(_origin.x, _origin.y, _origin.z), direction(_direction.x, _direction.y, _direction.z)
{
}

template<class T>
Eigen::Matrix<T, 3, 1>
Ray3<T>::point(T t) const
{
    return origin + direction * t;
}

// Compatibility
template<class T>
Ray3<T>
operator*(const Ray3<T> &r, const Eigen::Matrix3<T> &m)
{
    Eigen::Map<Eigen::Matrix<T, 3, 3>> m2(&m[0][0]);
    return Ray3<T>(m2 * r.origin, m2 * r.direction);
}

// Compatibility
template<class T>
Ray3<T>
operator*(const Ray3<T> &r, const Eigen::Matrix4<T> &m)
{
    Eigen::Map<Eigen::Matrix<T, 4, 4>> m2(&m[0][0]);
    return Ray3<T>(m2 * r.origin, m2 * r.direction);
}
} // end namespace celmath
