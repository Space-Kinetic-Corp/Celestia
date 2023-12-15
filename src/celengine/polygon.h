// polygon.h
//
// Polygon displayed on earth using Shadow Volumes method
//
// Copyright (C) 2008, the Celestia Development Team
// Initial version by Chris Laurel, claurel@gmail.com
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.


#ifndef _CELENGINE_POLYGON_H_
#define _CELENGINE_POLYGON_H_

#include <vector>
#include <celutil/color.h>
#include <celmath/vecgl.h>
#include <Eigen/Core>
#include <celmath/vecmath.h>
#include "glsupport.h"

using namespace Eigen;

class Polygon : public std::vector<Vector3d>
{
public:
    Polygon( const Color &color = Color(1.0f, 0.0f, 1.0f),
            float opacity = 1.0f,
            const Color &shadowBoxColor = Color(0.0f, 1.0f, 1.0f) );
    ~Polygon();

    void setColor(const Color &color);
    void setOpacity(float opacity);
    void render(bool drawShadowVolume) const;

    static void polygon2dtoPolygon3d(std::vector<Point2f>& geoPoly, float bodyRadius, Polygon& cartPoly) ;
    static float area(const std::vector<Point2f>& contour) ;
    static Vector3d spherePointLL(float longi, float lati, float bodyRadius) ;
    static bool crossDateLine( const Point2f &geoPt1, const Point2f &geoPt2, int *side ) ;

protected:
    void createDisplayList(bool drawShadowVolume) const;

    void Draw( bool drawShadowVolume) const;
    void DrawShadowVolume(const std::vector<Vector3d> &polygon, bool filled = true) const;
    void DrawFarPlane() const;
    void DrawShadowColor() const;

protected:
    mutable bool   m_updateDisplayList;
    Color          m_color;
    float          m_opacity;
    Color          m_shadowBoxColor;
    mutable GLuint m_displayList;
};

#endif // _CELENGINE_POLYGON_H_
