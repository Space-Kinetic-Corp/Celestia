// RegionOfInterest.cpp
//
// Visible region reference mark for ellipsoidal bodies.
//
// Copyright (C) 2008, the Celestia Development Team
// Initial version by Chris Laurel, claurel@gmail.com
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

/******************************************************************************/

#include "render.h"
#include "regionofinterest.h"
#include "body.h"
#include "selection.h"
#include "../celmath/vecgl.h"
#include <celmath/intersect.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdio>
#include <cmath>
#include <celengine/polygon.h>
#include <iostream>
#include <fstream>

/******************************************************************************/

using namespace Eigen;


/******************************************************************************/

/*! Construct a new reference mark with a polygon filled with cartesian
*   coordinates
 */
RegionOfInterest::RegionOfInterest(Body *body) :
    m_visible(true),
    m_body(body),
    m_traceOpacity(1.f),
    m_traceColor(1.f,0.f,0.f),
    m_LabelVisible(true)
{
    // Creation of the region's location that will display its label
    m_location = new Location();
    m_location->setFeatureType(Location::Observatory);
    m_location->setLabelVisible(true);
    m_location->setVisible(true);
    m_location->setSize( 200.0 );
    m_location->setDisplayName("");
    m_location->setLabelColorOverridden(true);

    body->addLocation(std::unique_ptr<Location>(m_location));
}


/******************************************************************************/

RegionOfInterest::~RegionOfInterest()
{
    PolygonList::iterator iter;
    for (iter = m_polygonList.begin(); iter != m_polygonList.end(); ++iter)
    {
        delete *iter;
    }
    m_polygonList.clear() ;
}

/******************************************************************************/

void RegionOfInterest::setVisible( bool visible )
{
    m_visible = visible;
    m_location->setVisible(m_visible && m_LabelVisible);
}

/******************************************************************************/

void RegionOfInterest::setLabelVisible( bool visible )
{
    m_LabelVisible = visible;
    m_location->setVisible(m_visible && m_LabelVisible);
}

/******************************************************************************/

bool RegionOfInterest::getLabelVisible()
{
    return m_LabelVisible;
}

/******************************************************************************/

std::string RegionOfInterest::label() const
{
    return m_location->getDisplayName();
}

/******************************************************************************/

void RegionOfInterest::setLabelColor( const Color& color )
{
    m_location->setLabelColor(color);
}

/******************************************************************************/

Color RegionOfInterest::getLabelColor() const
{
    return m_location->getLabelColor();
}

/******************************************************************************/

void RegionOfInterest::setLabel(const std::string& text)
{
    m_location->setDisplayName(std::string());
    m_location->setName(text);
}


/******************************************************************************/

void
RegionOfInterest::addPolygon(Polygon *polygon)
{
    // Nothing to do if no polygon is given
    if (polygon == NULL)
        return;

    // Store the polygon
    m_polygonList.push_back( polygon ) ;
}


/******************************************************************************/

void RegionOfInterest::setLabelOrigin(float longitude, float latitude, float alt)
{
    m_location->setPositionPlanetocentric(Vector3f(longitude,
                                                   latitude,
                                                   alt));
    m_location->setVisible(true);
}


/******************************************************************************/

void
RegionOfInterest::render(Renderer* /* renderer */,
                         const Vector3f& /* pos */,
                         float /* discSizeInPixels */,
                         double tdb,
                         const Matrices& m /* frustum */) const
{
    if( !isActive() || !m_visible || m_polygonList.empty() )
    {
        return;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Draw polygon

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    glPushMatrix();

    Quaterniond q = m_body->getOrientationCorrection3() * m_body->getEclipticToBodyFixed(tdb) ;
    auto qV = q.cast<float>().conjugate();
//    glRotate(q.cast<float>().conjugate());
    glRotatef(qV.w(), qV.x(), qV.y(), qV.z());


    PolygonList::const_iterator iter;
    for (iter = m_polygonList.begin(); iter != m_polygonList.end(); ++iter)
    {
        (*iter)->render(false);
    }

    glPopMatrix();

    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
}


/******************************************************************************/

float
RegionOfInterest::boundingSphereRadius() const
{
    // Make the bounding sphere large enough to include the projected points
    return m_body->getRadius() * getSizeFactor();
}


/******************************************************************************/

void
RegionOfInterest::setTraceColor(const Color &color)
{
    m_traceColor = color ;

    // Update polygons color
    PolygonList::iterator iter;
    for (iter = m_polygonList.begin(); iter != m_polygonList.end(); ++iter)
    {
        (*iter)->setColor(color);
    }
}


/******************************************************************************/

void
RegionOfInterest::setTraceOpacity(float opacity)
{
    m_traceOpacity = opacity ;

    // Update polygons opacity
    PolygonList::iterator iter;
    for (iter = m_polygonList.begin(); iter != m_polygonList.end(); ++iter)
    {
        (*iter)->setOpacity(opacity);
    }
}
