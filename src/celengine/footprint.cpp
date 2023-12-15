//// footprint.cpp
////
//// Visible region reference mark for ellipsoidal bodies.
////
//// Copyright (C) 2008, the Celestia Development Team
//// Initial version by Chris Laurel, claurel@gmail.com
////
//// This program is free software; you can redistribute it and/or
//// modify it under the terms of the GNU General Public License
//// as published by the Free Software Foundation; either version 2
//// of the License, or (at your option) any later version.

#include "footprint.h"

#include "body.h"
#include "render.h"
#include "selection.h"
#include "sensorgeometry.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <celengine/polygon.h>
#include <celmath/intersect.h>
#include <celmath/vecmath.h>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

using namespace Eigen;

double Footprint::s_sensorHighRes = 1 / 86400.;

/*! Construct a new reference mark with a polygon filled with cartesian
 *   coordinates
 */
Footprint::Footprint() :
    m_sensorGeometry(NULL), m_visible(false), m_body(NULL), m_targetType(LongLatPolygon_t),
    m_traceDuration(-1)
{
}

Footprint::~Footprint()
{
    std::map<double, Polygon *>::iterator iter;
    for (iter = m_polygonMap.begin(); iter != m_polygonMap.end(); ++iter)
    {
        delete iter->second;
    }
    m_polygonMap.clear();
}

void
Footprint::setTraceDuration(float traceDuration)
{
    m_traceDuration = traceDuration;

    // Add footprint to the body (depending on the trace duration)
    if (isActive() && m_body)
    {
        //        m_body->addReferenceMark(this);

        // make a unique pointer from this
        std::unique_ptr<ReferenceMark> thisPtr(this);
        m_body->addReferenceMark(std::move(thisPtr));
    }
}

void
Footprint::setTraceColorFile(std::string colorFileName)
{
    m_traceColorFileName = colorFileName;

    // If a color file exists, read it
    if (colorFileName != "")
    {
        std::ifstream inColor(colorFileName.c_str());
        if (!inColor.good())
            return;

        double lastSampleTime = -std::numeric_limits<double>::infinity();
        while (inColor.good())
        {
            double tdb, r, g, b, a;
            inColor >> tdb;
            inColor >> r;
            inColor >> g;
            inColor >> b;
            inColor >> a;

            if (inColor.good())
            {
                // Skip samples with duplicate times; such trajectories are invalid, but
                // are unfortunately used in some existing add-ons.
                if (tdb != lastSampleTime)
                {
                    m_colorMap.insert(std::pair<double, Color>(
                        tdb,
                        Color(
                            static_cast<float>(r),
                            static_cast<float>(g),
                            static_cast<float>(b),
                            static_cast<float>(a))));
                    lastSampleTime = tdb;
                }
            }
        }
        inColor.close();
    }
}

void
Footprint::addPolygon(Polygon *polygon, double jd)
{
    const double halfDuration_(3600 * m_traceDuration / 86400.);

    TimedItemMap::iterator it, limitIt;

    // Simultaneous polygon
    if (m_polygonMap.find(jd) != m_polygonMap.end())
    {
        delete polygon;
        return;
    }

    // Past trace
    limitIt = m_polygonMap.lower_bound(jd - halfDuration_);
    it      = m_polygonMap.begin();
    while (it != limitIt)
    {
        delete it->second;
        m_polygonMap.erase(it++);
    }

    // Future trace
    it = m_polygonMap.upper_bound(jd);
    while (it != m_polygonMap.end())
    {
        delete it->second;
        m_polygonMap.erase(it++);
    }

    // Nettoyage
    simplifyTrace(
        m_polygonMap.lower_bound(jd - halfDuration_),
        m_polygonMap.lower_bound(jd),
        s_sensorHighRes);

    // Set the polygon color
    if (m_colorMap.empty())
    {
        polygon->setColor(m_traceColor);
    }
    else
    {
        std::map<double, Color>::iterator pos;
        pos = m_colorMap.lower_bound(jd);
        if (pos != m_colorMap.begin())
        {
            --pos;
        }

        // No display with black color
        if (pos->second == Color(float(0.), float(0.), float(0.)))
        {
            // TODO : polygon will be lost
            return;
        }

        polygon->setColor(pos->second);
    }
    polygon->setOpacity(m_traceOpacity);

    // Insert the new polygon into the map
    m_polygonMap[jd] = polygon;
}

// -------------------------------------------------------------------------- //
// simplifyTrace
//
//! \brief Simplification d'une trace selon un crit�re de temps
// -------------------------------------------------------------------------- //

void
Footprint::simplifyTrace(
    const TimedItemMap::iterator &from,
    const TimedItemMap::iterator &to,
    double                        resolution)
{
    TimedItemMap::iterator it         = from;
    TimedItemMap::iterator lastKeepIt = from;
    TimedItemMap::iterator itNext;

    while (it != to)
    {
        if ((abs(it->first - lastKeepIt->first) < resolution) && (lastKeepIt != it))
        {
            delete it->second;
            m_polygonMap.erase(it++);
        }
        else
        {
            lastKeepIt = it;
            ++it;
        }
    }
}

void
Footprint::render(
    Renderer * /* renderer */,
    const Vector3f & /* pos */,
    float  discSizeInPixels,
    double tdb,
    const Matrices & /* m */) const
{
    if (!isActive())
    {
        return;
    }

    // Compute the instantaneous footprint coordinates
    m_sensorGeometry->computeFootprint(tdb);

    if (!m_visible)
    {
        return;
    }

    // No polygon, no rendering
    if (m_polygonMap.empty())
    {
        return;
    }

    // Don't render anything if the current time is not within the
    // target object's time window.
    if (m_targetType == SensorIntersection_t)
    {
        if (m_target.body() != NULL)
        {
            if (!m_target.body()->extant(tdb))
                return;
        }
    }

    // Don't render the trace if the it's smaller than the minimum size
    const float minDiscSize         = 5.0f;
    const float fullOpacityDiscSize = 10.0f;
    float       opacity = (discSizeInPixels - minDiscSize) / (fullOpacityDiscSize - minDiscSize);

    if (opacity <= 0.0f)
        return;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Draw polygon

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    glPushMatrix();

    auto q = m_body->getEclipticToBodyFixed(tdb).conjugate().cast<float>();

    glRotatef(q.w(), q.x(), q.y(), q.z());

    std::map<double, Polygon *>::const_iterator iter;
    for (iter = m_polygonMap.begin(); iter != m_polygonMap.end(); ++iter)
    {
        iter->second->render(false);
    }

    glPopMatrix();

    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
}

float
Footprint::boundingSphereRadius() const
{
    // Make the bounding sphere large enough to include the projected points
    return m_body->getRadius() * getSizeFactor();
}
