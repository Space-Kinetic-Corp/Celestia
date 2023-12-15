// lineofinterest.cpp
//
// Line reference mark for ellipsoidal bodies.
//
// Copyright (C) 2008, the Celestia Development Team
// Initial version by Chris Laurel, claurel@gmail.com
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#include "lineofinterest.h"

#include "body.h"
#include "location.h"

#include <celengine/render.h>
#include <celmath/mathlib.h>
#include <cstdio>

/******************************************************************************/

LineOfInterest::LineOfInterest(Body *_body) :
    body(_body), m_opacity(1.0f), highestAltitude(0.0), m_color(1.0f, 1.0f, 1.0f), m_visible(true),
    m_width(1.0), m_displayList(0), m_LabelVisible(true), m_location(NULL)
{
    // Initialization of the Location that will display the line's label
    //    m_location = new Location();
    auto location = std::make_unique<Location>();
    m_location    = location.get();
    m_location->setFeatureType(Location::Observatory);
    m_location->setLabelVisible(true);
    m_location->setVisible(true);
    m_location->setSize(200.0);
    m_location->setDisplayName("");
    m_location->setLabelColorOverridden(true);

    _body->addLocation(std::move(location));
}

/******************************************************************************/

LineOfInterest::~LineOfInterest()
{
    if (m_displayList != 0)
    {
        glDeleteLists(m_displayList, 1);
        m_displayList = 0;
    }
}

/******************************************************************************/
Eigen::Vector3d
LineOfInterest::spherePointLL(const Eigen::Vector3d &latLongAlt) const
{
    double theta = celmath::degToRad(180 - latLongAlt.y());
    double phi   = celmath::degToRad(latLongAlt.x());

    Eigen::Vector3d point(cos(phi) * cos(theta), sin(phi), cos(phi) * sin(theta));

    // Altitude cannot be below a certain value to avoid zfight
    double radius   = static_cast<double>(body->getRadius());
    double altitude = radius * std::max(1.0001, 1.0 + (latLongAlt.z() / radius));

    point *= altitude;

    return point;
}

/******************************************************************************/

void
LineOfInterest::createDisplayList() const
{
    // Create the display list
    m_displayList = glGenLists(1);
    glNewList(m_displayList, GL_COMPILE);

    // Drawing
    draw();

    // End of display list
    glEndList();
}

/******************************************************************************/

void
LineOfInterest::invalidateDisplayList() const
{
    if (m_displayList != 0)
    {
        glDeleteLists(m_displayList, 1);
        m_displayList = 0;
    }
}

/******************************************************************************/

void
LineOfInterest::drawLines() const
{
    glBegin(GL_LINE_STRIP);

    std::vector<LinePoint>::const_iterator it  = m_points.begin();
    std::vector<LinePoint>::const_iterator ite = m_points.end();

    // Line segmentation coefficient (the smaller the coarser)
    double invMaxSegmentLength = 1.0 / 500.0;

    Eigen::Vector3d geoLineStart;
    Eigen::Vector3d cartLineStart;
    bool            isFirst = true;

    for (; it != ite; ++it)
    {
        // LatLongAlt to cartesian conversion
        const Eigen::Vector3d &geoLineEnd  = it->LatLongAlt;
        Eigen::Vector3d        cartLineEnd = spherePointLL(geoLineEnd);

        // We need to split the line into smaller segments in order to follow the
        // Earth curvature
        if (!isFirst)
        {
            // Compute the number of segments we need
            Eigen::Vector3d vector        = cartLineEnd - cartLineStart;
            unsigned int    segmentsCount = ceil(vector.norm() * invMaxSegmentLength);
            Eigen::Vector3d segment
                = (geoLineEnd - geoLineStart) / static_cast<double>(segmentsCount);

            // We need to draw intermediate segments. We start from 1 (not 0) because
            // we need the intermediate segments!
            for (unsigned int i = 1; i < segmentsCount; ++i)
            {
                Eigen::Vector3d geoIntermediate  = static_cast<double>(i) * segment + geoLineStart;
                Eigen::Vector3d cartIntermediate = spherePointLL(geoIntermediate);

                // Push the current point
                //                glVertex(cartIntermediate.cast<float>());
                auto cartIntermediateV = cartIntermediate.cast<float>();
                glVertex3f(cartIntermediateV.x(), cartIntermediateV.y(), cartIntermediateV.z());
            }
        }

        // Push the current point
        auto cartLineEndV = cartLineEnd.cast<float>();
        glVertex3f(cartLineEndV.x(), cartLineEndV.y(), cartLineEndV.z());

        // We store the coordinates of the last point
        cartLineStart = cartLineEnd;
        geoLineStart  = geoLineEnd;

        // The next point wont be the first
        isFirst = false;
    }

    glEnd();
}

/******************************************************************************/

void
LineOfInterest::draw() const
{
    // Disable depth value offsetting
    glDisable(GL_POLYGON_OFFSET_FILL);

    // Manage depth if transparency is enabled
    if (m_opacity == 1.0f)
    {
        // Enable depth buffering
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
    else
    {
        // Alpha has to be enabled so we need to disable depth writing
        // and enable blend.
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
#ifdef USE_HDR
        glBlendFunc(GL_ONE_MINUS_SRC_ALPHA, GL_SRC_ALPHA);
#else
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
#endif
    }

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    // Draw the actual lines
    drawLines();

    // Disables depth
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    // Enables depth buffer offsetting
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(5.0, 15.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

/******************************************************************************/

void
LineOfInterest::render(
    Renderer              *renderer,
    const Eigen::Vector3f &position,
    float                  discSizeInPixels,
    double                 tdb,
    const Matrices        &m) const
{
    // If the line is not visible then nothing must be drawn
    if (!m_visible)
    {
        return;
    }

    // If the line is too small on the screen, skip its rendering
    if (discSizeInPixels <= 1)
    {
        return;
    }

    // Init the display list if necessary
    if (m_displayList == 0)
    {
        createDisplayList();
    }

    // Load the modelview matrix
    Eigen::Quaterniond q = body->getEclipticToBodyFixed(tdb).conjugate();

    glPushMatrix();
    //    glRotate(q.cast<float>());
    auto qV = q.cast<float>();
    glRotatef(qV.w(), qV.x(), qV.y(), qV.z());

    // Line color
    glColor4f(m_color.red(), m_color.green(), m_color.blue(), m_opacity);

    // Line width
    glLineWidth(m_width);

    // Draw-call
    glCallList(m_displayList);

    // Restore the modelview matrix
    glPopMatrix();
}

/******************************************************************************/

float
LineOfInterest::boundingSphereRadius() const
{
    float radius = body->getRadius();
    return 2.0f * (radius + highestAltitude);
}

/******************************************************************************/

void
LineOfInterest::setOpacity(float value)
{
    m_opacity = value;

    // Force the displaylist to be rebuilt in order to take into account this new value
    invalidateDisplayList();
}

/******************************************************************************/

void
LineOfInterest::setColor(const Color &color)
{
    m_color = color;
    m_location->setLabelColor(color);

    // Force the displaylist to be rebuilt in order to take into account this new value
    invalidateDisplayList();
}

/******************************************************************************/

void
LineOfInterest::setVisible(const bool visible)
{
    m_visible = visible;
    m_location->setVisible(m_visible && m_LabelVisible);
}

/******************************************************************************/

void
LineOfInterest::setLabelVisible(const bool visible)
{
    m_LabelVisible = visible;
    m_location->setVisible(m_visible && m_LabelVisible);
}

/******************************************************************************/

bool
LineOfInterest::getLabelVisible() const
{
    return m_LabelVisible;
}

/******************************************************************************/

std::string
LineOfInterest::label() const
{
    return m_location->getDisplayName();
}

/******************************************************************************/

void
LineOfInterest::setLabel(const std::string &text)
{
    m_location->setDisplayName(std::string());
    m_location->setName(text);
}

/******************************************************************************/

void
LineOfInterest::setWidth(const double width)
{
    m_width = width;
}

/******************************************************************************/

double
LineOfInterest::getWidth() const
{
    return m_width;
}

/******************************************************************************/

void
LineOfInterest::addPoints(const std::vector<LinePoint> &points)
{
    if ((m_points.size() == 0) && (points.size() > 0))
    {
        const LinePoint &first = points.front();
        m_location->setPositionPlanetocentric(
            Eigen::Vector3f((float)first.LatLongAlt.y(), (float)first.LatLongAlt.x(), 0.0f));
        m_location->setVisible(true);
    }

    // Adding the points
    m_points.insert(m_points.end(), points.begin(), points.end());

    // Looking for the highest point
    auto it  = points.begin();
    auto ite = points.end();

    for (; it != ite; ++it)
    {
        // We store the highest altitude of all points in order to compute the
        // bounding sphere radius
        if (it->LatLongAlt.z() > highestAltitude)
            highestAltitude = it->LatLongAlt.z();
    }

    // Force the displaylist to be rebuilt in order to take into account this new value
    invalidateDisplayList();
}
