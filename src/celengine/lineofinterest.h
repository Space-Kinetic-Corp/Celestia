// lineofinterest.h
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

#ifndef _CELENGINE_LINEOFINTEREST_H
#define _CELENGINE_LINEOFINTEREST_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <celengine/referencemark.h>
//#include <celengine/vecgl.h>
#include <celmath/frustum.h>
#include <celutil/color.h>
#include <vector>

class Body;
class CurvePlot;
class Location;

/*! LinePoint stores the information concerning one single point of a Line.
 */

class LinePoint
{
public:
    // Geographic coordinates of the point
    Eigen::Vector3d LatLongAlt;
};

/*! Line is a reference mark that draws a line with CurvePlot.
 *  Straight lines on projection are splitted into multiple straight segments
 *  along the body curvature.
 */

class LineOfInterest : public ReferenceMark
{
public:
    explicit LineOfInterest(Body *_body);
    virtual ~LineOfInterest();

    // Implementation of ReferenceMark
    virtual void render(
        Renderer              *renderer,
        const Eigen::Vector3f &position,
        float                  discSizeInPixels,
        double                 tdb,
        const Matrices& m) const;

    // Implementation of ReferenceMark
    virtual float boundingSphereRadius() const;

    // Implementation of ReferenceMark
    virtual bool isSurfaceMark() const
    {
        return true;
    }

    // Push multiple new points at the end of the line
    void addPoints(const std::vector<LinePoint> &points);

    // Sets the global line opacity
    void setOpacity(float value);

    // Sets the line color
    void setColor(const Color &color);

    // Sets the visibility of the line
    void setVisible(const bool visible);

    // Sets the visibility of the label
    void setLabelVisible(const bool visible);
    // Gets the visibility of the label
    bool getLabelVisible() const;

    // Gets the label
    std::string label() const;
    // Sets the label
    void setLabel(const std::string &text);

    // Sets the line width
    void setWidth(const double width);
    // Gets the line width
    double getWidth() const;

protected:
    // Computes the cartesian coordinates from a lat,long one
    Eigen::Vector3d spherePointLL(const Eigen::Vector3d &latLongAlt) const;

    // Actual code that draws the lines
    void drawLines() const;

    // Draws evverything
    void draw() const;

    // Create the OpenGL display list
    void createDisplayList() const;

    // Invalidate the display list so that it is rebuilt
    void invalidateDisplayList() const;

private:
    // Body whom this line is attached to
    const Body *body;

    // Line opacity
    float m_opacity;

    // Altitude of the highest point
    double highestAltitude;

    // Line color
    Color m_color;

    // Line visibility
    bool m_visible;

    // Line width
    double m_width;

    // Points
    std::vector<LinePoint> m_points;

    // Display list
    mutable uint32_t m_displayList;

    // Label visbility
    bool m_LabelVisible;

    // Location
    Location *m_location;
};

#endif // _CELENGINE_LINEOFINTEREST_H
