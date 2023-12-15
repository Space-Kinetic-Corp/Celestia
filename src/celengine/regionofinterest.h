// RegionOfInterest.h
//
// RegionOfInterest reference mark for ellipsoidal bodies.
//
// Copyright (C) 2008, the Celestia Development Team
// Initial version by Chris Laurel, claurel@gmail.com
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#ifndef _CELENGINE_REGIONOFINTEREST_H_
#define _CELENGINE_REGIONOFINTEREST_H_

#include <celengine/polygon.h>
#include <celengine/referencemark.h>
#include <celengine/selection.h>
#include <celutil/color.h>

class Body;

/*! RegionOfInterest is a reference mark that displays polygons
 *  on the surface of a body.
 */
class RegionOfInterest : public ReferenceMark
{
    typedef std::vector<Polygon *> PolygonList;

public:
    RegionOfInterest(Body *body);
    ~RegionOfInterest();

    void addPolygon(Polygon *polygon);

    void render(
        Renderer              *renderer,
        const Eigen::Vector3f &pos,
        float                  discSizeInPixels,
        double                 tdb,
        const Matrices& m) const;
    float boundingSphereRadius() const;
    // The RegionOfInterest must be rendered right after its body
    // Overloading isOpaque is not a good solution
    bool isSurfaceMark() const
    {
        return true;
    }

    bool isActive() const
    {
        return true;
    }

    void setVisible(bool visible);

    // Label visibility
    void setLabelVisible(bool visible);
    bool getLabelVisible();

    // Label color
    void  setLabelColor(const Color &color);
    Color getLabelColor() const;

    // Label text
    std::string label() const;
    void   setLabel(const std::string &text);

    void setTarget(Body *body)
    {
        m_body = body;
    }

    Color getTraceColor() const
    {
        return m_traceColor;
    }

    void setTraceColor(const Color &color);

    void setTraceOpacity(float opacity);

    float getTraceOpacity() const
    {
        return m_traceOpacity;
    }

    float getSizeFactor() const
    {
        return 10;
    }

    void setLabelOrigin(float longitude, float latitude, float alt);

protected:
    bool            m_visible;
    Body           *m_body;
    const Selection m_target;
    PolygonList     m_polygonList;

    float m_traceOpacity;
    Color m_traceColor;

    bool      m_LabelVisible;
    Location *m_location;
};

#endif // _CELENGINE_REGIONOFINTEREST_H_
