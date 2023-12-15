// Footprint.h
//
// Footprint reference mark for ellipsoidal bodies.
//
// Copyright (C) 2008, the Celestia Development Team
// Initial version by Chris Laurel, claurel@gmail.com
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#ifndef _CELENGINE_FOOTPRINT_H_
#define _CELENGINE_FOOTPRINT_H_

#include <celengine/polygon.h>
#include <celengine/referencemark.h>
#include <celengine/selection.h>
#include <celutil/color.h>
#include <celmath/frustum.h>
#include <map>

class Body;
class SensorGeometry;

/*! Footprint is a reference mark that displays polygons
 *  on the surface of a body.
 */
class Footprint : public ReferenceMark
{
    typedef std::map<double,Polygon*> TimedItemMap ;

public:
    enum TargetType
    {
        LongLatPolygon_t     = 0,
        SensorIntersection_t = 1
    };

    Footprint();
    ~Footprint();

    void setSensor(SensorGeometry *sensor){ m_sensorGeometry = sensor; }

    void addPolygon(Polygon *polygon, double jd ) ;

    void render(Renderer* renderer,
                 const Eigen::Vector3f& pos,
                 float discSizeInPixels,
                 double tdb,
                 const Matrices&) const override;
    float boundingSphereRadius() const;
    // The footprint must be rendered right after its body
    // Overloading isOpaque is not a good solution
    bool isSurfaceMark() const { return true; }

    bool isActive() const { return (m_traceDuration >=0); }

    void setVisible( bool visible ) { m_visible = visible; }

    void setTarget( Body *body )
    {
        m_body = body ;
    }

    float getTraceDuration() const
    {
        return m_traceDuration;
    }

    void setTraceDuration(float traceDuration);

    std::string getTraceColorFile() const
    {
        return m_traceColorFileName;
    }

    void setTraceColorFile(std::string colorFileName);

    Color getTraceColor() const
    {
        return m_traceColor;
    }

    void setTraceColor(const Color& color)
    {
        m_traceColor = color;
    }

    void setTraceOpacity(float opacity)
    {
        m_traceOpacity = opacity;
    }

    float getTraceOpacity() const
    {
        return m_traceOpacity;
    }

    float getSizeFactor() const { return 10; }

protected :
    void simplifyTrace( const TimedItemMap::iterator &from,
                       const TimedItemMap::iterator &to,
                       double resolution ) ;

protected:
    SensorGeometry           *m_sensorGeometry;
    bool                      m_visible;
    Body                     *m_body;
    TargetType                m_targetType;
    const Selection           m_target;
    TimedItemMap              m_polygonMap;

    float m_traceDuration;
    std::string m_traceColorFileName;
    float m_traceOpacity;
    Color m_traceColor;
    std::map< double, Color > m_colorMap ;

public:

    static double s_sensorHighRes ;
};

#endif // _CELENGINE_FOOTPRINT_H_

