// sensorgeometry.h
//
// Copyright (C) 2010, Celestia Development Team
// Original version by Chris Laurel <claurel@gmail.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#ifndef _CELENGINE_SENSOR_GEOMETRY_H_
#define _CELENGINE_SENSOR_GEOMETRY_H_

#include "geometry.h"

#include <celengine/footprint.h>
#include <celutil/color.h>
#include <celutil/resmanager.h>
#include <celmath/ray.h>

class Body;

class SensorGeometry : public Geometry
{
public:
    SensorGeometry();
    ~SensorGeometry();

    enum SensorShape
    {
        EllipticalShape,
        RectangularShape,
        AzimuthElevationShape,
    };

    bool pick(const Eigen::ParametrizedLine<double, 3> &r, double &distance) const override;

    void computeFootprint(double jd);

    //! Render the model in the current OpenGL context
    virtual void render(RenderContext &, double t = 0.0);

    virtual bool isOpaque() const;
    virtual bool isNormalized() const;

    virtual bool isMultidraw() const
    {
        return true;
    }

    virtual void setPartVisible(const std::string &partName, bool visible);
    virtual bool isPartVisible(const std::string &partName) const;

    Body *observer() const
    {
        return m_observer;
    }

    void setObserver(Body *observer);

    Body *target() const
    {
        return m_target;
    }

    void setTarget(Body *target);

    double range() const
    {
        return m_range;
    }

    void setRange(double range)
    {
        m_range = range;
    }

    SensorShape shape() const
    {
        return m_shape;
    }

    void setShape(SensorShape shape)
    {
        m_shape = shape;
    }

    Color frustumColor() const
    {
        return m_frustumColor;
    }

    void setFrustumColor(const Color &color)
    {
        m_frustumColor = color;
    }

    Color frustumBaseColor() const
    {
        return m_frustumBaseColor;
    }

    void setFrustumBaseColor(const Color &color)
    {
        m_frustumBaseColor = color;
    }

    float frustumOpacity() const
    {
        return m_frustumOpacity;
    }

    void setFrustumOpacity(float opacity)
    {
        m_frustumOpacity = opacity;
    }

    float gridOpacity() const
    {
        return m_gridOpacity;
    }

    void setGridOpacity(float opacity)
    {
        m_gridOpacity = opacity;
    }

    void setFOVs(double horizontalFov, double verticalFov);
    void setAzimuthElevation(
        double minAzimuth,
        double maxAzimuth,
        double minElevation,
        double maxElevation);

    float getTraceDuration() const
    {
        return m_footprint.getTraceDuration();
    }

    void setTraceDuration(float traceDuration)
    {
        m_footprint.setTraceDuration(traceDuration);
    }

    std::string getTraceColorFile() const
    {
        return m_footprint.getTraceColorFile();
    }

    void setTraceColorFile(std::string colorFileName)
    {
        m_footprint.setTraceColorFile(colorFileName);
    }

    Color getTraceColor() const
    {
        return m_footprint.getTraceColor();
    }

    void setTraceColor(const Color &color)
    {
        m_footprint.setTraceColor(color);
    }

    void setTraceOpacity(float opacity)
    {
        m_footprint.setTraceOpacity(opacity);
    }

    float getTraceOpacity() const
    {
        return m_footprint.getTraceOpacity();
    }

protected:
    //! Calcul d'un rayon sur une section
    Eigen::Vector3d computeSectionRay(double currentSection);

    //! Calcul d'un rayon sur une section d'arc
    Eigen::Vector3d computeArcSectionRay(double currentSection, double elevation);

    //! Calcul d'un rayon sur une section d'arc
    Eigen::Vector3d computeLineSectionRay(double t, double azimuth);

private:
    Body  *m_observer;
    Body  *m_target;
    double m_range;
    double m_horizontalFov;
    double m_verticalFov;
    double m_minAzimuth;
    double m_maxAzimuth;
    double m_minElevation;
    double m_maxElevation;
    Color  m_frustumColor;
    Color  m_frustumBaseColor;
    float  m_frustumOpacity;
    float  m_gridOpacity;

    SensorShape m_shape;
    bool        m_frustumVisible;
    bool        m_frustumBaseVisible;

    Footprint m_footprint;
    double    m_lastComputedFootprintDate;

    Eigen::Vector3d *m_footprintValues;
    Eigen::Vector3d *m_profile;

public:
    static unsigned int s_sectionCount;
};

#endif // !_CELENGINE_SENSOR_GEOMETRY_H_
