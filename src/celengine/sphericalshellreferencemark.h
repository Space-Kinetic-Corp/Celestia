// sphericalshellreferencemark.h
//
// Copyright (C) 2007-2009, Celestia Development Team
// Original version by Chris Laurel <claurel@gmail.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#ifndef _CELENGINE_SPHERICALSHELLREFERENCEMARK_H_
#define _CELENGINE_SPHERICALSHELLREFERENCEMARK_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <celengine/referencemark.h>
#include <celengine/selection.h>
#include <celmath/vecmath.h>
#include <celutil/color.h>

class Body;

class SphericalShellReferenceMark : public ReferenceMark
{
public:
    SphericalShellReferenceMark(const Body &_body);
    ~SphericalShellReferenceMark() override;

    void setGridSize(
        double latitudeMin,
        double latitudeMax,
        double cellLatitude,
        double longitudeMin,
        double longitudeMax,
        double cellLongitude,
        double altitudeMin,
        double altitudeMax,
        double cellAltitude)
    {
        // Compute maximums from the iteration count in order to get equal steps
        auto init = [](double  inMin,
                       double  inMax,
                       double  inCell,
                       double &outMin,
                       double &outMax,
                       int    &outIterationCount,
                       double &outStepSize)
        {
            // Guarantee the cell size won't be negative or null, in this case it will be equal to
            // the span size
            if ((inCell <= 0) | (inCell > (inMax - inMin)))
            {
                inCell = inMax - inMin;
            }
            outStepSize = inCell;
            // At least one iteration
            outIterationCount = std::max(1., round((inMax - inMin) / inCell));
            outMin            = inMin;
            outMax            = inMin + outIterationCount * inCell;
        };
        init(
            latitudeMin,
            latitudeMax,
            cellLatitude,
            m_latitudeMin,
            m_latitudeMax,
            m_latitudeIterationCount,
            m_latitudeStep);
        init(
            longitudeMin,
            longitudeMax,
            cellLongitude,
            m_longitudeMin,
            m_longitudeMax,
            m_longitudeIterationCount,
            m_longitudeStep);
        init(
            altitudeMin,
            altitudeMax,
            cellAltitude,
            m_altitudeMin,
            m_altitudeMax,
            m_altitudeIterationCount,
            m_altitudeStep);
    }

    void render(
        Renderer              *renderer,
        const Eigen::Vector3f &position,
        float                  discSize,
        double                 tdb,
        const Matrices        &m) const override;
    float boundingSphereRadius() const override;

    bool isOpaque() const override
    {
        return false;
    }

    //! Must be called before adding to scene after attribute changes
    void buildGeometry();

    //! Set plane label visibility
    void setLabelVisible(bool visible);

    //! Set opacity
    void setOpacity(float opacity);

    //! Set color
    void setColor(Color color);

private:
    struct Face_t
    {
        uint32_t m_vbo     = 0;
        uint32_t m_ibo     = 0;
        int      m_vboSize = 0;
        int      m_iboSize = 0;
    };

    struct Label_t
    {
        Eigen::Vector3f position;
        std::string     text;
    };

    Eigen::Quaterniond getOrientation(double tdb) const;

    void drawShell() const;
    void drawFace(uint32_t faceVbo, int faceVboCount, uint32_t lineIbo, int lineIboCount) const;

    void buildSphericalShell(
        double latitudeMin,
        double latitudeSpan,
        int    latitudeStepCount,
        double longitudeMin,
        double longitudeSpan,
        int    longitudeStepCount,
        double altitudeMin,
        double altitudeSpan,
        int    altitudeStepCount,

        std::function<
            void(int, int, int, int, int, int, double, double, double, std::vector<Eigen::Vector3f> &)>
            pushVertices,

        uint32_t &vbo,
        int    &vertexCount,
        uint32_t &ibo,
        int    &indexCount);

private:
    const Body &m_body;

    mutable double m_lastTime;

    Color m_color;
    float m_opacity;

    bool m_drawLabel;

    bool m_initDone;

    std::vector<Face_t>  m_grid;
    std::vector<Label_t> m_labels;

    double m_latitudeMin;
    double m_latitudeMax;
    double m_latitudeStep;
    int    m_latitudeIterationCount;

    double m_longitudeMin;
    double m_longitudeMax;
    double m_longitudeStep;
    int    m_longitudeIterationCount;

    double m_altitudeMin;
    double m_altitudeMax;
    double m_altitudeStep;
    int    m_altitudeIterationCount;

    double m_radius;
};

#endif // _CELENGINE_SPHERICALSHELLREFERENCEMARK_H_
