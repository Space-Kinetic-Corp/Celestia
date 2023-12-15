// sensorgeometry.cpp
//
// Copyright (C) 2010, Celestia Development Team
// Original version by Chris Laurel <claurel@gmail.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#include "sensorgeometry.h"

#include "astro.h"
#include "body.h"
#include "celmath/intersect.h"
#include "celmath/mathlib.h"
#include "celmath/vecgl.h"
#include "rendcontext.h"
#include "texmanager.h"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>

using namespace Eigen;
using namespace std;

unsigned int SensorGeometry::s_sectionCount = 512; // Must be a multiple of 4

SensorGeometry::SensorGeometry() :
    m_observer(NULL), m_target(NULL), m_range(0.0), m_horizontalFov(celmath::degToRad(5.0)),
    m_verticalFov(celmath::degToRad(5.0)), m_minAzimuth(celmath::degToRad(-20.0)),
    m_maxAzimuth(celmath::degToRad(20.0)), m_minElevation(celmath::degToRad(5.0)),
    m_maxElevation(celmath::degToRad(10.0)), m_frustumColor(1.0f, 1.0f, 1.0f),
    m_frustumBaseColor(1.0f, 1.0f, 1.0f), m_frustumOpacity(0.25f), m_gridOpacity(1.0f),
    m_shape(EllipticalShape), m_frustumVisible(true), m_frustumBaseVisible(true),
    m_lastComputedFootprintDate(0.)
{
    m_footprintValues = new Vector3d[s_sectionCount];
    m_profile         = new Vector3d[s_sectionCount];
    m_footprint.setSensor(this);
}

SensorGeometry::~SensorGeometry()
{
    delete m_footprintValues;
    delete m_profile;
}

void
SensorGeometry::setObserver(Body *observer)
{
    m_observer = observer;
}

void
SensorGeometry::setTarget(Body *target)
{
    m_target = target;

    // Create the footprint (TODO tag)
    m_footprint.setTarget(target);
    m_footprint.setTag("SensorFootprint");
}

bool
SensorGeometry::pick(const Eigen::ParametrizedLine<double, 3> &r, double & /* distance */) const
{
    return false;
}

void
SensorGeometry::setFOVs(double horizontalFov, double verticalFov)
{
    m_horizontalFov = horizontalFov;
    m_verticalFov   = verticalFov;
}

// Fonction d�finie dans QUtils, v�rifier si elle a �t� mise � jour !
void
wrapOrderedAngles(
    double  sourceAngleRad,
    double  destinationAngleRad,
    double &minWrappedAngleRad,
    double &maxWrappedAngleRad)
{
    const double EPSILON = 1e-12;

    // Premier angle wrapp� entre 0 et 2PI
    // On garde compare min+epsilon avec z�ro pour garder 0 -> 0
    minWrappedAngleRad = fmod(sourceAngleRad, 2.0 * celestia::numbers::pi);
    if (minWrappedAngleRad + EPSILON < 0.0)
    {
        minWrappedAngleRad += 2.0 * celestia::numbers::pi;
    }

    // Second angle wrapp� entre 0 et 2PI
    // On garde compare max-epsilon avec z�ro pour garder 0 -> 2PI
    maxWrappedAngleRad = fmod(destinationAngleRad, 2.0 * celestia::numbers::pi);
    if (maxWrappedAngleRad - EPSILON < 0.0)
    {
        maxWrappedAngleRad += 2.0 * celestia::numbers::pi;
    }

    // Si second angle wrapp� inf�rieur ou �gal � premier angle wrapp�
    // Alors on lui ajoute 2PI pour �tre compris entre 0 et 4PI et donc sup�rieur
    if (maxWrappedAngleRad <= minWrappedAngleRad)
    {
        maxWrappedAngleRad += 2.0 * celestia::numbers::pi;
    }
}

void
SensorGeometry::setAzimuthElevation(
    double minAzimuth,
    double maxAzimuth,
    double minElevation,
    double maxElevation)
{
    // Affectation des membres
    m_minAzimuth   = minAzimuth;
    m_maxAzimuth   = maxAzimuth;
    m_minElevation = minElevation;
    m_maxElevation = maxElevation;

    // Limitation des angles m_minAzimuth dans [0,2PI]
    // et maxAzimuth_ dans [first,4PI]
    wrapOrderedAngles(minAzimuth, maxAzimuth, m_minAzimuth, m_maxAzimuth);

    // MinElevation_ et maxElevation_ d�finis is entre 0 et 90 et min < max
    // Valid� uniquement par le checkvalidity
    // On limite les valeurs
    if (m_minElevation > m_maxElevation)
    {
        std::swap(m_minElevation, m_maxElevation);
    }
    if (m_minElevation < 0 || m_minElevation > (celestia::numbers::pi / 2))
    {
        m_minElevation = std::max(0., m_minElevation);
        m_minElevation = std::min(celestia::numbers::pi / 2, m_minElevation);
    }
    if (m_maxElevation < 0 || m_maxElevation > (celestia::numbers::pi / 2))
    {
        m_maxElevation = std::max(0., m_maxElevation);
        m_maxElevation = std::min(celestia::numbers::pi / 2, m_maxElevation);
    }
}

/** Compute the frustum points, used for the sensor geometry and the sensor footprint
 */
void
SensorGeometry::computeFootprint(double jd)
{
    // Test if we already computed the footprint during this step of rendering.
    // We need to decorelate the computation from the rendering because the
    // sensor trace might be rendered before the sensor frustum.
    if (m_lastComputedFootprintDate == jd)
    {
        return;
    }
    m_lastComputedFootprintDate = jd;

    if (m_target == NULL || m_observer == NULL)
    {
        return;
    }

    UniversalCoord obsPos    = m_observer->getPosition(jd);
    UniversalCoord targetPos = m_target->getPosition(jd);

    Vector3d pos = targetPos.offsetFromKm(obsPos);

    Quaterniond obsOrientation = m_observer->getOrientation(jd).conjugate()
                                 * m_observer->getGeometryOrientation().cast<double>().conjugate();
    Quaterniond         targetOrientation = m_target->getOrientation(jd).conjugate();
    Vector3d            origin            = targetOrientation.conjugate() * -pos;
    celmath::Ellipsoidd targetEllipsoid(m_target->getSemiAxes().cast<double>());

    Matrix3d obsRotation = obsOrientation.toRotationMatrix();

    // Compute the profile of the frustum; the profile is extruded over the range
    // of the sensor (or to the intersection) when rendering.
    if (m_shape == EllipticalShape)
    {
        double horizontalSize = tan(m_horizontalFov);
        double verticalSize   = tan(m_verticalFov);

        for (unsigned int i = 0; i < s_sectionCount; ++i)
        {
            double t     = double(i) / double(s_sectionCount);
            double theta = t * celestia::numbers::pi * 2.0;

            // Note: -sin() is used here to reverse the vertex order so that the _outside_
            // of the frustum is drawn.
            m_profile[i] = obsRotation
                           * Vector3d(cos(theta) * horizontalSize, -sin(theta) * verticalSize, 1.0)
                                 .normalized();
        }
    }
    else if (m_shape == RectangularShape)
    {
        double horizontalSize = tan(m_horizontalFov);
        double verticalSize   = tan(m_verticalFov);

        for (unsigned int i = 0; i < s_sectionCount; ++i)
        {
            double t     = double(i) / double(s_sectionCount);
            double theta = t * celestia::numbers::pi * 2.0;

            double u = double((i + s_sectionCount / 8) % (s_sectionCount / 4))
                       / double(s_sectionCount / 4);
            double phi = (u - 0.5) * celestia::numbers::pi / 2;

            // Note: -sin() is used here to reverse the vertex order so that the _outside_
            // of the frustum is drawn.
            double l = 1.0 / cos(phi);
            m_profile[i]
                = obsRotation
                  * Vector3d(cos(theta) * horizontalSize * l, -sin(theta) * verticalSize * l, 1.0)
                        .normalized();
        }
    }
    else // AzimuthElevationShape
    {
        for (unsigned int i = 0; i < s_sectionCount; ++i)
        {
            m_profile[i] = obsRotation * computeSectionRay(i);
        }
    }

    // Polygon for footprint
    Polygon *cartPoly = NULL;

    if (m_footprint.isActive())
    {
        cartPoly = new Polygon(Color(0.5f, 0.f, 0.f), 0.5);
        cartPoly->reserve(s_sectionCount);
    }

    // Compute the 'footprint' of the sensor by finding the intersection of all rays with
    // the target body. The rendering will not be correct unless the sensor frustum
    for (unsigned int i = 0; i < s_sectionCount; ++i)
    {
        Vector3d direction     = m_profile[i];
        Vector3d testDirection = targetOrientation.conjugate() * direction;

        // Draw the sensor frustum out to either the range or the point of
        // intersection with the target body--whichever is closer.
        double distance = 0.0;
        if (testIntersection(
                ParametrizedLine<double, 3>(origin, testDirection),
                targetEllipsoid,
                distance))
        {
            // Footprint position in Universal coordinates
            UniversalCoord footPos(obsPos.offsetKm(direction * distance));

            // Target -> Footprint in Universal coordinates
            Vector3d footVect = targetPos.offsetFromKm(footPos);

            // Target -> Footprint in Target  coordinates
            if (m_footprint.isActive())
            {
                cartPoly->push_back(
                    targetOrientation.conjugate() * -footVect * m_footprint.getSizeFactor());
            }

            // Adjust distance for sensor volume and contour
            distance = std::min(distance * 0.9999, m_range);
        }
        else
        {
            distance = m_range;
        }

        m_footprintValues[i] = distance * direction;
    }

    // Set footprint polygon (CW or CCW order is important for rendering)
    if (m_footprint.isActive())
    {
        m_footprint.addPolygon(cartPoly, jd);
    }
}

// -------------------------------------------------------------------------- //
// computeSectionRay
// -------------------------------------------------------------------------- //

Vector3d
SensorGeometry::computeSectionRay(double currentSection)
{
    Vector3d result;

    // t dans l'intervalle [ 0 ; 1 ]
    double t = currentSection / s_sectionCount;

    // D�coupage de l'intervalle en 4 et calcul de la section dans un intervalle
    // [ 0 ; 1 ] (correspond au ... * 4). Dans certains cas on veut une interpolation
    // dans un sens d�croissant (correspond au 1 - ...)
    if (t >= 0.75)
    {
        result = computeLineSectionRay(1 - (t - 0.75) * 4, m_minAzimuth);
    }
    else if (t >= 0.5)
    {
        result = computeArcSectionRay(1 - (t - 0.5) * 4, m_maxElevation);
    }
    else if (t >= 0.25)
    {
        result = computeLineSectionRay((t - 0.25) * 4, m_maxAzimuth);
    }
    else
    {
        result = computeArcSectionRay(t * 4, m_minElevation);
    }

    return result;
}

// -------------------------------------------------------------------------- //
// computeArcSectionRay
// -------------------------------------------------------------------------- //

Vector3d
SensorGeometry::computeArcSectionRay(double t, double elevation)
{
    // Azimut minimal corrig� de PI (on veut partir du haut)
    const double correctedMinAzimuth = m_minAzimuth;

    // El�vation (angle compl�mentaire PI/2)
    const double supplementaryElevation = (celestia::numbers::pi / 2) - elevation;

    // Distance totale de l'arc
    const double totalAzimuth = m_maxAzimuth - m_minAzimuth;

    // Calcul de l'angle d'azimut interpol�
    const double thetaAzimuth = totalAzimuth * t + correctedMinAzimuth;

    // Calcul de la direction du lancer de rayon autour de +X
    Vector3d rotVect = Vector3d(
        tan(supplementaryElevation) * cos(thetaAzimuth),
        tan(supplementaryElevation) * -sin(thetaAzimuth),
        1.0);

    // Retour du vecteur normalis�
    return rotVect.normalized();
}

// -------------------------------------------------------------------------- //
// computeLineSectionRay
// -------------------------------------------------------------------------- //

Vector3d
SensorGeometry::computeLineSectionRay(double t, double azimuth)
{
    // Azimut corrig� de PI (on veut partir du haut)
    const double correctedAzimuth = azimuth;

    // Distance totale de l'arc
    const double totalElevation = m_maxElevation - m_minElevation;

    // Calcul de l'angle d'�l�vation interpol� (angle compl�mentaire PI/2)
    const double thetaSupplementaryElevation
        = (celestia::numbers::pi / 2) - (totalElevation * t + m_minElevation);

    // Calcul de la direction du lancer de rayon autour de +X
    Vector3d rotVect = Vector3d(
        tan(thetaSupplementaryElevation) * cos(correctedAzimuth),
        tan(thetaSupplementaryElevation) * -sin(correctedAzimuth),
        1.0);

    // Retour du vecteur normalis�
    return rotVect.normalized();
}

/** Render the sensor geometry.
 */
void
SensorGeometry::render(RenderContext &rc, double tsec)
{
    if (m_target == NULL || m_observer == NULL)
    {
        return;
    }

    double jd = celestia::astro::secsToDays(tsec) + celestia::astro::J2000;

    computeFootprint(jd);

    Quaterniond obsOrientation = m_observer->getOrientation(jd).conjugate()
                                 * m_observer->getGeometryOrientation().cast<double>().conjugate();

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_CULL_FACE);

    glPushMatrix();

    // 'Undo' the rotation of the parent body. We are assuming that the observer is
    // the body to which the sensor geometry is attached.
//    glRotate(obsOrientation.conjugate());
    auto obsConj = obsOrientation.cast<float>().conjugate();
    glRotatef(obsConj.w(), obsConj.x(), obsConj.y(), obsConj.z());

    // Set to true if alternate sides of the frustum should be drawn with different
    // opacities.
    bool alternateShading = (m_shape == RectangularShape) || (m_shape == AzimuthElevationShape);

    if (alternateShading)
    {
        glShadeModel(GL_FLAT);
    }

    // Draw the frustum
    if (m_frustumVisible)
    {
        unsigned int sectionsPerRectSide = s_sectionCount / 4;
        // Offset pour l'ombrage des c�t�s.
        // Rectangle:sectionsPerRectSide / 2 - 1
        // AzimuthElevation:0
        // Elliptique:pas d'ombre
        unsigned int sectionsPerRectSideOffset
            = (m_shape == RectangularShape ? sectionsPerRectSide / 2 - 1 : 0);

        glColor4f(
            m_frustumColor.red(),
            m_frustumColor.green(),
            m_frustumColor.blue(),
            m_frustumOpacity);
        glBegin(GL_TRIANGLE_FAN);
        glVertex3d(0.0, 0.0, 0.0);
        for (unsigned int i = 0; i <= s_sectionCount; ++i)
        {
            if (alternateShading)
            {
                // Use different opacities for adjacent faces of rectangular frusta; this
                // makes the geometry easier to understand visually.
                float alpha = m_frustumOpacity;
                if (((i + sectionsPerRectSideOffset) / sectionsPerRectSide) % 2 == 1)
                {
                    alpha *= 0.5f;
                }
                glColor4f(
                    m_frustumColor.red(),
                    m_frustumColor.green(),
                    m_frustumColor.blue(),
                    alpha);
            }
            glVertex3dv(m_footprintValues[i % s_sectionCount].data());
        }
        glEnd();
    }

    if (alternateShading)
    {
        glShadeModel(GL_SMOOTH);
    }

    glEnable(GL_LINE_SMOOTH);

    // Draw the footprint outline
    if (m_frustumBaseVisible)
    {
        glColor4f(
            m_frustumBaseColor.red(),
            m_frustumBaseColor.green(),
            m_frustumBaseColor.blue(),
            m_gridOpacity);
        glLineWidth(2.0f);
        glBegin(GL_LINE_LOOP);
        for (unsigned int i = 0; i < s_sectionCount; ++i)
        {
            glVertex3dv(m_footprintValues[i].data());
        }
        glEnd();
        glLineWidth(1.0f);
    }

    if (m_frustumVisible)
    {
        const unsigned int sliceCount = 10;

        glColor4f(
            m_frustumColor.red(),
            m_frustumColor.green(),
            m_frustumColor.blue(),
            m_frustumOpacity);
        for (unsigned int slice = 1; slice < sliceCount; ++slice)
        {
            // Linear arrangement of slices
            // double t = double(slice) / double(sliceCount);

            // Exponential arrangement looks better
            double t = pow(2.0, -double(slice));

            glBegin(GL_LINE_LOOP);
            for (unsigned int i = 0; i < s_sectionCount; ++i)
            {
                Vector3d v = m_footprintValues[i] * t;
                glVertex3dv(v.data());
            }
            glEnd();
        }

        // NOTE: section count should be evenly divisible by 8
        glBegin(GL_LINES);
        if (m_shape == EllipticalShape)
        {
            unsigned int rayCount = 8;
            unsigned int step     = s_sectionCount / rayCount;
            for (unsigned int i = 0; i < s_sectionCount; i += step)
            {
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3dv(m_footprintValues[i].data());
            }
        }
        else
        {
            unsigned int step = s_sectionCount / 4;
            for (unsigned int i = s_sectionCount / 8; i < s_sectionCount; i += step)
            {
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3dv(m_footprintValues[i].data());
            }
        }
        glEnd();
    }

    glPopMatrix();

    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
}

bool
SensorGeometry::isOpaque() const
{
    return false;
}

bool
SensorGeometry::isNormalized() const
{
    return false;
}

void
SensorGeometry::setPartVisible(const std::string &partName, bool visible)
{
    if (partName == "Frustum")
    {
        m_frustumVisible = visible;
    }
    else if (partName == "FrustumBase")
    {
        m_frustumBaseVisible = visible;
    }
    else if (partName == "FrustumTrace")
    {
        m_footprint.setVisible(visible);
    }
}

bool
SensorGeometry::isPartVisible(const std::string &partName) const
{
    if (partName == "Frustum")
    {
        return m_frustumVisible;
    }
    else if (partName == "FrustumBase")
    {
        return m_frustumBaseVisible;
    }
    else
    {
        return false;
    }
}
