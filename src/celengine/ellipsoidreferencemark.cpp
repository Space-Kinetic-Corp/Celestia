// axisarrow.cpp
//
// Copyright (C) 2007-2009, Celestia Development Team
// Original version by Chris Laurel <claurel@gmail.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#include "ellipsoidreferencemark.h"

#include "body.h"
#include "timelinephase.h"

#include <celmath/vecgl.h>
#include <fstream>
#include <iostream>

EllipsoidReferenceMark::EllipsoidReferenceMark(const Body &_body) :
    m_body(_body), m_color(1.0f, 0.f, 0.f, 0.5f),
    m_axisSource(EllipsoidReferenceMark::AxisSource::FIXED),
    m_fixedSemiAxis(Eigen::Vector3d::Ones()), m_lastSample(0), m_radius(_body.getBoundingRadius()),
    m_scale(1.0f), m_lastTime(-1.0e30) //,m_quadratic(gluNewQuadric())
{
    setTag("ellipsoid");

    // Create smooth normals (one normal for each vertex)
    //    gluQuadricNormals(m_quadratic, GLU_SMOOTH);
}

EllipsoidReferenceMark::~EllipsoidReferenceMark()
{
    //    gluDeleteQuadric(m_quadratic);
}

void
EllipsoidReferenceMark::setColor(Color _color)
{
    m_color = _color;
}

void
EllipsoidReferenceMark::setScale(float _scale)
{
    m_scale = _scale;
}

void
EllipsoidReferenceMark::setSemiPrincipalAxis(const Vec3d &axis)
{
    m_fixedSemiAxis.x() = axis.x();
    m_fixedSemiAxis.y() = axis.y();
    m_fixedSemiAxis.z() = axis.z();
    m_axisSource        = EllipsoidReferenceMark::AxisSource::FIXED;
}

void
EllipsoidReferenceMark::loadSemiPrincipalFromFile(const std::string &filename)
{
    loadSampledData(filename);
}

void
EllipsoidReferenceMark::render(
    Renderer * /* renderer */,
    const Eigen::Vector3f & /* position */,
    float /* discSize */,
    double          tdb,
    const Matrices &m /* frustum */) const
{
    const Eigen::Vector3d semiAxis = getSemiPrincipalAxis(tdb);
    if (semiAxis.isZero())
    {
        // Skip rendering null ellipsoids
        return;
    }

    // Update bounding sphere radius
    if (tdb != m_lastTime)
    {
        m_lastTime = tdb;
        // Radius will always be the largest of the three semi axes
        m_radius = semiAxis.maxCoeff();
    }

    // Get orientation
    Eigen::Quaterniond q = getOrientation(tdb);

    if (m_color.alpha() == 1.0f)
    {
        // Enable depth buffering
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
    else
    {
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    // Double side
    glDisable(GL_CULL_FACE);

    // Scale the sphere to match semi principal axis
    glPushMatrix();
    auto qF = q.cast<float>();
    glRotatef(qF.w(), qF.x(), qF.y(), qF.z());
    glScalef(semiAxis.x() * m_scale, semiAxis.y() * m_scale, semiAxis.z() * m_scale);

    // Draw the filled shape
    //    gluQuadricDrawStyle(m_quadratic, GLU_FILL);
    //    glColor4f(m_color.red(), m_color.green(), m_color.blue(), m_color.alpha());
    //    gluSphere(m_quadratic, 1.f, 32, 32);

    // Double side end
    glEnable(GL_CULL_FACE);

    // Line alpha
    Color darker(
        m_color.red() + m_color.red() * 0.5f,
        m_color.green() + m_color.green() * 0.5f,
        m_color.blue() + m_color.blue() * 0.5f,
        m_color.alpha());

    // Lower limit
    float lineAlpha = m_color.alpha();

    if (lineAlpha == 1.0f)
    {
        // Enable depth buffering
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
    else
    {
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    // Draw the wired shape
//    glLineWidth(0.2);
//    gluQuadricDrawStyle(m_quadratic, GLU_LINE);
//    glColor4f(darker.red(), darker.green(), darker.blue(), lineAlpha);
//    gluSphere(m_quadratic, 1.f, 32, 32);

    // Restore context
    glPopMatrix();
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
}

float
EllipsoidReferenceMark::boundingSphereRadius() const
{
    return static_cast<float>(m_radius) * m_scale * 1000;
}

Eigen::Quaterniond
EllipsoidReferenceMark::getOrientation(double tdb) const
{
    return m_body.getEclipticToFrame(tdb).conjugate();
}

Eigen::Vector3d
EllipsoidReferenceMark::getSemiPrincipalAxis(double tdb) const
{
    if (m_axisSource == EllipsoidReferenceMark::AxisSource::FILE)
    {
        return computeSample(tdb);
    }

    return m_fixedSemiAxis;
}

void
EllipsoidReferenceMark::loadSampledData(const std::string &filename)
{
    // If a file exists, read it
    if (filename != "")
    {
        std::ifstream inStream(filename.c_str());
        if (!inStream.good())
            return;

        // SkipComments(inStream))

        double lastSampleTime = -std::numeric_limits<double>::infinity();
        while (inStream.good())
        {
            double tdb, x, y, z;
            inStream >> tdb;
            inStream >> x;
            inStream >> y;
            inStream >> z;

            if (inStream.good())
            {
                // Skip samples with duplicate times; such trajectories are invalid, but
                // are unfortunately used in some existing add-ons.
                if (tdb != lastSampleTime)
                {
                    Sample<double> samp;
                    samp.x = x;
                    samp.y = y;
                    samp.z = z;
                    samp.t = tdb;
                    m_samples.push_back(samp);

                    lastSampleTime = tdb;
                }
            }
        }

        inStream.close();

        m_axisSource = EllipsoidReferenceMark::AxisSource::FILE;
    }
}

Eigen::Vector3d
EllipsoidReferenceMark::computeSample(double jd) const
{
    Eigen::Vector3d retValue;
    if (m_samples.size() == 0)
    {
        retValue = Eigen::Vector3d::Zero();
    }
    else if (m_samples.size() == 1)
    {
        retValue = Eigen::Vector3d(m_samples[0].x, m_samples[0].y, m_samples[0].z);
    }
    else
    {
        Sample<double> samp;
        samp.t = jd;
        int n  = m_lastSample;

        if (n < 1 || n >= (int)m_samples.size() || jd < m_samples[n - 1].t || jd > m_samples[n].t)
        {
            auto iter = lower_bound(m_samples.begin(), m_samples.end(), samp);
            if (iter == m_samples.end())
                n = m_samples.size();
            else
                n = iter - m_samples.begin();

            m_lastSample = n;
        }

        if (n == 0)
        {
            retValue = Eigen::Vector3d(m_samples[n].x, m_samples[n].y, m_samples[n].z);
        }
        else if (n < (int)m_samples.size())
        {
            // Linear interpolation
            Sample<double> s0 = m_samples[n - 1];
            Sample<double> s1 = m_samples[n];

            double t = (jd - s0.t) / (s1.t - s0.t);
            retValue = Eigen::Vector3d(
                celmath::lerp(t, (double)s0.x, (double)s1.x),
                celmath::lerp(t, (double)s0.y, (double)s1.y),
                celmath::lerp(t, (double)s0.z, (double)s1.z));
        }
        else
        {
            retValue = Eigen::Vector3d(m_samples[n - 1].x, m_samples[n - 1].y, m_samples[n - 1].z);
        }
    }

    return retValue;
}
