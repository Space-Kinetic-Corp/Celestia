// ellipsoidreferencemark.cpp
//
// Copyright (C) 2007-2009, Celestia Development Team
// Original version by Chris Laurel <claurel@gmail.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#include "torusreferencemark.h"

#include "body.h"
#include "timelinephase.h"
#include "render.h"

#include <celmath/vecgl.h>
#include <fstream>
#include <iostream>

TorusReferenceMark::TorusReferenceMark(const Body &_body) :
    m_body(_body), m_color(1.0f, 0.f, 0.f, 0.5f),
    m_paramSource(TorusReferenceMark::ParamSource::FIXED), m_fixedMajorRadius(1.),
    m_fixedMinorRadius(1.), m_lastSample(0), m_radius(_body.getBoundingRadius()), m_scale(1.0f),
    m_lastTime(-1.0e30)
{
    setTag("torus");
}

TorusReferenceMark::~TorusReferenceMark()
{
}

void
TorusReferenceMark::setColor(Color _color)
{
    m_color = _color;
}

void
TorusReferenceMark::setScale(float _scale)
{
    m_scale = _scale;
}

void
TorusReferenceMark::setRadii(double majorRadius, double minorRadius)
{
    m_fixedMajorRadius = majorRadius;
    m_fixedMinorRadius = minorRadius;
    m_paramSource      = TorusReferenceMark::ParamSource::FIXED;
}

void
TorusReferenceMark::loadSemiPrincipalFromFile(const std::string &filename)
{
    loadSampledData(filename);
}

void
drawTorus(double r = 0.07, double c = 0.15, int rSeg = 16, int cSeg = 8, bool wired = false)
{
    glFrontFace(GL_CW);

    // glBindTexture(GL_TEXTURE_2D, texture);
    // glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    // const double PI = 3.1415926535897932384626433832795;
    const double TAU = 2 * celestia::numbers::pi;

    for (int i = 0; i < rSeg; i++)
    {
        if (wired)
        {
            glBegin(GL_LINE_STRIP);
        }
        else
        {
            glBegin(GL_QUAD_STRIP);
        }
        for (int j = 0; j <= cSeg; j++)
        {
            for (int k = 0; k <= 1; k++)
            {
                double s = (i + k) % rSeg + 0.5;
                double t = j % (cSeg + 1);

                double x = (c + r * cos(s * TAU / rSeg)) * cos(t * TAU / cSeg);
                double y = (c + r * cos(s * TAU / rSeg)) * sin(t * TAU / cSeg);
                double z = r * sin(s * TAU / rSeg);

                double u = (i + k) / (float)rSeg;
                double v = t / (float)cSeg;

                /// glTexCoord2d(u, v);
                glNormal3f(2 * x, 2 * y, 2 * z);
                glVertex3d(2 * x, 2 * y, 2 * z);
            }
        }
        glEnd();
    }

    glFrontFace(GL_CCW);
}

void
TorusReferenceMark::render(
    Renderer * /* renderer */,
    const Eigen::Vector3f & /* position */,
    float /* discSize */,
    double tdb,
    const Matrices& m /* frustum */) const
{
    const Eigen::Vector2d radii = getRadii(tdb);
    if (radii.x() == 0 || radii.y() == 0)
    {
        // Skip rendering null visualizers
        return;
    }

    // Update bounding sphere radius
    if (tdb != m_lastTime)
    {
        m_lastTime = tdb;
        // Radius will always be the largest of the three semi axes
        m_radius = radii.maxCoeff();
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
    auto qV = q.cast<float>();
    glRotatef(qV.w(), qV.x(), qV.y(), qV.z());

    // Color
    glColor4f(m_color.red(), m_color.green(), m_color.blue(), m_color.alpha());

    // Draw the filled shape
    drawTorus(radii.x() * m_scale, radii.y() * m_scale, 42, 32, false);

    // Double side end
    glEnable(GL_CULL_FACE);

    // Line alpha
    const Color darker(
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
    glLineWidth(0.2);
    glColor4f(darker.red(), darker.green(), darker.blue(), lineAlpha);
    drawTorus(radii.x() * m_scale, radii.y() * m_scale, 42, 32, true);

    // Restore context
    glPopMatrix();
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
}

float
TorusReferenceMark::boundingSphereRadius() const
{
    return static_cast<float>(m_radius) * m_scale * 1000;
}

Eigen::Quaterniond
TorusReferenceMark::getOrientation(double tdb) const
{
    return m_body.getEclipticToFrame(tdb).conjugate();
}

Eigen::Vector2d
TorusReferenceMark::getRadii(double tdb) const
{
    if (m_paramSource == TorusReferenceMark::ParamSource::FILE)
    {
        return computeSample(tdb);
    }

    return Eigen::Vector2d(m_fixedMajorRadius, m_fixedMinorRadius);
    ;
}

void
TorusReferenceMark::loadSampledData(const std::string &filename)
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
            double tdb, x, y;
            inStream >> tdb;
            inStream >> x;
            inStream >> y;

            if (inStream.good())
            {
                // Skip samples with duplicate times; such trajectories are invalid, but
                // are unfortunately used in some existing add-ons.
                if (tdb != lastSampleTime)
                {
                    TorusSample<double> samp;
                    samp.x = x;
                    samp.y = y;
                    samp.t = tdb;
                    m_samples.push_back(samp);

                    lastSampleTime = tdb;
                }
            }
        }

        inStream.close();

        m_paramSource = TorusReferenceMark::ParamSource::FILE;
    }
}

Eigen::Vector2d
TorusReferenceMark::computeSample(double jd) const
{
    Eigen::Vector2d retValue;
    if (m_samples.size() == 0)
    {
        retValue = Eigen::Vector2d::Zero();
    }
    else if (m_samples.size() == 1)
    {
        retValue = Eigen::Vector2d(m_samples[0].x, m_samples[0].y);
    }
    else
    {
        TorusSample<double> samp;
        samp.t = jd;
        int n  = m_lastSample;

        if (n < 1 || n >= (int)m_samples.size() || jd < m_samples[n - 1].t || jd > m_samples[n].t)
        {
            std::vector<TorusSample<double>>::const_iterator iter
                = lower_bound(m_samples.begin(), m_samples.end(), samp);
            if (iter == m_samples.end())
                n = m_samples.size();
            else
                n = iter - m_samples.begin();

            m_lastSample = n;
        }

        if (n == 0)
        {
            retValue = Eigen::Vector2d(m_samples[n].x, m_samples[n].y);
        }
        else if (n < (int)m_samples.size())
        {
            // Linear interpolation
            TorusSample<double> s0 = m_samples[n - 1];
            TorusSample<double> s1 = m_samples[n];

            double t = (jd - s0.t) / (s1.t - s0.t);
            retValue = Eigen::Vector2d(
                celmath::lerp(t, (double)s0.x, (double)s1.x),
                celmath::lerp(t, (double)s0.y, (double)s1.y));
        }
        else
        {
            retValue = Eigen::Vector2d(m_samples[n - 1].x, m_samples[n - 1].y);
        }
    }

    return retValue;
}
