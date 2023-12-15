// SphericalShellReferenceMark.cpp
//
// Copyright (C) 2007-2009, Celestia Development Team
// Original version by Chris Laurel <claurel@gmail.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#include "sphericalshellreferencemark.h"

#include "body.h"
#include "celmath/vecgl.h"
#include "render.h"
#include "timelinephase.h"

#include <celutil/utf8.h>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace
{

//! Ordered enum !
enum Planes_t
{
    mxmy = 0,
    mxpy,
    pxpy,
    pxmy,

    mymz,
    pymz,
    pypz,
    mypz,

    mzmx,
    pzmx,
    pzpx,
    mzpx
};

const uint32_t QUADRANT_COUNT = 12;
} // namespace

SphericalShellReferenceMark::SphericalShellReferenceMark(const Body &_body) :
    m_body(_body), m_lastTime(-1.0e30), m_opacity(0.35f), m_drawLabel(false), m_initDone(false),
    m_latitudeMin(0.), m_latitudeMax(0.), m_longitudeMin(0.), m_longitudeMax(0.), m_altitudeMin(0.),
    m_altitudeMax(0.), m_latitudeStep(0.), m_longitudeStep(0.), m_altitudeStep(0.), m_radius(0.)
{
    setTag("sphericalshell");
}

SphericalShellReferenceMark::~SphericalShellReferenceMark()
{
    for (const Face_t &plane : m_grid)
    {
        glDeleteBuffers(1, &plane.m_vbo);
        glDeleteBuffers(1, &plane.m_ibo);
    }
}

void
SphericalShellReferenceMark::setLabelVisible(bool visible)
{
    m_drawLabel = visible;
}

void
SphericalShellReferenceMark::setOpacity(float opacity)
{
    m_opacity = opacity;
}

void
SphericalShellReferenceMark::setColor(Color color)
{
    m_color = color;
}

void
SphericalShellReferenceMark::render(
    Renderer              *renderer,
    const Eigen::Vector3f &position,
    float                  discSize,
    double                 tdb,
    const Matrices        &m /* frustum */) const
{
    if (!m_initDone)
    {
        //        cerr << "SphericalShellReferenceMark geometry must be called before rendering!";
    }

    // Time paused
    if (tdb != m_lastTime)
    {
        m_lastTime = tdb;
    }

    // Enable depth test but not depth write
    glEnable(GL_DEPTH_TEST);

    // Always consider it's transparent
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    const Eigen::Quaterniond q = getOrientation(tdb);

    // Orientation
    glPushMatrix();
    auto qV = q.cast<float>();
    //    glRotate(q.cast<float>());
    glRotatef(qV.x(), qV.y(), qV.z(), qV.w());

    // Draw shell
    drawShell();

    glDisable(GL_DEPTH_TEST);

    // Restore context
    glPopMatrix();
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    // Draw labels
    if (m_drawLabel)
    {
        for (const Label_t &label : m_labels)
        {
            // Transform local position to universal coordinates
            auto p = q.cast<float>()
                     * Eigen::Vector3f(label.position.x(), label.position.y(), label.position.z());
            renderer
                ->addForegroundAnnotation(nullptr, label.text, Color(1.f, 1.f, 1.f), position + p);
        }
    }
}

void
SphericalShellReferenceMark::drawShell() const
{
    // Draw each face of the shell
    for (const Face_t &face : m_grid)
    {
        drawFace(face.m_vbo, face.m_vboSize, face.m_ibo, face.m_iboSize);
    }
}

void
SphericalShellReferenceMark::drawFace(
    GLuint faceVbo,
    int    faceVboCount,
    GLuint lineIbo,
    int    lineIboCount) const
{
    const float amount = 0.5f;
    Color       lineColor(color::lighten(m_color, amount));

    // Double side
    glDisable(GL_CULL_FACE);

    glBindBuffer(GL_ARRAY_BUFFER, faceVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    // Draw the surfaces with a small offset
    glPolygonOffset(1.f, 1.f);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glColor4f(m_color.red(), m_color.green(), m_color.blue(), m_opacity);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, faceVboCount);
    glPolygonOffset(0, 0);
    glDisable(GL_POLYGON_OFFSET_FILL);

    // Draw lines
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, lineIbo);
    glColor4f(lineColor.red(), lineColor.green(), lineColor.blue(), 1.f);
    glDrawElements(GL_LINE_STRIP, lineIboCount, GL_UNSIGNED_INT, NULL);

    // Unbind buffers
    glDisableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

float
SphericalShellReferenceMark::boundingSphereRadius() const
{
    return m_radius;
}

Eigen::Quaterniond
SphericalShellReferenceMark::getOrientation(double tdb) const
{
    return m_body.getEclipticToBodyFixed(tdb).conjugate()
           * Eigen::AngleAxisd(celestia::numbers::pi / 2, Eigen::Vector3d::UnitX());
}

void
SphericalShellReferenceMark::buildGeometry()
{
    // Spherical to cartesian coordinates
    auto sph2cart = [](double latitude, double longitude, double altitude)
    {
        // Inverse latitude
        latitude *= -1;
        const auto x
            = altitude * cos(celmath::degToRad(latitude)) * cos(celmath::degToRad(longitude));

        const auto y
            = altitude * cos(celmath::degToRad(latitude)) * sin(celmath::degToRad(longitude));

        const auto z = altitude * sin(celmath::degToRad(latitude));
        return Eigen::Vector3f(-x, y, z);
    };

    // Compute radius with one point
    m_radius = sph2cart(m_latitudeMin, m_longitudeMax, m_altitudeMax).norm();

    // Create un face
    auto faceFunc = [this, sph2cart](
                        int    i,
                        int    j,
                        int    k,
                        double latitudeStart,
                        double longitudeStart,
                        double altitudeStart)
    {
        return sph2cart(
            latitudeStart + m_latitudeStep * i,
            longitudeStart + m_longitudeStep * j,
            altitudeStart + m_altitudeStep * k);
    };

    // Wall Vertices
    auto pushWallVectices = [faceFunc, this](
                                int                           i,
                                int                           j,
                                int                           k,
                                int                           maxI,
                                int                           maxJ,
                                int                           maxK,
                                double                        latitudeStart,
                                double                        longitudeStart,
                                double                        altitudeStart,
                                std::vector<Eigen::Vector3f> &vertices)
    {
        int column = 0;
        if (j % 2 == 0)
        {
            column = i;
        }
        else
        {
            column = maxI - i - 1;
        }
        vertices.push_back(faceFunc(column, j, k, latitudeStart, longitudeStart, altitudeStart));
        vertices.push_back(
            faceFunc(column, j + 1, k, latitudeStart, longitudeStart, altitudeStart));

        // Latitude label
        if (j == 0 && k == 0)
        {
            // Remove trailing zeros
            std::ostringstream oss;
            oss << std::setprecision(8) << std::noshowpoint
                << latitudeStart + m_latitudeStep * column;
            std::string str = oss.str() + UTF8_DEGREE_SIGN;

            m_labels.push_back(
                { faceFunc(column, j, k, latitudeStart, longitudeStart, altitudeStart), str });
        }
    };

    // Floor Vertices
    auto pushFloorVectices = [faceFunc, this](
                                 int                           i,
                                 int                           j,
                                 int                           k,
                                 int                           maxI,
                                 int                           maxJ,
                                 int                           maxK,
                                 double                        latitudeStart,
                                 double                        longitudeStart,
                                 double                        altitudeStart,
                                 std::vector<Eigen::Vector3f> &vertices)
    {
        int column = 0;
        if (k % 2 == 0)
        {
            column = j;
        }
        else
        {
            column = maxJ - j - 1;
        }
        vertices.push_back(faceFunc(i, column, k, latitudeStart, longitudeStart, altitudeStart));
        vertices.push_back(
            faceFunc(i, column, k + 1, latitudeStart, longitudeStart, altitudeStart));

        // Longitude label
        if (i == 0 && k == 0)
        {
            // Remove trailing zeros
            std::ostringstream oss;
            oss << std::setprecision(8) << std::noshowpoint
                << longitudeStart + m_longitudeStep * column;
            std::string str = oss.str() + UTF8_DEGREE_SIGN;

            m_labels.push_back(
                { faceFunc(i, column, k, latitudeStart, longitudeStart, altitudeStart), str });
        }
    };

    // Border Vertices
    auto pushBorderVectices = [faceFunc, this](
                                  int                           i,
                                  int                           j,
                                  int                           k,
                                  int                           maxI,
                                  int                           maxJ,
                                  int                           maxK,
                                  double                        latitudeStart,
                                  double                        longitudeStart,
                                  double                        altitudeStart,
                                  std::vector<Eigen::Vector3f> &vertices)
    {
        int column = 0;
        if (k % 2 == 0)
        {
            column = i;
        }
        else
        {
            column = maxI - i - 1;
        }
        vertices.push_back(faceFunc(column, j, k, latitudeStart, longitudeStart, altitudeStart));
        vertices.push_back(
            faceFunc(column, j, k + 1, latitudeStart, longitudeStart, altitudeStart));

        // Altitude label
        if (i == 0 && j == 0)
        {
            // Remove trailing zeros
            std::ostringstream oss;
            oss << std::setprecision(8) << std::noshowpoint << altitudeStart + m_altitudeStep * k;
            std::string str = oss.str() + " Km";

            m_labels.push_back(
                { faceFunc(column, j, k, latitudeStart, longitudeStart, altitudeStart), str });
        }
    };

    // Temporary face
    Face_t face;

    // Bottom floor
    buildSphericalShell(
        m_latitudeMin,
        m_latitudeMax - m_latitudeMin,
        1,
        m_longitudeMin,
        m_longitudeMax - m_longitudeMin,
        m_longitudeIterationCount + 1,
        m_altitudeMin,
        m_altitudeMax - m_altitudeMin,
        m_altitudeIterationCount,

        pushFloorVectices,
        face.m_vbo,
        face.m_vboSize,
        face.m_ibo,
        face.m_iboSize);
    m_grid.push_back(face);

    // Top floor
    buildSphericalShell(
        m_latitudeMax,
        m_latitudeMax - m_latitudeMin,
        1,
        m_longitudeMin,
        m_longitudeMax - m_longitudeMin,
        m_longitudeIterationCount + 1,
        m_altitudeMin,
        m_altitudeMax - m_altitudeMin,
        m_altitudeIterationCount,
        pushFloorVectices,
        face.m_vbo,
        face.m_vboSize,
        face.m_ibo,
        face.m_iboSize);
    m_grid.push_back(face);

    // Front plane
    buildSphericalShell(
        m_latitudeMin,
        m_latitudeMax - m_latitudeMin,
        m_latitudeIterationCount + 1,
        m_longitudeMin,
        m_longitudeMax - m_longitudeMin,
        m_longitudeIterationCount,
        m_altitudeMin,
        m_altitudeMax - m_altitudeMin,
        1,
        pushWallVectices,
        face.m_vbo,
        face.m_vboSize,
        face.m_ibo,
        face.m_iboSize);
    m_grid.push_back(face);

    buildSphericalShell(
        m_latitudeMin,
        m_latitudeMax - m_latitudeMin,
        m_latitudeIterationCount + 1,
        m_longitudeMin,
        m_longitudeMax - m_longitudeMin,
        m_longitudeIterationCount,
        m_altitudeMax,
        m_altitudeMax - m_altitudeMin,
        1,
        pushWallVectices,
        face.m_vbo,
        face.m_vboSize,
        face.m_ibo,
        face.m_iboSize);
    m_grid.push_back(face);

    // Left Border
    buildSphericalShell(
        m_latitudeMin,
        m_latitudeMax - m_latitudeMin,
        m_latitudeIterationCount + 1,
        m_longitudeMin,
        m_longitudeMax - m_longitudeMin,
        1,
        m_altitudeMin,
        m_altitudeMax - m_altitudeMin,
        m_altitudeIterationCount,
        pushBorderVectices,
        face.m_vbo,
        face.m_vboSize,
        face.m_ibo,
        face.m_iboSize);
    m_grid.push_back(face);

    buildSphericalShell(
        m_latitudeMin,
        m_latitudeMax - m_latitudeMin,
        m_latitudeIterationCount + 1,
        m_longitudeMax,
        m_longitudeMax - m_longitudeMin,
        1,
        m_altitudeMin,
        m_altitudeMax - m_altitudeMin,
        m_altitudeIterationCount,
        pushBorderVectices,
        face.m_vbo,
        face.m_vboSize,
        face.m_ibo,
        face.m_iboSize);
    m_grid.push_back(face);

    m_initDone = true;
}

void
SphericalShellReferenceMark::buildSphericalShell(
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
    GLuint &vbo,
    int    &vertexCount,
    GLuint &ibo,
    int    &indexCount)
{
    // Compute triangles
    std::vector<Eigen::Vector3f> vertices;

    // 8-10 ...
    // |\|\|\|
    // 1-3-5-7
    // |\|\|\|
    // 0-2-4-6

    // Iteration lat/long/alt
    for (int k = 0; k < altitudeStepCount; ++k)
    {
        for (int j = 0; j < longitudeStepCount; ++j)
        {
            for (int i = 0; i < latitudeStepCount; ++i)
            {
                pushVertices(
                    i,
                    j,
                    k,
                    latitudeStepCount,
                    longitudeStepCount,
                    altitudeStepCount,
                    latitudeMin,
                    longitudeMin,
                    altitudeMin,
                    vertices);
            }
        }
    }
    vertexCount = vertices.size();

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        vertices.size() * sizeof(Eigen::Vector3f),
        vertices.data(),
        GL_STATIC_DRAW);

    // Compute quad lines
    std::vector<uint32_t> indices;
    for (int i = 0; i <= vertices.size() - 3; i += 2)
    {
        indices.push_back(i);
        indices.push_back(i + 1);
        indices.push_back(i + 3);
        indices.push_back(i + 2);
        indices.push_back(i);
    }
    indexCount = indices.size();

    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        indices.size() * sizeof(unsigned int),
        &indices[0],
        GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
