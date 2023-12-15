// gridreferencemark.cpp
//
// Copyright (C) 2007-2009, Celestia Development Team
// Original version by Chris Laurel <claurel@gmail.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#include "gridreferencemark.h"

#include "body.h"
#include "render.h"
#include "timelinephase.h"

#include <celmath/mathlib.h>
// #include "vecgl.h"

#include <fstream>
// #include <glm/gtc/type_ptr.hpp>
// #include <glm/gtx/rotate_vector.hpp>
#include <iomanip>
#include <iostream>

constexpr double PI
    = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899;

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

GridReferenceMark::GridReferenceMark(const Body &_body) :
    m_body(_body), m_lastTime(-1.0e30), m_gridXMin(0.), m_gridXMax(0.), m_gridYMin(0.),
    m_gridYMax(0.), m_xSpacing(0.), m_ySpacing(0.), m_gridSource(DataSource::FIXED),
    m_cellSource(DataSource::FIXED), m_opacity(0.35f), m_drawLabel(false), m_initDone(false),
    m_grid(QUADRANT_COUNT, Quadrant_t())
{
    setTag("grid");

    // Initialize all label positions to MAXIMUM
    m_labelPositions.fill(Position_t::MAXIMUM);

    // Initialize plane colors
    const float power          = 0.75f;
    m_planeColors[Plane_t::XY] = Color(power, 0.f, 0.f, m_opacity); // Red color for XY plane
    m_planeColors[Plane_t::YZ] = Color(0.f, power, 0.f, m_opacity); // Green color for YZ plane
    m_planeColors[Plane_t::XZ] = Color(0.f, 0.f, power, m_opacity); // Blue color for XZ plane

    // Initialize all planes visibles
    m_planeVisibilities.fill(true);
}

GridReferenceMark::~GridReferenceMark()
{
    resetBuffers();
}

void
GridReferenceMark::setGridSize(double xMin, double xMax, double yMin, double yMax)
{
    if (xMin > xMax || yMin > yMax)
    {
        //         << _("Invalid grid size parameters\n");
        return;
    }
    m_gridSource = DataSource::FIXED;
    m_gridXMin   = xMin;
    m_gridXMax   = xMax;
    m_gridYMin   = yMin;
    m_gridYMax   = yMax;

    invalidate();
}

void
GridReferenceMark::setCellSize(double cellMin, double cellMax)
{
    if (cellMin <= 0 || cellMax <= 0)
    {
        //        clog << _("Invalid grid size parameters\n");
        return;
    }
    m_cellSource = DataSource::FIXED;
    m_xSpacing   = fmax(0.0000001, cellMin);
    m_ySpacing   = fmax(0.0000001, cellMax);

    invalidate();
}

void
GridReferenceMark::loadGridSizeSampledData(const std::string &filename)
{
    // If a file exists, read it
    if (filename == "")
    {
        return;
    }
    std::ifstream inStream(filename.c_str());
    if (!inStream.good())
        return;

    double lastSampleTime = -std::numeric_limits<double>::infinity();
    while (inStream.good())
    {
        GridSample sample;
        inStream >> sample.t;
        inStream >> sample.xMin;
        inStream >> sample.yMin;
        inStream >> sample.xMax;
        inStream >> sample.yMax;

        if (inStream.good())
        {
            // Skip samples with duplicate times
            if (sample.t != lastSampleTime)
            {
                m_gridSamples.push_back(sample);
                lastSampleTime = sample.t;
            }
        }
    }
    inStream.close();

    // If at least one sample was read, change the data source to FILE and init attributes to the
    // first value
    if (!m_gridSamples.empty())
    {
        m_gridSource = DataSource::FILE;
        m_gridXMin   = m_gridSamples[0].xMin;
        m_gridYMin   = m_gridSamples[0].yMin;
        m_gridXMax   = m_gridSamples[0].xMax;
        m_gridYMax   = m_gridSamples[0].yMax;

        invalidate();
    }
}

void
GridReferenceMark::loadCellSizeSampledData(const std::string &filename)
{
    // If a file exists, read it
    if (filename == "")
    {
        return;
    }
    std::ifstream inStream(filename.c_str());
    if (!inStream.good())
        return;

    double lastSampleTime = -std::numeric_limits<double>::infinity();
    while (inStream.good())
    {
        CellSample sample;
        inStream >> sample.t;
        inStream >> sample.xSpacing;
        inStream >> sample.ySpacing;

        if (inStream.good())
        {
            // Skip samples with duplicate times
            if (sample.t != lastSampleTime)
            {
                m_cellSamples.push_back(sample);
                lastSampleTime = sample.t;
            }
        }
    }
    inStream.close();

    // If at least one sample was read, change the data source to FILE and init attributes to the
    // first value
    if (!m_cellSamples.empty())
    {
        m_cellSource = DataSource::FILE;
        m_xSpacing   = m_cellSamples[0].xSpacing;
        m_ySpacing   = m_cellSamples[0].ySpacing;

        invalidate();
    }
}

void
GridReferenceMark::setPlaneColor(Plane_t plane, Color color)
{
    // Update for new rendering
    Color newColor(color, m_opacity);
    m_planeColors[plane] = newColor;

    // Update for current rendering
    if (plane == Plane_t::XY)
    {
        m_grid[mxmy].m_color = newColor;
        m_grid[mxpy].m_color = newColor;
        m_grid[pxmy].m_color = newColor;
        m_grid[pxpy].m_color = newColor;
    }
    else if (plane == Plane_t::YZ)
    {
        m_grid[mymz].m_color = newColor;
        m_grid[mypz].m_color = newColor;
        m_grid[pymz].m_color = newColor;
        m_grid[pypz].m_color = newColor;
    }
    else if (plane == Plane_t::XZ)
    {
        m_grid[mzmx].m_color = newColor;
        m_grid[mzpx].m_color = newColor;
        m_grid[pzmx].m_color = newColor;
        m_grid[pzpx].m_color = newColor;
    }

    // Doesn't invalidate geometry
}

void
GridReferenceMark::setPlaneVisible(Plane_t plane, bool visible)
{
    // Update for new rendering
    m_planeVisibilities[plane] = visible;

    // Update for current rendering
    if (plane == Plane_t::XY)
    {
        m_grid[mxmy].m_visible = visible;
        m_grid[mxpy].m_visible = visible;
        m_grid[pxmy].m_visible = visible;
        m_grid[pxpy].m_visible = visible;
    }
    else if (plane == Plane_t::YZ)
    {
        m_grid[mymz].m_visible = visible;
        m_grid[mypz].m_visible = visible;
        m_grid[pymz].m_visible = visible;
        m_grid[pypz].m_visible = visible;
    }
    else if (plane == Plane_t::XZ)
    {
        m_grid[mzmx].m_visible = visible;
        m_grid[mzpx].m_visible = visible;
        m_grid[pzmx].m_visible = visible;
        m_grid[pzpx].m_visible = visible;
    }

    // Doesn't invalidate geometry
}

void
GridReferenceMark::setLabelVisible(bool visible)
{
    m_drawLabel = visible;

    // Doesn't invalidate geometry
}

void
GridReferenceMark::setLabelPosition(Plane_t plane, Position_t position)
{
    m_labelPositions[plane] = position;

    // Doesn't invalidate geometry
}

GridReferenceMark::Position_t
GridReferenceMark::getLabelPosition(Plane_t plane) const
{
    return m_labelPositions[plane];
}

void
GridReferenceMark::setOpacity(float opacity)
{
    // Update the opacity and reapply the colors
    m_opacity = opacity;
    setPlaneColor(Plane_t::XY, m_planeColors[Plane_t::XY]);
    setPlaneColor(Plane_t::YZ, m_planeColors[Plane_t::YZ]);
    setPlaneColor(Plane_t::XZ, m_planeColors[Plane_t::XZ]);
}

void
GridReferenceMark::drawQuadrants(GLuint vbo, GLuint ibo, GLuint size, const Color &color)
{
    const GLfloat quadAlpha  = color.alpha();
    const GLfloat lineAlpha  = std::min(quadAlpha + 0.1f, 1.0f);
    const GLfloat quadDarker = 0.75;
    const Color   lineColor(color.red(), color.green(), color.blue(), lineAlpha);
    const Color   quadColor(
        color.red() * quadDarker,
        color.green() * quadDarker,
        color.blue() * quadDarker,
        quadAlpha);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    // Draw the lines
    glColor4fv(lineColor.toVector4().data());
    glDrawArrays(GL_LINES, 0, size);

    // Draw the triangles with a small offset in depth
    glPolygonOffset(1, 0);
    glEnable(GL_POLYGON_OFFSET_FILL);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glColor4fv(quadColor.toVector4().data());
    glDrawElements(GL_TRIANGLES, (GLsizei)QUADRANT_COUNT, GL_UNSIGNED_INT, NULL);

    glPolygonOffset(0, 0);
    glDisable(GL_POLYGON_OFFSET_FILL);

    glDisableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void
GridReferenceMark::drawGrid() const
{
    Eigen::Matrix4d modelViewMatrix;
    glGetDoublev(GL_MODELVIEW_MATRIX, modelViewMatrix.data());
    //    auto       inverse = glm::inverse(modelViewMatrix);
    auto inverse = modelViewMatrix.inverse();

    // THIS MAY BE COLUMN NOT ROW
    auto trans = inverse.row(3);

    auto draw3 = [this](int q1, int q2, int q3)
    {
        if (m_grid[q1].m_visible)
            drawQuadrants(
                m_grid[q1].m_vbo,
                m_grid[q1].m_ibo,
                m_grid[q1].m_length,
                m_grid[q1].m_color);
        if (m_grid[q2].m_visible)
            drawQuadrants(
                m_grid[q2].m_vbo,
                m_grid[q2].m_ibo,
                m_grid[q2].m_length,
                m_grid[q2].m_color);
        if (m_grid[q3].m_visible)
            drawQuadrants(
                m_grid[q3].m_vbo,
                m_grid[q3].m_ibo,
                m_grid[q3].m_length,
                m_grid[q3].m_color);
    };

    auto draw2 = [this](int q1, int q2)
    {
        if (m_grid[q1].m_visible)
            drawQuadrants(
                m_grid[q1].m_vbo,
                m_grid[q1].m_ibo,
                m_grid[q1].m_length,
                m_grid[q1].m_color);
        if (m_grid[q2].m_visible)
            drawQuadrants(
                m_grid[q2].m_vbo,
                m_grid[q2].m_ibo,
                m_grid[q2].m_length,
                m_grid[q2].m_color);
    };

    if (trans.x() > 0)
    {
        if (trans.y() > 0)
        {
            if (trans.z() > 0)
            {
                // XYZ
                draw3(pxpy, pypz, pzpx);

                draw2(pymz, mzpx);
                draw2(mxpy, pzmx);
                draw2(pxmy, mypz);

                draw3(mxmy, mymz, mzmx);
            }
            else
            {
                // XY-Z
                draw3(pxpy, pymz, mzpx);

                draw2(pypz, pzpx);
                draw2(mxpy, mzmx);
                draw2(pxmy, mymz);

                draw3(mxmy, mypz, pzmx);
            }
        }
        else
        {
            if (trans.z() > 0)
            {
                // X-YZ
                draw3(pxmy, mypz, pzpx);

                draw2(mymz, mzpx);
                draw2(mxmy, pzmx);
                draw2(pxpy, pypz);

                draw3(mxpy, pymz, mzmx);
            }
            else
            {
                // X-Y-Z
                draw3(pxmy, mymz, mzpx);

                draw2(mypz, pzpx);
                draw2(mxmy, mzmx);
                draw2(pxpy, pymz);

                draw3(mxpy, pypz, pzmx);
            }
        }
    }
    else
    {
        if (trans.y() > 0)
        {
            if (trans.z() > 0)
            {
                // -XYZ
                draw3(mxpy, pypz, pzmx);

                draw2(pymz, mzmx);
                draw2(pxpy, pzpx);
                draw2(mxmy, mypz);

                draw3(pxmy, mymz, mzpx);
            }
            else
            {
                // -XY-Z
                draw3(mxpy, pymz, mzmx);

                draw2(pypz, pzmx);
                draw2(pxpy, mzpx);
                draw2(mxmy, mymz);

                draw3(pxmy, mypz, pzpx);
            }
        }
        else
        {
            if (trans.z() > 0)
            {
                // -X-YZ
                draw3(mxmy, mypz, pzmx);

                draw2(mymz, mzmx);
                draw2(pxmy, pzpx);
                draw2(mxpy, pypz);

                draw3(pxpy, pymz, mzpx);
            }
            else
            {
                // -X-Y-Z
                draw3(mxmy, mymz, mzmx);

                draw2(mypz, pzmx);
                draw2(pxmy, mzpx);
                draw2(mxpy, pymz);

                draw3(pxpy, pypz, pzpx);
            }
        }
    }
}

namespace
{
/**
 * @brief Renders labels on a 3D plane, adjusting their positions based on the visibility of the
 * planes.
 *
 * @param[in] renderer        Target Renderer for the labels.
 * @param[in] position        Base position for labels.
 * @param[in] rotation        Rotation applied to each label.
 * @param[in] min             Minimum value for label range.
 * @param[in] max             Maximum value for label range.
 * @param[in] spacing         Distance between each label.
 * @param[in] textDepth       Depth placement of labels.
 * @param[in] valueVector     Vector for value placement.
 * @param[in] depthVector     Vector for depth placement.
 * @param[in] color           Color of the labels.
 * @param[in] isMainPlaneVisible      Visibility flag for the main plane.
 * @param[in] isSecondaryPlaneVisible Visibility flag for the secondary plane.
 *
 */
void
drawLabels(
    Renderer                 *renderer,
    const Eigen::Vector3f    &position,
    const Eigen::Quaternionf &rotation,
    double                    min,
    double                    max,
    double                    spacing,
    const Eigen::Vector3f    &valueVector,
    const Eigen::Vector3f    &depthVector,
    const Color              &color,
    bool                      isMainPlaneVisible,
    bool                      isSecondaryPlaneVisible)
{
    if (isMainPlaneVisible || isSecondaryPlaneVisible)
    {
        const int count = (std::abs(min) + std::abs(max)) / spacing + 1;
        for (int i = 0; i < count; ++i)
        {
            double value = min + i * spacing;
            if (value < min || value > max)
            {
                continue;
            }
            // Remove trailing zeros (depends on m_?Spacing for 0 displaying scientific notation)
            if (abs(value) < (spacing / 10.))
                value = 0.;
            std::ostringstream oss;
            oss << std::setprecision(8) << std::noshowpoint << value;
            std::string str = oss.str();

            const Eigen::Vector3f p = rotation * ((valueVector * (float)value) + depthVector);
            renderer->addForegroundAnnotation(NULL, str, color, position + p);
        }
    }
}
} // namespace

void
GridReferenceMark::render(
    Renderer              *renderer,
    const Eigen::Vector3f &position,
    float                  discSize,
    double                 tdb,
    const Matrices        &m /* frustum */) const
{
    const_cast<GridReferenceMark *>(this)->recomputeGeometry(tdb);

    if ((m_gridXMax - m_gridXMin <= 0) || (m_gridYMax - m_gridYMin <= 0) || (m_xSpacing == 0)
        || (m_ySpacing == 0))
    {
        // Skip rendering null grids
        return;
    }

    // Update bounding sphere radius
    if (tdb != m_lastTime)
    {
        m_lastTime = tdb;
    }

    // Enable depth test but not depth write
    glEnable(GL_DEPTH_TEST);

    // Enable stencil to discard drawing planes behind others (depth write doesn't do the trick)
    glEnable(GL_STENCIL_TEST);
    glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
    glStencilFunc(GL_EQUAL, 0, -1);

    // Clear stencil buffer and init value to 0
    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);

    // Always consider it's transparent
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    const Eigen::Quaterniond q = getOrientation(tdb);

    // Rotate to match parent frame
    const Eigen::Quaterniond q2 = q * Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitY())
                                  * Eigen::AngleAxisd(-PI / 2, Eigen::Vector3d::UnitX());

    if (discSize > 100)
    {
        const float epsilon = 0.1f * std::min(m_xSpacing, m_ySpacing);

        const double startY = std::floor(m_gridYMin / m_ySpacing) * m_ySpacing;
        const double endY   = std::ceil(m_gridYMax / m_ySpacing) * m_ySpacing;
        const double startX = std::floor(m_gridXMin / m_xSpacing) * m_xSpacing;
        const double endX   = std::ceil(m_gridXMax / m_xSpacing) * m_xSpacing;

        const GLfloat lineAlpha = std::min(m_opacity + 0.1f, 1.0f);
        const float   amount    = 0.4f;
        const Color   xyColor(color::lighten(m_planeColors[XY], amount), lineAlpha);
        const Color   yzColor(color::lighten(m_planeColors[YZ], amount), lineAlpha);
        const Color   xzColor(color::lighten(m_planeColors[XZ], amount), lineAlpha);

        const Eigen::Vector3f xAxis(1.f, 0.f, 0.f);
        const Eigen::Vector3f yAxis(0.f, 1.f, 0.f);
        const Eigen::Vector3f zAxis(0.f, 0.f, 1.f);

        const Eigen::Quaternionf fixY = q2.cast<float>();

        if (m_drawLabel)
        {
            Eigen::Vector3f depthVector;

            // For the XY plane
            depthVector = yAxis.cwiseProduct(getDepthVector(
                getLabelPosition(Plane_t::XY),
                startY - epsilon,
                endY + epsilon,
                m_planeVisibilities[XY]));
            drawLabels(
                renderer,
                position,
                fixY,
                startX,
                endX,
                m_xSpacing,
                xAxis,
                depthVector,
                xyColor,
                m_planeVisibilities[XY],
                m_planeVisibilities[XZ]);

            // For the YZ plane
            depthVector = zAxis.cwiseProduct(getDepthVector(
                getLabelPosition(Plane_t::YZ),
                startY - epsilon,
                endY + epsilon,
                m_planeVisibilities[YZ]));
            drawLabels(
                renderer,
                position,
                fixY,
                startX,
                endX,
                m_xSpacing,
                yAxis,
                depthVector,
                yzColor,
                m_planeVisibilities[YZ],
                m_planeVisibilities[XY]);

            // For the XZ plane
            depthVector = xAxis.cwiseProduct(getDepthVector(
                getLabelPosition(Plane_t::XZ),
                startY - epsilon,
                endY + epsilon,
                m_planeVisibilities[XZ]));
            drawLabels(
                renderer,
                position,
                fixY,
                startX,
                endX,
                m_xSpacing,
                zAxis,
                depthVector,
                xzColor,
                m_planeVisibilities[XZ],
                m_planeVisibilities[YZ]);
        }
    }

    // Orientation
    glPushMatrix();
    auto qF = q.cast<float>();
    glRotatef(qF.w(), qF.x(), qF.y(), qF.z());

    // Rotate to match parent frame
    glRotatef(180.f, 0.f, 1.f, 0.f);
    glRotatef(-90.f, 1.f, 0.f, 0.f);

    // Draw
    drawGrid();

    glDisable(GL_DEPTH_TEST);

    // Restore context
    glPopMatrix();
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
}

float
GridReferenceMark::boundingSphereRadius() const
{
    Eigen::Vector2d bottomLeft(m_gridXMin, m_gridYMin);
    Eigen::Vector2d topRight(m_gridXMax, m_gridYMax);

    auto s = topRight - bottomLeft;

    return static_cast<float>(s.norm());
}

void
GridReferenceMark::resetBuffers() const
{
    for (int i = 0; i < QUADRANT_COUNT; ++i)
    {
        glDeleteBuffers(1, &m_grid[i].m_vbo);
        glDeleteBuffers(1, &m_grid[i].m_ibo);
    }
}

Eigen::Quaterniond
GridReferenceMark::getOrientation(double tdb) const
{
    return m_body.getEclipticToBodyFixed(tdb).conjugate();
}

void
GridReferenceMark::buildQuadrant(
    double                  xMin,
    double                  yMin,
    double                  xMax,
    double                  yMax,
    double                  xSpacing,
    double                  ySpacing,
    const Eigen::Matrix4f &rotation,
    Color                   color,
    bool                    isVisible,
    Quadrant_t             *quadrant)
{
    // Init vertices count to 0 to prevent drawing empty quadrant
    quadrant->m_length  = 0;
    quadrant->m_visible = isVisible;

    // Store the colors
    quadrant->m_color = color;

    // Detect if this quadrant must be ommited
    if (xMin >= xMax || yMin >= yMax)
    {
        return;
    }

    // Adjust the coordinates as the nearest multiple of spacing
    const float xMinAdjusted = static_cast<float>((int)floor(xMin / xSpacing) * xSpacing);
    const float xMaxAdjusted = static_cast<float>((int)ceil(xMax / xSpacing) * xSpacing);

    const float yMinAdjusted = static_cast<float>((int)floor(yMin / ySpacing) * ySpacing);
    const float yMaxAdjusted = static_cast<float>((int)ceil(yMax / ySpacing) * ySpacing);

    // Grid Width
    const float width  = xMaxAdjusted - xMinAdjusted;
    const float height = yMaxAdjusted - yMinAdjusted;

    const uint32_t m_xGridSubdivision = (unsigned int)floor(width / xSpacing) + 1;
    const uint32_t m_yGridSubdivision = (unsigned int)floor(height / ySpacing) + 1;

    // Compute bottom left coordinate in grid
    double xBottomLeft, yBottomLeft;
    if (xMinAdjusted < 0)
    {
        xBottomLeft = m_xGridSubdivision * -xSpacing;
    }
    else
    {
        xBottomLeft = xMinAdjusted;
    }
    if (yMinAdjusted < 0)
    {
        yBottomLeft = m_yGridSubdivision * -ySpacing;
    }
    else
    {
        yBottomLeft = yMinAdjusted;
    }

    // vertices by columns and rows
    std::vector<Eigen::Vector3f> vertices;

    // Top and bottom
    for (unsigned int i = 0; i <= m_xGridSubdivision; ++i)
    {
        auto x = float(i * xSpacing + xBottomLeft);
        x      = std::max(xMinAdjusted, std::min(xMaxAdjusted, x));
        vertices.emplace_back((rotation * Eigen::Vector4f(x, yMinAdjusted, 0.0f, 1.f)).head<3>());
        vertices.emplace_back((rotation * Eigen::Vector4f(x, yMaxAdjusted, 0.0f, 1.f)).head<3>());
    }

    // Left and right
    for (unsigned int i = 0; i <= m_yGridSubdivision; ++i)
    {
        auto y = float(i * ySpacing + yBottomLeft);
        y      = std::max(yMinAdjusted, std::min(yMaxAdjusted, y));
        vertices.emplace_back((rotation * Eigen::Vector4f(xMinAdjusted, y, 0.0f, 1.f)).head<3>());
        vertices.emplace_back((rotation * Eigen::Vector4f(xMaxAdjusted, y, 0.0f, 1.f)).head<3>());
    }

    // Update vertices count
    quadrant->m_length = vertices.size();

    // 4 triangles because we want to draw both the front and back of the
    // plane.
    std::vector<unsigned int> indices;
    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(m_xGridSubdivision * 2 + 1);

    indices.push_back(0);
    indices.push_back(m_xGridSubdivision * 2 + 1);
    indices.push_back(m_xGridSubdivision * 2);

    indices.push_back(m_xGridSubdivision * 2 + 1);
    indices.push_back(1);
    indices.push_back(0);

    indices.push_back(m_xGridSubdivision * 2);
    indices.push_back(m_xGridSubdivision * 2 + 1);
    indices.push_back(0);

    glGenBuffers(1, &quadrant->m_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, quadrant->m_vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        vertices.size() * sizeof(Eigen::Vector3f),
        vertices[0].data(),
        GL_STATIC_DRAW);

    glGenBuffers(1, &quadrant->m_ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, quadrant->m_ibo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        indices.size() * sizeof(unsigned int),
        &indices[0],
        GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void
GridReferenceMark::buildGeometry()
{
    /*
            (0,8)|        (5,8)
                 |
      (-1,0)     |(0,0)
     --------------------------
            (0,0)|        (5,0)
                 |
      (-1,-2)    |(0,-2)
    */

    // Clear/delete previous buffers
    resetBuffers();

    // Vertices rotator
    const Eigen::Vector3f xAxis(1.f, 0.f, 0.f);
    const Eigen::Vector3f yAxis(0.f, 1.f, 0.f);
    const auto            xyRotation = Eigen::Matrix4f::Identity();

    auto yzAA       = Eigen::AngleAxisf(celmath::degToRad(-90.f), Eigen::Vector3f::UnitY());
    auto xzAA       = Eigen::AngleAxisf(celmath::degToRad(90.f), Eigen::Vector3f::UnitX());
    auto yzRotation = celmath::rotate(yzAA);
    auto xzRotation = celmath::rotate(xzAA);

    //-X-Y
    buildQuadrant(
        m_gridXMin,
        m_gridYMin,
        0,
        0,
        m_xSpacing,
        m_ySpacing,
        xyRotation,
        m_planeColors[Plane_t::XY],
        m_planeVisibilities[Plane_t::XY],
        &m_grid[mxmy]);
    //-X+Y
    buildQuadrant(
        m_gridXMin,
        0,
        0,
        m_gridYMax,
        m_xSpacing,
        m_ySpacing,
        xyRotation,
        m_planeColors[Plane_t::XY],
        m_planeVisibilities[Plane_t::XY],
        &m_grid[mxpy]);
    //+X+Y
    buildQuadrant(
        0,
        0,
        m_gridXMax,
        m_gridYMax,
        m_xSpacing,
        m_ySpacing,
        xyRotation,
        m_planeColors[Plane_t::XY],
        m_planeVisibilities[Plane_t::XY],
        &m_grid[pxpy]);
    //+X-Y
    buildQuadrant(
        0,
        m_gridYMin,
        m_gridXMax,
        0,
        m_xSpacing,
        m_ySpacing,
        xyRotation,
        m_planeColors[Plane_t::XY],
        m_planeVisibilities[Plane_t::XY],
        &m_grid[pxmy]);

    // -Y-Z
    buildQuadrant(
        m_gridYMin,
        m_gridXMin,
        0,
        0,
        m_ySpacing,
        m_xSpacing,
        yzRotation,
        m_planeColors[Plane_t::YZ],
        m_planeVisibilities[Plane_t::YZ],
        &m_grid[mymz]);
    // +Y-Z
    buildQuadrant(
        m_gridYMin,
        0,
        0,
        m_gridXMax,
        m_ySpacing,
        m_xSpacing,
        yzRotation,
        m_planeColors[Plane_t::YZ],
        m_planeVisibilities[Plane_t::YZ],
        &m_grid[pymz]);
    // +Y+Z
    buildQuadrant(
        0,
        0,
        m_gridYMax,
        m_gridXMax,
        m_ySpacing,
        m_xSpacing,
        yzRotation,
        m_planeColors[Plane_t::YZ],
        m_planeVisibilities[Plane_t::YZ],
        &m_grid[pypz]);
    // -Y+Z
    buildQuadrant(
        0,
        m_gridXMin,
        m_gridYMax,
        0,
        m_ySpacing,
        m_xSpacing,
        yzRotation,
        m_planeColors[Plane_t::YZ],
        m_planeVisibilities[Plane_t::YZ],
        &m_grid[mypz]);

    //-Z-X
    buildQuadrant(
        m_gridYMin,
        m_gridXMin,
        0,
        0,
        m_ySpacing,
        m_xSpacing,
        xzRotation,
        m_planeColors[Plane_t::XZ],
        m_planeVisibilities[Plane_t::XZ],
        &m_grid[mzmx]);
    //+Z-X
    buildQuadrant(
        m_gridYMin,
        0,
        0,
        m_gridXMax,
        m_ySpacing,
        m_xSpacing,
        xzRotation,
        m_planeColors[Plane_t::XZ],
        m_planeVisibilities[Plane_t::XZ],
        &m_grid[pzmx]);
    //+Z+X
    buildQuadrant(
        0,
        0,
        m_gridYMax,
        m_gridXMax,
        m_ySpacing,
        m_xSpacing,
        xzRotation,
        m_planeColors[Plane_t::XZ],
        m_planeVisibilities[Plane_t::XZ],
        &m_grid[pzpx]);
    //-Z+X
    buildQuadrant(
        0,
        m_gridXMin,
        m_gridYMax,
        0,
        m_ySpacing,
        m_xSpacing,
        xzRotation,
        m_planeColors[Plane_t::XZ],
        m_planeVisibilities[Plane_t::XZ],
        &m_grid[mzpx]);
}

void
GridReferenceMark::recomputeGeometry(double tdb)
{
    // New values
    double xMin, xMax, yMin, yMax;
    double xSpacing, ySpacing;

    // Get new grid spacing and size
    getGridSpacing(tdb, &xSpacing, &ySpacing);
    getGridSize(tdb, &xMin, &xMax, &yMin, &yMax);

    // Flag for checking if recompute is needed
    bool needRecompute = false;

    // Check if values have changed or first init
    if (m_initDone == false || xMin != m_gridXMin || xMax != m_gridXMax || yMin != m_gridYMin
        || yMax != m_gridYMax || xSpacing != m_xSpacing || ySpacing != m_ySpacing)
    {
        needRecompute = true;
    }

    // If a recompute is needed, update the values
    if (needRecompute)
    {
        m_gridXMin = xMin;
        m_gridXMax = xMax;
        m_gridYMin = yMin;
        m_gridYMax = yMax;
        m_xSpacing = xSpacing;
        m_ySpacing = ySpacing;

        m_initDone = true;

        buildGeometry();
    }
}

void
GridReferenceMark::invalidate()
{
    m_initDone = false;
}

void
GridReferenceMark::getGridSpacing(double tdb, double *xSpacing, double *ySpacing) const
{
    if (m_cellSource == DataSource::FIXED)
    {
        *xSpacing = m_xSpacing;
        *ySpacing = m_ySpacing;
    }
    else if (m_cellSource == DataSource::FILE)
    {
        auto it = std::lower_bound(
            m_cellSamples.begin(),
            m_cellSamples.end(),
            tdb,
            [](const CellSample &sample, const double &val) { return sample.t < val; });

        if (it == m_cellSamples.end())
        {
            --it; // if tdb is beyond last element, point to the last element
        }
        else if (it != m_cellSamples.begin() && it->t > tdb)
        {
            --it; // if tdb is between elements, move to the last element that is <= tdb
        }

        *xSpacing = it->xSpacing;
        *ySpacing = it->ySpacing;
    }
}

void
GridReferenceMark::getGridSize(double tdb, double *xMin, double *xMax, double *yMin, double *yMax)
    const
{
    if (m_gridSource == DataSource::FIXED)
    {
        *xMin = m_gridXMin;
        *xMax = m_gridXMax;
        *yMin = m_gridYMin;
        *yMax = m_gridYMax;
    }
    else if (m_gridSource == DataSource::FILE)
    {
        auto it = std::lower_bound(
            m_gridSamples.begin(),
            m_gridSamples.end(),
            tdb,
            [](const GridSample &sample, const double &val) { return sample.t < val; });

        if (it == m_gridSamples.end())
        {
            --it; // if tdb is beyond last element, point to the last element
        }
        else if (it != m_gridSamples.begin() && it->t > tdb)
        {
            --it; // if tdb is between elements, move to the last element that is <= tdb
        }

        *xMin = it->xMin;
        *xMax = it->xMax;
        *yMin = it->yMin;
        *yMax = it->yMax;
    }
}

Eigen::Vector3f
GridReferenceMark::getDepthVector(
    Position_t position,
    float      min,
    float      max,
    bool       isMainPlaneVisible) const
{
    // Si le plan principal est masqué on projette le texte à l'origine
    if (!isMainPlaneVisible)
    {
        return { 0.f, 0.f, 0.f };
    }
    switch (position)
    {
    case Position_t::MINIMUM:
        return { min, min, min };
    case Position_t::CENTER:
        return { 0.f, 0.f, 0.f };
    case Position_t::MAXIMUM:
        return { max, max, max };
    default:
        throw std::invalid_argument("Invalid position type");
    }
}
