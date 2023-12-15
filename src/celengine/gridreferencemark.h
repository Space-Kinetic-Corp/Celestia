// gridreferencemark.h
//
// Copyright (C) 2007-2009, Celestia Development Team
// Original version by Chris Laurel <claurel@gmail.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#ifndef _CELENGINE_GRIDREFERENCEMARK_H_
#define _CELENGINE_GRIDREFERENCEMARK_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <celengine/referencemark.h>
#include <celengine/selection.h>
#include <celutil/color.h>
#include <celmath/vecmath.h>

class Body;

struct GridSample
{
    double t;
    double xMin, xMax, yMin, yMax;
};

struct CellSample
{
    double t;
    double xSpacing, ySpacing;
};

/**
 * @class GridReferenceMark
 * @brief This class represents a grid-based reference mark in a 3D environment.
 *
 * The grid's dimensions and characteristics can be manipulated and visualized.
 */
class GridReferenceMark : public ReferenceMark
{
public:
    /**
     * @enum Plane_t
     * @brief Enumeration representing the XY, YZ, and XZ planes.
     */
    enum Plane_t
    {
        XY,
        YZ,
        XZ,
        Plane_t_SIZE
    };

    /**
     * @enum Position_t
     * @brief Enumeration representing the the label position on a plane.
     */
    enum class Position_t
    {
        MINIMUM,
        CENTER,
        MAXIMUM
    };

    /**
     * @brief Default constructor for the class.
     */
    GridReferenceMark(const Body &_body);

    /**
     * @brief Default destructor for the class.
     */
    ~GridReferenceMark() override;

    /**
     * @brief Sets the grid size based on the input parameters.
     */
    void setGridSize(double xMin, double xMax, double yMin, double yMax);

    /**
     * @brief Sets the cell size based on the input parameters.
     */
    void setCellSize(double cellMin, double cellMax);

    /**
     * @brief Loads grid size sampled data from a specified file.
     */
    void loadGridSizeSampledData(const std::string &filename);

    /**
     * @brief Loads cell size sampled data from a specified file.
     */
    void loadCellSizeSampledData(const std::string &filename);

    /**
     * @brief Sets the color of the plane.
     */
    void setPlaneColor(Plane_t plane, Color color);

    /**
     * @brief Sets the visibility of the plane.
     */
    void setPlaneVisible(Plane_t plane, bool visible);

    /**
     * @brief Sets the visibility of the label.
     */
    void setLabelVisible(bool visible);

    /**
     * @brief Sets the label position on the specified plane.
     */
    void setLabelPosition(Plane_t plane, Position_t position);

    /**
     * @brief Gets the label position on the specified plane.
     */
    Position_t getLabelPosition(Plane_t plane) const;

    /**
     * @brief Sets the opacity of the plane.
     */
    void setOpacity(float opacity);

    /**
     * @brief Renders the grid reference mark using a specified renderer.
     */
    void render(
        Renderer              *renderer,
        const Eigen::Vector3f &position,
        float                  discSize,
        double                 tdb,
        const Matrices& m) const override;

    /**
     * @brief Returns the radius of the bounding sphere.
     */
    float boundingSphereRadius() const override;

    /**
     * @brief Checks if the grid reference mark is opaque.
     */
    bool isOpaque() const override
    {
        return false;
    }

private:
    /**
     * @Struct @Quadrant_t
     * @brief  Stores parameters for a quadrant
     */
    struct Quadrant_t
    {
        uint32_t m_vbo    = 0;
        uint32_t m_ibo    = 0;
        uint32_t m_length = 0;
        double xSpacing = 0;
        double ySpacing = 0;
        Color  m_color;
        bool   m_visible = true;
    };

    /**
     * @brief Recomputes the geometry based on the current time.
     */
    void recomputeGeometry(double tdb);

    /**
     * @brief Invalidates the geometry on property change.
     */
    void invalidate();

    /**
     * @brief Resets the buffers.
     */
    void resetBuffers() const;

    /**
     * @brief Returns the current orientation.
     */
    Eigen::Quaterniond getOrientation(double tdb) const;

    /**
     * @brief Builds the geometry of the grid reference mark.
     */
    void buildGeometry();

    /**
     * @brief Draws onee quadrant of the grid.
     */
    static void drawQuadrants(uint32_t vbo, uint32_t ibo, uint32_t size, const Color &color);

    /**
     * @brief Draws the quadrants of the grid.
     */
    void drawGrid() const;

    /**
     * @brief Builds a specific quadrant of the grid.
     */
    void buildQuadrant(
        double           xMin,
        double           yMin,
        double           xMax,
        double           yMax,
        double           xSpacing,
        double           ySpacing,
        const Eigen::Matrix4f &rotation,
        Color            color,
        bool             isVisible,
        Quadrant_t      *quadrant);

    /**
     * @brief Retrieves the grid spacing at a specified time.
     */
    void getGridSpacing(double tdb, double *xSpacing, double *ySpacing) const;

    /**
     * @brief Retrieves the grid size at a specified time.
     */
    void getGridSize(double tdb, double *xMin, double *xMax, double *yMin, double *yMax) const;

    /**
     * @brief Get depth vector for text label positionnig.
     */
    Eigen::Vector3f
    getDepthVector(Position_t position, float min, float max, bool isMainPlaneVisible) const;

private:
    /**
     * @enum DataSource
     * @brief Enumeration representing the source of the data (either fixed or from a file).
     */
    enum class DataSource
    {
        FIXED,
        FILE
    };

    const Body    &m_body;     //!< The body object to which the grid reference mark is attached.
    mutable double m_lastTime; //!< The last known time.
    std::vector<Quadrant_t> m_grid; //!< The collection of grid quadrants.

    // Grid and cell size parameters
    double m_gridXMin, m_gridXMax, m_gridYMin, m_gridYMax;
    double m_xSpacing, m_ySpacing;

    std::vector<GridSample> m_gridSamples; //!< The grid samples.
    std::vector<CellSample> m_cellSamples; //!< The cell samples.

    // Source of grid and cell size data
    DataSource m_gridSource, m_cellSource;

    float m_opacity; //!< The opacity of the grid reference mark.

    bool m_drawLabel; //!< Determines whether to draw the label or not.

    std::array<Position_t, Plane_t_SIZE> m_labelPositions; //!< The position of the labels by plane
    std::array<Color, Plane_t_SIZE>      m_planeColors;    //!< The colos of the planes
    std::array<bool, Plane_t_SIZE>       m_planeVisibilities; //!< The visibility of the planes

    bool m_initDone; //!< Flags whether initialization is done.
};

#endif // _CELENGINE_GRIDREFERENCEMARK_H_
