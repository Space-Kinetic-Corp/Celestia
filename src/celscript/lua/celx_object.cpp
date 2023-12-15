// celx_object.cpp
//
// Copyright (C) 2003-2009, the Celestia Development Team
//
// Lua script extensions for Celestia: object
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

#include "celx_object.h"

#include "celx.h"
#include "celx_category.h"
#include "celx_internal.h"

#include <celengine/atmosphere.h>
#include <celengine/axisarrow.h>
#include <celengine/billboardgeometry.h>
#include <celengine/body.h>
#include <celengine/cluster.h>
#include <celengine/ellipsoidreferencemark.h>
#include <celengine/gridreferencemark.h>
#include <celengine/lineofinterest.h>
#include <celengine/location.h>
#include <celengine/meshmanager.h>
#include <celengine/multitexture.h>
#include <celengine/planetgrid.h>
#include <celengine/regionofinterest.h>
#include <celengine/sphericalshellreferencemark.h>
#include <celengine/timelinephase.h>
#include <celengine/torusreferencemark.h>
#include <celengine/umbra.h>
#include <celengine/visibleregion.h>
#include <celestia/celestiacore.h>
#include <celscript/common/scriptmaps.h>
#include <celutil/logger.h>
#include <celutil/stringutils.h>
#include <cstring>
#include <iostream>

using namespace Eigen;
using namespace std;
using celestia::util::GetLogger;

static int object_setgridreferencemarkproperty(lua_State *l);

#ifdef _WIN32
// #ifdef _WIN32
#include <windows.h>
extern HWND  mainWindow;
extern HMENU menuBar;
extern bool  hideMenuBar;
#else
#include <QMenuBar>
#include <QWidget>
#endif

static const char *
bodyTypeName(int cl)
{
    switch (cl)
    {
    case Body::Planet:
        return "planet";
    case Body::DwarfPlanet:
        return "dwarfplanet";
    case Body::Moon:
        return "moon";
    case Body::MinorMoon:
        return "minormoon";
    case Body::Asteroid:
        return "asteroid";
    case Body::Comet:
        return "comet";
    case Body::Spacecraft:
        return "spacecraft";
    case Body::Invisible:
        return "invisible";
    case Body::SurfaceFeature:
        return "surfacefeature";
    case Body::Component:
        return "component";
    case Body::Diffuse:
        return "diffuse";
    }
    return "unknown";
}

static const char *
dsoTypeName(DeepSkyObjectType dsoType)
{
    switch (dsoType)
    {
    case DeepSkyObjectType::Galaxy:
        return "galaxy";
    case DeepSkyObjectType::Globular:
        return "globular";
    case DeepSkyObjectType::Nebula:
        return "nebula";
    case DeepSkyObjectType::OpenCluster:
        return "opencluster";
    default:
        return "unknown";
    }
}

static celestia::MarkerRepresentation::Symbol
parseMarkerSymbol(const string &name)
{
    using namespace celestia;

    if (compareIgnoringCase(name, "diamond") == 0)
        return MarkerRepresentation::Diamond;
    if (compareIgnoringCase(name, "triangle") == 0)
        return MarkerRepresentation::Triangle;
    if (compareIgnoringCase(name, "square") == 0)
        return MarkerRepresentation::Square;
    if (compareIgnoringCase(name, "filledsquare") == 0)
        return MarkerRepresentation::FilledSquare;
    if (compareIgnoringCase(name, "plus") == 0)
        return MarkerRepresentation::Plus;
    if (compareIgnoringCase(name, "x") == 0)
        return MarkerRepresentation::X;
    if (compareIgnoringCase(name, "leftarrow") == 0)
        return MarkerRepresentation::LeftArrow;
    if (compareIgnoringCase(name, "rightarrow") == 0)
        return MarkerRepresentation::RightArrow;
    if (compareIgnoringCase(name, "uparrow") == 0)
        return MarkerRepresentation::UpArrow;
    if (compareIgnoringCase(name, "downarrow") == 0)
        return MarkerRepresentation::DownArrow;
    if (compareIgnoringCase(name, "circle") == 0)
        return MarkerRepresentation::Circle;
    if (compareIgnoringCase(name, "disk") == 0)
        return MarkerRepresentation::Disk;
    else
        return MarkerRepresentation::Diamond;
}

// ==================== Object ====================
// star, planet, or deep-sky object
int
object_new(lua_State *l, const Selection &sel)
{
    CelxLua celx(l);

    Selection *ud = static_cast<Selection *>(lua_newuserdata(l, sizeof(Selection)));
    *ud           = sel;

    celx.setClass(Celx_Object);

    return 1;
}

Selection *
to_object(lua_State *l, int index)
{
    CelxLua celx(l);
    return static_cast<Selection *>(celx.checkUserData(index, Celx_Object));
}

static Selection *
this_object(lua_State *l)
{
    CelxLua celx(l);

    Selection *sel = to_object(l, 1);
    if (sel == nullptr)
    {
        celx.doError("Bad position object!");
    }

    return sel;
}

static int
object_tostring(lua_State *l)
{
    lua_pushstring(l, "[Object]");

    return 1;
}

// Return true if the object is visible, false if not.
static int
object_visible(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected to function object:visible");

    Selection *sel = this_object(l);
    lua_pushboolean(l, sel->isVisible());

    return 1;
}

// Set the object visibility flag.
static int
object_setvisible(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One argument expected to object:setvisible()");

    Selection *sel = this_object(l);
    bool       visible
        = celx.safeGetBoolean(2, AllErrors, "Argument to object:setvisible() must be a boolean");
    if (sel->body() != nullptr)
    {
        sel->body()->setVisible(visible);
    }
    else if (sel->deepsky() != nullptr)
    {
        sel->deepsky()->setVisible(visible);
    }

    return 0;
}

// Set the object label visibility flag.
static int
object_setlabelvisible(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One argument expected to object:setlabelvisible()");

    Selection *sel     = this_object(l);
    bool       visible = celx.safeGetBoolean(
        2,
        AllErrors,
        "Argument to object:setlabelvisible() must be a boolean");
    if (sel->body() != nullptr)
    {
        sel->body()->setLabelVisible(visible);
    }

    return 0;
}

// Set the object hierarchy visibility flag.
static int
object_sethierarchyvisible(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One argument expected to object:sethierarchyvisible()");

    Selection *sel     = this_object(l);
    bool       visible = celx.safeGetBoolean(
        2,
        AllErrors,
        "Argument to object:sethierarchyvisible() must be a boolean");
    if (sel->body() != nullptr)
    {
        sel->body()->setHierarchyVisible(visible);
    }

    return 0;
}

// Check the visibility flag for an object component; returns false if the object doesn't
// have a component with the specified name
static int
object_partvisible(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One argument expected to object:setpartvisible()");

    Selection *sel = this_object(l);
    string     partName
        = celx.safeGetString(2, AllErrors, "Argument of object:setpartvisible() must be a string");

    bool visible = false;

    if (sel->body() != nullptr && sel->body()->getGeometry() != InvalidResource)
    {
        Geometry *geom = GetGeometryManager()->find(sel->body()->getGeometry());
        if (geom)
        {
            visible = geom->isPartVisible(partName);
        }
    }

    lua_pushboolean(l, visible ? 1 : 0);
    return 1;
}

// Set the visibility flag for an object component; has no effect if the object doesn't
// have any defined components.
static int
object_setpartvisible(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(3, 3, "Two argument expected to object:setpartvisible()");

    Selection *sel      = this_object(l);
    string     partName = celx.safeGetString(
        2,
        AllErrors,
        "Argument 1 of object:setpartvisible() must be a string");

    bool visible = celx.safeGetBoolean(
        3,
        AllErrors,
        "Argument 2 of object:setpartvisible() must be a boolean");
    if (sel->body() != nullptr && sel->body()->getGeometry() != InvalidResource)
    {
        Geometry *geom = GetGeometryManager()->find(sel->body()->getGeometry());
        if (geom)
        {
            geom->setPartVisible(partName, visible);
        }
    }

    return 0;
}

static int
object_setorbitcolor(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(4, 4, "Red, green, and blue color values exepected for object:setorbitcolor()");

    Selection *sel = this_object(l);
    float      r   = (float)celx.safeGetNumber(
        2,
        WrongType,
        "Argument 1 to object:setorbitcolor() must be a number",
        0.0);
    float g = (float)celx.safeGetNumber(
        3,
        WrongType,
        "Argument 2 to object:setorbitcolor() must be a number",
        0.0);
    float b = (float)celx.safeGetNumber(
        4,
        WrongType,
        "Argument 3 to object:setorbitcolor() must be a number",
        0.0);
    Color orbitColor(r, g, b);

    if (sel->body() != nullptr)
    {
        sel->body()->setOrbitColor(orbitColor);
    }

    return 0;
}

static int
object_orbitcoloroverridden(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected to object:orbitcoloroverridden");

    bool       isOverridden = false;
    Selection *sel          = this_object(l);
    if (sel->body() != nullptr)
    {
        isOverridden = sel->body()->isOrbitColorOverridden();
    }

    lua_pushboolean(l, isOverridden);

    return 1;
}

static int
object_setorbitcoloroverridden(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One argument expected to object:setorbitcoloroverridden");

    Selection *sel       = this_object(l);
    bool       _override = celx.safeGetBoolean(
        2,
        AllErrors,
        "Argument to object:setorbitcoloroverridden() must be a boolean");

    if (sel->body() != nullptr)
    {
        sel->body()->setOrbitColorOverridden(_override);
    }

    return 0;
}

static int
object_orbitvisibility(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected to object:orbitvisibility");

    Body::VisibilityPolicy visibility = Body::UseClassVisibility;

    Selection *sel = this_object(l);
    if (sel->body() != nullptr)
    {
        visibility = sel->body()->getOrbitVisibility();
    }

    const char *s = "normal";
    if (visibility == Body::AlwaysVisible)
        s = "always";
    else if (visibility == Body::NeverVisible)
        s = "never";

    lua_pushstring(l, s);

    return 1;
}

static int
object_setorbitvisibility(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One argument expected to object:setorbitcoloroverridden");

    if (!lua_isstring(l, 2))
    {
        celx.doError("First argument to object:setorbitvisibility() must be a string");
    }

    Selection *sel = this_object(l);

    string key;
    key = lua_tostring(l, 2);

    auto &OrbitVisibilityMap = celx.appCore(AllErrors)->scriptMaps()->OrbitVisibilityMap;
    if (OrbitVisibilityMap.count(key) == 0)
    {
        GetLogger()->warn("Unknown visibility policy: {}\n", key);
    }
    else
    {
        auto visibility = static_cast<Body::VisibilityPolicy>(OrbitVisibilityMap[key]);

        if (sel->body() != nullptr)
        {
            sel->body()->setOrbitVisibility(visibility);
        }
    }

    return 0;
}

static int
object_addreferencemark(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "Expected one table as argument to object:addreferencemark()");

    if (!lua_istable(l, 2))
    {
        celx.doError("Argument to object:addreferencemark() must be a table");
    }

    Selection *sel  = this_object(l);
    Body      *body = sel->body();

    lua_pushstring(l, "type");
    lua_gettable(l, 2);
    const char *rmtype = celx.safeGetString(3, NoErrors, "");
    lua_settop(l, 2);

    lua_pushstring(l, "size");
    lua_gettable(l, 2);
    float rmsize
        = (float)celx.safeGetNumber(3, NoErrors, "", body->getRadius()) + body->getRadius();
    lua_settop(l, 2);

    lua_pushstring(l, "opacity");
    lua_gettable(l, 2);
    // -1 indicates that the opacity wasn't set and the default value
    // should be used.
    float rmopacity = (float)celx.safeGetNumber(3, NoErrors, "", -1.0f);
    lua_settop(l, 2);

    lua_pushstring(l, "color");
    lua_gettable(l, 2);
    const char *rmcolorstring = celx.safeGetString(3, NoErrors, "");
    Color       rmcolor(0.0f, 1.0f, 0.0f);
    if (rmcolorstring != nullptr)
        Color::parse(rmcolorstring, rmcolor);
    lua_settop(l, 2);

    lua_pushstring(l, "tag");
    lua_gettable(l, 2);
    const char *rmtag = celx.safeGetString(3, NoErrors, "");
    if (rmtag == nullptr)
        rmtag = rmtype;
    lua_settop(l, 2);

    lua_pushstring(l, "target");
    lua_gettable(l, 2);
    Selection *rmtarget = to_object(l, 3);
    lua_settop(l, 2);

    // VTS
    lua_pushstring(l, "scale");
    lua_gettable(l, 2);
    // -1 indicates that the scale wasn't set and the default value
    // should be used.
    float rmscale = (float)celx.safeGetNumber(3, NoErrors, "", -1.0f);
    lua_settop(l, 2);

    if (rmtype != nullptr)
    {
        body->removeReferenceMark(rmtype);

        if (compareIgnoringCase(rmtype, "body axes") == 0)
        {
            auto arrow = std::make_unique<BodyAxisArrows>(*body);
            arrow->setTag(rmtag);
            arrow->setSize(rmsize);
            if (rmopacity >= 0.0f)
                arrow->setOpacity(rmopacity);
            body->addReferenceMark(std::move(arrow));
        }
        else if (compareIgnoringCase(rmtype, "frame axes") == 0)
        {
            auto arrow = std::make_unique<FrameAxisArrows>(*body);
            arrow->setTag(rmtag);
            arrow->setSize(rmsize);
            if (rmopacity >= 0.0f)
                arrow->setOpacity(rmopacity);
            body->addReferenceMark(std::move(arrow));
        }
        else if (compareIgnoringCase(rmtype, "sun direction") == 0)
        {
            auto arrow = std::make_unique<SunDirectionArrow>(*body);
            arrow->setTag(rmtag);
            arrow->setSize(rmsize);
            if (rmcolorstring != nullptr)
                arrow->setColor(rmcolor);
            body->addReferenceMark(std::move(arrow));
        }
        else if (compareIgnoringCase(rmtype, "velocity vector") == 0)
        {
            auto arrow = std::make_unique<VelocityVectorArrow>(*body);
            arrow->setTag(rmtag);
            arrow->setSize(rmsize);
            if (rmcolorstring != nullptr)
                arrow->setColor(rmcolor);
            body->addReferenceMark(std::move(arrow));
        }
        else if (compareIgnoringCase(rmtype, "spin vector") == 0)
        {
            auto arrow = std::make_unique<SpinVectorArrow>(*body);
            arrow->setTag(rmtag);
            arrow->setSize(rmsize);
            if (rmcolorstring != nullptr)
                arrow->setColor(rmcolor);
            body->addReferenceMark(std::move(arrow));
        }
        else if (compareIgnoringCase(rmtype, "body to body direction") == 0 && rmtarget != nullptr)
        {
            auto arrow = std::make_unique<BodyToBodyDirectionArrow>(*body, *rmtarget);
            arrow->setTag(rmtag);
            arrow->setSize(rmsize);
            if (rmcolorstring != nullptr)
                arrow->setColor(rmcolor);
            body->addReferenceMark(std::move(arrow));
        }
        else if (compareIgnoringCase(rmtype, "visible region") == 0 && rmtarget != nullptr)
        {
            auto region = std::make_unique<VisibleRegion>(*body, *rmtarget);
            region->setTag(rmtag);
            if (rmopacity >= 0.0f)
                region->setOpacity(rmopacity);
            if (rmcolorstring != nullptr)
                region->setColor(rmcolor);
            body->addReferenceMark(std::move(region));
        }
        else if (compareIgnoringCase(rmtype, "planetographic grid") == 0)
        {
            body->addReferenceMark(std::make_unique<PlanetographicGrid>(*body));
        }
        // ellipsoid
        else if (compareIgnoringCase(rmtype, "ellipsoid") == 0)
        {
            auto ellipsoid = std::make_unique<EllipsoidReferenceMark>(*body);
            ellipsoid->setTag(rmtag);
            if (rmcolorstring != nullptr)
                ellipsoid->setColor(rmcolor);
            if (rmscale != -1.f)
                ellipsoid->setScale(rmscale);

            lua_pushstring(l, "file");
            lua_gettable(l, 2);
            const char *elFile = celx.safeGetString(3, NoErrors, "");
            lua_settop(l, 2);
            if (elFile != nullptr)
            {
                ellipsoid->loadSemiPrincipalFromFile(string(elFile));
            }
            else
            {
                lua_pushstring(l, "value");
                lua_gettable(l, 2);
                Vec3d *elAxis = celx.toVector(3);
                lua_settop(l, 2);
                if (elAxis != nullptr)
                    ellipsoid->setSemiPrincipalAxis(*elAxis);
            }

            body->addReferenceMark(std::move(ellipsoid));
        }
        else if (compareIgnoringCase(rmtype, "torus") == 0)
        {
            //            TorusReferenceMark *torus = new TorusReferenceMark(*body);
            auto torus = std::make_unique<TorusReferenceMark>(*body);
            torus->setTag(rmtag);
            if (rmcolorstring != nullptr)
                torus->setColor(rmcolor);
            if (rmscale != -1.f)
                torus->setScale(rmscale);

            lua_pushstring(l, "file");
            lua_gettable(l, 2);
            const char *elFile = celx.safeGetString(3, NoErrors, "");
            if (elFile != nullptr)
                torus->loadSemiPrincipalFromFile(string(elFile));
            lua_settop(l, 2);

            lua_pushstring(l, "majorradius");
            lua_gettable(l, 2);
            double rmmajorradius = celx.safeGetNumber(3, NoErrors, "", .0f);
            lua_settop(l, 2);

            lua_pushstring(l, "minorradius");
            lua_gettable(l, 2);
            double rminorradius = celx.safeGetNumber(3, NoErrors, "", .0f);
            lua_settop(l, 2);

            torus->setRadii(rmmajorradius, rminorradius);

            body->addReferenceMark(std::move(torus));
        }
        else if (compareIgnoringCase(rmtype, "sphericalshell") == 0)
        {
            //            SphericalShellReferenceMark *shell = new
            //            SphericalShellReferenceMark(*body);
            auto shell = std::make_unique<SphericalShellReferenceMark>(*body);
            shell->setTag(rmtag);
            if (rmcolorstring != nullptr)
                shell->setColor(rmcolor);

            lua_pushstring(l, "latitudemin");
            lua_gettable(l, 2);
            double latitudeMin = celx.safeGetNumber(3, NoErrors, "", .0f);
            lua_settop(l, 2);

            lua_pushstring(l, "latitudemax");
            lua_gettable(l, 2);
            double latitudeMax = celx.safeGetNumber(3, NoErrors, "", .0f);
            lua_settop(l, 2);

            lua_pushstring(l, "latitudecell");
            lua_gettable(l, 2);
            double cellLatitude = celx.safeGetNumber(3, NoErrors, "", .0f);
            lua_settop(l, 2);

            ///

            lua_pushstring(l, "altitudemin");
            lua_gettable(l, 2);
            double altitudeMin = celx.safeGetNumber(3, NoErrors, "", .0f);
            lua_settop(l, 2);

            lua_pushstring(l, "altitudemax");
            lua_gettable(l, 2);
            double altitudeMax = celx.safeGetNumber(3, NoErrors, "", .0f);
            lua_settop(l, 2);

            lua_pushstring(l, "altitudecell");
            lua_gettable(l, 2);
            double cellAltitude = celx.safeGetNumber(3, NoErrors, "", .0f);
            lua_settop(l, 2);

            ///

            lua_pushstring(l, "longitudemin");
            lua_gettable(l, 2);
            double longitudeMin = celx.safeGetNumber(3, NoErrors, "", .0f);
            lua_settop(l, 2);

            lua_pushstring(l, "longitudemax");
            lua_gettable(l, 2);
            double longitudeMax = celx.safeGetNumber(3, NoErrors, "", .0f);
            lua_settop(l, 2);

            lua_pushstring(l, "longitudecell");
            lua_gettable(l, 2);
            double cellLongitude = celx.safeGetNumber(3, NoErrors, "", .0f);
            lua_settop(l, 2);

            shell->setGridSize(
                latitudeMin,
                latitudeMax,
                cellLatitude,
                longitudeMin,
                longitudeMax,
                cellLongitude,
                altitudeMin,
                altitudeMax,
                cellAltitude);

            // Init
            shell->buildGeometry();
            body->addReferenceMark(std::move(shell));
        }
        else if (compareIgnoringCase(rmtype, "grid") == 0)
        {
            //            GridReferenceMark *grid = new GridReferenceMark(*body);
            auto grid = std::make_unique<GridReferenceMark>(*body);
            grid->setTag(rmtag);

            lua_pushstring(l, "gridfile");
            lua_gettable(l, 2);
            const char *gridfile = celx.safeGetString(3, NoErrors, "");
            lua_settop(l, 2);
            if (gridfile != nullptr)
            {
                grid->loadGridSizeSampledData(string(gridfile));
            }
            else
            {
                lua_pushstring(l, "gridxmin");
                lua_gettable(l, 2);
                double rmgridxmin = celx.safeGetNumber(3, NoErrors, "", .0f);
                lua_settop(l, 2);

                lua_pushstring(l, "gridymin");
                lua_gettable(l, 2);
                double rmgridymin = celx.safeGetNumber(3, NoErrors, "", .0f);
                lua_settop(l, 2);

                lua_pushstring(l, "gridxmax");
                lua_gettable(l, 2);
                double rmgridxmax = celx.safeGetNumber(3, NoErrors, "", .0f);
                lua_settop(l, 2);

                lua_pushstring(l, "gridymax");
                lua_gettable(l, 2);
                double rmgridymax = celx.safeGetNumber(3, NoErrors, "", .0f);
                lua_settop(l, 2);

                grid->setGridSize(rmgridxmin, rmgridxmax, rmgridymin, rmgridymax);
            }

            lua_pushstring(l, "cellfile");
            lua_gettable(l, 2);
            const char *cellfile = celx.safeGetString(3, NoErrors, "");
            lua_settop(l, 2);
            if (cellfile != nullptr)
            {
                grid->loadCellSizeSampledData(string(cellfile));
            }
            else
            {
                lua_pushstring(l, "cellwidth");
                lua_gettable(l, 2);
                double rmcellwidth = celx.safeGetNumber(3, NoErrors, "", .0f);
                lua_settop(l, 2);

                lua_pushstring(l, "cellheight");
                lua_gettable(l, 2);
                double rmcellheight = celx.safeGetNumber(3, NoErrors, "", .0f);
                lua_settop(l, 2);

                grid->setCellSize(rmcellwidth, rmcellheight);
            }
            // Add the reference mark to the body
            body->addReferenceMark(std::move(grid));
            // Take into account the grid properties on the stack after adding it
            object_setgridreferencemarkproperty(l);
        }
        else if (compareIgnoringCase(rmtype, "roi") == 0)
        {
            lua_pushstring(l, "polygon");
            lua_gettable(l, 2);
            string polyStr = celx.safeGetString(
                3,
                AllErrors,
                "Reference mark polygon argument must be a string");
            lua_settop(l, 2);

            lua_pushstring(l, "isclosed");
            lua_gettable(l, 2);
            bool isclosed = (celx.safeGetNumber(3, NoErrors, "", 1) > 0.5);
            lua_settop(l, 2);

            // Build an istream that holds the input string
            std::istringstream iss(polyStr);

            // Iterate over the istream, using >> to grab floats
            // and push_back to store them in the vector
            std::vector<float> values;
            std::copy(
                std::istream_iterator<float>(iss),
                std::istream_iterator<float>(),
                std::back_inserter(values));

            // an roi can be either a polygon on a surface (which is an actual ROI) if it is closed
            // or an loi (a line in space) which it is open.

            if (isclosed)
            {
                // Creating a ROI

                // Test if the number of values is even
                if (values.size() % 2 != 0)
                {
                    clog << "The size of roi polygon coordinates list must be even! (Got "
                         << values.size() << ")\n";
                    return 0;
                }

                // Fill the polygon list
                vector<Point2f> polygon2d;
                for (auto it = values.begin(); it != values.end(); it += 2)
                {
                    polygon2d.emplace_back(*it, *(it + 1));
                }

                if (rmopacity < 0.0f || rmcolorstring == nullptr)
                {
                    clog << "Wrong parameters for ROI";
                    return 0;
                }

                // Create a geopoly
                auto cartPoly = new class Polygon(rmcolor, rmopacity);

                // TODO : use dynamic factor
                const float factor = 10;

                // Convert it to cartesian
                Polygon::polygon2dtoPolygon3d(polygon2d, body->getRadius() * factor, *cartPoly);

                // Set footprint polygon
                //                auto *roi = new RegionOfInterest(body);
                auto roi = std::make_unique<RegionOfInterest>(body);
                roi->addPolygon(cartPoly);
                roi->setTag(rmtag);
                roi->setLabel(rmtag);

                // Compute the 2d polygon centroid
                Point2f centroid;
                for (auto point : polygon2d)
                {
                    centroid.x += point.x;
                    centroid.y += point.y;
                }
                float ratio = 1.0f / (float)polygon2d.size();
                centroid.x *= ratio;
                centroid.y *= ratio;
                roi->setLabelOrigin(centroid.x, centroid.y, 0.0f);
                roi->setLabelColor(rmcolor);

                // Add footprint
                body->addReferenceMark(std::move(roi));
            }
            else
            {
                // Creating a LOI

                // Test if the number of values is odd
                if (values.size() % 3 != 0)
                {
                    clog << "The size of loi points coordinates list must be odd! (Got "
                         << values.size() << ")\n";
                    return 0;
                }

                // Fill the points list
                vector<LinePoint> points;
                for (auto it = values.begin(); it != values.end(); it += 3)
                {
                    LinePoint point;
                    point.LatLongAlt = Vector3d(*(it + 2), *(it + 1), *it);
                    points.push_back(point);
                }

                if (rmopacity < 0.0f || rmcolorstring == NULL)
                {
                    clog << "Wrong parameters for LOI";
                    return 0;
                }

                // Set footprint polygon
                //                LineOfInterest *loi = new LineOfInterest(body);
                auto loi = std::make_unique<LineOfInterest>(body);
                loi->addPoints(points);
                loi->setTag(rmtag);
                loi->setColor(rmcolor);
                loi->setOpacity(rmopacity);
                loi->setLabel(rmtag);

                // Add footprint
                body->addReferenceMark(std::move(loi));
            }
        }
        else if (compareIgnoringCase(rmtype, "umbra cone") == 0)
        {
            auto umbra = dynamic_cast<Umbra *>(body->findReferenceMarkMut(rmtag));

            if (umbra == nullptr)
            {
                auto umbraPtr = std::make_unique<Umbra>(*body);
                umbra->setTag(rmtag);
                body->addReferenceMark(std::move(umbraPtr));
            }

            lua_pushstring(l, "visible");
            lua_gettable(l, 2);
            bool visible
                = celx.safeGetBoolean(3, AllErrors, "Umbra visible argument shall be a boolean");
            lua_settop(l, 2);

            umbra->setUmbraVisible(visible);
            umbra->setUmbraColor(rmcolor);
            umbra->setUmbraMaxExtent(rmsize);
        }
        else if (compareIgnoringCase(rmtype, "penumbra cone") == 0)
        {
            Umbra *umbra = dynamic_cast<Umbra *>(body->findReferenceMarkMut(rmtag));
            if (umbra == nullptr)
            {
                auto umbraPtr = std::make_unique<Umbra>(*body);
                umbra->setTag(rmtag);
                body->addReferenceMark(std::move(umbraPtr));
            }

            lua_pushstring(l, "visible");
            lua_gettable(l, 2);
            bool visible
                = celx.safeGetBoolean(3, AllErrors, "Penumbra visible argument shall be a boolean");
            lua_settop(l, 2);

            umbra->setPenumbraVisible(visible);
            umbra->setPenumbraColor(rmcolor);
            umbra->setPenumbraMaxExtent(rmsize);
        }
    }

    return 0;
}

static int
object_removereferencemark(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1000, "Invalid number of arguments in object:removereferencemark");
    CelestiaCore *appCore = celx.appCore(AllErrors);

    Selection *sel  = this_object(l);
    Body      *body = sel->body();

    int argc = lua_gettop(l);
    for (int i = 2; i <= argc; i++)
    {
        string refMark = celx.safeGetString(
            i,
            AllErrors,
            "Arguments to object:removereferencemark() must be strings");

        if (body->findReferenceMark(refMark))
            appCore->toggleReferenceMark(refMark, *sel);
    }

    return 0;
}

static int
object_hasreferencemark(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "Invalid number of arguments in object:removereferencemark");
    CelestiaCore *appCore = celx.appCore(AllErrors);

    Selection *sel  = this_object(l);
    Body      *body = sel->body();

    int    argc = lua_gettop(l);
    string refMark
        = celx.safeGetString(2, AllErrors, "Argument to object:haseferencemark() must be strings");
    if (body->findReferenceMark(refMark))
    {
        celx.push(true);
    }
    else
    {
        celx.push(false);
    }
    return 1;
}

// Set the visualizer color value.
static int
object_visualizercolor(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(3, 3, "Two arguments expected to object:setvisualizercolor()");

    Selection *sel = this_object(l);
    if (sel->body() != NULL)
    {
        Body *body = sel->body();

        string refMarkTag = celx.safeGetString(
            2,
            AllErrors,
            "Argument 1 to object:setvisualizercolor() must be a string");
        string colorString = celx.safeGetString(
            3,
            AllErrors,
            "Argument 2 to object:setvisualizercolor() must be a string");

        Color color;
        Color::parse(colorString.c_str(), color);

        auto refMark = body->findReferenceMarkMut(refMarkTag);
        if (refMark != nullptr)
        {
            auto ellipsoid = dynamic_cast<EllipsoidReferenceMark *>(refMark);
            if (ellipsoid != nullptr)
            {
                ellipsoid->setColor(color);
            }
            else
            {
                auto torus = dynamic_cast<TorusReferenceMark *>(refMark);
                if (torus != nullptr)
                {
                    torus->setColor(color);
                }
            }
        }
    }
    return 0;
}

// Set the visualizer scale value.
static int
object_visualizerscale(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(3, 3, "Two arguments expected to object:setvisualizerscale()");

    Selection *sel = this_object(l);
    if (sel->body() != nullptr)
    {
        Body *body = sel->body();

        string refMarkTag = celx.safeGetString(
            2,
            AllErrors,
            "Argument 1 to object:setvisualizerscale() must be a string");
        double scale = celx.safeGetNumber(
            3,
            AllErrors,
            "Argument 2 to object:setvisualizerscale() must be a numer");

        auto refMark = body->findReferenceMarkMut(refMarkTag);
        if (refMark != nullptr)
        {
            auto ellipsoid = dynamic_cast<EllipsoidReferenceMark *>(refMark);
            if (ellipsoid != nullptr)
            {
                ellipsoid->setScale(scale);
            }
            else
            {
                auto torus = dynamic_cast<TorusReferenceMark *>(refMark);
                if (torus != nullptr)
                {
                    torus->setScale(scale);
                }
            }
        }
    }
    return 0;
}

// Set the grid property

static int
object_setgridreferencemarkproperty(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "Expected one table as argument to object:setgridreferencemarkproperty()");

    if (!lua_istable(l, 2))
    {
        celx.doError("Argument to object:setgridreferencemarkproperty() must be a table");
    }

    lua_pushstring(l, "tag");
    lua_gettable(l, 2);
    string refMarkTag = celx.safeGetString(
        3,
        AllErrors,
        "Missing 'tag' key in object:setgridreferencemarkproperty()");
    lua_settop(l, 2);

    // Properties list
    const char *properties[]
        = { "GridXyPlaneVisible",  "GridYzPlaneVisible",  "GridXzPlaneVisible", "GridXyPlaneColor",
            "GridYzPlaneColor",    "GridXzPlaneColor",    "GridOpacity",        "GridLabelVisible",
            "GridXyLabelPosition", "GridYzLabelPosition", "GridXzLabelPosition" };

    Selection *sel = this_object(l);
    if (sel->body() != NULL)
    {
        Body *body = sel->body();

        auto refMark = body->findReferenceMarkMut(refMarkTag);
        if (refMark != NULL)
        {
            auto grid = dynamic_cast<GridReferenceMark *>(refMark);
            if (grid != NULL)
            {
                // Loop through each property
                for (auto &propertie : properties)
                {
                    lua_pushstring(l, propertie);
                    lua_gettable(l, 2);

                    std::string propertyStr(propertie); // convert to std::string

                    // Use the appropriate logic for each property type...
                    if (lua_isboolean(l, -1) && propertyStr.find("Visible") != std::string::npos)
                    {
                        bool visible = lua_toboolean(l, -1);
                        if (propertyStr == "GridXyPlaneVisible")
                            grid->setPlaneVisible(GridReferenceMark::Plane_t::XY, visible);
                        else if (propertyStr == "GridYzPlaneVisible")
                            grid->setPlaneVisible(GridReferenceMark::Plane_t::YZ, visible);
                        else if (propertyStr == "GridXzPlaneVisible")
                            grid->setPlaneVisible(GridReferenceMark::Plane_t::XZ, visible);
                        else if (propertyStr == "GridLabelVisible")
                            grid->setLabelVisible(visible);
                    }
                    else if (lua_isnumber(l, -1) && propertyStr == "GridOpacity")
                    {
                        double opacity = lua_tonumber(l, -1);
                        grid->setOpacity(opacity);
                    }
                    else if (lua_isstring(l, -1))
                    {
                        // String property (color or label position)
                        const char *strVal = lua_tostring(l, -1);
                        if (propertyStr.find("Color") != std::string::npos)
                        {
                            Color rmcolor(0.0f, 1.0f, 0.0f);
                            if (strVal != NULL)
                                Color::parse(strVal, rmcolor);

                            if (propertyStr == "GridXyPlaneColor")
                                grid->setPlaneColor(GridReferenceMark::Plane_t::XY, rmcolor);
                            else if (propertyStr == "GridYzPlaneColor")
                                grid->setPlaneColor(GridReferenceMark::Plane_t::YZ, rmcolor);
                            else if (propertyStr == "GridXzPlaneColor")
                                grid->setPlaneColor(GridReferenceMark::Plane_t::XZ, rmcolor);
                        }
                        else if (propertyStr.find("LabelPosition") != std::string::npos)
                        {
                            GridReferenceMark::Position_t position;
                            if (strcmp(strVal, "MINIMUM") == 0)
                                position = GridReferenceMark::Position_t::MINIMUM;
                            else if (strcmp(strVal, "CENTER") == 0)
                                position = GridReferenceMark::Position_t::CENTER;
                            else if (strcmp(strVal, "MAXIMUM") == 0)
                                position = GridReferenceMark::Position_t::MAXIMUM;
                            else
                                celx.doError("LabelPosition value to "
                                             "object:setgridreferencemarkproperty() "
                                             "must be one of MINIMUM, CENTER, MAXIMUM");

                            if (propertyStr == "GridXyLabelPosition")
                                grid->setLabelPosition(GridReferenceMark::Plane_t::XY, position);
                            else if (propertyStr == "GridYzLabelPosition")
                                grid->setLabelPosition(GridReferenceMark::Plane_t::YZ, position);
                            else if (propertyStr == "GridXzLabelPosition")
                                grid->setLabelPosition(GridReferenceMark::Plane_t::XZ, position);
                        }
                    }
                    lua_pop(l, 1);
                }
            }
        }
    }
    return 0;
}

// Set the grid property
static int
object_setgridreferencemarkproperty2(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "Expected one table as argument to object:setgridreferencemarkproperty()");

    if (!lua_istable(l, 2))
    {
        celx.doError("Argument to object:setgridreferencemarkproperty() must be a table");
    }

    lua_pushstring(l, "tag");
    lua_gettable(l, 2);
    string refMarkTag = celx.safeGetString(
        3,
        AllErrors,
        "Missing 'tag' key in object:setgridreferencemarkproperty()");
    lua_settop(l, 2);

    lua_pushstring(l, "property");
    lua_gettable(l, 2);
    string refMarkProp = celx.safeGetString(
        3,
        AllErrors,
        "Missing 'property' key in object:setgridreferencemarkproperty()");
    lua_settop(l, 2);

    Selection *sel = this_object(l);
    if (sel->body() != nullptr)
    {
        Body *body = sel->body();

        auto refMark = body->findReferenceMarkMut(refMarkTag);
        if (refMark != nullptr)
        {
            auto grid = dynamic_cast<GridReferenceMark *>(refMark);
            if (grid != nullptr)
            {
                if (refMarkProp.find("Color") != std::string::npos)
                {
                    lua_pushstring(l, "color");
                    lua_gettable(l, 2);
                    const char *colorString = celx.safeGetString(
                        3,
                        AllErrors,
                        "Invalid 'color' value for "
                        "object:setgridreferencemarkproperty()");
                    Color rmcolor(0.0f, 1.0f, 0.0f);
                    if (colorString != nullptr)
                        Color::parse(colorString, rmcolor);
                    lua_settop(l, 2);

                    if (refMarkProp == "GridXyPlaneColor")
                        grid->setPlaneColor(GridReferenceMark::Plane_t::XY, rmcolor);
                    else if (refMarkProp == "GridYzPlaneColor")
                        grid->setPlaneColor(GridReferenceMark::Plane_t::YZ, rmcolor);
                    else if (refMarkProp == "GridXzPlaneColor")
                        grid->setPlaneColor(GridReferenceMark::Plane_t::XZ, rmcolor);
                }

                else if (refMarkProp.find("Visible") != std::string::npos)
                {
                    lua_pushstring(l, "visible");
                    lua_gettable(l, 2);
                    bool visible = celx.safeGetBoolean(3, NoErrors, "");
                    lua_settop(l, 2);

                    if (refMarkProp == "GridXyPlaneVisible")
                        grid->setPlaneVisible(GridReferenceMark::Plane_t::XY, visible);
                    else if (refMarkProp == "GridYzPlaneVisible")
                        grid->setPlaneVisible(GridReferenceMark::Plane_t::YZ, visible);
                    else if (refMarkProp == "GridXzPlaneVisible")
                        grid->setPlaneVisible(GridReferenceMark::Plane_t::XZ, visible);
                    else if (refMarkProp == "GridLabelVisible")
                        grid->setLabelVisible(visible);
                }

                else if (refMarkProp == "GridOpacity")
                {
                    lua_pushstring(l, "opacity");
                    lua_gettable(l, 2);
                    double opacity = celx.safeGetNumber(3, NoErrors, "");
                    lua_settop(l, 2);

                    grid->setOpacity(opacity);
                }

                else if (refMarkProp.find("LabelPosition") != std::string::npos)
                {
                    lua_pushstring(l, "labelposition");
                    lua_gettable(l, 2);
                    string labelPosition = celx.safeGetString(
                        3,
                        AllErrors,
                        "Invalid 'labelposition' value for "
                        "object:setgridreferencemarkproperty()");
                    lua_settop(l, 2);

                    GridReferenceMark::Position_t position;
                    if (labelPosition == "MINIMUM")
                        position = GridReferenceMark::Position_t::MINIMUM;
                    else if (labelPosition == "CENTER")
                        position = GridReferenceMark::Position_t::CENTER;
                    else if (labelPosition == "MAXIMUM")
                        position = GridReferenceMark::Position_t::MAXIMUM;
                    else
                        celx.doError("LabelPosition value to "
                                     "object:setgridreferencemarkproperty() "
                                     "must be one of MINIMUM, CENTER, MAXIMUM");

                    if (refMarkProp == "GridXyLabelPosition")
                        grid->setLabelPosition(GridReferenceMark::Plane_t::XY, position);
                    else if (refMarkProp == "GridYzLabelPosition")
                        grid->setLabelPosition(GridReferenceMark::Plane_t::YZ, position);
                    else if (refMarkProp == "GridXzLabelPosition")
                        grid->setLabelPosition(GridReferenceMark::Plane_t::XZ, position);
                }
            }
        }
    }
    return 0;
}

// Set the visualizer property
static int
object_setvisualizerreferencemarkproperty(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(
        2,
        2,
        "Expected one table as argument to object:setvisualizerreferencemarkproperty()");

    if (!lua_istable(l, 2))
    {
        celx.doError("Argument to object:setvisualizerreferencemarkproperty() must be a table");
    }

    lua_pushstring(l, "tag");
    lua_gettable(l, 2);
    string refMarkTag = celx.safeGetString(3, NoErrors, "");
    lua_settop(l, 2);

    lua_pushstring(l, "property");
    lua_gettable(l, 2);
    string refMarkProp = celx.safeGetString(3, NoErrors, "");
    lua_settop(l, 2);

    lua_pushstring(l, "color");
    lua_gettable(l, 2);
    const char *colorString = celx.safeGetString(3, NoErrors, "");
    Color       rmcolor(0.0f, 1.0f, 0.0f);
    if (colorString != NULL)
        Color::parse(colorString, rmcolor);
    lua_settop(l, 2);

    lua_pushstring(l, "visible");
    lua_gettable(l, 2);
    bool visible = celx.safeGetBoolean(3, NoErrors, "");
    lua_settop(l, 2);

    lua_pushstring(l, "opacity");
    lua_gettable(l, 2);
    double opacity = celx.safeGetNumber(3, NoErrors, "");
    lua_settop(l, 2);

    Selection *sel = this_object(l);
    if (sel->body() != nullptr)
    {
        Body *body = sel->body();

        auto refMark = body->findReferenceMarkMut(refMarkTag);
        if (refMark != nullptr)
        {
            auto shell = dynamic_cast<SphericalShellReferenceMark *>(refMark);
            if (shell != nullptr)
            {
                if (refMarkProp == "labelvisible")
                    shell->setLabelVisible(visible);
                else if (refMarkProp == "opacity")
                    shell->setOpacity(opacity);
                else if (refMarkProp == "color")
                    shell->setColor(rmcolor);
            }
        }
    }
    return 0;
}

// Set the referencemark visibility flag.
static int
object_setregionofinterestvisible(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(3, 3, "Two arguments expected to object:setregionofinterestvisible()");

    Selection *sel = this_object(l);
    if (sel->body() != NULL)
    {
        Body *body = sel->body();

        string refMarkTag = celx.safeGetString(
            2,
            AllErrors,
            "Argument 1 to object:removereferencemark() must be a string");
        bool visible = celx.safeGetBoolean(
            3,
            AllErrors,
            "Argument 2 to object:setregionofinterestvisible() must be a boolean");

        auto refMark = body->findReferenceMarkMut(refMarkTag);
        if (refMark != nullptr)
        {
            // In case of performance issue with the dynamic_cast, create a common base
            // class for both LOI and ROI with a virtual type() and static_cast
            // depending on this type value.
            auto roi = dynamic_cast<RegionOfInterest *>(refMark);
            if (roi != nullptr)
            {
                roi->setVisible(visible);
            }
            else
            {
                auto loi = dynamic_cast<LineOfInterest *>(refMark);
                if (loi != nullptr)
                {
                    loi->setVisible(visible);
                }
            }
        }
    }
    return 0;
}

// Set the referencemark label visiblity flag
static int
object_setregionofinteresttextvisible(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(3, 3, "Two arguments expected to object:setregionofinterestvisible()");

    Selection *sel = this_object(l);
    if (sel->body() != NULL)
    {
        Body *body = sel->body();

        string refMarkTag = celx.safeGetString(
            2,
            AllErrors,
            "Argument 1 to object:removereferencemark() must be a string");
        bool visible = celx.safeGetBoolean(
            3,
            AllErrors,
            "Argument 2 to object:setregionofinterestvisible() must be a boolean");

        auto refMark = body->findReferenceMarkMut(refMarkTag);
        if (refMark != nullptr)
        {
            // In case of performance issue with the dynamic_cast, create a common base
            // class for both LOI and ROI with a virtual type() and static_cast
            // depending on this type value.
            auto roi = dynamic_cast<RegionOfInterest *>(refMark);
            if (roi != nullptr)
            {
                roi->setLabelVisible(visible);
            }
            else
            {
                auto loi = dynamic_cast<LineOfInterest *>(refMark);
                if (loi != nullptr)
                {
                    loi->setLabelVisible(visible);
                }
            }
        }
    }
    return 0;
}

// Set the referencemark color value.
static int
object_setregionofinterestcolor(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(3, 3, "Two arguments expected to object:setregionofinterestcolor()");

    Selection *sel = this_object(l);
    if (sel->body() != NULL)
    {
        Body *body = sel->body();

        string refMarkTag = celx.safeGetString(
            2,
            AllErrors,
            "Argument 1 to object:setregionofinterestcolor() must be a string");
        string colorString = celx.safeGetString(
            3,
            AllErrors,
            "Argument 2 to object:setregionofinterestcolor() must be a string");

        Color roiColor;
        Color::parse(colorString.c_str(), roiColor);

        auto refMark = body->findReferenceMarkMut(refMarkTag);
        if (refMark != nullptr)
        {
            // In case of performance issue with the dynamic_cast, create a common base
            // class for both LOI and ROI with a virtual type() and static_cast
            // depending on this type value.
            auto roi = dynamic_cast<RegionOfInterest *>(refMark);
            if (roi != nullptr)
            {
                roi->setTraceColor(roiColor);
            }
            else
            {
                auto loi = dynamic_cast<LineOfInterest *>(refMark);
                if (loi != nullptr)
                {
                    loi->setColor(roiColor);
                }
            }
        }
    }
    return 0;
}

// Set the referencemark contour width.
static int
object_setregionofinterestcontourwidth(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(
        3,
        3,
        "Two arguments expected to object:object_setregionofinterestcontourwidth()");

    Selection *sel = this_object(l);
    if (sel->body() != NULL)
    {
        Body *body = sel->body();

        string refMarkTag = celx.safeGetString(
            2,
            AllErrors,
            "Argument 1 to object:object_setregionofinterestcontourwidth() must be a string");
        double width = celx.safeGetNumber(
            3,
            AllErrors,
            "Argument 2 to object:object_setregionofinterestcontourwidth() must be a number");

        auto refMark = body->findReferenceMarkMut(refMarkTag);
        if (refMark != nullptr)
        {
            // No contour width for the ROI (only LOI)
            auto loi = dynamic_cast<LineOfInterest *>(refMark);
            if (loi != nullptr)
            {
                loi->setWidth(width);
            }
        }
    }
    return 0;
}

// Set the referencemark opacity value.
static int
object_setregionofinterestopacity(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(3, 3, "Two arguments expected to object:setregionofinterestopacity()");

    Selection *sel = this_object(l);
    if (sel->body() != NULL)
    {
        Body *body = sel->body();

        string refMarkTag = celx.safeGetString(
            2,
            AllErrors,
            "Argument 1 to object:removereferencemark() must be a string");
        double opacity = celx.safeGetNumber(
            3,
            AllErrors,
            "Argument 2 to object:setregionofinterestopacity() must be a number");

        auto refMark = body->findReferenceMarkMut(refMarkTag);
        if (refMark != nullptr)
        {
            // In case of performance issue with the dynamic_cast, create a common base
            // class for both LOI and ROI with a virtual type() and static_cast
            // depending on this type value.
            auto roi = dynamic_cast<RegionOfInterest *>(refMark);
            if (roi != nullptr)
            {
                roi->setTraceOpacity(opacity);
            }
            else
            {
                auto loi = dynamic_cast<LineOfInterest *>(refMark);
                if (loi != nullptr)
                {
                    loi->setOpacity(opacity);
                }
            }
        }
    }
    return 0;
}

// Set the location visibility
static int
object_setpointofinterestvisible(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(3, 3, "Two arguments expected to object:setpointofinterestvisible()");

    Selection *sel  = this_object(l);
    Body      *body = sel->body();
    if (body != NULL)
    {
        string locationName = celx.safeGetString(
            2,
            AllErrors,
            "Argument 1 to object:setpointofinterestvisible() must be a string");
        bool visible = celx.safeGetBoolean(
            3,
            AllErrors,
            "Argument 2 to object:setpointofinterestvisible() must be a boolean");

        auto locations = body->getLocations();
        for (auto it = locations->begin(); it != locations->end(); ++it)
        {
            if ((*it)->getName() == locationName)
            {
                (*it)->setVisible(visible);
            }
        }
    }
    return 0;
}

// Set the location label  visibility
static int
object_setpointofinteresttextvisible(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(3, 3, "Two arguments expected to object:object_setpointofinteresttextvisible()");

    Selection *sel  = this_object(l);
    Body      *body = sel->body();
    if (body != NULL)
    {
        string locationName = celx.safeGetString(
            2,
            AllErrors,
            "Argument 1 to object:object_setpointofinteresttextvisible() must be a string");
        bool visible = celx.safeGetBoolean(
            3,
            AllErrors,
            "Argument 2 to object:object_setpointofinteresttextvisible() must be a boolean");

        auto locations = body->getLocations();
        for (auto it = locations->begin(); it != locations->end(); ++it)
        {
            if ((*it)->getName() == locationName)
            {
                (*it)->setLabelVisible(visible);
            }
        }
    }
    return 0;
}

// Set the POI (location) color value.
static int
object_setpointofinterestcolor(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(3, 3, "Two arguments expected to object:setregionofinterestcolor()");

    Selection *sel  = this_object(l);
    Body      *body = sel->body();
    if (body != NULL)
    {
        string locationName = celx.safeGetString(
            2,
            AllErrors,
            "Argument 1 to object:setpointofinterestcolor() must be a string");
        string colorString = celx.safeGetString(
            3,
            AllErrors,
            "Argument 2 to object:setpointofinterestcolor() must be a string");

        Color color;
        Color::parse(colorString.c_str(), color);

        auto locations = body->getLocations();
        for (auto it = locations->begin(); it != locations->end(); ++it)
        {
            if ((*it)->getName() == locationName)
            {
                (*it)->setLabelColor(color);
            }
        }
    }
    return 0;
}

static int
object_setmagcoeff(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(3, 3, "Two argument expected to object:setmagcoeff()");

    Selection *sel = this_object(l);
    if (sel->body() != nullptr)
    {
        Body  *body    = sel->body();
        float  iradius = body->getRadius();
        double radius  = celx.safeGetNumber(
            2,
            AllErrors,
            "Argument 1 to object:setmagcoeff() must be a number");
        double coeff = celx.safeGetNumber(
            3,
            AllErrors,
            "Argument 2 to object:setmagcoeff() must be a number");

        if ((radius > 0))
        {
            Geometry *geometry = nullptr;
            if (body->getGeometry() != InvalidResource)
            {
                geometry = GetGeometryManager()->find(body->getGeometry());
            }
            if (geometry != nullptr && !geometry->isNormalized())
            {
                body->setGeometryScale(body->getGeometryScale() * ((float)radius / iradius));
            }
            body->setSemiAxes(body->getSemiAxes() * ((float)radius / iradius));

            // MagCoeff permet de positionner correctement le FixedPosition
            if (body->getClassification() == Body::Component)
            {
                body->setMagCoeff(coeff);
            }

            if (body->getRings() != nullptr)
            {
                //                RingSystem rings(0.0f, 0.0f);
                auto  rings        = std::unique_ptr<RingSystem>(body->getRings());
                float inner        = rings->innerRadius;
                float outer        = rings->outerRadius;
                rings->innerRadius = inner * (float)radius / iradius;
                rings->outerRadius = outer * (float)radius / iradius;
                body->setRings(std::move(rings));
            }
        }
    }

    return 0;
}

static int
object_radius(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected to function object:radius");

    Selection *sel = this_object(l);
    lua_pushnumber(l, sel->radius());

    return 1;
}

static int
object_setradius(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One argument expected to object:setradius()");

    Selection *sel = this_object(l);
    if (sel->body() == nullptr)
        return 0;

    Body  *body    = sel->body();
    float  iradius = body->getRadius();
    double radius
        = celx.safeGetNumber(2, AllErrors, "Argument to object:setradius() must be a number");
    if (radius <= 0)
        return 0;

    float scaleFactor = static_cast<float>(radius) / iradius;
    body->setSemiAxes(body->getSemiAxes() * scaleFactor);
    body->scaleRings(scaleFactor);

    return 0;
}

static int
object_type(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected to function object:type");

    Selection  *sel = this_object(l);
    const char *tname;
    switch (sel->getType())
    {
    case SelectionType::Body:
        tname = bodyTypeName(sel->body()->getClassification());
        break;
    case SelectionType::Star:
        tname = "star";
        break;
    case SelectionType::DeepSky:
        tname = dsoTypeName(sel->deepsky()->getObjType());
        break;
    case SelectionType::Location:
        tname = "location";
        break;
    case SelectionType::None:
        tname = "null";
        break;
    default:
        assert(0);
        tname = "unknown";
        break;
    }

    lua_pushstring(l, tname);

    return 1;
}

static int
object_name(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected to function object:name");

    Selection *sel = this_object(l);
    switch (sel->getType())
    {
    case SelectionType::Body:
        lua_pushstring(l, sel->body()->getName().c_str());
        break;
    case SelectionType::DeepSky:
        lua_pushstring(
            l,
            celx.appCore(AllErrors)
                ->getSimulation()
                ->getUniverse()
                ->getDSOCatalog()
                ->getDSOName(sel->deepsky())
                .c_str());
        break;
    case SelectionType::Star:
        lua_pushstring(
            l,
            celx.appCore(AllErrors)
                ->getSimulation()
                ->getUniverse()
                ->getStarCatalog()
                ->getStarName(*(sel->star()))
                .c_str());
        break;
    case SelectionType::Location:
        lua_pushstring(l, sel->location()->getName().c_str());
        break;
    default:
        lua_pushstring(l, "?");
        break;
    }

    return 1;
}

static int
object_localname(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected to function object:localname");

    Selection *sel = this_object(l);
    switch (sel->getType())
    {
    case SelectionType::Body:
        lua_pushstring(l, sel->body()->getName(true).c_str());
        break;
    case SelectionType::DeepSky:
        lua_pushstring(
            l,
            celx.appCore(AllErrors)
                ->getSimulation()
                ->getUniverse()
                ->getDSOCatalog()
                ->getDSOName(sel->deepsky(), true)
                .c_str());
        break;
    case SelectionType::Star:
        lua_pushstring(
            l,
            celx.appCore(AllErrors)
                ->getSimulation()
                ->getUniverse()
                ->getStarCatalog()
                ->getStarName(*(sel->star()), true)
                .c_str());
        break;
    case SelectionType::Location:
        lua_pushstring(l, sel->location()->getName(true).c_str());
        break;
    default:
        lua_pushstring(l, "?");
        break;
    }

    return 1;
}

static int
object_spectraltype(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected to function object:spectraltype");

    Selection *sel = this_object(l);
    if (sel->star() != nullptr)
    {
        char buf[16];
        strncpy(buf, sel->star()->getSpectralType(), sizeof buf);
        buf[sizeof(buf) - 1] = '\0'; // make sure it's zero terminate
        lua_pushstring(l, buf);
    }
    else
    {
        lua_pushnil(l);
    }

    return 1;
}

static int
object_getinfo(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected to function object:getinfo");

    lua_newtable(l);

    Selection *sel = this_object(l);
    if (sel->star() != nullptr)
    {
        Star *star = sel->star();
        celx.setTable("type", "star");
        celx.setTable(
            "name",
            celx.appCore(AllErrors)
                ->getSimulation()
                ->getUniverse()
                ->getStarCatalog()
                ->getStarName(*(sel->star()))
                .c_str());
        celx.setTable("catalogNumber", star->getIndex());
        celx.setTable("stellarClass", star->getSpectralType());
        celx.setTable("absoluteMagnitude", (lua_Number)star->getAbsoluteMagnitude());
        celx.setTable("luminosity", (lua_Number)star->getLuminosity());
        celx.setTable("radius", (lua_Number)star->getRadius());
        celx.setTable("temperature", (lua_Number)star->getTemperature());
        celx.setTable("rotationPeriod", (lua_Number)star->getRotationModel()->getPeriod());
        celx.setTable("bolometricMagnitude", (lua_Number)star->getBolometricMagnitude());

        const celestia::ephem::Orbit *orbit = star->getOrbit();
        if (orbit != nullptr)
            celx.setTable("orbitPeriod", orbit->getPeriod());

        if (star->getOrbitBarycenter() != nullptr)
        {
            Selection parent((Star *)(star->getOrbitBarycenter()));
            lua_pushstring(l, "parent");
            object_new(l, parent);
            lua_settable(l, -3);
        }
    }
    else if (sel->body() != nullptr)
    {
        Body       *body  = sel->body();
        const char *tname = bodyTypeName(body->getClassification());

        celx.setTable("type", tname);
        celx.setTable("name", body->getName().c_str());
        celx.setTable("mass", (lua_Number)body->getMass());
        celx.setTable("albedo", (lua_Number)body->getGeomAlbedo());
        celx.setTable("geomAlbedo", (lua_Number)body->getGeomAlbedo());
        celx.setTable("bondAlbedo", (lua_Number)body->getBondAlbedo());
        celx.setTable("reflectivity", (lua_Number)body->getReflectivity());
        celx.setTable("infoURL", body->getInfoURL().c_str());
        celx.setTable("radius", (lua_Number)body->getRadius());

        // TODO: add method to return semiaxes
        Vector3f semiAxes = body->getSemiAxes();
        // Note: oblateness is an obsolete field, replaced by semiaxes;
        // it's only here for backward compatibility.
        float polarRadius = semiAxes.y();
        float eqRadius    = max(semiAxes.x(), semiAxes.z());
        celx.setTable("oblateness", (eqRadius - polarRadius) / eqRadius);

        double lifespanStart, lifespanEnd;
        body->getLifespan(lifespanStart, lifespanEnd);
        celx.setTable("lifespanStart", (lua_Number)lifespanStart);
        celx.setTable("lifespanEnd", (lua_Number)lifespanEnd);
        // TODO: atmosphere, surfaces ?

        PlanetarySystem *system = body->getSystem();
        if (system->getPrimaryBody() != nullptr)
        {
            Selection parent(system->getPrimaryBody());
            lua_pushstring(l, "parent");
            object_new(l, parent);
            lua_settable(l, -3);
        }
        else
        {
            Selection parent(system->getStar());
            lua_pushstring(l, "parent");
            object_new(l, parent);
            lua_settable(l, -3);
        }

        lua_pushstring(l, "hasRings");
        lua_pushboolean(l, body->getRings() != nullptr);
        lua_settable(l, -3);

        // TIMELINE-TODO: The code to retrieve orbital and rotation periods only works
        // if the object has a single timeline phase. This should hardly ever
        // be a problem, but it still may be best to set the periods to zero
        // for objects with multiple phases.
        const celestia::ephem::RotationModel *rm = body->getRotationModel(0.0);
        celx.setTable("rotationPeriod", (double)rm->getPeriod());

        const celestia::ephem::Orbit *orbit = body->getOrbit(0.0);
        celx.setTable("orbitPeriod", orbit->getPeriod());
        const Atmosphere *atmosphere = body->getAtmosphere();
        if (atmosphere != nullptr)
        {
            celx.setTable("atmosphereHeight", (double)atmosphere->height);
            celx.setTable("atmosphereCloudHeight", (double)atmosphere->cloudHeight);
            celx.setTable("atmosphereCloudSpeed", (double)atmosphere->cloudSpeed);
        }
    }
    else if (sel->deepsky() != nullptr)
    {
        DeepSkyObject *deepsky     = sel->deepsky();
        const char    *objTypeName = dsoTypeName(deepsky->getObjType());
        celx.setTable("type", objTypeName);

        celx.setTable(
            "name",
            celx.appCore(AllErrors)
                ->getSimulation()
                ->getUniverse()
                ->getDSOCatalog()
                ->getDSOName(deepsky)
                .c_str());
        celx.setTable("catalogNumber", deepsky->getIndex());

        if (!strcmp(objTypeName, "galaxy"))
            celx.setTable("hubbleType", deepsky->getType());

        celx.setTable("absoluteMagnitude", (lua_Number)deepsky->getAbsoluteMagnitude());
        celx.setTable("radius", (lua_Number)deepsky->getRadius());
    }
    else if (sel->location() != nullptr)
    {
        celx.setTable("type", "location");
        Location *location = sel->location();
        celx.setTable("name", location->getName().c_str());
        celx.setTable("size", (lua_Number)location->getSize());
        celx.setTable("importance", (lua_Number)location->getImportance());
        celx.setTable("infoURL", location->getInfoURL().c_str());

        auto  featureType     = location->getFeatureType();
        auto &LocationFlagMap = celx.appCore(AllErrors)->scriptMaps()->LocationFlagMap;
        auto  iter            = std::find_if(
            LocationFlagMap.begin(),
            LocationFlagMap.end(),
            [&featureType](auto &it) { return it.second == featureType; });
        if (iter != LocationFlagMap.end())
            celx.setTable("featureType", std::string(iter->first).c_str());
        else
            celx.setTable("featureType", "Unknown");

        Body *parent = location->getParentBody();
        if (parent != nullptr)
        {
            Selection selection(parent);
            lua_pushstring(l, "parent");
            object_new(l, selection);
            lua_settable(l, -3);
        }
    }
    else
    {
        celx.setTable("type", "null");
    }
    return 1;
}

static int
object_absmag(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected to function object:absmag");

    Selection *sel = this_object(l);
    if (sel->star() != nullptr)
        lua_pushnumber(l, sel->star()->getAbsoluteMagnitude());
    else
        lua_pushnil(l);

    return 1;
}

static int
object_mark(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 7, "Need 0 to 6 arguments for object:mark");

    Selection    *sel     = this_object(l);
    CelestiaCore *appCore = celx.appCore(AllErrors);

    Color       markColor(0.0f, 1.0f, 0.0f);
    const char *colorString
        = celx.safeGetString(2, WrongType, "First argument to object:mark must be a string");
    if (colorString != nullptr)
        Color::parse(colorString, markColor);

    celestia::MarkerRepresentation::Symbol markSymbol = celestia::MarkerRepresentation::Diamond;
    const char                            *markerString
        = celx.safeGetString(3, WrongType, "Second argument to object:mark must be a string");
    if (markerString != nullptr)
        markSymbol = parseMarkerSymbol(markerString);

    float markSize
        = (float)
              celx.safeGetNumber(4, WrongType, "Third arg to object:mark must be a number", 10.0);
    if (markSize < 1.0f)
        markSize = 1.0f;
    else if (markSize > 10000.0f)
        markSize = 10000.0f;

    float markAlpha
        = (float)
              celx.safeGetNumber(5, WrongType, "Fourth arg to object:mark must be a number", 0.9);
    if (markAlpha < 0.0f)
        markAlpha = 0.0f;
    else if (markAlpha > 1.0f)
        markAlpha = 1.0f;

    Color markColorAlpha(0.0f, 1.0f, 0.0f, 0.9f);
    markColorAlpha = Color(markColor, markAlpha);

    const char *markLabel
        = celx.safeGetString(6, WrongType, "Fifth argument to object:mark must be a string");
    if (markLabel == nullptr)
        markLabel = "";

    bool occludable = celx.safeGetBoolean(
        7,
        WrongType,
        "Sixth argument to object:mark must be a boolean",
        true);

    Simulation *sim = appCore->getSimulation();

    celestia::MarkerRepresentation markerRep(markSymbol);
    markerRep.setSize(markSize);
    markerRep.setColor(markColorAlpha);
    markerRep.setLabel(markLabel);
    sim->getUniverse()->markObject(*sel, markerRep, 1, occludable);

    return 0;
}

static int
object_unmark(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected to function object:unmark");

    Selection    *sel     = this_object(l);
    CelestiaCore *appCore = celx.appCore(AllErrors);

    Simulation *sim = appCore->getSimulation();
    sim->getUniverse()->unmarkObject(*sel, 1);

    return 0;
}

// Return the object's current position.  A time argument is optional;
// if not provided, the current master simulation time is used.
static int
object_getposition(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 2, "Expected no or one argument to object:getposition");

    Selection    *sel     = this_object(l);
    CelestiaCore *appCore = celx.appCore(AllErrors);

    double t = celx.safeGetNumber(
        2,
        WrongType,
        "Time expected as argument to object:getposition",
        appCore->getSimulation()->getTime());
    celx.newPosition(sel->getPosition(t));

    return 1;
}

static int
object_getchildren(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments expected for object:getchildren()");

    Selection    *sel     = this_object(l);
    CelestiaCore *appCore = celx.appCore(AllErrors);

    Simulation *sim = appCore->getSimulation();

    lua_newtable(l);
    if (sel->star() != nullptr)
    {
        const SolarSystem *solarSys = sim->getUniverse()->getSolarSystem(sel->star());
        if (solarSys != nullptr)
        {
            for (int i = 0; i < solarSys->getPlanets()->getSystemSize(); i++)
            {
                Body     *body = solarSys->getPlanets()->getBody(i);
                Selection satSel(body);
                object_new(l, satSel);
                lua_rawseti(l, -2, i + 1);
            }
        }
    }
    else if (sel->body() != nullptr)
    {
        const PlanetarySystem *satellites = sel->body()->getSatellites();
        if (satellites != nullptr && satellites->getSystemSize() != 0)
        {
            for (int i = 0; i < satellites->getSystemSize(); i++)
            {
                Body     *body = satellites->getBody(i);
                Selection satSel(body);
                object_new(l, satSel);
                lua_rawseti(l, -2, i + 1);
            }
        }
    }

    return 1;
}

static int
object_preloadtexture(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No argument expected to object:preloadtexture");
    CelestiaCore *appCore = celx.appCore(AllErrors);

    Renderer  *renderer = appCore->getRenderer();
    Selection *sel      = this_object(l);

    if (sel->body() != nullptr && renderer != nullptr)
    {
        LuaState *luastate = celx.getLuaStateObject();
        // make sure we don't timeout because of texture-loading:
        double timeToTimeout = luastate->timeout - luastate->getTime();

        renderer->loadTextures(sel->body());

        // no matter how long it really took, make it look like 0.1s:
        luastate->timeout = luastate->getTime() + timeToTimeout - 0.1;
    }

    return 0;
}

/*! object:catalognumber(string: catalog_prefix)
 *
 *  Look up the catalog number for a star in one of the supported catalogs,
 *  currently HIPPARCOS, HD, or SAO. The single argument is a string that
 *  specifies the catalog number, either "HD", "SAO", or "HIP".
 *  If the object is a star, the catalog string is valid, and the star
 *  is present in the catalog, the catalog number is returned on the stack.
 *  Otherwise, nil is returned.
 *
 * \verbatim
 * -- Example: Get the SAO and HD catalog numbers for Rigel
 * --
 * rigel = celestia:find("Rigel")
 * sao = rigel:catalognumber("SAO")
 * hd = rigel:catalognumber("HD")
 *
 * \endverbatim
 */
static int
object_catalognumber(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One argument expected to object:catalognumber");
    CelestiaCore *appCore = celx.appCore(AllErrors);

    Selection  *sel = this_object(l);
    const char *catalogName
        = celx.safeGetString(2, WrongType, "Argument to object:catalognumber must be a string");

    // The argument is a string indicating the catalog.
    bool        validCatalog = false;
    bool        useHIPPARCOS = false;
    StarCatalog catalog      = StarCatalog::HenryDraper;
    if (catalogName != nullptr)
    {
        if (compareIgnoringCase(catalogName, "HD") == 0)
        {
            catalog      = StarCatalog::HenryDraper;
            validCatalog = true;
        }
        else if (compareIgnoringCase(catalogName, "SAO") == 0)
        {
            catalog      = StarCatalog::SAO;
            validCatalog = true;
        }
        else if (compareIgnoringCase(catalogName, "HIP") == 0)
        {
            useHIPPARCOS = true;
            validCatalog = true;
        }
    }

    uint32_t catalogNumber = AstroCatalog::InvalidIndex;
    if (sel->star() != nullptr && validCatalog)
    {
        uint32_t internalNumber = sel->star()->getIndex();

        if (useHIPPARCOS)
        {
            // Celestia's internal catalog numbers /are/ HIPPARCOS numbers
            if (internalNumber < StarDatabase::MAX_HIPPARCOS_NUMBER)
                catalogNumber = internalNumber;
        }
        else
        {
            const StarDatabase *stardb = appCore->getSimulation()->getUniverse()->getStarCatalog();
            catalogNumber              = stardb->crossIndex(catalog, internalNumber);
        }
    }

    if (catalogNumber != AstroCatalog::InvalidIndex)
        lua_pushnumber(l, catalogNumber);
    else
        lua_pushnil(l);

    return 1;
}

// Locations iterator function; two upvalues expected. Used by
// object:locations method.
static int
object_locations_iter(lua_State *l)
{
    CelxLua    celx(l);
    Selection *sel = to_object(l, lua_upvalueindex(1));
    if (sel == nullptr)
    {
        celx.doError("Bad object!");
        return 0;
    }

    // Get the current counter value
    uint32_t i = (uint32_t)lua_tonumber(l, lua_upvalueindex(2));

    Body *body = sel->body();
    if (body == nullptr)
        return 0;

    auto locations = body->getLocations();
    if (!locations.has_value() || i >= locations->size())
    {
        // Return nil when we've enumerated all the locations (or if
        // there were no locations associated with the object.)
        return 0;
    }

    // Increment the counter
    lua_pushnumber(l, i + 1);
    lua_replace(l, lua_upvalueindex(2));

    Location *loc = (*locations)[i];
    if (loc == nullptr)
        lua_pushnil(l);
    else
        object_new(l, Selection(loc));

    return 1;
}

/*! object:locations()
 *
 * Return an iterator over all the locations associated with an object.
 * Only solar system bodies have locations; for all other object types,
 * this method will return an empty iterator.
 *
 * \verbatim
 * -- Example: print locations of current selection
 * --
 * for loc in celestia:getselection():locations() do
 *     celestia:log(loc:name())
 * end
 *
 * \endverbatim
 */
static int
object_locations(lua_State *l)
{
    CelxLua celx(l);
    // Push a closure with two upvalues: the object and a counter
    lua_pushvalue(l, 1);  // object
    lua_pushnumber(l, 0); // counter
    lua_pushcclosure(l, object_locations_iter, 2);

    return 1;
}

/*! object:bodyfixedframe()
 *
 * Return the body-fixed frame for this object.
 *
 * \verbatim
 * -- Example: get the body-fixed frame of the Earth
 * --
 * earth = celestia:find("Sol/Earth")
 * ebf = earth:bodyfixedframe()
 *
 * \endverbatim
 */
static int
object_bodyfixedframe(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments allowed for object:bodyfixedframe");

    Selection *sel = this_object(l);
    celx.newFrame(ObserverFrame(ObserverFrame::BodyFixed, *sel));

    return 1;
}

/*! object:equatorialframe()
 *
 * Return the mean equatorial frame for this object.
 *
 * \verbatim
 * -- Example: getthe equatorial frame of the Earth
 * --
 * earth = celestia:find("Sol/Earth")
 * eme = earth:equatorialframe()
 *
 * \endverbatim
 */
static int
object_equatorialframe(lua_State *l)
{
    // TODO: allow one argument specifying a freeze time
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments allowed for to object:equatorialframe");

    Selection *sel = this_object(l);
    celx.newFrame(ObserverFrame(ObserverFrame::Equatorial, *sel));

    return 1;
}

/*! object:orbitframe(time: t)
 *
 * Return the frame in which the orbit for an object is defined at a particular
 * time. If time isn't specified, the current simulation time is assumed. The
 * positions of stars and deep sky objects are always defined in the universal
 * frame.
 *
 * \verbatim
 * -- Example: get the orbital frame for the Earth at the current time.
 * --
 * earth = celestia:find("Sol/Earth")
 * eof = earth:orbitframe()
 *
 * \endverbatim
 */
static int
object_orbitframe(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 2, "One or no arguments allowed for to object:orbitframe");

    Selection    *sel     = this_object(l);
    CelestiaCore *appCore = celx.appCore(AllErrors);

    double t = celx.safeGetNumber(
        2,
        WrongType,
        "Time expected as argument to object:orbitframe",
        appCore->getSimulation()->getTime());

    if (sel->body() == nullptr)
    {
        // The default universal frame
        celx.newFrame(ObserverFrame());
    }
    else
    {
        const auto &f = sel->body()->getOrbitFrame(t);
        celx.newFrame(ObserverFrame(f));
    }

    return 1;
}

/*! object:bodyframe(time: t)
 *
 * Return the frame in which the orientation for an object is defined at a
 * particular time. If time isn't specified, the current simulation time is
 * assumed. The positions of stars and deep sky objects are always defined
 * in the universal frame.
 *
 * \verbatim
 * -- Example: get the curren body frame for the International Space Station.
 * --
 * iss = celestia:find("Sol/Earth/ISS")
 * f = iss:bodyframe()
 *
 * \endverbatim
 */
static int
object_bodyframe(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 2, "One or no arguments allowed for to object:bodyframe");

    Selection    *sel     = this_object(l);
    CelestiaCore *appCore = celx.appCore(AllErrors);

    double t = celx.safeGetNumber(
        2,
        WrongType,
        "Time expected as argument to object:orbitframe",
        appCore->getSimulation()->getTime());

    if (sel->body() == nullptr)
    {
        // The default universal frame
        celx.newFrame(ObserverFrame());
    }
    else
    {
        const auto &f = sel->body()->getBodyFrame(t);
        celx.newFrame(ObserverFrame(f));
    }

    return 1;
}

/*! object:getphase(time: t)
 *
 * Get the active timeline phase at the specified time. If no time is
 * specified, the current simulation time is used. This method returns
 * nil if the object is not a solar system body, or if the time lies
 * outside the range covered by the timeline.
 *
 * \verbatim
 * -- Example: get the timeline phase for Cassini at midnight January 1, 2000 UTC.
 * --
 * cassini = celestia:find("Sol/Cassini")
 * tdb = celestia:utctotdb(2000, 1, 1)
 * phase = cassini:getphase(tdb)
 *
 * \endverbatim
 */
static int
object_getphase(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 2, "One or no arguments allowed for to object:getphase");

    Selection    *sel     = this_object(l);
    CelestiaCore *appCore = celx.appCore(AllErrors);

    double t = celx.safeGetNumber(
        2,
        WrongType,
        "Time expected as argument to object:getphase",
        appCore->getSimulation()->getTime());

    if (sel->body() == nullptr)
    {
        lua_pushnil(l);
    }
    else
    {
        const Timeline *timeline = sel->body()->getTimeline();
        if (timeline->includes(t))
        {
            celx.newPhase(timeline->findPhase(t));
        }
        else
        {
            lua_pushnil(l);
        }
    }

    return 1;
}

// Phases iterator function; two upvalues expected. Used by
// object:phases method.
static int
object_phases_iter(lua_State *l)
{
    CelxLua    celx(l);
    Selection *sel = to_object(l, lua_upvalueindex(1));
    if (sel == nullptr)
    {
        celx.doError("Bad object!");
        return 0;
    }

    // Get the current counter value
    uint32_t i = (uint32_t)lua_tonumber(l, lua_upvalueindex(2));

    const Timeline *timeline = nullptr;
    if (sel->body() != nullptr)
    {
        timeline = sel->body()->getTimeline();
    }

    if (timeline != nullptr && i < timeline->phaseCount())
    {
        // Increment the counter
        lua_pushnumber(l, i + 1);
        lua_replace(l, lua_upvalueindex(2));

        const auto &phase = timeline->getPhase(i);
        celx.newPhase(phase);

        return 1;
    }

    // Return nil when we've enumerated all the phases (or if
    // if the object wasn't a solar system body.)
    return 0;
}

/*! object:phases()
 *
 * Return an iterator over all the phases in an object's timeline.
 * Only solar system bodies have timeline; for all other object types,
 * this method will return an empty iterator. The phases in a timeline
 * are always sorted from earliest to latest, and always coverage a
 * continuous span of time.
 *
 * \verbatim
 * -- Example: copy all of an objects phases into the array timeline
 * --
 * timeline = { }
 * count = 0
 * for phase in celestia:getselection():phases() do
 *     count = count + 1
 *     timeline[count] = phase
 * end
 *
 * \endverbatim
 */
static int
object_phases(lua_State *l)
{
    CelxLua celx(l);
    // Push a closure with two upvalues: the object and a counter
    lua_pushvalue(l, 1);  // object
    lua_pushnumber(l, 0); // counter
    lua_pushcclosure(l, object_phases_iter, 2);

    return 1;
}

static int
object_setringstexture(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 2, "One or two arguments are expected for object:setringstexture()");

    Selection *sel = this_object(l);

    if (sel->body() == nullptr || sel->body()->getRings() == nullptr)
        return 0;

    const char *textureName = celx.safeGetString(2);
    if (*textureName == '\0')
    {
        celx.doError("Empty texture name passed to object:setringstexture()");
        return 0;
    }
    const char *path = celx.safeGetString(3, WrongType);
    if (path == nullptr)
        path = "";

    sel->body()->getRings()->texture = MultiResTexture(textureName, path);

    return 0;
}

static int
object_getmass(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments are expected for object:getmass()");

    Selection *sel = this_object(l);

    if (sel->body() == nullptr)
        return 0;

    return celx.push(sel->body()->getMass());
}

static int
object_getdensity(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments are expected for object:getdensity()");

    Selection *sel = this_object(l);

    if (sel->body() == nullptr)
        return 0;

    return celx.push(sel->body()->getDensity());
}

static int
object_gettemperature(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(1, 1, "No arguments are expected for object:getdensity()");

    Selection *sel = this_object(l);

    float temp = 0;
    if (sel->body() != nullptr)
    {
        CelestiaCore *appCore = celx.appCore(AllErrors);
        double        time    = appCore->getSimulation()->getTime();
        temp                  = sel->body()->getTemperature(time);
    }
    else if (sel->star() != nullptr)
    {
        temp = sel->star()->getTemperature();
    }

    if (temp > 0)
        return celx.push(temp);

    return 0;
}

void
CreateObjectMetaTable(lua_State *l)
{
    CelxLua celx(l);

    celx.createClassMetatable(Celx_Object);

    celx.registerMethod("__tostring", object_tostring);
    celx.registerMethod("visible", object_visible);
    celx.registerMethod("setvisible", object_setvisible);
    celx.registerMethod("setlabelvisible", object_setlabelvisible);
    celx.registerMethod("sethierarchyvisible", object_sethierarchyvisible);
    celx.registerMethod("partvisible", object_partvisible);
    celx.registerMethod("setpartvisible", object_setpartvisible);
    celx.registerMethod("orbitcoloroverridden", object_orbitcoloroverridden);
    celx.registerMethod("setorbitcoloroverridden", object_setorbitcoloroverridden);
    celx.registerMethod("setorbitcolor", object_setorbitcolor);
    celx.registerMethod("orbitvisibility", object_orbitvisibility);
    celx.registerMethod("setorbitvisibility", object_setorbitvisibility);
    celx.registerMethod("addreferencemark", object_addreferencemark);
    celx.registerMethod("removereferencemark", object_removereferencemark);
    celx.registerMethod("setvisualizerscale", object_visualizerscale);
    celx.registerMethod("setgridreferencemarkproperty", object_setgridreferencemarkproperty);
    celx.registerMethod(
        "object_setgridreferencemarkproperty2",
        object_setgridreferencemarkproperty2);
    celx.registerMethod(
        "setvisualizerreferencemarkproperty",
        object_setvisualizerreferencemarkproperty);
    celx.registerMethod("setregionofinterestvisible", object_setregionofinterestvisible);
    celx.registerMethod("setregionofinteresttextvisible", object_setregionofinteresttextvisible);
    celx.registerMethod("setregionofinterestcolor", object_setregionofinterestcolor);
    celx.registerMethod("setregionofinterestcontourwidth", object_setregionofinterestcontourwidth);
    celx.registerMethod("setregionofinterestopacity", object_setregionofinterestopacity);
    celx.registerMethod("setpointofinterestvisible", object_setpointofinterestvisible);
    celx.registerMethod("setpointofinteresttextvisible", object_setpointofinteresttextvisible);
    celx.registerMethod("setpointofinterestcolor", object_setpointofinterestcolor);
    celx.registerMethod("radius", object_radius);
    celx.registerMethod("setradius", object_setradius);
    celx.registerMethod("setmagcoeff", object_setmagcoeff);
    celx.registerMethod("type", object_type);
    celx.registerMethod("spectraltype", object_spectraltype);
    celx.registerMethod("getinfo", object_getinfo);
    celx.registerMethod("catalognumber", object_catalognumber);
    celx.registerMethod("absmag", object_absmag);
    celx.registerMethod("name", object_name);
    celx.registerMethod("localname", object_localname);
    celx.registerMethod("mark", object_mark);
    celx.registerMethod("unmark", object_unmark);
    celx.registerMethod("getposition", object_getposition);
    celx.registerMethod("getchildren", object_getchildren);
    celx.registerMethod("locations", object_locations);
    celx.registerMethod("bodyfixedframe", object_bodyfixedframe);
    celx.registerMethod("equatorialframe", object_equatorialframe);
    celx.registerMethod("orbitframe", object_orbitframe);
    celx.registerMethod("bodyframe", object_bodyframe);
    celx.registerMethod("getphase", object_getphase);
    celx.registerMethod("phases", object_phases);
    celx.registerMethod("preloadtexture", object_preloadtexture);
    celx.registerMethod("setringstexture", object_setringstexture);
    celx.registerMethod("gettemperature", object_gettemperature);
    celx.registerMethod("getmass", object_getmass);
    celx.registerMethod("getdensity", object_getdensity);

    lua_pop(l, 1); // pop metatable off the stack
}

// ==================== object extensions ====================

// TODO: This should be replaced by an actual Atmosphere object
static int
object_setatmosphere(lua_State *l)
{
    CelxLua celx(l);

    celx.checkArgs(23, 23, "22 arguments (!) expected to function object:setatmosphere");

    Selection *sel = this_object(l);
    // CelestiaCore* appCore = getAppCore(l, AllErrors);

    Body *body = sel->body();
    if (body == nullptr)
        return 0;

    Atmosphere *atmosphere = body->getAtmosphere();
    if (atmosphere == nullptr)
        return 0;

    auto r = static_cast<float>(
        celx.safeGetNumber(2, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    auto g = static_cast<float>(
        celx.safeGetNumber(3, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    auto b = static_cast<float>(
        celx.safeGetNumber(4, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    //            Color testColor(0.0f, 1.0f, 0.0f);
    Color testColor(r, g, b);
    atmosphere->lowerColor = testColor;
    r                      = static_cast<float>(
        celx.safeGetNumber(5, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    g = static_cast<float>(
        celx.safeGetNumber(6, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    b = static_cast<float>(
        celx.safeGetNumber(7, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    atmosphere->upperColor = Color(r, g, b);
    r                      = static_cast<float>(
        celx.safeGetNumber(8, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    g = static_cast<float>(
        celx.safeGetNumber(9, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    b = static_cast<float>(
        celx.safeGetNumber(10, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    atmosphere->skyColor = Color(r, g, b);
    r                    = static_cast<float>(
        celx.safeGetNumber(11, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    g = static_cast<float>(
        celx.safeGetNumber(12, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    b = static_cast<float>(
        celx.safeGetNumber(13, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    atmosphere->sunsetColor = Color(r, g, b);
    /* r = */ celx.safeGetNumber(
        14,
        AllErrors,
        "Arguments to observer:setatmosphere() must be numbers");
    /* g = */ celx.safeGetNumber(
        15,
        AllErrors,
        "Arguments to observer:setatmosphere() must be numbers");
    /* b = */ celx.safeGetNumber(
        16,
        AllErrors,
        "Arguments to observer:setatmosphere() must be numbers");
    // HWR            atmosphere->rayleighCoeff = Vector3(r, g, b);
    /* r = */ celx.safeGetNumber(
        17,
        AllErrors,
        "Arguments to observer:setatmosphere() must be numbers");
    /* g = */ celx.safeGetNumber(
        18,
        AllErrors,
        "Arguments to observer:setatmosphere() must be numbers");
    /* b = */ celx.safeGetNumber(
        19,
        AllErrors,
        "Arguments to observer:setatmosphere() must be numbers");
    // HWR            atmosphere->absorptionCoeff = Vector3(r, g, b);
    b = static_cast<float>(
        celx.safeGetNumber(20, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    atmosphere->mieCoeff = b;
    b                    = static_cast<float>(
        celx.safeGetNumber(21, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    atmosphere->mieScaleHeight = b;
    b                          = static_cast<float>(
        celx.safeGetNumber(22, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    atmosphere->miePhaseAsymmetry = b;
    b                             = static_cast<float>(
        celx.safeGetNumber(23, AllErrors, "Arguments to observer:setatmosphere() must be numbers"));
    atmosphere->rayleighScaleHeight = b;

    body->recomputeCullingRadius();

    return 0;
}

#define checkEmpty(c, s)                         \
    if (s->empty())                              \
    {                                            \
        c.doError("Selection object is empty!"); \
        return 0;                                \
    }

static int
object_getcategories(lua_State *l)
{
    CelxLua celx(l);

    Selection *s = celx.getThis<Selection>();
    checkEmpty(celx, s);
    const auto *set = UserCategory::getCategories(*s);
    return celx.pushIterable<UserCategoryId>(set);
}

static int
object_addtocategory(lua_State *l)
{
    CelxLua celx(l);

    Selection *s = celx.getThis<Selection>();
    checkEmpty(celx, s);
    bool ret;
    if (celx.isUserData(2))
    {
        auto c = *celx.getUserData<UserCategoryId>(2);
        ret    = UserCategory::addObject(*s, c);
    }
    else
    {
        const char *n = celx.safeGetString(
            2,
            AllErrors,
            "Argument to object:addtocategory() must be string or userdata");
        if (n == nullptr)
            return celx.push(false);
        auto c = UserCategory::find(n);
        ret    = UserCategory::addObject(*s, c);
    }
    return celx.push(ret);
}

static int
object_removefromcategory(lua_State *l)
{
    CelxLua celx(l);

    Selection *s = celx.getThis<Selection>();
    checkEmpty(celx, s);
    bool ret;
    if (celx.isUserData(2))
    {
        UserCategoryId c = *celx.getUserData<UserCategoryId>(2);
        ret              = UserCategory::removeObject(*s, c);
    }
    else
    {
        const char *n = celx.safeGetString(
            2,
            AllErrors,
            "Argument to object:addtocategory() must be string or userdata");
        if (n == nullptr)
            return celx.push(false);
        auto c = UserCategory::find(n);
        ret    = UserCategory::removeObject(*s, c);
    }
    return celx.push(ret);
}

static int
object_setclustersymbol(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One allowed for to object:setclustersymbol");

    Selection    *sel     = this_object(l);
    CelestiaCore *appCore = celx.appCore(AllErrors);

    std::string path           = celx.safeGetString(2);
    auto        referenceMarks = sel->body()->getReferenceMarksMut();
    if (!referenceMarks.has_value())
    {
        return 0;
    }

    // Get reference marks from the optional value
    auto referenceMarksOk = referenceMarks.value();

    //        auto clusterEntry = std::find_if(
    //            referenceMarksOk.begin(),
    //            referenceMarksOk.end(),
    //            []( ReferenceMark* mark) { return dynamic_cast<const Cluster *>(mark) != nullptr;
    //            });

    // get clusterEntry
    bool clusterFound{ false };
    for (auto mark : referenceMarksOk)
    {
        auto cluster = dynamic_cast<Cluster *>(mark);
        if (cluster != nullptr)
        {
            cluster->setSymbol(path);
            clusterFound = true;

            break;
        }
    }

    if (!clusterFound)
    {
        return 0;
    }

    //    auto cluster = static_cast<Cluster *>(*clusterEntry);

    return 0;
}

static int
object_setclustersymbolsize(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One allowed for to object:setclustersymbolsize");

    Selection    *sel     = this_object(l);
    CelestiaCore *appCore = celx.appCore(AllErrors);

    double symbolSize     = celx.safeGetNumber(2);
    auto   referenceMarks = sel->body()->getReferenceMarksMut();
    if (!referenceMarks)
    {
        return 0;
    }

    auto referenceMarksOk = referenceMarks.value();

    //    auto clusterEntry = std::find_if(
    //        referenceMarks->begin(),
    //        referenceMarks->end(),
    //        [](const ReferenceMark *mark) { return dynamic_cast<const Cluster *>(mark) != nullptr;
    //        });
    //    if (clusterEntry == referenceMarks->end())
    //    {
    //        return 0;
    //    }

    bool clusterFound{ false };
    for (auto mark : referenceMarksOk)
    {
        auto cluster = dynamic_cast<Cluster *>(mark);
        if (cluster != nullptr)
        {
            cluster->setSymbolSize(symbolSize);
            clusterFound = true;

            break;
        }
    }

    if (!clusterFound)
    {
        return 0;
    }

    //    auto cluster = static_cast<Cluster*>(*clusterEntry);
    //    auto cluster = *clusterEntry;

    return 0;
}

static int
object_setclusterobjectsymbol(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One allowed for to object:setclusterobjectsymbol");

    Selection    *sel     = this_object(l);
    CelestiaCore *appCore = celx.appCore(AllErrors);

    //    std::string path           = celx.safeGetString(2);
    auto resourceHandle = celx.safeGetNumber(2);
    auto geometryHandle = sel->body()->getGeometry();
    auto geometry       = GetGeometryManager()->find(geometryHandle);
    auto billboard      = dynamic_cast<BillboardGeometry *>(geometry);
    if (!billboard)
    {
        return 0;
    }

    billboard->setSymbol(resourceHandle);
    return 0;
}

static int
object_setclusterobjectsymbolsize(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One allowed for to object:setclusterobjectsymbolsize");

    Selection    *sel     = this_object(l);
    CelestiaCore *appCore = celx.appCore(AllErrors);

    double symbolSize     = celx.safeGetNumber(2);
    auto   geometryHandle = sel->body()->getGeometry();
    auto   geometry       = GetGeometryManager()->find(geometryHandle);
    auto   billboard      = dynamic_cast<BillboardGeometry *>(geometry);
    if (!billboard)
    {
        return 0;
    }

    billboard->setSymbolSize(symbolSize);
    return 0;
}

static int
object_resetclustervisibility(lua_State *l)
{
    CelxLua celx(l);
    celx.checkArgs(2, 2, "One argument allowed for to object:resetclustervisibility");

    Selection *sel            = this_object(l);
    auto       referenceMarks = sel->body()->getReferenceMarksMut();
    if (!referenceMarks)
    {
        return 0;
    }

    auto referenceMarksOk = referenceMarks.value();

    //    auto clusterEntry = std::find_if(
    //        referenceMarks->begin(),
    //        referenceMarks->end(),
    //        [](const ReferenceMark *mark) { return dynamic_cast<const Cluster *>(mark) != nullptr;
    //        });
    //    if (clusterEntry == referenceMarks->end())
    //    {
    //        return 0;
    //    }

    bool clusterFound{ false };
    for (auto mark : referenceMarksOk)
    {
        auto cluster = dynamic_cast<Cluster *>(mark);
        if (cluster != nullptr)
        {
            bool visible = celx.safeGetBoolean(2);
            cluster->resetObjectVisibility(visible);
            clusterFound = true;
            break;
        }
    }

    if (!clusterFound)
    {
        return 0;
    }

    //    auto cluster = static_cast<Cluster *>(*clusterEntry);

    return 0;
}

void
ExtendObjectMetaTable(lua_State *l)
{
    CelxLua celx(l);
    celx.pushClassName(Celx_Object);
    lua_rawget(l, LUA_REGISTRYINDEX);
    if (lua_type(l, -1) != LUA_TTABLE)
        std::cout << "Metatable for " << CelxLua::ClassNames[Celx_Object] << " not found!\n";
    celx.registerMethod("setatmosphere", object_setatmosphere);
    celx.registerMethod("getcategories", object_getcategories);
    celx.registerMethod("addtocategory", object_addtocategory);
    celx.registerMethod("removefromcategory", object_removefromcategory);
    // VTS
    celx.registerMethod("hasreferencemark", object_hasreferencemark);
    celx.registerMethod("setclustersymbol", object_setclustersymbol);
    celx.registerMethod("setclustersymbolsize", object_setclustersymbolsize);
    celx.registerMethod("setclusterobjectsymbol", object_setclusterobjectsymbol);
    celx.registerMethod("setclusterobjectsymbolsize", object_setclusterobjectsymbolsize);
    celx.registerMethod("resetclustervisibility", object_resetclustervisibility);
    celx.pop(1);
}
