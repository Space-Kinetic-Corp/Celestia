#include "umbra.h"

#include "body.h"
#include "glshader.h"
#include "meshmanager.h"
#include "render.h"
#include "timelinephase.h"
#include <celmath/vecgl.h>

Umbra::Umbra(Body &body) :
    ReferenceMark(), _body(body), _umbraVisible(false), _umbraMaxExtent(3 * body.getRadius()),
    _umbraColor(), _penumbraVisible(false), _penumbraMaxExtent(3 * body.getRadius()),
    _penumbraColor()
{
}

/*! Draw the reference mark geometry at the specified time.
 *
 */
void
Umbra::render(
    Renderer              *renderer,
    const Eigen::Vector3f &position,
    float                  discSizeInPixels,
    double                 tdb,
    const Matrices         &m) const
{
    Eigen::Quaterniond q  = _body.getEclipticToBodyFixed(tdb);
    Eigen::Quaternionf qf = q.cast<float>().conjugate();

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glRotatef(qf.w(), qf.x(), qf.y(), qf.z());

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);

    // 8<---------------- Code taken from visibleregion.cpp ----------------------------------------
    //  In order to avoid precision problems and extremely large values, scale
    //  the target position and semiaxes such that the largest semiaxis is 1.0.
    Eigen::Vector3d lightDir
        = _body.getPosition(tdb).offsetFromKm(_body.getSystem()->getStar()->getPosition(tdb));
    lightDir = lightDir / _body.getRadius();

    // Another measure to prevent precision problems: if the distance to the
    // object is much greater than the largest semi axis, clamp it to 1e4 times
    // the radius, as body-to-target rays at that distance are nearly parallel anyhow.
    if (lightDir.norm() > 10000.0)
        lightDir *= (10000.0 / lightDir.norm());

    // Pick two orthogonal axes both normal to the light direction
    // Vector3d lightDirNorm = lightDir.normalized();
    auto dir = lightDir.normalized();
    //---------------- Code taken from visibleregion.cpp ---------------------------------------->8

    if (_umbraVisible)
    {
        glColor3f(_umbraColor.red(), _umbraColor.green(), _umbraColor.blue());
        makeTranslucent(_umbraColor.alpha() > 0.95);

        auto umbraConeRange = getUmbraRange(_body.getAstrocentricPosition(tdb).norm());
        drawUmbraCone(tdb, umbraConeRange.first, umbraConeRange.second, dir, discSizeInPixels);
    }

    if (_penumbraVisible)
    {
        glColor3f(_penumbraColor.red(), _penumbraColor.green(), _penumbraColor.blue());
        makeTranslucent(_penumbraColor.alpha() > 0.95);

        auto penumbraConeRange = getPenumbraRange(_body.getAstrocentricPosition(tdb).norm());
        drawPenumbraCone(
            tdb,
            penumbraConeRange.first,
            penumbraConeRange.second,
            dir,
            discSizeInPixels);
    }

    glPopMatrix();
}

void
Umbra::setUmbraVisible(bool isVisible)
{
    if (isVisible != _umbraVisible)
    {
        _umbraVisible = isVisible;
        _body.recomputeCullingRadius();
    }
}

void
Umbra::setUmbraColor(const Color &c)
{
    _umbraColor = c;
}
void
Umbra::setUmbraMaxExtent(double extent)
{
    if (extent != _umbraMaxExtent)
    {
        _umbraMaxExtent = extent;
        _body.recomputeCullingRadius();
    }
}

void
Umbra::setPenumbraVisible(bool isVisible)
{
    if (isVisible != _penumbraVisible)
    {
        _penumbraVisible = isVisible;
        _body.recomputeCullingRadius();
    }
}
void
Umbra::setPenumbraColor(const Color &c)
{
    _penumbraColor = c;
}
void
Umbra::setPenumbraMaxExtent(double extent)
{
    if (extent != _penumbraMaxExtent)
    {
        _penumbraMaxExtent = extent;
        _body.recomputeCullingRadius();
    }
}

/*! Return the radius of a bounding sphere (in kilometers) large enough
 *  to contain the reference mark geometry.
 */
float
Umbra::boundingSphereRadius() const
{
    return std::max(
        _umbraVisible ? _umbraMaxExtent : 0.0,
        _penumbraVisible ? _penumbraMaxExtent : 0.0);
}

/*! Return true if the reference mark contains no translucent geometry.
 *  The default implementation always returns true (i.e. completely
 *  opaque geometry is assumed).
 */
bool
Umbra::isOpaque() const
{
    return _umbraColor.alpha() > 0.95f && _penumbraColor.alpha() > 0.95f;
}
/*! Return true if the reference mark sticks to the surface of its body.
 *  This ensures that it gets rendered right after its body (which is
 *  required e.g. for marks making use of the stencil buffer).
 *  The default implementation always returns false.
 */
bool
Umbra::isSurfaceMark() const
{
    return false;
}

std::pair<double, double>
Umbra::getUmbraRange(double distance) const
{
    //
    double Rs = _body.getSystem()->getStar()->getRadius();
    double Rp = _body.getRadius();

    // Distance Sun Planet
    double d = celestia::astro::kilometersToLightYears(distance);

    // Distance from planet P to the apex of the cone
    double d1 = celestia::astro::lightYearsToKilometers(d * (Rp / (Rs - Rp)));

    // Height of the cone considering its base is tangent to the planet
    double h = d1 - Rp * (Rp / d1);

    // Distance planet center to cone base center
    double d2 = d1 - h;

    return { d2, d1 };
}

std::pair<double, double>
Umbra::getPenumbraRange(double distance) const
{
    //
    double Rs = _body.getSystem()->getStar()->getRadius();
    double Rp = _body.getRadius();

    // Distance Sun Planet
    double d = celestia::astro::kilometersToLightYears(distance);

    // Distance from planet P to the apex of the cone
    double d1 = celestia::astro::lightYearsToKilometers(d * (1 - Rs / (Rs + Rp)));

    // Height of the cone considering its base is tangent to the planet
    double h = d1 - Rp * (Rp / d1);

    // Distance planet center to cone base center
    double d2 = d1 - h;

    return { d2, h };
}

typedef std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, double, Eigen::Vector3d> EllipseParam;
EllipseParam
getPlaneEllipsoidIntersection(const Eigen::Vector3d &normal, double distance, const Eigen::Vector3d &semiAxes)
{
    Eigen::Vector3d m      = normal.cwiseProduct(semiAxes);
    double   length = m.norm();
    m.normalize();
    double delta = distance / length;
    double rho   = sqrt(1 - delta * delta);

    Eigen::Vector3d c = m * delta;

    Eigen::Vector3d s, t;
    if (abs(1 - abs(c[2])) < 1e-5)
    {
        s = { 1, 0, 0 };
        t = { 0, 1, 0 };
    }
    else
    {
        double length = sqrt(m[0] * m[0] + m[1] * m[1]);
        s             = { -m[1], m[0], 0 };
        s.normalize();

        t = m.cross(s);
    }

    return { c, s, t, rho, semiAxes };
}

Eigen::Vector3d
plotEllipse(const EllipseParam &params, double scale, double angle)
{
    auto &c   = std::get<0>(params);
    auto &s   = std::get<1>(params);
    auto &t   = std::get<2>(params);
    auto &rho = std::get<3>(params);
    auto &abc = std::get<4>(params);

    return (c + (s * cos(angle) + t * sin(angle)) * rho * scale).cwiseProduct(abc);
}
void
Umbra::drawUmbraCone(
    double          tdb,
    double          centerDistance,
    double          apexDistance,
    const Eigen::Vector3d &lightDir,
    double          discSizeInPixels) const
{
    Eigen::Vector3d n   = _body.getEclipticToBodyFixed(tdb) * lightDir;
    Eigen::Vector3d abc = _body.getSemiAxes().cast<double>();

    auto ellipseParam = getPlaneEllipsoidIntersection(n, centerDistance, abc);

    double scale = (discSizeInPixels + 3) / discSizeInPixels;
    scale        = std::max(scale, 1.0001);

    Eigen::Vector3d apex = n * apexDistance;

    if (apexDistance > _umbraMaxExtent)
    {
        GLdouble planeEquation[] = { -n[0], -n[1], -n[2], _umbraMaxExtent };
        glClipPlane(GL_CLIP_PLANE0, planeEquation);
        glEnable(GL_CLIP_PLANE0);
    }

    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(apex[0], apex[1], apex[2]);
    const size_t maxPixelError = 1;
    const size_t count         = std::min(
        static_cast<unsigned int>(
            ceil(2 * celestia::numbers::pi / acos(1 - maxPixelError / (discSizeInPixels * 0.5)))),
        1024U);
    for (size_t i = 0; i <= count; ++i)
    {
        const double r = i * 2.0 * celestia::numbers::pi / count;
        Eigen::Vector3d     p = plotEllipse(ellipseParam, scale, r);
        glVertex3f(p[0], p[1], p[2]);
    }

    glEnd();

    if (apexDistance > _umbraMaxExtent)
    {
        glDisable(GL_CLIP_PLANE0);
    }
}

void
Umbra::drawPenumbraCone(
    double          tdb,
    double          centerDistance,
    double          apexDistance,
    const Eigen::Vector3d &lightDir,
    double          discSizeInPixels) const
{
    Eigen::Vector3d n   = _body.getEclipticToBodyFixed(tdb) * lightDir;
    n            = -n;
    Eigen::Vector3d abc = _body.getSemiAxes().cast<double>();

    auto ellipseParam = getPlaneEllipsoidIntersection(n, centerDistance, abc);

    double scale = (discSizeInPixels + 5) / discSizeInPixels;
    scale        = std::max(scale, 1.0002);

    glBegin(GL_TRIANGLE_STRIP);

    Eigen::Vector3d apex = n * apexDistance;

    const size_t maxPixelError = 1;
    const size_t count         = std::min(
        static_cast<unsigned int>(
            ceil(2 * celestia::numbers::pi / acos(1 - maxPixelError / (discSizeInPixels * 0.5)))),
        1024U);
    for (size_t i = 0; i <= count; ++i)
    {
        const double r = i * 2.0 * celestia::numbers::pi / count;
        Eigen::Vector3d     p = plotEllipse(ellipseParam, scale, r);
        Eigen::Vector3d     p1
            = apex
              + (p - apex) * (apexDistance + _penumbraMaxExtent) / (apexDistance - centerDistance);
        glVertex3f(p[0], p[1], p[2]);
        glVertex3f(p1[0], p1[1], p1[2]);
    }

    glEnd();
}

void
Umbra::makeTranslucent(bool isTranslucent) const
{
    if (isOpaque())
    {
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
    else
    {
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
}