#include "billboardgeometry.h"

#include "texmanager.h"
#include "render.h"
#include "celmath/ray.h"
#include "celmath/intersect.h"
#include "body.h"


BillboardGeometry::BillboardGeometry(Body &body, size_t symbolSize, ResourceHandle textureHandle)
    : Geometry()
    , _body(body)
    , _symbolSize(symbolSize)
    , _textureHandle(textureHandle)
    , _lastPixelSize()
{
}
//! Render the geometry in the specified OpenGL context
void BillboardGeometry::render(RenderContext& rc, double t)
{
    // Compute the pixel size at a given distance from the observer
    // Used later during a picking
    _lastPixelSize = 2*_body.getRadius() / rc.getPointScale();


    //load the texture
    auto symbol = GetTextureManager()->find(_textureHandle);


    // Bind the texture and quad rendering
    if (symbol)
    {
        glEnable(GL_POINT_SPRITE_ARB);
        glActiveTextureARB(GL_TEXTURE0_ARB);
        glEnable(GL_TEXTURE_2D);
        symbol->bind();
        glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);


        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    // Render the quad
    glPointSize(_symbolSize);
    glBegin(GL_POINTS);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glVertex3f(0.f, 0.f, 0.f);
    glEnd();
    glPointSize(1.f);

    // Unbind the texture
    if (symbol)
    {
        glDisable(GL_BLEND);
        glDisable(GL_POINT_SPRITE_ARB);
        glActiveTextureARB(GL_TEXTURE0_ARB);
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);
    }

}

/*! Find the closest intersection between the ray and the
*  model.  If the ray intersects the model, return true
*  and set distance; otherwise return false and leave
*  distance unmodified.
 */
bool BillboardGeometry::pick(const Eigen::ParametrizedLine<double, 3>& r, double& distance) const
{
    // Rough appoximation of the sprite as a sphere
    double physSpriteRadius = r.origin().norm() * _lastPixelSize * _symbolSize;
    return testIntersection(r, celmath::Sphered(physSpriteRadius), distance);
}

bool BillboardGeometry::isOpaque() const
{
    return false;
}

bool BillboardGeometry::isMultidraw() const
{
    return false;
}

//void BillboardGeometry::setSymbol(const std::string& path)
//{
////    auto currentInfo = GetTextureManager()->getResourceInfo(_textureHandle);
////    auto currentInfo = GetTextureManager()->find(_textureHandle);
//    auto texmang = GetTextureManager();
//
//    std::string base_dir;
//    if (texmang)
//    {
//        base_dir = texmang->path;
//
//    }
//    _textureHandle = GetTextureManager()->getHandle(TextureInfo(path, base_dir, 0U));
//
//}
void BillboardGeometry::setSymbol(ResourceHandle textureHandle)
{
    _textureHandle = textureHandle;
}
void BillboardGeometry::setSymbolSize(size_t size)
{
    _symbolSize = size;
}

size_t BillboardGeometry::getSymbolSize() const
{
    return _symbolSize;
}