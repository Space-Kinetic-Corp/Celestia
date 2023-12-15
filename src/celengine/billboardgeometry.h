#ifndef CELESTIA_BILLBOARDGEOMETRY_H
#define CELESTIA_BILLBOARDGEOMETRY_H

#include <celengine/geometry.h>
#include <celutil/reshandle.h>

class Body;
class Texture;

/*!
 * @class BillboardGeometry
 *
 * Geometry dedicated to render a body as a screen space quad as a texture
 * The displayed texture is expected to be square.
 */
class BillboardGeometry : public Geometry
{
public:
    BillboardGeometry(Body &body, size_t symbolSize, ResourceHandle textureHandle);
    virtual ~BillboardGeometry(){};

    //! Render the geometry in the specified OpenGL context
    void render(RenderContext &rc, double t = 0.0) override;

    /*! Find the closest intersection between the ray and the
     *  model.  If the ray intersects the model, return true
     *  and set distance; otherwise return false and leave
     *  distance unmodified.
     */
    virtual bool pick(const Eigen::ParametrizedLine<double, 3>& r, double &distance) const override;

    //! Never opaque
    bool isOpaque() const override;

    //! No multidraw
    bool isMultidraw() const override;

    // Configure the rendered quad
//    void   setSymbol(const std::string& path);
    void   setSymbol(ResourceHandle textureHandle);
    void   setSymbolSize(size_t size);
    size_t getSymbolSize() const;

private:
    Body          &_body;
    ResourceHandle _textureHandle;
    size_t         _symbolSize;
    double         _lastPixelSize;
};
#endif // CELESTIA_BILLBOARDGEOMETRY_H
