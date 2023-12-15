#pragma once

#include <celengine/referencemark.h>
#include <celengine/selection.h>
#include <celutil/color.h>


class Body;


class Umbra : public ReferenceMark
{
public:
    Umbra(Body &body);
    ~Umbra() override = default;

    void setUmbraVisible(bool isVisible);
    void setUmbraColor(const Color &c);
    void setUmbraMaxExtent(double extent);

    void setPenumbraVisible(bool isVisible);
    void setPenumbraColor(const Color &c);
    void setPenumbraMaxExtent(double extent);

    /*! Draw the reference mark geometry at the specified time.
	 *
     */
    void render(Renderer             * renderer,
                const Eigen::Vector3f& position,
                float                  discSizeInPixels,
                double                 tdb,
                const Matrices& m) const override;

    /*! Return the radius of a bounding sphere (in kilometers) large enough
	 *  to contain the reference mark geometry.
     */
    float boundingSphereRadius() const override;

    /*! Return true if the reference mark contains no translucent geometry.
	 *  The default implementation always returns true (i.e. completely
	 *  opaque geometry is assumed).
     */
    bool isOpaque() const override;

    /*! Return true if the reference mark sticks to the surface of its body.
	*  This ensures that it gets rendered right after its body (which is
	*  required e.g. for marks making use of the stencil buffer).
	*  The default implementation always returns false.
     */
    bool isSurfaceMark() const override;
protected:
    Body &_body;

private:
    bool _umbraVisible;
    double _umbraMaxExtent;
    Color _umbraColor;

    bool _penumbraVisible;
    double _penumbraMaxExtent;
    Color _penumbraColor;


    std::pair<double, double> getUmbraRange(double distance) const;
    std::pair<double, double>  getPenumbraRange(double distance) const;
    void drawUmbraCone(double tdb, double centerDistance, double apexDistance, const Eigen::Vector3d &lightDir, double discSizeInPixels) const;
    void drawPenumbraCone(double tdb, double centerDistance, double apexDistance, const Eigen::Vector3d &lightDir, double discSizeInPixels) const;


    void makeTranslucent(bool isTranslucent) const;
};