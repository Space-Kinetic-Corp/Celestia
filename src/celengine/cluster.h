#ifndef CELESTIA_CLUSTER_H
#define CELESTIA_CLUSTER_H

#include <celengine/referencemark.h>
#include <celengine/selection.h>
#include <celmath/frustum.h>
// maybes
#include <celengine/body.h>
#include <celephem/orbit.h>

//class Orbit;
class Body;
struct Catalog;
class Universe;

/*!
 * @class Cluster
 * The notion of refernce mark is here totally ignored
 * Cluster shoud be considered as help class which populates the body it is attached to.
 * A catalog is used to populate the cluster of body. The catalog is a text file. Each line
 * is a catalog entry which consists of:
 * - an object name
 * - an orbit file
 * separated by ONE tab.
 * With in the catalog, all elements have the same radius.
 *
 */
class Cluster : public ReferenceMark
{
public:
    Cluster(
        Universe          &u,
        Body              &body,
        const std::string &path,
        const std::string &catalog,
        double             objectRadius,
        const std::string &symbol,
        size_t             symbolSize);
    ~Cluster() override;

    /*! Draw the reference mark geometry at the specified time.
     *
     */
    void render(
        Renderer              *renderer,
        const Eigen::Vector3f &position,
        float                  discSizeInPixels,
        double                 tdb,
        const Matrices&         m) const override;

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

    void setSymbol(const std::string path);
    void setSymbolSize(size_t size);

    /*!
     * Resets the visibility of the object composing the cluster.
     */
    void resetObjectVisibility(bool visible);

private:
    Body       &_body;
    Catalog    *_cat;
    std::string _path;
};

#endif // CELESTIA_CLUSTER_H
