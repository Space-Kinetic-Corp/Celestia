#include "cluster.h"

#include <celengine/billboardgeometry.h>
#include <celengine/meshmanager.h>
#include <celengine/texmanager.h>
#include <celengine/trajmanager.h>
#include <celephem/chunkedorbit.h>
#include <celutil/resmanager.h>
#include <fstream>
#include <vector>
// #include <celutil/debug.h>
#include <celengine/frametree.h>
#include <celengine/render.h>
#include <celengine/timelinephase.h>

// Catalog resource
struct Catalog
{
    struct Object
    {
        Object(Object &&other) = default;

        Object(std::string &&name, std::string &&trajectory) : name(name), trajectory(trajectory)
        {
        }

        std::string name;
        std::string trajectory;
        celestia::ephem::Orbit      *orbit;
    };
    std::vector<Object> objects;
};

namespace
{
// Catalog resource info
struct CatalogInfo
{
    //    typedef Catalog ResourceType;
    using ResourceType = Catalog;
    using ResourceKey  = std::string;

    CatalogInfo(const std::string &path, const std::string &source) : _filename(source), _path(path)
    {
    }

    ~CatalogInfo()
    {
    }

    bool operator<(const CatalogInfo &other) const
    {
        return _filename < other._filename;
    }
    std::string resolve(const std::filesystem::path &baseDir)
    {
        std::string   filename = _path + "/" + baseDir.string() + "/" + _filename;
        std::ifstream ifs;
        ifs.open(filename, std::ios_base::in);

        return ifs.good() ? filename : "";
    }
    std::unique_ptr<Catalog> load(const std::string &filename)
    {
        std::ifstream ifs;
        ifs.open(filename, std::ios_base::in);

        if (!ifs.good())
        {
            return nullptr;
        }

        auto        catalog = std::make_unique<Catalog>();
        std::string line;
        while (ifs.good())
        {
            std::getline(ifs, line);

            auto sep = line.find_first_of('\t');
            if (sep == std::string::npos)
            {
                continue;
            }
            auto name         = line.substr(0, sep);
            auto sampledOrbit = line.substr(sep + 1);
            catalog->objects.emplace_back(std::move(name), std::move(sampledOrbit));
        }

        return catalog;
    }

    std::string _filename;
    std::string _path;
};

// Catalog manager
typedef ResourceManager<CatalogInfo> CatalogManager;

// Catalog manager singleton
CatalogManager &
catalogMgr()
{
    static CatalogManager mgr("data");
    return mgr;
}
} // namespace

Cluster::Cluster(
    Universe          &u,
    Body              &body,
    const std::string &path,
    const std::string &catalog,
    double             objectRadius,
    const std::string &symbol,
    size_t             symbolSize) :
    ReferenceMark(),
    _body(body), _cat(), _path(path)
{
    // Query a handle for the texture
    auto textureHandle = GetTextureManager()->getHandle(TextureInfo(symbol, path, 0U, 1U));

    // Load the catalog
    auto catalogHandle = catalogMgr().getHandle(CatalogInfo(path, catalog));
    _cat               = catalogMgr().find(catalogHandle);

    // Make the cluster object not pickable
    _body.setClickable(false);

    // Prepare the body to host the children
    auto system = _body.getSatellites();
    if (system == nullptr)
    {
        //        system = new PlanetarySystem(&_body);
        auto systemPtr = std::make_unique<PlanetarySystem>(&_body);
        _body.setSatellites(std::move(systemPtr));
    }

    for (auto &&obj : _cat->objects)
    {
        // Creating the body
        auto elt = new Body(system, obj.name);
        elt->setClassification(Body::ClusterObject);
        elt->setSemiAxes(Eigen::Vector3f(objectRadius, objectRadius, objectRadius));
        elt->setVisibleAsPoint(false);
        elt->setSecondaryIlluminator(false);

        // Loading the orbit
        auto trajHandle = GetTrajectoryManager()->getHandle(TrajectoryInfo(obj.trajectory, path));
//        auto orbit      = GetTrajectoryManager()->find(trajHandle);
        obj.orbit       = GetTrajectoryManager()->find(trajHandle);
        //        obj.orbit = (Orbit*)orbit;

        // Configuring the orbit for the element
        double cbegin, cend;
        obj.orbit->getValidRange(cbegin, cend);
        Selection center(&_body);
        Selection frame(&_body);
//        auto      orbitFrame = new J2000EquatorFrame(center);
        auto orbitFrame = std::make_shared<J2000EquatorFrame>(center);
        // don't need to addRefs as they are managed by smart pointers
//        orbitFrame->addRef();
//        auto bodyFrame = new J2000EquatorFrame(center);
        auto bodyFrame = std::make_shared<J2000EquatorFrame>(center);
//        bodyFrame->addRef();
        auto rotationModel
            = new celestia::ephem::ConstantOrientation(Eigen::Quaterniond::Identity());
        auto phase = TimelinePhase::CreateTimelinePhase(
            u,
            elt,
            cbegin,
            cend,
            orbitFrame,
            *obj.orbit,
            bodyFrame,
            *rotationModel);
//        bodyFrame->release();
//        orbitFrame->release();

        if (phase)
        {
//            auto timeline = new Timeline();
            auto timeline = std::make_unique<Timeline>();
            timeline->appendPhase(phase);
            elt->setTimeline(std::move(timeline));
        }

        // Configuring the orbit path of the element
        elt->setPlotDuration(body.getPlotDuration());
        elt->setPlotFade(body.getPlotFade());
        elt->setPlotLead(body.getPlotLead());
        elt->setPlotPattern(body.getPlotPattern());
        elt->setPlotWidth(body.getPlotWidth());
        elt->setOrbitColor(body.getOrbitColor());
        elt->setOrbitColorOverridden(true);
        elt->setOrbitVisibility(Body::NeverVisible);

        // Assigning the gerometry to the element
        GeometryInfo info(obj.name + "_billboard", path, Eigen::Vector3f::Zero(), 1.0f, false);
        info.resource                 = new BillboardGeometry(*elt, symbolSize, textureHandle);
        info.state                    = ResourceState::Loaded;
        ResourceHandle geometryHandle = GetGeometryManager()->getHandle(info);
        elt->setGeometry(geometryHandle);

        // Always visible
        elt->setMinPixelSize(1e-5);
    }
}

Cluster::~Cluster()
{
}

void
Cluster::render(
    Renderer              *renderer,
    const Eigen::Vector3f &position,
    float                  discSizeInPixels,
    double                 tdb,
    const Matrices        &m) const
{
    return;
}

float
Cluster::boundingSphereRadius() const
{
    return _body.getRadius();
}

bool
Cluster::isOpaque() const
{
    return true;
}

bool
Cluster::isSurfaceMark() const
{
    return false;
}

void
Cluster::setSymbol(const std::string path)
{
    auto textureHandle = GetTextureManager()->getHandle(TextureInfo(path, _path, 0U));

    auto subTree = _body.getFrameTree();
    auto count   = subTree->childCount();

    for (size_t i = 0; i < count; ++i)
    {
        auto child          = subTree->getChild(i)->body();
        auto geometryHandle = child->getGeometry();
        auto geometry       = GetGeometryManager()->find(geometryHandle);
        auto billboard      = dynamic_cast<BillboardGeometry *>(geometry);
        if (billboard)
        {
            billboard->setSymbol(textureHandle);
        }
    }
}

void
Cluster::setSymbolSize(size_t size)
{
    auto subTree = _body.getFrameTree();
    auto count   = subTree->childCount();

    for (size_t i = 0; i < count; ++i)
    {
        auto child          = subTree->getChild(i)->body();
        auto geometryHandle = child->getGeometry();
        auto geometry       = GetGeometryManager()->find(geometryHandle);
        auto billboard      = dynamic_cast<BillboardGeometry *>(geometry);
        if (billboard)
        {
            billboard->setSymbolSize(size);
        }
    }
}

void
Cluster::resetObjectVisibility(bool visible)
{
    auto subTree = _body.getFrameTree();
    auto count   = subTree->childCount();

    for (size_t i = 0; i < count; ++i)
    {
        auto child = subTree->getChild(i)->body();
        child->setVisible(visible);
    }
}