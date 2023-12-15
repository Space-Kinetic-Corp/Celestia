#include "chunkedorbit.h"

#include <array>
#include <celmath/mathlib.h>
#include <fstream>

template<>
struct std::iterator_traits<ChunkedOrbit::Chunk>
{
    typedef size_t                          difference_type;
    typedef ChunkedOrbit::Chunk             value_type;
    typedef ChunkedOrbit::Chunk            *pointer;
    typedef ChunkedOrbit::Chunk            &reference;
    typedef std::random_access_iterator_tag iterator_category;
};

// Defined in samporbit.cpp
// extern Eigen::Vector3d celestia::ephem::cubicInterpolate(
//    const Eigen::Vector3d &p0,
//    const Eigen::Vector3d &v0,
//    const Eigen::Vector3d &p1,
//    const Eigen::Vector3d &v1,
//    double                 t);
//
//// Defined in samporbit.cpp
// extern Eigen::Vector3d celestia::ephem::cubicInterpolateVelocity(
//     const Eigen::Vector3d &p0,
//     const Eigen::Vector3d &v0,
//     const Eigen::Vector3d &p1,
//     const Eigen::Vector3d &v1,
//     double                 t);

Eigen::Vector3d
cubicInterpolate(
    const Eigen::Vector3d &p0,
    const Eigen::Vector3d &v0,
    const Eigen::Vector3d &p1,
    const Eigen::Vector3d &v1,
    double                 t)
{
    return p0
           + (((2.0 * (p0 - p1) + v1 + v0) * (t * t * t))
              + ((3.0 * (p1 - p0) - 2.0 * v0 - v1) * (t * t)) + (v0 * t));
}

ChunkedOrbit::Chunk::Chunk() : _stream()
{
}

ChunkedOrbit::Chunk::Chunk(std::istream &stream, size_t id) :
    _loaded(false), _id(id), _stream(&stream)
{
}

double
ChunkedOrbit::Chunk::date() const
{
    load();
    return _data[0];
}
double
ChunkedOrbit::Chunk::x() const
{
    load();
    return _data[1];
}
double
ChunkedOrbit::Chunk::y() const
{
    load();
    return _data[2];
}
double
ChunkedOrbit::Chunk::z() const
{
    load();
    return _data[3];
}

size_t
ChunkedOrbit::Chunk::id() const
{
    return _id;
}

bool
ChunkedOrbit::Chunk::isValid() const
{
    return _stream && _stream->good();
}
bool
ChunkedOrbit::Chunk::operator==(const Chunk &other) const
{
    return _id == other._id;
}
bool
ChunkedOrbit::Chunk::operator!=(const Chunk &other) const
{
    return !(other == *this);
}
bool
ChunkedOrbit::Chunk::operator<(double tjd) const
{
    return date() < tjd;
}

bool
ChunkedOrbit::Chunk::operator<(const Chunk &other) const
{
    return id() < other.id();
}
ChunkedOrbit::Chunk &
ChunkedOrbit::Chunk::operator++()
{
    ++_id;
    _loaded = false;
    return *this;
}
ChunkedOrbit::Chunk &
ChunkedOrbit::Chunk::operator=(const Chunk &other)
{
    if (other != *this || !_loaded)
    {
        _loaded = other._loaded;
        _id     = other._id;
        _stream = other._stream;
        if (_loaded)
        {
            _data = other._data;
        }
    }
    return *this;
}

ChunkedOrbit::Chunk &
ChunkedOrbit::Chunk::operator+=(size_t value)
{
    _id += value;
    _loaded = false;
    return *this;
}

size_t
ChunkedOrbit::Chunk::operator-(const Chunk &other) const
{
    return _id - other._id;
}
const ChunkedOrbit::Chunk &
ChunkedOrbit::Chunk::operator*() const
{
    return *this;
}

void
ChunkedOrbit::Chunk::load() const
{
    if (!_loaded)
    {
        _stream->seekg(_id * 32, std::ios_base::beg);
        _stream->read(reinterpret_cast<char *>(_data.data()), 32);
        _loaded = true;
    }
}

ChunkedOrbit::ChunkedOrbit(
    const std::string                        filename,
    celestia::ephem::TrajectoryInterpolation interpolation) :
    CachingOrbit(),
    _interpolation(interpolation), _xyzfilename(filename), _begin(-1.0), _end(-1.0), _count(),
    _ifs()
{
    std::ifstream ifs;
    ifs.open(filename, std::ios_base::in | std::ios_base::binary);

    if (ifs.good())
    {
        ifs.seekg(0, std::ios_base::end);
        _count = ifs.tellg() / 32;

        _begin = Chunk(ifs, 0).date();
        _end   = Chunk(ifs, _count - 1).date();
    }
}

ChunkedOrbit::~ChunkedOrbit()
{
    delete _ifs;
}

Eigen::Vector3d
ChunkedOrbit::computePosition(double tjd) const
{
    if (!_ifs)
    {
        _ifs = new std::ifstream(_xyzfilename, std::ios_base::in | std::ios_base::binary);
    }
    Eigen::Vector3d pos;
    if (_count == 0)
    {
        pos = Eigen::Vector3d::Zero();
    }
    else if (_count == 1)
    {
        Chunk first(*_ifs, 0);
        pos = Eigen::Vector3d(first.x(), first.y(), first.z());
    }
    else
    {
        Chunk first(*_ifs, 0);
        Chunk last(*_ifs, _count);
        if (!_chunkN.isValid() || tjd < _chunkN_1.date() || tjd > _chunkN.date())
        {
            auto entry = std::lower_bound(first, last, tjd);
            if (entry == last)
            {
                _chunkN_2 = Chunk(*_ifs, _count - 3);
                _chunkN_1 = Chunk(*_ifs, _count - 2);
                _chunkN   = Chunk(*_ifs, _count - 1);
                _chunkN1  = Chunk(*_ifs, _count - 1);
            }
            else
            {
                _chunkN   = entry;
                _chunkN_1 = Chunk(*_ifs, std::max(0, static_cast<int>(_chunkN.id()) - 1));
                _chunkN_2 = Chunk(*_ifs, std::max(0, static_cast<int>(_chunkN.id()) - 2));
                _chunkN1  = Chunk(*_ifs, std::min(_count - 1, _chunkN.id() + 1));
            }
        }

        if (_chunkN == first)
        {
            pos = Eigen::Vector3d(_chunkN.x(), _chunkN.y(), _chunkN.z());
        }
        else if (_chunkN != last)
        {
            if (_interpolation == celestia::ephem::TrajectoryInterpolation::Linear)
            {
                Chunk &s0 = _chunkN_1;
                Chunk &s1 = _chunkN;

                double t = (tjd - s0.date()) / (s1.date() - s0.date());
                pos      = Eigen::Vector3d(
                    celmath::lerp(t, (double)s0.x(), (double)s1.x()),
                    celmath::lerp(t, (double)s0.y(), (double)s1.y()),
                    celmath::lerp(t, (double)s0.z(), (double)s1.z()));
            }
            else if (_interpolation == celestia::ephem::TrajectoryInterpolation::Cubic)
            {
                Chunk &s0 = _chunkN_2;
                Chunk &s1 = _chunkN_1;
                Chunk &s2 = _chunkN;
                Chunk &s3 = _chunkN1;

                double          h  = s2.date() - s1.date();
                double          ih = 1.0 / h;
                double          t  = (tjd - s1.date()) * ih;
                Eigen::Vector3d p0(s1.x(), s1.y(), s1.z());
                Eigen::Vector3d p1(s2.x(), s2.y(), s2.z());

                Eigen::Vector3d v10(
                    (double)s1.x() - (double)s0.x(),
                    (double)s1.y() - (double)s0.y(),
                    (double)s1.z() - (double)s0.z());
                Eigen::Vector3d v21(
                    (double)s2.x() - (double)s1.x(),
                    (double)s2.y() - (double)s1.y(),
                    (double)s2.z() - (double)s1.z());
                Eigen::Vector3d v32(
                    (double)s3.x() - (double)s2.x(),
                    (double)s3.y() - (double)s2.y(),
                    (double)s3.z() - (double)s2.z());

                // Estimate velocities by averaging the differences at adjacent spans
                // (except at the end spans, where we just use a single velocity.)
                Eigen::Vector3d v0;
                if (_chunkN.id() > 1)
                {
                    v0 = v10 * (0.5 / (s1.date() - s0.date())) + v21 * (0.5 * ih);
                    v0 *= h;
                }
                else
                {
                    v0 = v21;
                }

                Eigen::Vector3d v1;
                if (_chunkN.id() < (_count - 1))
                {
                    v1 = v21 * (0.5 * ih) + v32 * (0.5 / (s3.date() - s2.date()));
                    v1 *= h;
                }
                else
                {
                    v1 = v21;
                }

                pos = cubicInterpolate(p0, v0, p1, v1, t);
            }
            else
            {
                // Unknown interpolation type
                pos = Eigen::Vector3d::Zero();
            }
        }
        else
        {
            pos = Eigen::Vector3d(_chunkN_1.x(), _chunkN_1.y(), _chunkN_1.z());
        }
    }

    // Add correction for Celestia's coordinate system
    return { pos.x(), pos.z(), -pos.y() };
}
bool
ChunkedOrbit::isPeriodic() const
{
    return false;
}
double
ChunkedOrbit::getPeriod() const
{
    return _end - _begin;
}
double
ChunkedOrbit::getBoundingRadius() const
{
    return 8000;
}
void
ChunkedOrbit::getValidRange(double &begin, double &end) const
{
    begin = _begin;
    end   = _end;
}

std::unique_ptr<ChunkedOrbit::Orbit>
ChunkedOrbit::createSampledOrbit() const
{
    auto xyz = _xyzfilename;
    xyz.replace(xyz.find_last_of("."), std::string::npos, ".xyz");

    {
        std::ofstream ofs;
        ofs.open(xyz);
        ofs.precision(20);

        std::ifstream ifs;
        ifs.open(_xyzfilename, std::ios_base::in | std::ios_base::binary);

        for (size_t i = 0; i < _count; ++i)
        {
            Chunk c(ifs, i);

            ofs << c.date() << ' ' << c.x() << ' ' << c.y() << ' ' << c.z() << '\n';
        }
    }

    return LoadSampledTrajectoryDoublePrec(xyz, "", _interpolation);
}

void
ChunkedOrbit::sample(double startTime, double endTime, celestia::ephem::OrbitSampleProc &proc) const
{
    auto sampledOrbit = createSampledOrbit();
    sampledOrbit->sample(startTime, endTime, proc);
    //    delete sampledOrbit;
}