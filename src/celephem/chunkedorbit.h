#pragma once

#include "orbit.h"
#include "samporbit.h"
#include <Eigen/Core>

#include <array>

/*!
 * @class ChunkedOrbit
 *
 * New orbit which will import ephemerids from a file.
 * The file shall be a *.cbxyz file type.
 * *.cbxyz is a binary little-endian encoded file. Each entry in the file consists
 * in 4 64t bit floating point number :
 * - timestamp
 * - position.x
 * - position.y
 * - position.z
 *
 *
 * @note /!\ The file handle will stay open after the 1st request for a position
 * and this until the end
 *
 * @note the code is exactly the same as the Sampled Orbit
 */
class ChunkedOrbit : public celestia::ephem::CachingOrbit
{
public:
    // A chunk of the file : ie a position entry
    // It also implement an RamdomAccessIterator
    // To be used in the computePosition method
    class Chunk
    {
    public:
        Chunk();
        Chunk(std::istream &stream, size_t id);
        double       date() const;
        double       x() const;
        double       y() const;
        double       z() const;
        bool         isValid() const;
        size_t       id() const;
        Chunk       &operator+=(size_t value);
        bool         operator==(const Chunk &other) const;
        bool         operator!=(const Chunk &other) const;
        bool         operator<(double tjd) const;
        bool         operator<(const Chunk &other) const;
        size_t       operator-(const Chunk &other) const;
        const Chunk &operator*() const;
        Chunk       &operator++();
        Chunk       &operator=(const Chunk &other);

    private:
        mutable bool                  _loaded;
        size_t                        _id;
        mutable std::array<double, 4> _data;
        mutable std::istream         *_stream;

        void load() const;
    };

    ChunkedOrbit(
        const std::string                        filename,
        celestia::ephem::TrajectoryInterpolation interpolation);
    ~ChunkedOrbit() override;

    virtual Eigen::Vector3d computePosition(double tjd) const override;
    virtual bool            isPeriodic() const override;
    virtual double          getPeriod() const override;
    virtual double          getBoundingRadius() const override;
    virtual void            getValidRange(double &begin, double &end) const override;
    virtual void
    sample(double startTime, double endTime, celestia::ephem::OrbitSampleProc &proc) const override;

    std::unique_ptr<Orbit> createSampledOrbit() const;

private:
    celestia::ephem::TrajectoryInterpolation _interpolation;

    std::string           _xyzfilename;
    mutable std::istream *_ifs;
    double                _begin, _end;
    size_t                _count;

    // Chunk #N
    mutable Chunk _chunkN;
    // Chunk #N-1
    mutable Chunk _chunkN_1;
    // Chunk #N-2
    mutable Chunk _chunkN_2;
    // Chunk #N+1
    mutable Chunk _chunkN1;
};