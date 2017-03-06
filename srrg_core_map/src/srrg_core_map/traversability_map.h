#pragma once
#include <iostream>
#include <srrg_types/defs.h>
#include <srrg_boss/blob.h>

namespace srrg_core_map {
struct TraversabilityMap: public srrg_boss::BLOB{
    TraversabilityMap(const srrg_core::UnsignedCharImage& img = srrg_core::UnsignedCharImage::zeros(0,0)) { img.copyTo(_img);}
    //! saves the cloud in a binary stream, optimized for speed
    virtual void write(std::ostream& os) const;
    //! loads the cloud from a binary stream, optimized for speed
    virtual bool read(std::istream& is);
    inline const srrg_core::UnsignedCharImage& image() const { return _img;}
    srrg_core::UnsignedCharImage _img;
};
typedef srrg_boss::BLOBReference<TraversabilityMap> TraversabilityMapBLOBReference;
}
