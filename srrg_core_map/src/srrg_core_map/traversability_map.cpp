#include "traversability_map.h"
#include <string>

namespace srrg_core_map {

using namespace srrg_boss;
using namespace std;

void TraversabilityMap::write(ostream &os) const{
    // Header
    int type_ = _img.type();
    int channels_ = _img.channels();
    os.write((char*)&_img.rows, sizeof(int));    // rows
    os.write((char*)&_img.cols, sizeof(int));    // cols
    os.write((char*)&type_, sizeof(int));        // type
    os.write((char*)&channels_, sizeof(int));    // channels
    // Data
    if (_img.isContinuous())
        os.write(_img.ptr<char>(0), (_img.dataend - _img.datastart));
    else {
        int rowsz = CV_ELEM_SIZE(type_) * _img.cols;
        for (int r = 0; r < _img.rows; ++r)
            os.write(_img.ptr<char>(r), rowsz);
    }
}

bool TraversabilityMap::read(istream &is){
    // Header
    int rows, cols, type, channels;
    is.read((char*)&rows, sizeof(int));         // rows
    is.read((char*)&cols, sizeof(int));         // cols
    is.read((char*)&type, sizeof(int));         // type
    is.read((char*)&channels, sizeof(int));     // channels
    // Data
    _img.create(rows, cols);
    is.read((char*)_img.data, CV_ELEM_SIZE(type) * rows * cols);
    return true;
}

BOSS_REGISTER_BLOB(TraversabilityMap);

}
