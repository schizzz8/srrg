#include "frequency_map.h"
#include "grid_line_traversal.h"

using namespace srrg_core;

FrequencyMapCell::FrequencyMapCell(int hits_, int misses_) {
    _hits = hits_;
    _misses = misses_;
}

FrequencyMap::FrequencyMap() : Eigen::Matrix<FrequencyMapCell, 
                               Eigen::Dynamic,
                               Eigen::Dynamic>(0, 0) {
    _offset = Eigen::Vector2f::Zero();
    _resolution = 0.05f;
}

FrequencyMap::FrequencyMap(float resolution_, 
                           Eigen::Vector2f &offset_,
                           Eigen::Vector2i &size,
                           FrequencyMapCell &unknown) : Eigen::Matrix<FrequencyMapCell,
                                                        Eigen::Dynamic,
                                                        Eigen::Dynamic>(size.x(), size.y()) {
    _offset = offset_;
    _resolution = resolution_;
    this->fill(unknown);
}

void FrequencyMap::integrateScan(srrg_core::LaserMessage *laser, const Eigen::Isometry3f &robotPose, 
                                 double maxRange, double usableRange, int gain, int squareSize) {
    if(maxRange < 0) { maxRange = laser->maxRange(); }
    if(usableRange < 0) { usableRange = maxRange; }

    Eigen::Isometry3f laserCenter = robotPose * laser->offset();
    Eigen::Vector2f rp = Eigen::Vector2f((float) laserCenter.translation().x(), (float) laserCenter.translation().y());

    Eigen::Vector2i start = world2map(rp);
    for(size_t i = 0; i < laser->ranges().size(); i++) {
        float r = (float) laser->ranges()[i];
        if(r >= maxRange) { continue; }
        bool cropped = false;
        if(r > usableRange) {
            r = usableRange;
            cropped = true;
        }
        static GridLineTraversalLine line;
        Eigen::Vector3f base_point3d(r * cosf(laser->minAngle() + i * laser->angleIncrement()),
                                     r * sinf(laser->minAngle() + i * laser->angleIncrement()),1);
        base_point3d = laserCenter * base_point3d;
        Eigen::Vector2f bp (base_point3d.x(),base_point3d.y());
        Eigen::Vector2i end = world2map(bp.cast<float>());

        GridLineTraversal::gridLine(start, end, &line);
        for(int i = 0; i < line.numPoints; i++) {
            if(isInside(line.points[i])) { this->coeffRef(line.points[i].x(), line.points[i].y()).incrementMisses(1); }
        }
        if(!isInside(end)) { continue; }
        if(!cropped) {
            for(int c = -squareSize; c <= squareSize; c++) {
                for(int r = -squareSize; r <= squareSize; r++) {
                    Eigen::Vector2i subCell(end.x() + r, end.y() + c);
                    if(isInside(subCell)) {
                        this->coeffRef(subCell.x(), subCell.y()).incrementHits(gain);
                    }
                }
            }
        }
    }
}

void FrequencyMap::applyGain(int gain) {
    for(int c = 0; c < cols(); c++) {
        for (int r = 0; r < rows(); r++) {
            FrequencyMapCell &cell = this->coeffRef(r, c);
            int h = cell.hits() * (gain - 1);
            cell.incrementHits(h);
            cell.incrementMisses(h);
        }
    }
}
