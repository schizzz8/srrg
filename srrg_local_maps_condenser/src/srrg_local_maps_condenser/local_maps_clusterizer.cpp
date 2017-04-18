#include "local_maps_clusterizer.h"

using namespace std;

namespace std {
template<>
bool std::less<Eigen::Vector2f>::operator ()(const Eigen::Vector2f& a,const Eigen::Vector2f& b) const {
    for(size_t i=0;i<2;++i) {
        if(a[i]<b[i]) return true;
        if(a[i]>b[i]) return false;
    }
    return false;
}
}

namespace srrg_local_maps_condenser {

using namespace srrg_core;
using namespace srrg_core_map;

void LocalMapsClusterizer::computeBoundingBox(MapNodeList* local_maps_){
    int n = 0;
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    Eigen::Vector2f max,min;
    float xmin=std::numeric_limits<float>::max();
    float xmax=std::numeric_limits<float>::min();
    float ymin=std::numeric_limits<float>::max();
    float ymax=std::numeric_limits<float>::min();
    for(MapNodeList::iterator it = local_maps_->begin(); it != local_maps_->end(); it++){
        LocalMap* lmap = dynamic_cast<LocalMap*> (it->get());
        Eigen::Vector2f point = Eigen::Vector2f (lmap->transform().translation().x(),
                                                 lmap->transform().translation().y());
        _points.push_back(point);
        _local_maps.insert(std::pair<Eigen::Vector2f,MapNode*>(point,it->get()));
        xmax = xmax > point.x()+_range ? xmax : point.x()+_range;
        ymax = ymax > point.y()+_range ? ymax : point.y()+_range;
        xmin = xmin < point.x()-_range ? xmin : point.x()-_range;
        ymin = ymin < point.y()-_range ? ymin : point.y()-_range;
        centroid += point;
        n++;
    }
    centroid /= n;
    min = Eigen::Vector2f (xmin,ymin);
    max = Eigen::Vector2f (xmax,ymax);

    float radius = (max-centroid).norm() + 1*_resolution;
    _min = Eigen::Vector2f(centroid.x()-radius,centroid.y()-radius);
    _max = Eigen::Vector2f(centroid.x()+radius,centroid.y()+radius);

    cerr << "Bounding Box: " << endl;
    cerr << "\t>>Lower: " << _min.transpose() << endl;
    cerr << "\t>>Upper: " << _max.transpose() << endl;

    _size = Eigen::Vector2i ((_max.x()-_min.x())/_resolution,
                             (_max.x()-_min.x())/_resolution);

    cerr << "Grid size: " << _size.x() << "x" << _size.y() << endl;

}

void LocalMapsClusterizer::visualize(QuadtreeNode *quadtree){
    if(quadtree){
        if(quadtree->depth() == 0){

            float xmin = quadtree->square().min().x();
            float ymin = quadtree->square().min().y();
            float xmax = quadtree->square().max().x();
            float ymax = quadtree->square().max().y();
            cv::rectangle(_image,
                          cv::Point ((xmin-_min.x())/_resolution,(ymin-_min.y())/_resolution),
                          cv::Point ((xmax-_min.x())/_resolution,(ymax-_min.y())/_resolution),
                          cv::Scalar (0,0,255));

            for(vector<Eigen::Vector2f>::const_iterator it = quadtree->points().begin();
                it != quadtree->points().end();
                it++){
                Eigen::Vector2f point = *it;
                cv::circle(_image,
                           cv::Point ((point.x()-_min.x())/_resolution,(point.y()-_min.y())/_resolution),
                           1,
                           cv::Scalar (255,0,0));
            }

        }
        visualize(quadtree->getChild1());
        visualize(quadtree->getChild2());
        visualize(quadtree->getChild3());
        visualize(quadtree->getChild4());
    } else {
        return;
    }
}

void LocalMapsClusterizer::visit(QuadtreeNode* quadtree){
    if(quadtree){
        if(quadtree->depth() == 0) {
            _count++;
            LocalMapWithTraversability* lmap = new LocalMapWithTraversability();
            bool first = true;
            for(vector<Eigen::Vector2f>::const_iterator it = quadtree->points().begin();
                it != quadtree->points().end();
                it++){
                Vector2fMapNodeMap::iterator iter = _local_maps.find(*it);
                if(iter != _local_maps.end()){
                    LocalMap* current = dynamic_cast<LocalMap*> (iter->second);
                    if(first){
                        //lmap->setId(current->getId());
                        //lmap->setTransform(current->transform());
                        lmap->setCloud(current->cloud());
                        lmap->nodes() = current->nodes();
                        lmap->relations() = current->relations();
                        first = false;
                    } else {
                        Cloud cloud;
                        current->cloud()->transform(cloud,lmap->transform().inverse()*current->transform());
                        lmap->cloud()->add(cloud);
                        for(MapNodeList::iterator jt = current->nodes().begin(); jt != current->nodes().end(); jt++){
                            MapNode* node = jt->get();
                            node->parents().clear();
                            node->parents().insert(lmap);
                            node->setTransform(lmap->transform().inverse()*node->transform());
                            lmap->nodes().addElement(node);
                        }
                        for(BinaryNodeRelationSet::iterator jt = current->relations().begin(); jt != current->relations().end(); jt++){
                            BinaryNodeRelation* rel = jt->get();
                            rel->setParent(lmap);
                            lmap->relations().insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));
                        }
                    }
                }
            }
            voxelize(*lmap->cloud(),_voxel_size);
            _structure_analyzer.compute(lmap->cloud());
            lmap->setLower(_structure_analyzer.lower());
            lmap->setUpper(_structure_analyzer.upper());
            lmap->setResolution(_resolution);
            lmap->setTraversabilityMap(new TraversabilityMap(_structure_analyzer.classified()));
            _merged_local_maps->addElement(lmap);
        }
        visit(quadtree->getChild1());
        visit(quadtree->getChild2());
        visit(quadtree->getChild3());
        visit(quadtree->getChild4());
    }
    else
        return;
}

}
