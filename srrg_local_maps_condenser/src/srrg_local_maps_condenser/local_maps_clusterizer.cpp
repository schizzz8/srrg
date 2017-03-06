#include "local_maps_clusterizer.h"

using namespace std;

namespace std {
template<>
bool std::less<Eigen::Vector3f>::operator ()(const Eigen::Vector3f& a,const Eigen::Vector3f& b) const {
    for(size_t i=0;i<3;++i) {
        if(a[i]<b[i]) return true;
        if(a[i]>b[i]) return false;
    }
    return false;
}
}

namespace srrg_local_maps_condenser {

using namespace srrg_core;
using namespace srrg_core_map;

void LocalMapsClusterizer::setInput(MapNodeList* local_maps_){
    //_local_maps = local_maps_;
    Eigen::Vector3f barycenter = Eigen::Vector3f::Zero();
    const float low=-std::numeric_limits<float>::max();
    const float up=std::numeric_limits<float>::max();
    Eigen::Vector3f min=Eigen::Vector3f(up, up, up);
    Eigen::Vector3f max=Eigen::Vector3f(low, low, low);
    int n = 0;
    for(MapNodeList::iterator it = local_maps_->begin(); it != local_maps_->end(); it++){
        LocalMap* lmap = dynamic_cast<LocalMap*> (it->get());

        _points.push_back(lmap->transform().translation());
        _local_maps.insert(std::pair<Eigen::Vector3f,MapNode*>(lmap->transform().translation(),it->get()));

        Eigen::Vector3f lower,upper;
        Cloud cloud;
        lmap->cloud()->transform(cloud,lmap->transform());
        cloud.computeBoundingBox(lower,upper);
        if (min.x()>lower.x())
            min.x()=lower.x();
        if (min.y()>lower.y())
            min.y()=lower.y();
        if (min.z()>lower.z())
            min.z()=lower.z();
        if (max.x()<lower.x())
            max.x()=lower.x();
        if (max.y()<lower.y())
            max.y()=lower.y();
        if (max.z()<lower.z())
            max.z()=lower.z();
        if (min.x()>upper.x())
            min.x()=upper.x();
        if (min.y()>upper.y())
            min.y()=upper.y();
        if (min.z()>upper.z())
            min.z()=upper.z();
        if (max.x()<upper.x())
            max.x()=upper.x();
        if (max.y()<upper.y())
            max.y()=upper.y();
        if (max.z()<upper.z())
            max.z()=upper.z();
        barycenter += lower;
        barycenter += upper;
        n += 2;
    }
    barycenter /= n;
    Eigen::Vector3f diff = max - barycenter;
    float radius = -std::numeric_limits<float>::max();
    for(int i = 0; i < 3; i++)
        if(diff(i) > radius)
            radius = diff(i);
    _min = Eigen::Vector3f(barycenter.x()-radius,barycenter.y()-radius,barycenter.z()-radius);
    _max = Eigen::Vector3f(barycenter.x()+radius,barycenter.y()+radius,barycenter.z()+radius);
}

void LocalMapsClusterizer::execute(){
    cerr << "BB: " << _min.transpose() << "\t" << _max.transpose() << endl;
    _octree = new OctreeNode(_depth,_points,_min,_max);
    visit(_octree);
    cerr << "Created " << _count << " clusters" << endl;
}

void LocalMapsClusterizer::visit(OctreeNode *octree){
    if(octree){
        if(octree->depth() == 0) {
            _count++;
            LocalMapWithTraversability* lmap = new LocalMapWithTraversability();
            bool first = true;
            //MapNodeList* local_maps = octree->localMaps();
            vector<Eigen::Vector3f> points = octree->points();
            for(vector<Eigen::Vector3f>::iterator it = points.begin(); it != points.end(); it++) {
                //for(MapNodeList::iterator it = local_maps->begin(); it != local_maps->end(); it++) {
                Vector3fMapNodeMap::iterator iter = _local_maps.find(*it);
                if(iter != _local_maps.end()){
                    LocalMap* current = dynamic_cast<LocalMap*> (iter->second);
                    if(first){
                        lmap->setId(current->getId());
                        lmap->setTransform(current->transform());
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
        visit(octree->getChild1());
        visit(octree->getChild2());
        visit(octree->getChild3());
        visit(octree->getChild4());
        visit(octree->getChild5());
        visit(octree->getChild6());
        visit(octree->getChild7());
        visit(octree->getChild8());
    }
    else
        return;
}

}
