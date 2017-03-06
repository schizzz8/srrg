#include "connectivity_refiner.h"

namespace srrg_local_maps_condenser {

using namespace std;
using namespace srrg_core;
using namespace srrg_core_map;
using namespace srrg_structure_analyzer;

void ConnectivityRefiner::execute(){
    for(MapNodeList::iterator it = _local_maps->begin(); it != _local_maps->end(); it++) {
        LocalMapWithTraversability* lmap1 = dynamic_cast<LocalMapWithTraversability*> (it->get());
        for(MapNodeList::iterator jt = _local_maps->begin(); jt != _local_maps->end(); jt++){
            LocalMapWithTraversability* lmap2 = dynamic_cast<LocalMapWithTraversability*> (jt->get());
            if(!same(lmap1,lmap2) && closeEnough(lmap1,lmap2) && !alreadyConnected(lmap1,lmap2))
                if(addEdge(lmap1,lmap2)) {
                    BinaryNodeRelation* rel = new BinaryNodeRelation(lmap1,lmap2,lmap1->transform().inverse()*lmap2->transform());
                    _relations->insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));
                }
        }
    }
    cerr << "Added " << _relations->size() << " edges" << endl;
}

bool ConnectivityRefiner::addEdge(LocalMapWithTraversability *lmap1, LocalMapWithTraversability *lmap2){
    int count = 0;
    StructureAnalyzer structure_analyzer;
    structure_analyzer.setCellResolution(_resolution);
    Cloud* cloud = new Cloud;
    Cloud transformed_cloud;
    lmap1->cloud()->transform(transformed_cloud,lmap1->transform());
    cloud->add(transformed_cloud);
    int size1 = cloud->size();
    transformed_cloud.clear();
    lmap2->cloud()->transform(transformed_cloud,lmap2->transform());
    cloud->add(transformed_cloud);
    structure_analyzer.compute(cloud);
    const IntImage indices = structure_analyzer.indices();
    const UnsignedCharImage classified = structure_analyzer.classified();
    for (int r=0; r<indices.rows; r++)
        for (int c=0; c<indices.cols; c++) {
            int idx1 = indices.at<int>(r,c);
            if (idx1<0)
                continue;
            if(classified.at<unsigned char>(r,c) != 0)
                continue;
            int rmin = r-1<0?0:r-1;
            int rmax = r+1>indices.rows-1?indices.rows-1:r+1;
            int cmin = c-1<0?0:c-1;
            int cmax = c+1>indices.cols-1?indices.cols-1:c+1;
            for (int rr =rmin; rr<=rmax; rr++)
                for (int cc =cmin; cc<=cmax; cc++)
                    if (rr!=r || cc!=c) {
                        int idx2 = indices.at<int>(rr,cc);
                        if (idx2<0)
                            continue;
                        if(classified.at<unsigned char>(rr,cc) != 0)
                            continue;
                        if((idx1 < size1)^(idx2 < size1)){
                            count++;
                        }
                    }
        }
    if(count > _connected_points)
        return true;
    else
        return false;
}

bool ConnectivityRefiner::same(LocalMapWithTraversability *lmap1, LocalMapWithTraversability *lmap2){
    return (lmap1->getId() == lmap2->getId()) ? true : false;
}

bool ConnectivityRefiner::closeEnough(LocalMapWithTraversability *lmap1, LocalMapWithTraversability *lmap2){
    return ((lmap1->transform().translation() - lmap2->transform().translation()).norm() <= _distance_threshold) ? true : false;
}

bool ConnectivityRefiner::alreadyConnected(LocalMapWithTraversability *lmap1, LocalMapWithTraversability *lmap2){
    for(BinaryNodeRelationSet::iterator kt = _relations->begin(); kt != _relations->end(); kt++)
        if((*kt)->from()->getId() == lmap1->getId() && (*kt)->to()->getId() == lmap2->getId() ||
                (*kt)->from()->getId() == lmap2->getId() && (*kt)->to()->getId() == lmap1->getId() ) {
            return true;
        }
    return false;
}

}
