#include "local_map_with_traversability.h"
#include "GL/gl.h"
#include <srrg_gl_helpers/opengl_primitives.h>
#include <iostream>
namespace srrg_core_map{

using namespace std;
using namespace srrg_boss;
using namespace srrg_gl_helpers;

LocalMapWithTraversability::LocalMapWithTraversability(const Eigen::Isometry3f& transform,
                                                       int id,
                                                       srrg_boss::IdContext* context) : MapNode(transform, id, context){
}

LocalMapWithTraversability::~LocalMapWithTraversability(){
    _cloud_ref.set(0);
    _traversability_map_ref.set(0);
}

void LocalMapWithTraversability::serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    MapNode::serialize(data,context);
    ObjectData * cloudBlobData=new ObjectData();
    data.setField("cloud", cloudBlobData);
    _cloud_ref.serialize(*cloudBlobData,context);
    ArrayData* nodesArray = new ArrayData;
    for (MapNodeList::iterator it = _nodes.begin(); it!=_nodes.end(); it++){
        MapNode* node=it->get();
        nodesArray->add(new PointerData(node));
    }
    data.setField("nodes", nodesArray);
    ArrayData* relationsArray = new ArrayData;
    for (BinaryNodeRelationSet::iterator it = _relations.begin(); it!=_relations.end(); it++){
        BinaryNodeRelation* rel=it->get();
        relationsArray->add(new PointerData(rel));
    }
    data.setField("relations", relationsArray);
    ObjectData * traversabilityBlobData=new ObjectData();
    data.setField("traversability", traversabilityBlobData);
    _traversability_map_ref.serialize(*traversabilityBlobData,context);
    data.setDouble("resolution",_resolution);
    _lower.toBOSS(data,"lower");
    _upper.toBOSS(data,"upper");
}

void LocalMapWithTraversability::deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    MapNode::deserialize(data,context);
    // deserialize the cloud
    ObjectData * cloudBlobData = static_cast<ObjectData*>(data.getField("cloud"));
    _cloud_ref.deserialize(*cloudBlobData,context);
    ArrayData& nodesArray=data.getField("nodes")->getArray();
    for (size_t i =0; i< nodesArray.size(); i++){
        ValueData& v = nodesArray[i];
        Identifiable* id = v.getPointer();
        MapNode* n = dynamic_cast<MapNode*>(id);
        _nodes.addElement(n);
    }
    ArrayData& relationsArray=data.getField("relations")->getArray();
    for (size_t i =0; i< relationsArray.size(); i++){
        ValueData& v = relationsArray[i];
        Identifiable* id = v.getPointer();
        BinaryNodeRelation* r = dynamic_cast<BinaryNodeRelation*>(id);
    }
    // deserialize the traversability map
    ObjectData * traversabilityBlobData = static_cast<ObjectData*>(data.getField("traversability"));
    _traversability_map_ref.deserialize(*traversabilityBlobData,context);
    _resolution = data.getDouble("resolution");
    _lower.fromBOSS(data,"lower");
    _upper.fromBOSS(data,"upper");
}

void LocalMapWithTraversability::push() {
    MapNode::push();
    for(MapNodeList::iterator nodes_it = _nodes.begin(); nodes_it != _nodes.end(); ++nodes_it) {
        (*nodes_it)->push();
    }
}

void LocalMapWithTraversability::pop() {
    MapNode::pop();
    for(MapNodeList::iterator nodes_it = _nodes.begin(); nodes_it != _nodes.end(); ++nodes_it) {
        (*nodes_it)->pop();
    }
}

void LocalMapWithTraversability::draw(DrawAttributesType attributes, int name){
    if (! (attributes&ATTRIBUTE_SHOW))
        return;
    if (name>-1)
        glPushName(name);
    glPushMatrix();
    glMultMatrix(_transform);
    if (attributes&ATTRIBUTE_SELECTED) {
        if(!(attributes&ATTRIBUTE_ONLY)) {
            cloud()->draw(attributes);
        }
        _nodes.draw(attributes);
        for (BinaryNodeRelationSet::iterator it = _relations.begin(); it!=_relations.end(); it++) {
            (*it)->draw();
        }
    }
    glPushMatrix();
    glScalef(0.2, 0.2, 0.2);
    glPushAttrib(GL_COLOR);
    drawReferenceSystem();
    glPopAttrib();
    glPopMatrix();
    glPopMatrix();
    if (name>-1)
        glPopName();
}

bool LocalMapWithTraversability::intersects(LocalMapWithTraversability &n){
    //cerr << "BB1: " << _lower.transpose() << "\t" << _upper.transpose() << endl;
    //cerr << "BB2: " << n.lower().transpose() << "\t" << n.upper().transpose() << endl;
    Eigen::Vector3f lower1 = _transform*_lower;
    Eigen::Vector3f upper1 = _transform*_upper;
    Eigen::Vector3f lower2 = n.transform()*n.lower();
    Eigen::Vector3f upper2 = n.transform()*n.upper();
    //cerr << "BB1: " << lower1.transpose() << "\t" << upper1.transpose() << endl;
    //cerr << "BB2: " << lower2.transpose() << "\t" << upper2.transpose() << endl;
    if(lower1.x()<=upper2.x() && lower2.x()<=upper1.x() &&
            lower1.y()<=upper2.y() && lower2.y()<=upper1.y() &&
            lower1.z()<=upper2.z() && lower2.z()<=upper1.z()){
        cerr << "Local maps intersect!" << endl;
        return true;
    }
    else {
        cerr << "Local maps do not intersect!" << endl;
        return false;
    }
}

void LocalMapWithTraversability::findMinMax(LocalMapWithTraversability &n,
                                            Eigen::Vector3f &common_min, Eigen::Vector3f &common_max) {
    Eigen::Vector3f lower1 = _transform*_lower;
    Eigen::Vector3f upper1 = _transform*_upper;
    Eigen::Vector3f lower2 = n.transform()*n.lower();
    Eigen::Vector3f upper2 = n.transform()*n.upper();
    if(lower1.x() > lower2.x())
        common_min.x() = lower1.x();
    else
        common_min.x() = lower2.x();
    if(upper1.x() < upper2.x())
        common_max.x() = upper1.x();
    else
        common_max.x() = upper2.x();
    if(lower1.y() > lower2.y())
        common_min.y() = lower1.y();
    else
        common_min.y() = lower2.y();
    if(upper1.y() < upper2.y())
        common_max.y() = upper1.y();
    else
        common_max.y() = upper2.y();
    if(lower1.z() > lower2.z())
        common_min.z() = lower1.z();
    else
        common_min.z() = lower2.z();
    if(upper1.z() < upper2.z())
        common_max.z() = upper1.z();
    else
        common_max.z() = upper2.z();
}

BOSS_REGISTER_CLASS(LocalMapWithTraversability);

}
