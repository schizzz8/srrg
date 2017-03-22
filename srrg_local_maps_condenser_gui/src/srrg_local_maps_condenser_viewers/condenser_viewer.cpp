#include "condenser_viewer.h"
#include "qevent.h"
#include <cstring>
#include <srrg_gl_helpers/opengl_primitives.h>

namespace srrg_local_maps_condenser_gui{

using namespace std;
using namespace Eigen;
using namespace srrg_core_viewers;
using namespace srrg_core_map;

void CondenserViewer::draw() {
    int k = 0;
    for (MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++) {
        MapNode* n = it->get();
        int attrs = ATTRIBUTE_SHOW;

        if (_selected_objects.count(n)) {
            glColor3f(0.8, 0.5, 0.5);
            attrs |= ATTRIBUTE_SELECTED;
        } else {
            glColor3f(0.5, 0.8, 0.5);
        }
        n->draw(attrs);
        k++;
    }

    for (BinaryNodeRelationSet::iterator it = relations.begin(); it!=relations.end(); it++) {
        if (! (*it)->parent())
            (*it)->draw();
    }

}

void CondenserViewer::drawWithNames() {
    _names_map.clear();
    int name = 0;
    for (MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++) {
        MapNode* n = it->get();
        n->draw(ATTRIBUTE_SHOW, name);
        _names_map.insert(make_pair(name, n));
        name++;
    }
}

void CondenserViewer::postSelection(const QPoint&){
    int id = selectedName();
    if (id < 0)
        return;
    MapNode* node = _names_map[id];
    std::cout << "[INFO]: the id of the selected object is " << id << endl;
    std::cout << "[INFO]: selected object " << node->getId() << endl;
    if (_selected_objects.count(node))
        _selected_objects.erase(node);
    else
        _selected_objects.insert(node);
}

}

