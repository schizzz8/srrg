#include "base_aligner.h"

namespace srrg_nicp {

  using namespace std;
  using namespace srrg_boss;
  using namespace srrg_core_map;
  
  BaseAligner::Trigger::Trigger(BaseAligner* t, int e, int p) {
    _aligner = t;
    _event = e;
    _priority = p;
    cerr << "Trigger ctor, Events: " << e << endl;
    for (BaseAligner::EventTriggeMap::iterator it =_aligner->_triggers.begin();
	 it!=_aligner->_triggers.end(); it++){
      if ( (e & it->first) ) {
	BaseAligner::PriorityTriggerMap::iterator pit = it->second.find(_priority);
	if (pit != it->second.end())
	  throw std::runtime_error("error, trigger with the same priority already exists");
	it->second.insert(make_pair(_priority, this));
	cerr << "  adding aligner trigger:" << this << " for event: " << it->first << " priority: " << p;
      }
    }
  }

  BaseAligner::Trigger::~Trigger() {
    for (BaseAligner::EventTriggeMap::iterator it =_aligner->_triggers.begin();
	 it!=_aligner->_triggers.end(); it++){
      if (_event & it->first ) {
	BaseAligner::PriorityTriggerMap::iterator pit = it->second.find(_priority);
	if (pit == it->second.end())
	  throw std::runtime_error("error, deleting a non existing trigger");
	it->second.erase(pit); 
	cerr << "destroying trigger" << endl;
      }
    }
 }

  BaseAligner::BaseAligner(){
    _solver.setMaxError(.01);
    _solver.setDamping(100);
    _solver.setGICP(false);
    _reference_compression_enabled = true;
    
    // populate trigger map
    _triggers.insert(make_pair(Initialized, PriorityTriggerMap()));
    _triggers.insert(make_pair(Correspondences, PriorityTriggerMap()));
    _triggers.insert(make_pair(Iteration, PriorityTriggerMap()));
    _triggers.insert(make_pair(Optimization, PriorityTriggerMap()));
  }

  BaseAligner::~BaseAligner(){}
  

  void BaseAligner::setCurrentModel( const Cloud* m) {
    _solver.setCurrentModel(m);
  }
  
  void BaseAligner::setReferenceModel( const Cloud* m) {
    _solver.setReferenceModel(m);
  }

  void BaseAligner::callTriggers(TriggerEvent event, void* params){
    BaseAligner::EventTriggeMap::iterator it = _triggers.find(event);
    if (it == _triggers.end()) {
      throw std::runtime_error("error, unsupported event in aligner");
    }
    for (PriorityTriggerMap::iterator pit = it->second.begin(); pit!= it->second.end(); pit++){
      pit->second->action(event, params);
    }
  }

}
