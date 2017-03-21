#pragma once
#include "solver.h"

#include <fstream>

namespace srrg_nicp {

  class BaseAligner {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    friend class Trigger;

    enum TriggerEvent {Initialized=0x1, Iteration=0x2, Correspondences=0x4, Optimization=0x8};
    class Trigger{
    public:
      Trigger(BaseAligner* aligner, int event, int priorty);
      inline int event() const {return _event;} 
      inline int priority() const {return _priority;}
      virtual void action(TriggerEvent event, void* parameters=0)=0;
      inline BaseAligner* aligner()  {return _aligner;}
      inline const BaseAligner* aligner() const {return _aligner;}
      virtual ~Trigger();
    protected:
      int _event;
      int _priority;
      BaseAligner* _aligner;
    };

    BaseAligner();
    virtual ~BaseAligner();

    inline const srrg_core_map::Cloud* currentModel() const { return _solver.currentModel();}

    inline const srrg_core_map::Cloud* referenceModel() const { return _solver.referenceModel();}
    
    virtual void setCurrentModel( const srrg_core_map::Cloud* m);
    virtual void setReferenceModel( const srrg_core_map::Cloud* m);

    virtual void setMaxDistance(float) = 0;
    virtual float maxDistance() const = 0;
    inline const Eigen::Isometry3f& T() const {return _solver.T();}

    inline const srrg_core::Matrix6f& informationMatrix() const {return _solver.informationMatrix();}

    virtual void align(const Eigen::Isometry3f& initial_guess = Eigen::Isometry3f::Identity(),
		       const srrg_core::Matrix6f& initial_guess_information = srrg_core::Matrix6f::Zero()) = 0;

    inline Solver& solver() { return _solver; }

    inline void setReferenceCompressionEnabled(bool enable) {_reference_compression_enabled=enable;}
    inline bool referenceCompressionEnabled() const {return _reference_compression_enabled;}
  protected:
    typedef std::map<int, Trigger*> PriorityTriggerMap;
    typedef std::map<TriggerEvent, PriorityTriggerMap> EventTriggeMap;
    void callTriggers(TriggerEvent event, void* parameters=0);
    EventTriggeMap _triggers;

    Solver _solver;
    srrg_core_map::Cloud _compressed_reference;
    bool _reference_compression_enabled;
    Trigger* _trigger;
  };

}
