#pragma once

#include <srrg_types/defs.h>

#include "pinhole_projector.h"
#include "base_correspondence_finder.h"
#include "unscented.h"

namespace srrg_nicp {

 /**
     Implements an iterative least squares solver for 3D points with
     normal. <br> The use of this class is
     straightforward.  After construction the object has to be loaded
     with two models: the reference and the current.  Then repeated
     calls to oneIteration() refine an inner transformation. After
     each iteration, the user can monitor the state of convergence by
     checking the cumulative error, the number and the identity of
     inliers.
     
   */
  class Solver{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef _SigmaPoint<srrg_core::Vector6f> SigmaPoint;
    typedef std::vector<SigmaPoint, Eigen::aligned_allocator<SigmaPoint> > SigmaPointVector;

    Solver();
    virtual ~Solver();
    
    //! computes the system matrix and the coefficient vector of a least squares problem
    //! it does side effect in the _H and _b variables of this class
    virtual void linearize(const BaseCorrespondenceFinder::CorrespondenceVector& correspondences);
    

    //! sets/gets the model that stays fixed
    void setReferenceModel(const srrg_core_map::Cloud* m);
    inline const srrg_core_map::Cloud* referenceModel() const { return _reference;}


    //! sets/gets the model that moves
    void setCurrentModel(const srrg_core_map::Cloud* m);
    inline const srrg_core_map::Cloud* currentModel() const {return _current;}


    //! omega of the planar patches (gets rotated to be aligned to the normal)
    inline const Eigen::Matrix3f& flatOmega() const {return _flat_omega;}
    inline void setFlatOmega(const Eigen::Matrix3f& o) {_flat_omega=o;}

    //! omega of the normals to the planar patches (gets rotated to be aligned on the normal)
    inline const Eigen::Matrix3f& longLinearOmega() const {return _long_linear_omega;}
    inline void setLongLinearOmega(const Eigen::Matrix3f& o) {_long_linear_omega=o;}

    //! system matrix
    inline const srrg_core::Matrix6f& H() const {return _H;}
    
    //! coefficient vector
    inline const srrg_core::Vector6f& b() const {return _b;}

    //! current transform
    inline const Eigen::Isometry3f& T() const {return _T;}
    
    //! sets the current transform
    inline void setT(const Eigen::Isometry3f& T_)  {_T=T_;}

    //! sets the current transform, and the prior
    inline void setT(const Eigen::Isometry3f& T_, const srrg_core::Matrix6f& info)  {
      _T = T_;
      _prior_transform = T_;
      _inverse_prior_transform = T_.inverse();
      _prior_information = info;
      _has_prior = true;
    }

    //! returns the chi2 after the current linearization
    inline float error() const {return _error;}

    //! returns the information matrix of the solution, computed from the hessian
    //! must be called after linearize();
    const srrg_core::Matrix6f& informationMatrix() const {return _information_matrix;}

    //! chi2 of every point
    inline const std::vector<float>& errors() const {return _errors;}

    //! damping in the solution of the linear system
    inline float damping() const {return _damping;}

    //! set the damping
    inline void setDamping(float d)  { _damping = d; }


    //! max error after which the robust kernel will kick anc regard the point as outlier
    inline float maxError() const {return _max_error;}
    //! set the max error 
    inline void setMaxError(float me) {_max_error = me;}

    // one round of linearization/solution. Side effect in T()
    virtual void oneRound(const BaseCorrespondenceFinder::CorrespondenceVector& correspondences, bool computeStats = false);

    //! sets gicp mode (ignores the normals in the error vector)
    void setGICP(bool gicp) {_use_only_points = gicp;}


    //! gives a hint to the system on which omegas need to be recomputed
    //! needs to be called after setting the current model
    void setReferencePointsHint(const std::vector<int>& currentPointsHint);

    inline const Eigen::Isometry3f& priorTransform() const {return _prior_transform;}
    inline const srrg_core::Matrix6f& priorInformationMatrix() const {return _prior_information;}

    inline void clearPrior() { _has_prior= false;} 

    srrg_core::Matrix6f  remapInformationMatrix(const Eigen::Isometry3f& T);
    
  protected:
    srrg_core::Matrix6f _information_matrix;
    srrg_core::Matrix6f _H;
    srrg_core::Vector6f _b;
    float _max_error;
    std::vector<float> _errors;
    float _error;
    float _damping;
    Eigen::Isometry3f _T; // position of the world w.r.t the camera
    bool _use_only_points;
    const srrg_core_map::Cloud* _reference, *_current;
    srrg_core::Matrix3fVector _omega_points, _omega_normals;
    Eigen::Matrix3f _flat_omega;
    Eigen::Matrix3f _long_linear_omega;
    bool _omega_tainted;
    Eigen::Isometry3f _prior_transform;
    srrg_core::Matrix6f _prior_information;

    void errorAndJacobian(Eigen::Vector3f&  pointError, 
			  Eigen::Vector3f&  normalError, 
			  srrg_core::Matrix6f&  J, 
			  const Eigen::Vector3f& referencePoint, 
			  const Eigen::Vector3f& referenceNormal, 
			  const Eigen::Vector3f& currentPoint,
			  const Eigen::Vector3f& currentNormal) const;

    void errorAndJacobianPoint(Eigen::Vector3f&  pointError, 
			       srrg_core::Matrix3_6f&  J, 
			       const Eigen::Vector3f& referencePoint, 
			       const Eigen::Vector3f& currentPoint) const;

    // computes the hessian and the coefficient vector of the reference points between imin and imax
    // it also computes the error and the number of inliers
    virtual void linearize(srrg_core::Matrix6f& H, srrg_core::Vector6f& b, 
			   float& error, const BaseCorrespondenceFinder::CorrespondenceVector& correspondences,
			   size_t imin=0, size_t imax=std::numeric_limits<size_t>::max());    

    // if no argument is given it updates all omegas, otherwise it updates the omegas of the passed vector of indice  
    void computeOmegas(const std::vector<int> updateList = std::vector<int>());


    // computes the informatio matrix
    void computeInformationMatrix();

    
  private:
    srrg_core::Vector6f priorError(const srrg_core::Vector6f& dt = srrg_core::Vector6f::Zero());
    srrg_core::Matrix6f priorJacobian();
    SigmaPointVector _raw_sigma_points;
    Eigen::Isometry3f _inverse_prior_transform;
    bool _has_prior; 
 };

}
