#pragma once

#include <iostream>

#include <srrg_types/defs.h>

#include <srrg_boss/blob.h>

namespace srrg_core_map {

struct RichPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  inline RichPoint(const Eigen::Vector3f& p = Eigen::Vector3f::Zero(),
                   const Eigen::Vector3f& n = Eigen::Vector3f::Zero(),
                   float a = 0.0f,
                   const Eigen::Vector3f& c = Eigen::Vector3f::Zero()) {
    _point = p;
    _normal = n;
    _rgb = c;
    _accumulator = a;
    _is_normalized = true;
  }

  inline RichPoint& operator+=(const RichPoint& rp) {
    denormalize();
    RichPoint rp2(rp);
    rp2.denormalize();
    _point += rp2._point;
    _normal += rp2._normal;
    _rgb += rp2._rgb;
    _accumulator += rp2._accumulator;
    return *this;
  }

  inline bool isNormalized() const { return _is_normalized; }
  inline const Eigen::Vector3f& point() const { return _point; }
  inline const Eigen::Vector3f& normal() const { return _normal; }
  inline const Eigen::Vector3f& rgb() const { return _rgb; }

  inline float accumulator() const { return _accumulator; }
  inline void transformInPlace(Eigen::Isometry3f iso) {
    _point = iso * _point;
    _normal = iso.linear() * _normal;
  }
  inline RichPoint transform(const Eigen::Isometry3f& iso) const {
    return RichPoint(iso * _point, iso.linear() * _normal, _accumulator, _rgb);
  }

  inline void denormalize() {
    if (!_is_normalized) {
      return;
    }
    _point *= _accumulator;
    _normal *= _accumulator;
    _rgb *= _accumulator;
    _is_normalized = false;
  }

  inline void normalize() {
    if (_is_normalized) {
      return;
    }
    if (_accumulator > 0) {
      float iv = 1. / _accumulator;
      _point *= iv;
      _normal *= iv;
      _rgb *= iv;
      if(_normal.squaredNorm()>0)
	_normal.normalize();
    } else {
      _point.setZero();
      _normal.setZero();
      _rgb.setZero();
    }
    _is_normalized = true;
  }

  Eigen::Vector3f _point;
  Eigen::Vector3f _normal;
  Eigen::Vector3f _rgb;
  float _accumulator;
  bool _is_normalized;
};

typedef std::vector<RichPoint, Eigen::aligned_allocator<RichPoint> >
    RichPointVector;

/**)
   This class represents a 3D model, as a collection of rich points
 */

struct Cloud : public srrg_boss::BLOB, public RichPointVector {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  //! applies the transformation to each entity in the model, doing side effect.
  //! @param dest: output
  //! @param T: the transform

  void transformInPlace(const Eigen::Isometry3f& T);
  //! applies the transformation to each entity in the model, but writes the
  //output in dest.
  //! @param dest: output
  //! @param T: the transform
  void transform(Cloud& dest, const Eigen::Isometry3f& T) const;

  //! adds other to this point cloud, doing side effect
  void add(const Cloud& other);

  //! draws a cloud if invoked in a gl context
  virtual void draw(int name = -1) const;

  //! saves the cloud in a binary stream, optimized for speed
  virtual void write(std::ostream& os) const;

  //! loads the cloud from a binary stream, optimized for speed
  virtual bool read(std::istream& is);

  //! clips to a maxRange around a pose
  void clip(float range,
            const Eigen::Isometry3f& pose = Eigen::Isometry3f::Identity());

  //! computes the bounding box of a cloud.
  //! p1: lower x, lower y, lower z
  void computeBoundingBox(Eigen::Vector3f& lower,
                          Eigen::Vector3f& higher) const;
};

typedef srrg_boss::BLOBReference<Cloud> CloudBLOBReference;

//! does the merge of src in dest
//! it requires the index image of dest and of src, seen from the same point
//! and also the depth buffers
//! the clouds should be aligned
//! points that are closer than distanceThreshold are merged based on the
//scaling values
//! if the normals are compatible
 void merge(srrg_core::FloatImage& destBuffer, srrg_core::IndexImage& destIndices, Cloud& dest,
           srrg_core::FloatImage& srcBuffer, srrg_core::IndexImage& srcIndices, Cloud& src,
           float normalThreshold = 1, float distanceThreshold = 0.2);

//! prunes the points in model, computing a scaled average
//! one point will survive for each voxel of side res
 void voxelize(Cloud& model, float res);
}
