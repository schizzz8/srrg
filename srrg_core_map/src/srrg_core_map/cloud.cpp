#include "cloud.h"

#ifdef _GO_PARALLEL_
#include <omp.h>
#endif

#include <GL/gl.h>
#include <string>
#include <stdexcept>

// #define _NAN_CHECK_

namespace srrg_core_map {

  using namespace std;
  using namespace srrg_boss;
  using namespace srrg_core;

  void Cloud::add(const Cloud& other) {
    if (&other == this)
      return;
    size_t k = size();
    resize(k + other.size());
#ifdef _GO_PARALLEL_
#pragma omp parallel for
#endif
    for (size_t i = 0; i < other.size(); i++) {
      at(k + i) = other.at(i);
    }
  }


  void Cloud::draw(int name) const {
    if (name > -1)
      glPushName(name);

    glBegin(GL_POINTS);
    for (size_t i = 0; i<size(); i++){
      const RichPoint& rp = at(i);
      const Eigen::Vector3f& p = rp.point();
      const Eigen::Vector3f& n = rp.normal();
      if (n.squaredNorm()) {
	const Eigen::Vector3f& rgb = rp.rgb();
	if (rgb.squaredNorm()){
	  glPushAttrib(GL_COLOR);
	  glColor3f(rgb.x(), rgb.y(), rgb.z());
	  glNormal3f(n.x(), n.y(), n.z());
	  glVertex3f(p.x(), p.y(), p.z());
	  glPopAttrib();
	} else {
	  glNormal3f(n.x(), n.y(), n.z());
	  glVertex3f(p.x(), p.y(), p.z());
	}
      } else {
	Eigen::Vector3f n = -rp.normal().normalized();
	glNormal3f(n.x(), n.y(), n.z());
	glVertex3f(p.x(), p.y(), p.z());
      }
    }
    glEnd();
    if (name > -1)
      glPopName();
  }

  void Cloud::transformInPlace(const Eigen::Isometry3f& T) {
    Eigen::Matrix3f R = T.linear();
#ifdef _GO_PARALLEL_
#pragma omp parallel for
#endif
    for (size_t i = 0; i < size(); i++) {
      if (at(i).accumulator() <= 0) {
	cerr << "size: " << size() << endl;
	cerr << "index: " << i << endl;
	cerr << "p: " << at(i).point().transpose() << endl;
	cerr << "n: " << at(i).normal().transpose() << endl;
	cerr << "cvi: " << at(i).accumulator() << endl;
	throw std::runtime_error("cum numm");
      }
      
      at(i).transformInPlace(T);
    }
  }

  void Cloud::transform(Cloud& other, const Eigen::Isometry3f& T) const {
    other.resize(size());
    Eigen::Matrix3f R = T.linear();
#ifdef _GO_PARALLEL_
#pragma omp parallel for
#endif
    for (size_t i = 0; i < size(); i++) {
      other[i] = at(i).transform(T);
    }
  }

#ifdef _DEBUG_MERGE_

#define checkCumVal(x,msg)						\
  if (x.cumulativeValues[x##Idx]<=0){					\
    cerr << "idx: " << x##Idx << endl;					\
      cerr << "p:" << x.pointAccumulators[x##Idx].transpose() << endl;	\
      cerr << "n:" << x.normalAccumulators[x##Idx].transpose() << endl; \
      cerr << "c:" << x.cumulativeValues[x##Idx]<< endl;		\
      throw std::runtime_error(msg);					\
  }
#else
#define checkCumVal(x,msg)
#endif

  void merge(FloatImage& destBuffer, IndexImage& destIndices, Cloud& dest,
	     FloatImage& srcBuffer, IndexImage& srcIndices, Cloud& src,
	     float normalThreshold,
	     float distanceThreshold) {

#ifdef _DEBUG_MERGE_
    cerr << "Merge: " << endl;
    cerr << "destSize: " << dest.size() << " srcSize: " << src.size() << endl;
#endif //_DEBUG_MERGE_
    
    normalThreshold = cos(normalThreshold);
    int newPoints = 0;
    for (int c = 0; c < destBuffer.cols; c++) {
      for (int r = 0; r < destBuffer.rows; r++) {
	int& srcIdx = srcIndices.at<int>(r, c);
	int& destIdx = destIndices.at<int>(r, c);

	// add a point if it appars only in the src and are undefined in the dest
	if (srcIdx < 0 || destIdx < 0) {
	  if (srcIdx > -1 && destIdx < 0)
	    newPoints ++;
	  continue;
	}

	checkCumVal(dest, "before dest");
	checkCumVal(src, "before src");

	float destDepth = destBuffer.at<float>(r, c);
	float srcDepth = srcBuffer.at<float>(r, c);
	float davg = 0.5 * (destDepth + srcDepth);
	// if a new point appears a lot in front an old one add it
	float scaledDistance = (destDepth - srcDepth) / davg;

	if ( scaledDistance > distanceThreshold) {
	  destIdx = -1;
	  newPoints++;
	  continue;
	}

	// if a new point appears a lot behind an old replace the old
	if ( scaledDistance < -distanceThreshold) {
	  dest[destIdx] = src[srcIdx];
	  srcIdx = -1;
	  destIdx = -1;
	  continue;
	}

	// if the normals do not match do nothing
	Eigen::Vector3f destNormal = dest[destIdx].normal();
	if (!dest[destIdx].isNormalized())
	  destNormal /= dest[destIdx].accumulator();

	Eigen::Vector3f srcNormal = src[srcIdx].normal();
	if (!src[srcIdx].isNormalized())
	  srcNormal /= src[srcIdx].accumulator();
	if (destNormal.dot(srcNormal) < normalThreshold) {
	  destIdx = -1;
	  //newPoints++;
	  srcIdx = -1;
	  continue;
	}

	// merge the points
	dest[destIdx] += src[srcIdx];
	srcIdx = -1;

	checkCumVal(dest, "merge in bounds");
      }
    }

#ifdef _DEBUG_MERGE_
    cerr << "dest expected final size: " << dest.size() + newPoints << endl;
#endif //_DEBUG_MERGE_
    
    // recompute all the touched points
    for (int c = 0; c < destBuffer.cols; c++) {
      for (int r = 0; r < destBuffer.rows; r++) {
	int& destIdx = destIndices.at<int>(r, c);
	if (destIdx < 0)
	  continue;
	dest[destIdx].normalize();
      }
    }

    int k = dest.size();
    dest.resize(dest.size() + newPoints);
    for (int c = 0; c < srcBuffer.cols; c++) {
      for (int r = 0; r < srcBuffer.rows; r++) {
	int& srcIdx = srcIndices.at<int>(r, c);
	if (srcIdx < 0)
	  continue;
	dest[k] = src[srcIdx];
	k++;
      }
    }
#ifdef _DEBUG_MERGE_
    cerr << "expected size: " << dest.size() << " newPoints: " << newPoints << " finalSize: " << k << endl;
#endif //_DEBUG_MERGE_
  }

  struct IndexTriplet {
    int x, y, z, index;
    IndexTriplet() {
      x = y = z = 0;
      index = -1;
    }

    IndexTriplet(const Eigen::Vector3f& v, int idx, float ires) {
      x = (int) (ires * v.x());
      y = (int) (ires * v.y());
      z = (int) (ires * v.z());
      index = idx;
    }
    bool operator < (const IndexTriplet& o) const {
      if (z < o.z)
	return true;
      if (z > o.z)
	return false;
      if (x < o.x)
	return true;
      if (x > o.x)
	return false;
      if (y < o.y)
	return true;
      if (y > o.y)
	return false;
      if (index < o.index)
	return true;
      return false;
    }

    bool sameCell( const IndexTriplet& o) const {
      return x == o.x && y == o.y && z == o.z;
    }
  };

  //! clips to a maxRange around a pose
  void Cloud::clip(float range, const Eigen::Isometry3f& pose) {
    Eigen::Isometry3f T = pose.inverse();
    range *= range;
    int k = 0;
    for (size_t i = 0; i < size(); i++) {
      const RichPoint& p = at(i);
      Eigen::Vector3f other_p = T * p.point();
      if (other_p.squaredNorm() < range) {
	at(k) = p;
	k++;
      }
    }
    resize(k);
  }


  void voxelize(Cloud& model, float res) {
    //cerr << "voxelize: " << model.size() << " -> ";
    float ires = 1. / res;
    std::vector<IndexTriplet> voxels(model.size());
    for (int i = 0; i < (int)model.size(); i++) {
      voxels[i] = IndexTriplet(model[i].point(), i , ires);
      //cerr << i << " " << model.points[i].transpose() << endl;
      //cerr << voxels[i].x << " " << voxels[i].y << " " << voxels[i].x <<  endl;
    }
    Cloud sparseModel;
    sparseModel.resize(model.size());
    std::sort(voxels.begin(), voxels.end());
    int k = -1;
    for (size_t i = 0; i < voxels.size(); i++) {
      IndexTriplet& triplet = voxels[i];
      int idx = triplet.index;
      if (k >= 0  && voxels[i].sameCell(voxels[i - 1])) {
	sparseModel[k] += model[idx];
      } else {
	k++;
	sparseModel[k] = model[idx];
      }
      //cerr << voxels[i].x << " " << voxels[i].y << " " << voxels[i].x <<  endl;
    }
    sparseModel.resize(k);
    for (size_t i = 0; i < sparseModel.size(); i++) {
      if (sparseModel[i].accumulator() <= 0)
	cerr << "Il male sia con te" << endl;
      sparseModel[i].normalize();
    }
    model = sparseModel;
    //cerr << sparseModel.size() << endl;
  }

  template <class T>
  void writeDatum(ostream& os, const T& d) {
    const char * dp = reinterpret_cast<const char*>(&d);
    os.write(dp, sizeof(T));
  }

  void Cloud::write(ostream& os) const {
    size_t s=size();
    writeDatum(os, s);
    for(size_t i=0; i<s; i++) {
      const RichPoint& p=at(i);
      writeDatum(os, p.point().x());
      writeDatum(os, p.point().y());
      writeDatum(os, p.point().z());
      writeDatum(os, p.normal().x());
      writeDatum(os, p.normal().y());
      writeDatum(os, p.normal().z());
      writeDatum(os, p.rgb().x());
      writeDatum(os, p.rgb().y());
      writeDatum(os, p.rgb().z());
      float acc = p.accumulator();
      writeDatum(os, acc);
    } 
  }

  template <class T>
  void readDatum(istream& is, T& d) {
    char * dp = reinterpret_cast<char*>(&d);
    is.read(dp, sizeof(T));
  }


  void Cloud::computeBoundingBox(Eigen::Vector3f& lower, Eigen::Vector3f& higher) const {
    const float low=-std::numeric_limits<float>::max();
    const float up=std::numeric_limits<float>::max();
    lower=Eigen::Vector3f(up, up, up);
    higher=Eigen::Vector3f(low, low, low);
    for(size_t i=0; i<size(); i++) {
      const RichPoint& p=at(i);
      if (lower.x()>p.point().x())
	lower.x()=p.point().x();
      if (lower.y()>p.point().y())
	lower.y()=p.point().y();
      if (lower.z()>p.point().z())
	lower.z()=p.point().z();
      if (higher.x()<p.point().x())
	higher.x()=p.point().x();
      if (higher.y()<p.point().y())
	higher.y()=p.point().y();
      if (higher.z()<p.point().z())
	higher.z()=p.point().z();
    }
  }

  bool Cloud::read(istream& is) {
    clear();
    size_t s;
    readDatum(is, s);
    resize(s);
    for(size_t i=0; i<s; i++) {
      RichPoint p=at(i);
      float x,y,z, nx, ny, nz, cx, cy,cz, a;
      readDatum(is, x);
      readDatum(is, y);
      readDatum(is, z);
      readDatum(is, nx);
      readDatum(is, ny);
      readDatum(is, nz);
      readDatum(is, cx);
      readDatum(is, cy);
      readDatum(is, cz);
      readDatum(is, a);
      at(i) = RichPoint(Eigen::Vector3f(x,y,z), Eigen::Vector3f(nx, ny, nz), a, Eigen::Vector3f(cx, cy, cz));
    } 
    return true;
  }

  BOSS_REGISTER_BLOB(Cloud);

}

