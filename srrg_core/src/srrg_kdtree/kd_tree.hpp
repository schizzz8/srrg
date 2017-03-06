#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues> 

namespace srrg_core {
  
  /**
     KDTree: implements a KDTree, it requires a type T 
     and a dimension D
  */  
  template <class T, size_t D>
    class KDTree {
  public:
    // typedef for defining a vector variable sized points
    typedef Eigen::Matrix<T, D, 1> VectorTD;
    typedef std::vector<VectorTD, Eigen::aligned_allocator<VectorTD> > VectorTDVector;

    /**
       BaseTreeNode class. Represents a base class for a node in the search tree.     
    */
    class BaseTreeNode {
    public:
      //! ctor
      BaseTreeNode(KDTree<T, D>* tree_): _tree(tree_){}

      //! dtor
      virtual ~BaseTreeNode() {}
  
      //! function to search for the neighbor
      //! @param answer: the neighbor found
      //! @param query: the point to search
      //! @param maximum distance allowed for a point
      //! @returns the distance of the closest point. -1 if no point found within range
      virtual T findNeighbor(VectorTD& answer,
			     int& index,
			     const VectorTD& query, 
			     const T max_distance) const = 0;

      virtual size_t numPoints() const = 0;
      KDTree<T, D>* _tree;
    };

    /**
       Leaf node: it contains a vector of points on which
       the search function performs a linear search in the list
    */
    class LeafNode : public BaseTreeNode {
    public:
      //! ctor
      LeafNode(KDTree<T, D>* tree_) : BaseTreeNode(tree_) {}

      //! dtor
      virtual ~LeafNode() {}
    
      //! function to search for the neighbor. Performs a linear search in the vector
      //! @param answer: the neighbor found
      //! @param query: the point to search
      //! @param maximum distance allowed for a point
      //! @returns the distance of the closest point. -1 if no point found within range
      virtual T findNeighbor(VectorTD& answer,
			     int& index,
			     const VectorTD& query, 
			     const T max_distance) const {
	T d_max = std::numeric_limits<T>::max();
	const VectorTDVector& points=this->_tree->_points;
	const std::vector<int>& indices=this->_tree->_indices;
	for(size_t i = _min_index; i < _max_index; ++i) {
	  T d = (points[i] - query).squaredNorm();
	  if (d < d_max) {
	    answer = points[i];
	    index = indices[i];
	    d_max = d;
	  }
	}

	if (d_max > max_distance * max_distance) {
	  index = -1;
	  return -1;
	}
	return d_max;
      }

      virtual size_t numPoints() const {return _max_index-_min_index;}

      inline size_t minIndex() const { return _min_index; }
      inline size_t maxIndex() const { return _max_index; }

      inline void setMinIndex(size_t min_index) { _min_index = min_index; }
      inline void setMaxIndex(size_t max_index) { _max_index = max_index; }

    protected:
      size_t _min_index;   
      size_t _max_index;   

    };

    /**
       Middle node: it represents a splitting plane, and has 2 child nodes
       that refer to the set of points to the two sides of the splitting plane.
       A splitting plane is parameterized as a point on a plane and as a normal to the plane.
    */
    class MiddleNode : public BaseTreeNode {
    public:
      //! ctor
      //! @param mean_: point on the split plane
      //! @param normal_: normal vector to the plane
      //! param left_child: left subtree
      //! param right_child: right subtree
      MiddleNode(KDTree<T,D>* tree_,
		 const VectorTD& mean_,
		 const VectorTD& normal_,
		 BaseTreeNode* left_child = 0,
		 BaseTreeNode* right_child = 0): BaseTreeNode(tree_)
      {
	assert(normal_.rows() == D);
	assert(mean_.rows() == D);
	_normal = normal_;
	_mean = mean_;
	_num_points = 0;
	_left_child = left_child;
	_right_child = right_child;
	if (_left_child) {
	  _num_points += _left_child->numPoints();
	}
	if (_right_child) {
	  _num_points += _right_child->numPoints();
	}
      }
	  
      //! dtor
      virtual ~MiddleNode() {
	if (_left_child) {
	  delete _left_child;
	  _left_child = 0;
	}
	if (_right_child) {
	  delete _right_child;
	  _right_child = 0;
	}
      }

      //! mean const accessor
      inline const VectorTD& mean() const { return _mean; }

      //! normal const accessor
      inline const VectorTD& normal() const { return _normal; }

      //! mean left accessor
      inline BaseTreeNode* leftChild() const { return _left_child; }

      //! mean right accessor
      inline BaseTreeNode* rightChild() const { return _right_child; }

      virtual size_t numPoints() const {return _num_points;}
      
      //! checks if a point lies to the left or the right of a plane
      //! @param query_point: the point to be checked
      //! @returns true if a point lies to the left, false otherwise
      inline bool side(const VectorTD& query_point) const { return _normal.dot(query_point - _mean) < 0; }
      
      //! binary search for the neighbors. 
      //! @param answer: the neighbor found
      //! @param query: the point to search
      //! @param maximum distance allowed for a point
      //! @returns the distance of the closest point. -1 if no point found within range
      virtual T findNeighbor(VectorTD& answer,
			     int& index,
			     const VectorTD& query, 
			     const T max_distance) const {
	bool is_left = side(query);
	if (is_left && _left_child) {
	  return _left_child->findNeighbor(answer, index, query, max_distance);
	}
	if (!is_left && _right_child) {
	  return _right_child->findNeighbor(answer, index, query, max_distance);
	}
	return -1;    
      }

    protected:
      VectorTD _normal;
      VectorTD _mean;
      size_t _num_points;
      BaseTreeNode* _left_child;
      BaseTreeNode* _right_child;
    };
    
    //! ctor
    KDTree(const VectorTDVector& points_,
	   T max_leaf_range) {
      _num_nodes = 0;
      _points = points_;
      _aux_points = points_;
      _indices.resize(points_.size());      
      _aux_indices.resize(points_.size());
      for(size_t i = 0; i < _indices.size(); ++i) {
	_indices[i] = i;
      }
      _root = _buildTree(0,
			 points_.size(),
			 max_leaf_range);
    }
    //! dtor
    ~KDTree() {
      if (_root) {
	delete _root;
      }
      _root = 0;
    }

    //! num_nodes accessor
    inline size_t numNodes() const { return _num_nodes; }
        
    //! function to search for the neighbor
    //! @param answer: the neighbor found
    //! @param query: the point to search
    //! @param maximum distance allowed for a point
    //! @returns the distance of the closest point. -1 if no point found within range
    inline T findNeighbor(VectorTD& answer,
			  int& index, 
			  const VectorTD& query, 
			  const T max_distance) const { return _root->findNeighbor(answer, index, query, max_distance); }

    inline void printKDTree() { _printKDTree(_root); }
    
    protected:    
    /**
       Partitions a point vector in two vectors, computing the splitting plane
       as the largest eigenvalue of the point covariance
       @param mean: the returned mean of the splitting plane
       @param normal: the normal of the splitting plane
       @param left: the returned left vector of points
       @param right: the returned right vector of points
       @param points: the array of points
       @returns the distance of the farthest point from the plane
    */
    T _splitPoints(VectorTD& mean, VectorTD& normal,
		   size_t& num_left_points,
		   const size_t min_index, const size_t max_index) {
      // if points empty, nothing to do      
      if (min_index == max_index) {
	return 0;
      }

      size_t num_points = max_index - min_index;
      T inverse_num_points = 1.0 / num_points;
      VectorTD sum = VectorTD::Zero();
      Eigen::Matrix<T, D, D> squared_sum = Eigen::Matrix<T, D, D>::Zero();
      Eigen::Matrix<T, D, D> covariance = Eigen::Matrix<T, D, D>::Zero();
      for(size_t i = min_index; i < max_index; ++i) {
	sum += _points[i];
	squared_sum += _points[i] * _points[i].transpose();
      }
      mean = sum * inverse_num_points;
      covariance = squared_sum * inverse_num_points - mean * mean.transpose();
      
      // eigenvalue decomposition
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, D, D> > solver;
      solver.compute(covariance, Eigen::ComputeEigenvectors);
      normal = solver.eigenvectors().col(D - 1).normalized();

      // the following var will contain the range of points along the normal vector
      T max_distance_from_plane = 0;

      // run through the points and split them in the left or the right set
      size_t left_index = min_index;
      size_t right_index = max_index;

      size_t num_left = 0;
      size_t num_right = 0;
      for(size_t i = min_index; i < max_index; ++i) {
	T distance_from_plane = normal.dot(_points[i] - mean);
	if (fabs(distance_from_plane) > max_distance_from_plane) {
	  max_distance_from_plane = fabs(distance_from_plane);
	}

	bool side = distance_from_plane < 0;
	if (side) {
	  _aux_points[left_index] = _points[i];
	  _aux_indices[left_index] = _indices[i];
	  left_index++;
	  num_left++;
	} else {	  
	  right_index--;
	  _aux_points[right_index] = _points[i];
	  _aux_indices[right_index] = _indices[i];
	  num_right++;
	}
      }

      for(size_t i = min_index; i < max_index; ++i) {
	_points[i] = _aux_points[i];
	_indices[i] = _aux_indices[i];
      }

      num_left_points = num_left;
      return max_distance_from_plane;
    }

    //! function to build the tree
    //! @param points: the points
    //! @param max_leaf_range: specify the size of the "box" below which a leaf node is generated
    //! returns the root of the search tree
    BaseTreeNode* _buildTree(const size_t min_index,
			     const size_t max_index,
			     const T max_leaf_range) {
      size_t num_points = max_index-min_index;
      if (! num_points) {
	return 0;
      }

      VectorTD mean;
      VectorTD normal;
      size_t  num_left_points = 0;

      T range = _splitPoints(mean, normal,
			     num_left_points,
			     min_index,
			     max_index);

      if (range < max_leaf_range) {
	LeafNode* node = new LeafNode(this);
	node->setMinIndex(min_index);
	node->setMaxIndex(max_index);
	_num_nodes++;
	return node;
      }

      MiddleNode* node = new MiddleNode(this,
					mean,
					normal,
					_buildTree(min_index, min_index + num_left_points, max_leaf_range),
					_buildTree(min_index + num_left_points, max_index, max_leaf_range));
      _num_nodes++;
	
      return node;
    }

    void _printKDTree(BaseTreeNode* node) {
      LeafNode* leaf = dynamic_cast<LeafNode*>(node);
      if (leaf) {
	std::cerr << "Leaf: " << std::endl;
	for(size_t i = 0; i < leaf->points().size(); ++i) {
	  std::cerr << leaf->points()[i].transpose() << std::endl;
	}
	std::cerr << std::endl;
      }
      
      
      MiddleNode* middle_node = dynamic_cast<MiddleNode*>(node);
      if (middle_node) {
	_printKDTree(middle_node->leftChild());
	_printKDTree(middle_node->rightChild());
      }
    }

    size_t _num_nodes;
    BaseTreeNode* _root;    
    VectorTDVector _aux_points;
    VectorTDVector _points;
    std::vector<int> _indices;
    std::vector<int> _aux_indices;
  };
  
}
