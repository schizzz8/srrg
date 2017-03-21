#pragma once

#include <srrg_types/defs.h>

#include <srrg_core_map/cloud.h>

namespace srrg_nicp {

  void shrinkRawDepth(srrg_core::RawDepthImage& dest_buffer, const srrg_core::RawDepthImage& src_buffer, int k);

  void shrinkDepth(srrg_core::FloatImage& dest_buffer, srrg_core::IntImage& dest_indices, 
		   const srrg_core::FloatImage& src_buffer, const srrg_core::IntImage& src_indices, int k);

  void compareDepths(float& in_distance, 
		     int& in_num,
		     float& out_distance, 
		     int& out_num,
		     const srrg_core::FloatImage& depths1, const srrg_core::IntImage& indices1, 
		     const srrg_core::FloatImage& depths2, const srrg_core::IntImage& indices2, 
		     float dist=0.05, bool scale_z = false, srrg_core::FloatImage* result=0);

  void convert_32FC1_to_16UC1(cv::Mat& dest, const cv::Mat& src, float scale = 1000.0f);
  void convert_32FC1_to_16UC1(cv::Mat& dest, const cv::Mat& src, const cv::Mat& mask, float scale = 1000.0f);
  void convert_16UC1_to_32FC1(cv::Mat& dest, const cv::Mat& src, float scale = 0.001f);
  void convert_16UC1_to_32FC1(cv::Mat& dest, const cv::Mat& src, const cv::Mat& mask, float scale = 0.001f);

  // [Workaround: OpenCV 3.0.0 image subtraction warnings]
  void add_16UC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2);
  void sub_16UC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2);
  void add_32FC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2);
  void sub_32FC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2);

}
