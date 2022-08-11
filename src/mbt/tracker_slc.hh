#pragma once

#include "mbt/tracker.hh"

class PointHandler;

class SLCTracker : public SLTracker {
public:
	SLCTracker(const cv::Matx33f& K, std::vector<Object3D*>& objects);

protected:
	virtual void Track(std::vector<cv::Mat>& imagePyramid, std::vector<Object3D*>& objects, int runs = 1) override;
	void RunIteration(std::vector<Object3D*>& objects, const std::vector<cv::Mat>& imagePyramid, int level, 
		int sl_len, int sl_seg, float band_width, float ss, int run_type = 0);
	void ComputeJac(Object3D* object, int m_id, const cv::Mat& frame,  const cv::Mat& mask_map, const cv::Mat& masks_map, const cv::Mat& depth_map, const cv::Mat& depth_inv_map, 
		cv::Matx66f& wJTJM, cv::Matx61f& JTM, float band_width, float ss);
	void FindMatchPoint(float diff);
	void FindMatchPointMaxConv(SearchLine* search_line, float diff);
	virtual void PreProcess() override;
};