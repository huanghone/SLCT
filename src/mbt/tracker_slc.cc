#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <glog/logging.h>

#include "base/global_param.hh"
#include "mbt/m_func.hh"
#include "mbt/histogram.hh"
#include "mbt/search_line.hh"
#include "mbt/tracker_slc.hh"

//#define COLOR_MATCHING
//#define USE_MIXED_OPTIMIZE

//#define SHOW_SLC_DEBUG
#define SHOW_SLC_PROB_MAP
//#define SHOW_SLC_SEARCH_LINE
//#define SHOW_SLC_BUNDLE
//#define SHOW_SLC_CONTOUR_POINTS
//#define SHOW_SLC_MODELLED_OCCLUSION
//#define SHOW_SLC_SEARCH_LINE_WEIGHT
//#define SHOW_RUNTIME

enum {
	RUN_TRACK = 0,
	RUN_DEBUG = 1,
};

SLCTracker::SLCTracker(const cv::Matx33f& K, std::vector<Object3D*>& objects)
	: SLTracker(K, objects)
{

}

void SLCTracker::PreProcess() {
	UpdateHist();
}

void SLCTracker::Track(std::vector<cv::Mat>& imagePyramid, std::vector<Object3D*>& objects, int runs) {
#ifdef SHOW_SLC_DEBUG
	RunIteration(objects, imagePyramid, 2, 12, 2, 8.0, 1.2);
	RunIteration(objects, imagePyramid, 0, 12, 2, 8.0, 1.2, RUN_DEBUG);
	for (int iter = 0; iter < runs * 3; iter++) {
#else
	for (int iter = 0; iter < runs * 4; iter++) {
#endif
		RunIteration(objects, imagePyramid, 2, 12, 2, 8.0, 1.2, 0.2f);
	}

	for (int iter = 0; iter < runs * 2; iter++) {
		RunIteration(objects, imagePyramid, 1, 10, 2, 6.0, 1.0, 0.2f);
	}

	for (int iter = 0; iter < runs * 1; iter++) {
		RunIteration(objects, imagePyramid, 0, 8, 2, 4.0, 0.8, 0.2f);
	}
}

bool IsOccluded(int oid, int pixel_idx, int contour_idx, uchar* mask_data, float* depth_data) {
	uchar oidc = mask_data[pixel_idx];
	if (oidc != 0 && oidc != oid && depth_data[contour_idx] < depth_data[pixel_idx]) {
			return true;
	}
	return false;
}

inline float ColorWeight(float c, float x) {
	//return exp(c*(1 - x));
	return exp(1.2*(x - 1));
	return 1-(x * x - 1);
	return (1 - c * (x - 1) * c * (x - 1)) * (1 - c * (x - 1) * c * (x - 1));
}

inline float DistanceWeight(float lambda, float x) {
	return exp(lambda * x);
}

void SLCTracker::ComputeJac(
	Object3D* object,
	int m_id, 
	const cv::Mat& frame,
	const cv::Mat& mask_map,
	const cv::Mat& masks_map,
	const cv::Mat& depth_map,
	const cv::Mat& depth_inv_map, 
	cv::Matx66f& wJTJM, cv::Matx61f& JTM,
	float band_width,
	float ss) 
{
	float* depth_data = (float*)depth_map.ptr<float>();
	float* depth_inv_data = (float*)depth_inv_map.ptr<float>();
	uchar* frame_data = frame.data;
	uchar* mask_data = mask_map.data;
	uchar* masks_data = masks_map.data;
	const std::vector<std::vector<cv::Point> >& search_points = search_line->search_points;
	const std::vector<std::vector<cv::Point2f> >& bundle_prob = search_line->bundle_prob;

	JTM = cv::Matx61f::zeros();
	wJTJM = cv::Matx66f::zeros();
	float* JT = JTM.val;
	float* wJTJ = wJTJM.val;

	float zf = view->getZFar();
	float zn = view->getZNear();
	cv::Matx33f K = view->GetCalibrationMatrix().get_minor<3, 3>(0, 0);
	cv::Matx33f K_inv = K.inv();
	float* K_inv_data =  K_inv.val;
	float fx = K(0,0);
	float fy = K(1,1);

	for (int r = 0; r < search_points.size(); r++) {
		if (!search_line->actives[r])
			continue;

		int mid = search_points[r][search_points[r].size()-1].x;
		int eid = search_points[r][search_points[r].size()-1].y;

		float nx = search_line->norms[r].x;
		float ny = search_line->norms[r].y;

		//if (eid < 0)
		//	continue;

		float lambda1 = -1.2f;
		float we = eid < 0 ? 0.1f : ColorWeight(lambda1, scores[r]);

		int mx = search_points[r][mid].x;
		int my = search_points[r][mid].y;
		int zidx = my * depth_map.cols + mx;

		for (int c = 0; c < search_points[r].size()-1; c++) {
			int pidx = search_points[r][c].y * frame.cols + search_points[r][c].x;
			if ((c < mid && mask_data[pidx]) || (c > mid && !mask_data[pidx]))
				continue;

			float pyf = bundle_prob[r][c].x;
			float pyb = bundle_prob[r][c].y;
			if (eid > 0 && (c < eid && pyb < pyf || c > eid && pyf < pyb))
				continue;

			float dist = GetDistance(search_points[r][c], search_points[r][mid]);
			if (dist > band_width)
				continue;

			//if (m_id > 0 && IsOccluded(m_id, pidx, zidx, masks_data, depth_data))
			//	continue;

			if (c > mid)
				dist = -dist;

			float lambda2 = -0.25f;
			float wd = eid < 0 ? DistanceWeight(lambda2, 8.0f) : DistanceWeight(lambda2, GetDistance(search_points[r][c], search_points[r][eid]));

			// sl + cdw
			float wa = we * wd;
			
			// sl + dw
			//float wa = wd;

			// sl + cw
			//float wa = we;

			// sl
			//float wa = 1.0f;

			float s = ss;
			float s2 = s * s;
			float heaviside = 1.0f / float(CV_PI) * (-atan(dist * s)) + 0.5f;
			float dirac = (1.0f / float(CV_PI)) * (s / (dist * s2 * dist + 1.0f));
			float e = heaviside * (pyf - pyb) + pyb + 0.000001;
			float DlogeDe = -(pyf - pyb) / e;
			float constant_deriv = DlogeDe * dirac;
			float c2 = constant_deriv * constant_deriv;
			float w = -1.0f / log(e) * wa;

			float depth = 1.0f - depth_data[zidx];
			float D = 2.0f * zn * zf / (zf + zn - (2.0f * depth - 1.0) * (zf - zn));
			float Xc = D * (K_inv_data[0] * mx + K_inv_data[2]);
			float Yc = D * (K_inv_data[4] * my + K_inv_data[5]);
			float Zc = D;

			float J[6];

			float Zc2 = Zc*Zc;
			J[0] = nx * (-Xc*fx*Yc/Zc2) +     ny * (-fy -Yc*Yc*fy/Zc2);
			J[1] = nx * (fx + Xc*Xc*fx/Zc2) + ny * (Xc*Yc*fy/Zc2);
			J[2] = nx * (-fx*Yc/Zc)+          ny * (Xc*fy/Zc);
			J[3] = nx * (fx/Zc);
			J[4] =                            ny * (fy/Zc);
			J[5] = nx * (-Xc*fx/Zc2) +        ny * (-Yc*fy/Zc2);

			for (int n = 0; n < 6; n++) {
				JT[n] += constant_deriv * J[n] * wa;
			}

			for (int n = 0; n < 6; n++)
			for (int m = n; m < 6; m++) {
				wJTJ[n * 6 + m] += w * J[n] * c2 * J[m];
			}

			depth = 1.0f - depth_inv_data[zidx];
			D = 2.0f * zn * zf / (zf + zn - (2.0f * depth - 1.0) * (zf - zn));
			Xc = D * (K_inv_data[0] * mx + K_inv_data[2]);
			Yc = D * (K_inv_data[4] * my + K_inv_data[5]);
			Zc = D;

			Zc2 = Zc*Zc;
			J[0] = nx * (-Xc*fx*Yc/Zc2) +     ny * (-fy -Yc*Yc*fy/Zc2);
			J[1] = nx * (fx + Xc*Xc*fx/Zc2) + ny * (Xc*Yc*fy/Zc2);
			J[2] = nx * (-fx*Yc/Zc)+          ny * (Xc*fy/Zc);
			J[3] = nx * (fx/Zc);
			J[4] =                            ny * (fy/Zc);
			J[5] = nx * (-Xc*fx/Zc2) +        ny * (-Yc*fy/Zc2);

			for (int n = 0; n < 6; n++) {
				JT[n] += constant_deriv * J[n] * wa;
			}

			for (int n = 0; n < 6; n++)
			for (int m = n; m < 6; m++) {
				wJTJ[n * 6 + m] += w* J[n] * c2 * J[m];
			}
		}
	}

	for (int i = 0; i < wJTJM.rows; i++)
	for (int j = i + 1; j < wJTJM.cols; j++) {
		wJTJM(j, i) = wJTJM(i, j);
	}
}

#if 1
void SLCTracker::FindMatchPoint(float diff) {
	std::vector<std::vector<cv::Point> >& search_points = search_line->search_points;
	const std::vector<std::vector<cv::Point2f> >& bundle_prob = search_line->bundle_prob;
	scores.resize(search_points.size());

	for (int r = 0; r < bundle_prob.size(); ++r) {
		float score_min = 1000000;
		search_points[r][search_points[r].size()-1].y = -1;
		scores[r] = 0;
		for (int c = 3; c < bundle_prob[r].size()-3; ++c) {
			// SLE = 0.2; SLC = fabs(0.5)
			if (fabs(bundle_prob[r][c + 1].x - bundle_prob[r][c - 1].x) > 0.5f) {
			//if (bundle_prob[r][c + 1].x - bundle_prob[r][c - 1].x > diff) {
				float prbf = 
					bundle_prob[r][c-3].x*
					bundle_prob[r][c-2].x*
					bundle_prob[r][c-1].x;
				float prbb = 
					bundle_prob[r][c-3].y*
					bundle_prob[r][c-2].y*
					bundle_prob[r][c-1].y;
				float prff = 
					bundle_prob[r][c+1].x*
					bundle_prob[r][c+2].x*
					bundle_prob[r][c+3].x;
				float prfb = 
					bundle_prob[r][c+1].y*
					bundle_prob[r][c+2].y*
					bundle_prob[r][c+3].y;

				float pr_C = prbb*prff;
				float pr_F = prbf*prff;
				float pr_B = prbb*prfb;

				if ((pr_C > pr_F) && (pr_C > pr_B)) {
					float score = -log(prbb) - log(prff);
					if (score < score_min) {
						score_min = score;
						scores[r] = pr_C;
						search_points[r][search_points[r].size()-1].y = c;
					}
				}
			}
		}
	}
}
#endif

void SLCTracker::FindMatchPointMaxConv(SearchLine* search_line, float diff) {
	std::vector<std::vector<cv::Point> >& search_points = search_line->search_points;
	const std::vector<std::vector<cv::Point2f> >& bundle_prob = search_line->bundle_prob;
	scores.resize(search_points.size());

	for (int r = 0; r < bundle_prob.size(); ++r) {
		float prob_max = 0.0f;
		search_points[r][search_points[r].size()-1].y = -1;
		scores[r] = 0.0f;

		int mid = search_points[r][search_points[r].size() - 1].x;

		float nx = search_line->norms[r].x;
		float ny = search_line->norms[r].y;

		for (int c = 3; c < bundle_prob[r].size()-3; ++c) {
			// SLE = 0.2; SLC = fabs(0.5)
			//if (fabs(bundle_prob[r][c + 1].x - bundle_prob[r][c - 1].x) > 0.5f) {
			if (bundle_prob[r][c + 1].x - bundle_prob[r][c - 1].x > diff) {
				float prbf = 
					bundle_prob[r][c-3].x*
					bundle_prob[r][c-2].x*
					bundle_prob[r][c-1].x;
				float prbb = 
					bundle_prob[r][c-3].y*
					bundle_prob[r][c-2].y*
					bundle_prob[r][c-1].y;
				float prff = 
					bundle_prob[r][c+1].x*
					bundle_prob[r][c+2].x*
					bundle_prob[r][c+3].x;
				float prfb = 
					bundle_prob[r][c+1].y*
					bundle_prob[r][c+2].y*
					bundle_prob[r][c+3].y;

				float pr_C = prbb*prff;
				float pr_F = prbf*prff;
				float pr_B = prbb*prfb;

				// both tukey_weight(ex, 10) * slweight(pr_C, 1.0f); cam_regular 89.3
				if ((pr_C > pr_F) && (pr_C > pr_B)) {
					float convb =
						bundle_prob[r][c - 1].x +
						bundle_prob[r][c - 2].x +
						bundle_prob[r][c-3].x;

					float convf =
						bundle_prob[r][c + 1].x +
						bundle_prob[r][c + 2].x +
						bundle_prob[r][c+3].x;

					float conv = (convf - convb) / 3.0f;

					float ex = nx * (search_points[r][mid].x - search_points[r][c].x) + ny * (search_points[r][mid].y - search_points[r][c].y);
					//float we = edweight(ex, -0.2) * tukey_weight(1 - conv, 0.9f);
					//float we = tukey_weight(ex, 10) * ecweight(conv, 1.5f);
					float we = tukey_weight(ex, 10) * tukey_weight(1 - conv, 0.9f);
					//float we = tukey_weight(ex, 12) * ecweight(conv, 1.0f);
					//float we = edweight(ex, -0.25) * ecweight(conv, 1.2);

					if (we > prob_max) {
						prob_max = we;
						//scores[r] = we;
						scores[r] = conv;
						search_points[r][search_points[r].size()-1].y = c;
					}
				}
			}
		}
	}
}

static void ConvertMask(const cv::Mat& src_mask, cv::Mat& mask, uchar oid) {
	mask = cv::Mat(src_mask.size(), CV_8UC1, cv::Scalar(0));
	uchar depth = src_mask.type() & CV_MAT_DEPTH_MASK;

	if (CV_8U == depth && oid > 0) {
		for (int r = 0; r < src_mask.rows; ++r)
		for (int c = 0; c < src_mask.cols; ++c) {
			if (oid == src_mask.at<uchar>(r,c))
				mask.at<uchar>(r,c) = 255;
		}
	} else 
	if (CV_32F == depth) {
		for (int r = 0; r < src_mask.rows; ++r)
		for (int c = 0; c < src_mask.cols; ++c) {
			if (src_mask.at<float>(r,c))
				mask.at<uchar>(r,c) = 255;
		}
	}	else {
		LOG(ERROR) << "WRONG IMAGE TYPE";
	}
}

void SLCTracker::RunIteration(std::vector<Object3D*>& objects, const std::vector<cv::Mat>& imagePyramid, int level, int sl_len, int sl_seg, float band_width, float ss, int run_type) {
	int width = view->GetWidth();
	int height = view->GetHeight();
	view->setLevel(level);
	int numInitialized = 0;
	for (int o = 0; o < objects.size(); o++) {
		if (!objects[o]->isInitialized())
			continue;

		numInitialized++;

		cv::Rect roi = Compute2DROI(objects[o], cv::Size(width / pow(2, level), height / pow(2, level)), 8);
		if (roi.area() == 0)
			continue;

		while (roi.area() < 3000 && level > 0) {
			level--;
			view->setLevel(level);
			roi = Compute2DROI(objects[o], cv::Size(width / pow(2, level), height / pow(2, level)), 8);
		}
	}

	view->setLevel(level);
	view->RenderSilhouette(std::vector<Model*>(objects.begin(), objects.end()), GL_FILL);
	cv::Mat depth_map = view->DownloadFrame(View::DEPTH);

	cv::Mat masks_map;
	if (numInitialized > 1) {
		masks_map = view->DownloadFrame(View::MASK);
	}	else {
		masks_map = depth_map;
	}

	for (int o = 0; o < objects.size(); o++) {
		if (!objects[o]->isInitialized())
			continue;

		cv::Rect roi = Compute2DROI(objects[o], cv::Size(width / pow(2, level), height / pow(2, level)), 8);
		if (roi.area() == 0)
			continue;

		int m_id = (numInitialized <= 1) ? -1 : objects[o]->getModelID();
		cv::Mat mask_map;
		ConvertMask(masks_map, m_id, mask_map);

		search_line->FindSearchLine(mask_map, imagePyramid[level], sl_len, sl_seg, true);

		if (numInitialized > 1) {
			FilterOccludedPoint(masks_map, depth_map);
		}

		GetBundleProb(imagePyramid[level], o);

		FindMatchPoint(0.5);

		view->RenderSilhouette(objects[o], GL_FILL, true);
		cv::Mat depth_inv_map = view->DownloadFrame(View::DEPTH);

		cv::Matx66f wJTJ;
		cv::Matx61f JT;
		ComputeJac(objects[o], m_id, imagePyramid[level], mask_map, masks_map, depth_map, depth_inv_map, wJTJ, JT, band_width, ss);

		cv::Matx44f T_cm = Transformations::exp(-wJTJ.inv(cv::DECOMP_CHOLESKY) * JT) * objects[o]->getPose();
		objects[o]->setPose(T_cm);
	}
}