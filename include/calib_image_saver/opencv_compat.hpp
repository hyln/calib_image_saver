#ifndef CALIB_IMAGE_SAVER_OPENCV_COMPAT_HPP
#define CALIB_IMAGE_SAVER_OPENCV_COMPAT_HPP

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#ifndef CV_AA
#define CV_AA cv::LINE_AA
#endif

#ifndef CV_ADAPTIVE_THRESH_MEAN_C
#define CV_ADAPTIVE_THRESH_MEAN_C cv::ADAPTIVE_THRESH_MEAN_C
#endif

#ifndef CV_CALIB_CB_ADAPTIVE_THRESH
#define CV_CALIB_CB_ADAPTIVE_THRESH cv::CALIB_CB_ADAPTIVE_THRESH
#endif

#ifndef CV_CALIB_CB_FAST_CHECK
#define CV_CALIB_CB_FAST_CHECK cv::CALIB_CB_FAST_CHECK
#endif

#ifndef CV_CALIB_CB_FILTER_QUADS
#define CV_CALIB_CB_FILTER_QUADS cv::CALIB_CB_FILTER_QUADS
#endif

#ifndef CV_CALIB_CB_NORMALIZE_IMAGE
#define CV_CALIB_CB_NORMALIZE_IMAGE cv::CALIB_CB_NORMALIZE_IMAGE
#endif

#ifndef CV_CHAIN_APPROX_SIMPLE
#define CV_CHAIN_APPROX_SIMPLE cv::CHAIN_APPROX_SIMPLE
#endif

#ifndef CV_FONT_HERSHEY_SIMPLEX
#define CV_FONT_HERSHEY_SIMPLEX cv::FONT_HERSHEY_SIMPLEX
#endif

#ifndef CV_GRAY2RGB
#define CV_GRAY2RGB cv::COLOR_GRAY2RGB
#endif

#ifndef CV_RETR_CCOMP
#define CV_RETR_CCOMP cv::RETR_CCOMP
#endif

#ifndef CV_SHAPE_CROSS
#define CV_SHAPE_CROSS cv::MORPH_CROSS
#endif

#ifndef CV_SHAPE_RECT
#define CV_SHAPE_RECT cv::MORPH_RECT
#endif

#ifndef CV_TERMCRIT_EPS
#define CV_TERMCRIT_EPS cv::TermCriteria::EPS
#endif

#ifndef CV_TERMCRIT_ITER
#define CV_TERMCRIT_ITER cv::TermCriteria::COUNT
#endif

#ifndef CV_THRESH_BINARY
#define CV_THRESH_BINARY cv::THRESH_BINARY
#endif

#ifndef CV_THRESH_BINARY_INV
#define CV_THRESH_BINARY_INV cv::THRESH_BINARY_INV
#endif

#ifndef CV_WINDOW_NORMAL
#define CV_WINDOW_NORMAL cv::WINDOW_NORMAL
#endif

#ifndef cvStartWindowThread
#define cvStartWindowThread() ((void)0)
#endif

#endif // CALIB_IMAGE_SAVER_OPENCV_COMPAT_HPP
