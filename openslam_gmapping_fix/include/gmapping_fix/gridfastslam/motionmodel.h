#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include <gmapping_fix/utils/point.h>
#include <gmapping_fix/utils/stat.h>
#include <gmapping_fix/utils/macro_params.h>

namespace  GMapping_Fix { 

struct MotionModel{
	OrientedPoint drawFromMotion(const OrientedPoint& p, double linearMove, double angularMove) const;
	OrientedPoint drawFromMotion(const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold) const;
	Covariance3 gaussianApproximation(const OrientedPoint& pnew, const OrientedPoint& pold) const;
	double srr, str, srt, stt;
};

};

#endif
