#ifndef CERES_UTILS_HPP
#define CERES_UTILS_HPP

struct parameterBlockTrajectoryWP{
	double parameter[6];
};

struct parameterBlockPathWP{
	double parameter[3];
};

struct parameterBlockContinuousPath{
    double parameter[15];
};

struct parameterBlockContinuousPathConstant{
    double parameter[3];
};

#endif