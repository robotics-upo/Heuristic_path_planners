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

struct parameterBlockChebyshev{
    double parameter[18];
};

struct parameterBlockReducedChebyshev{
    double parameter[12];
};

struct parameterBlockChebyshevTime{
    double parameter[19];
};

#endif