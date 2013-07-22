#include "KalmanSingleState.h"

/**
* Constructors.
*/
KalmanSingleState::KalmanSingleState():
    initialized(false),q(0.0),r(0.0),k(0.0),p(0.0),x(0.0) {}
KalmanSingleState::KalmanSingleState(double initQ, double initR, double initP, double initValue)
{
    init(initQ, initR, initP, initValue);
}

/**
* Destructor.
*/
KalmanSingleState::~KalmanSingleState() {}

/**
* Initialize the filter.
*/
void KalmanSingleState::init(double initQ, double initR, double initP, double initValue)
{
    q = initQ;
    r = initR;
    p = initP;
    x = initValue;
    raw = initValue;
    initialized = true;
}

/**
* Update the filter with the latest raw measurement.
*/
double KalmanSingleState::update(double measurement)
{
    raw = measurement;
    p = p + q;
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1 - k) * p;
    return x;
}

/**
* Get the current filtered value.
*/
double KalmanSingleState::getValue()
{
    return x;
}
