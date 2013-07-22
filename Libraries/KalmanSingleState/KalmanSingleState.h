#ifndef KALMANSINGLESTATE_H
#define KALMANSINGLESTATE_H

/**
* A Kalman Filter with only one State to track (no matrix).
*/
class KalmanSingleState
{
    public:
        KalmanSingleState();
        KalmanSingleState(double initQ, double initR, double initP, double initValue);
        virtual ~KalmanSingleState();
        void init(double initQ, double initR, double initP, double initValue);
        double update(double measurement);
        double getValue();
        bool initialized;
		double x;
		double raw;
    protected:
    private:
        double q;
        double r;
        double p;
        double k;
};

#endif // KALMANSINGLESTATE_H
