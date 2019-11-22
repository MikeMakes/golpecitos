#ifndef PID_h
#define PID_h

class PID{

    public:
        PID(float _kp, float _ki, float _kd , float _minSat, float _maxSat); // interesante añadirle antiwindup

        float reference() { return mReference; }
        void reference(float _ref) { mReference = _ref; mAccumErr = 0; mLastError = 0; mLastResult = 0;}

        float update(float &_val, float _incT);

        void setSaturations(float _min, float _max) { mMinSat = _min; mMaxSat = _max; }

        float mKp, mKi, mKd;
        float mMinSat, mMaxSat;
    private:
        float mReference;


        float mLastResult, mLastError, mAccumErr;

};


#endif