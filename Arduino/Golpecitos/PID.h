#ifndef PID_h
#define PID_h

class PID{

    public:
        PID(float _kp, float _ki, float _kd , float _minSat, float _maxSat); // interesante añadirle antiwindup


        float kp() const { return mKp; }
        float ki() const { return mKi; }
        float kd() const { return mKd; }

        void kp(float _kp) { mKp = _kp; }
        void ki(float _ki) { mKi = _ki; }
        void kd(float _kd) { mKd = _kd; }

        float reference() { return mReference; }
        void reference(float _ref) { mReference = _ref; mAccumErr = 0; mLastError = 0; mLastResult = 0;}

        float update(float &_val, float _incT);

        void setSaturations(float _min, float _max) { mMinSat = _min; mMaxSat = _max; }
        void getSaturations(float _min, float _max) { _min = mMinSat; _max = mMaxSat; }


    private:
        float mReference;
        float mKp, mKi, mKd;

        float mMinSat, mMaxSat;

        float mLastResult, mLastError, mAccumErr;

};


#endif