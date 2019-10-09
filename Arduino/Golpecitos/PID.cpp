#include <PID.h>


//---------------------------------------------------------------------------------------------------------------------
PID::PID(float _kp, float _ki, float _kd ,  float _minSat, float _maxSat) {
    mKp = _kp;
    mKi = _ki;
    mKd = _kd;
    mMinSat =  _minSat;
    mMaxSat =  _maxSat;
    
}

//---------------------------------------------------------------------------------------------------------------------
float PID::update(float & _val, float _incT) {
    float dt = _incT; // TODO 666 input arg?

    // Calculate error
	float err = mReference - _val;

    mAccumErr += err*dt;

	// Compute PID
    mLastResult = mKp*err + mKi*mAccumErr + mKd*(err- mLastError)/dt;
    mLastError = err;

	// Saturate signal
    if (mLastResult > mMaxSat){
        mLastResult = mMaxSat;
    }else if (mLastResult < mMinSat){
        mLastResult = mMinSat;
    }

    return mLastResult;
}