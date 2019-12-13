#include "PID.h"


//---------------------------------------------------------------------------------------------------------------------
PID::PID(float _kp, float _ki, float _kd ,  float _minSat, float _maxSat, float _maxAntiW, float _minAntiW) {
    mKp = _kp;
    mKi = _ki;
    mKd = _kd;
    mMinSat =  _minSat;
    mMaxSat =  _maxSat;
    mMinAntiW = _minAntiW;
    mMaxAntiW = _maxAntiW;
    
}

//---------------------------------------------------------------------------------------------------------------------
float PID::update(float & _val, float _incT) {
    
    float dt = _incT; 

    // Calculate error
	float err = mReference - _val;

    mAccumErr += err*dt; //Integrativo
    
	mAccumErr = min( max(mAccumErr, mMinAntiW) , mMaxAntiW );

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