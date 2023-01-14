/* By Tyler Clarke. Base class for polymorphing highly compatible motors */
#pragma once

class BaseMotor {
public:
    virtual void SetPercent(double percent) = 0;
    virtual void setInverted() = 0;
    virtual void SetP(double kP) = 0;
    virtual void SetI(double kI) = 0;
    virtual void SetD(double kD) = 0;
    virtual void SetF(double kF) = 0;
    virtual void SetOutputRange(double kPeakOF, double kPeakOR, double kNominalOF, double kNominalOR) = 0;
    virtual double GetPosition() = 0;
    virtual double GetVelocity() = 0;
    virtual void SetPositionPID(double position) = 0;
    virtual void SetSpeedPID(double speed) = 0;
    virtual bool isAtZero() = 0;
};
