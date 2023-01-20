/* Spark MAX motor; can be used as a BaseMotor. */


#include <rev/CANSparkMax.h> /* Requires REVLib */
#include "BaseMotor.hpp"


struct _SparkMotorEncoderControlContainer{
    rev::SparkMaxRelativeEncoder encoder;
    rev::SparkMaxPIDController pid;
};


class SparkMotor : public BaseMotor {
    bool invert = false;
public:
    rev::CANSparkMax* spark;
    _SparkMotorEncoderControlContainer* controls;

    SparkMotor(int canID){
        spark = new rev::CANSparkMax (canID, rev::CANSparkMax::MotorType::kBrushless);
        controls = new _SparkMotorEncoderControlContainer {spark -> GetEncoder(), spark -> GetPIDController()};
    }

    void setInverted() {
        invert =! invert;
        spark -> SetInverted(invert);
    }

    void setInverted(bool inverted){ // Function overloads are really cool.
        spark -> SetInverted(inverted);
    }
    
    void SetPercent(double percent){
        spark -> Set(percent);
    }

    void SetP(double kP){
        controls -> pid.SetP(kP);
    }

    void SetI(double kI){
        controls -> pid.SetI(kI);
    }

    void SetD(double kD){
        controls -> pid.SetD(kD);
    }

    void SetF(double kF){
        controls -> pid.SetFF(kF);
    }

    void SetOutputRange(double kPeakOF, double kPeakOR, double kNominalOF = 0, double kNominalOR = 0){
        controls -> pid.SetOutputRange(kPeakOR, kPeakOF); // Ignore nominal values; this is Spark.
    }
        
    double GetPosition() {
        return controls -> encoder.GetPosition();
    }
    
    double GetVelocity() {
        return controls -> encoder.GetVelocity();
    }
        
    void SetPositionPID(double position){
        controls -> pid.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
    }

    void SetSpeedPID(double speed){
        controls -> pid.SetReference(speed, rev::CANSparkMax::ControlType::kVelocity);
    }
        
    bool isAtZero() {
        return GetPosition() == 0;
    }

    void SetEncoderPosition(double pos) {
        controls -> encoder.SetPosition(pos);
    }
};