#pragma once

#include <SparkMotor.hpp>
#include <ctre/Phoenix.h>

#define BANGBANG_ERROR_SPEED .1
#define PID_ERROR_MARGIN  5
#define PI 3.14159

struct SwerveMotor : public SparkMotor {
    double coeff = 1;

    void _configPIDPositionAt(double pos) {
        spark -> setPosition(pos);
    }

    void configurePIDCoeff(double coef) {
        coeff = coef;
    }

    void setPosPIDTo(double pos) {
        pos *= coeff;
        SetPositionPID(pos);
    }

    double getPIDPos() {
        return GetPosition();
    }
    
    bool isAtPos(double pos, double errorMarg) {
        pos *= coeff;
        return GetPosition() <= (errorMarg - pos) && GetPosition() >= (errorMarg + pos);
    }
};

class SwerveModule {
    SwerveMotor* speed;
    SwerveMotor* direction;
    CANCoder* cancoder;
    CANCoderConfiguration config;
    
    short role;           /* This is determined by what side the module is on, and it does different stuff based off that.  
                                    Key: 1 = Frontleft, 2 = Frontright, 3 = Backright, 4 = Backleft                               
                          */
    bool linked = false;  
    bool isAtAngle; 
    bool zeroed;
    
    SwerveModule* linkSwerve;

public:
    SwerveModule(int speedID, int direcID, int CanCoderID, short roll, bool speedInverted=false, bool direcInverted=false) {
        speed = new SwerveMotor {speedID};
        direction = new SwerveMotor {direcID};
        cancoder = new CANCoder {CanCoderID};
        role = roll;
        
        if (speedInverted) {
            speed -> setInverted();
        }
        if (direcInverted) {
            direction -> setInverted();
        }
        
        config.sensorCoefficient = 360.0/4096.0;
        config.unitString = "degrees";
        config.sensorTimeBase = SensorTimeBase::PerSecond;
    }
    
    void link(SwerveModule* Link) {
        linked = true;           
        linkSwerve = Link; 
    }

    double coterminal(double thang){
        while (thang >= 360){
            thang -= 360;
        }
        while (thang < 0){
            thang += 360;
        }
        return thang;
    }
        
    double coterminalShortest(double current, double goal) {
        if (fabs(goal - current) > 180) {
            return 360 - coterminal(current - goal);
        }
        else {
            return goal - current;
        }
    }
            
    void calibrate(double offset) {            // Intented to be ran once, or after the build team destroys the robot. Calibrates the CanCoder 
        cancoder -> ConfigAllSettings(config);
        cancoder -> ConfigMagnetOffset(offset);

        if (linked) {
            linkSwerve -> calibrate(offset);
        }
    }       
    
    void zero() {
        double currentPos = cancoder -> GetAbsolutePosition();
        double target = coterminalShortest(currentPos, 0);
        
        if (cancoder -> HasResetOccurred()) {
            zeroed = false;
        }

        if (!zeroed) { 
            if (target > 0) {
                direction -> SetPercent(BANGBANG_ERROR_SPEED);
            }
            else if (target < 0) {
                direction -> SetPercent(-BANGBANG_ERROR_SPEED);
            }
            else {
                direction -> SetPercent(0);
                zeroed = true;
            }
        }
        
        if (linked) {
            linkSwerve -> zero();
        }
    }
    
    void notZeroed() {           // Run when TeleopInit() or DisabledLoop(); this will tell zero() to zero when called again
        zeroed = false;
    }

    void SetDirectionAngle(double angle) {
        double currentPos = cancoder -> GetAbsolutePosition();
        double desiredPos = coterminalShortest(currentPos, angle);
        
        direction -> _configPIDPositionAt(currentPos);
        
        direction -> setPIDPosTo(currentPos + desiredPos);
        
        if (direction -> isAtPos(currentPos + desiredPos, PID_ERROR_MARGIN)) {
            
        }
        if (linked) {
            linkSwerve -> SetDirectionAngle(angle);
        }
    }     

    void orient(float percent, bool left) {
        if (left) {
            if (role == 1) {
                direction -> SetPositionPID()
            }
            else if (role == 2) {
                SetDirectionAngle(90);
            }
            else if (role == 3) {
                SetDirectionAngle(0);
            }
            else if (role == 4) {
                SetDirectionAngle(270);
            }
        }

        else {
            if (role == 1) {
                SetDirectionAngle(270);
            }
            else if (role == 2) {
                SetDirectionAngle(180);
            }
            else if (role == 3) {
                SetDirectionAngle(90);
            }
            else if (role == 4) {
                SetDirectionAngle(0);
            }
        }
        speed -> SetPercent(percent);

        if (linked) {
            linkSwerve -> orient(percent, left);
        }
    }  

    /*                            Tests                             */        

    double GetAbsoluteCANCoderPos() {
        return cancoder -> GetAbsolutePosition();
    }                               
};
