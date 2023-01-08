#pragma once

#include <SparkMotor.hpp>
#include <ctre/Phoenix.h>

#define BANGBANG_ERROR_SPEED .1

class SwerveModule {
    SparkMotor* speed;
    SparkMotor* direction;
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
        speed = new SparkMotor {speedID};
        direction = new SparkMotor {direcID};
        cancoder = new CANCoder {CanCoderID};
        role = roll;
        
        if (speedInverted) {
            speed -> setInverted();
        }
        if (direcInverted) {
            direction -> setInverted();
        }
        
        config.sensorCoefficient = 2 * M_PI / 4096.0;      // Convert to radians
        config.unitString = "rad";
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
            
    void calibrate() {            // Intented to be ran once, or after the build team destroys the robot. Calibrates the CanCoder 
        cancoder -> ConfigAllSettings(config);
        cancoder -> SetPosition(0);

        if (linked) {
            linkSwerve -> calibrate();
        }
    }       
    
    void zero() {
        double target = coterminalShortest(cancoder -> GetAbsolutePosition(), 0);
        
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
        double target = coterminalShortest(currentPos, angle);
        
        if ( target < 0 ) {
            direction -> SetPercent(BANGBANG_ERROR_SPEED);
        }
        else if ( target > 0 ) {
            direction -> SetPercent(-BANGBANG_ERROR_SPEED);
        }
        else {
            direction -> SetPercent(0);
        }
        
        if (linked) {
            linkSwerve -> SetDirectionAngle(angle);
        }
    }     

    void orient(float speed, bool left) {
        if (left) {
            if (role == 1) {
                direction -> SetDirectionAngle(180);
            }
            else if (role == 2) {
                direction -> SetDirectionAngle(90);
            }
            else if (role == 3) {
                direction -> SetDirectionAngle(0);
            }
            else if (role == 4) {
                direction -> SetDirectionAngle(270);
            }
        }

        else {
            if (role == 1) {
                direction -> SetDirectionAngle(270);
            }
            else if (role == 2) {
                direction -> SetDirectionAngle(180);
            }
            else if (role == 3) {
                direction -> SetDirectionAngle(90);
            }
            else if (role == 4) {
                direction -> SetDirectionAngle(0);
            }
        }
        speed -> SetPercent(speed);
    }                                       
};
