#pragma once

#include <SparkMotor.hpp>
#include <ctre/Phoenix.h>

#define BANGBANG_ERROR_SPEED 68


class SwerveModule {
    SparkMotor* speed;
    SparkMotor* direction;
    CANCoder* cancoder;
    CANCoderConfiguration config;
    
    short role;           // This is determined by what side the module is on, and it does different stuff based off that.
    bool linked = false;   // There is a 'main' module that is linked to other modules
    bool zeroed = false;
    float bangErrorSpeed = .1;
    
    SwerveModule* linkSwerve;
    
    
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

    void calibrate() {            // Intented to be ran once, or after the build team destroys the robot. Calibrates the CanCoder 
        cancoder -> ConfigAllSettings(config);
        cancoder -> SetPosition(0);

        if (linked) {
            linkSwerve -> calibrate();
        }
    }       
    
    void zero() {
        if (cancoder -> HasResetOccurred()) {
            zeroed = false;
        }

        if (!zeroed) { 
            if (cancoder -> GetPosition() > 0) {
                direction -> SetPercent(BANGBANG_ERROR_SPEED);
            }
            else if (cancoder -> GetPosition() < 0) {
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
        double currentPos = cancoder -> GetPosition();

        if ( currentPos < angle ) {
            direction -> SetPercent(BANGBANG_ERROR_SPEED);
        }
        else if ( currentPos > angle ) {
            direction -> SetPercent(-BANGBANG_ERROR_SPEED);
        }
        else {
            direction -> SetPercent(0);
        }
        
        if (linked) {
            linkSwerve -> SetDirectionAngle(angle);
        }
    }                                            
};
