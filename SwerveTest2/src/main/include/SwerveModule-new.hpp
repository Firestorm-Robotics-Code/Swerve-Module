/*
    Updated swerve module. See the master version in FirestormRoboticsLibrary.
    Keeping this here so (new) code compiles.
    I'm working on migrating out all the new code to my own project but in case I need to compile this with the verified swerve code
*/


#pragma once

#include "SparkMax.hpp"
#include <ctre/Phoenix.h>
#include <iostream>
#include <PidController.hpp>

#define BANGBANG_ERROR_SPEED .05
#define PID_ERROR_MARGIN 5
#define BRAKE_SPEED .4                                 // To run the wheels against each other; this of course wil have to be tune

struct SwerveMotor : public SparkMotor {
    double coeff = 1;

    SwerveMotor(int id) : SparkMotor(id) {                      // Constructor stuff

    }

    void _configPIDPositionAt(double pos) {
        controls -> encoder.SetPosition(pos);
    }

    void configurePIDCoeff(double coef) {
        coeff = coef;
    }

    void setPosPIDTo(double pos) {
        pos *= coeff;
        SetPositionPID(pos);
    }
};


class SwerveModule {
    SwerveMotor* speed;
    SwerveMotor* direction;
    PIDPositionController* directionController;
    CANCoder* cancoder;
    CANCoderConfiguration config;

    double curPercent; // So multiple commands can alter speed
    
    short role;           /* This is determined by what side the module is on, and it does different stuff based off that.  
                                    Key: 1 = Frontleft, 2 = Frontright, 3 = Backright, 4 = Backleft                               
                          */
    bool isLinked = false;  
    const double angleConversion = 360.0 / 4096.0;
    const double positionConversion = 1/angleConversion; // For any fraction x/y, 1/(x/y) == y/x. Proof: 1/2 = 0.5, 1/0.5 = 2 = 2/1; 1/3 = 0.33333, 1/0.33333 = 3 = 3/1.
    
    SwerveModule* linkSwerve;
    bool zeroed;

    double encoderOffset; // Being CTRE their calibration code doesn't work. Like, at all. So this is the "custom" version.

public:
    SwerveModule(int speedID, int directionIdentifier, int CanCoderID, short roll, double offset, bool speedInverted=false, bool direcInverted=false) {
        encoderOffset = offset;
        std::cout << "Swerve: " << speedID << ", " << directionIdentifier << ", " << CanCoderID << ", " << roll << std::endl;
        speed = new SwerveMotor {speedID};
        direction = new SwerveMotor {directionIdentifier};
        cancoder = new CANCoder {CanCoderID};
        role = roll;

        directionController = new PIDPositionController (direction);
        directionController -> constants.P = 0.0005;
        //directionController -> constants.I = 0.0001;
        directionController -> constants.MaxOutput = 0.2;
        directionController -> constants.MinOutput = -0.2;
        directionController -> SetCircumference(4096);
        
        if (speedInverted) {
            speed -> setInverted(true);
        }
        else{
            speed -> setInverted(false);
        }
        if (direcInverted) {
            direction -> setInverted(true);
        }
        else{
            direction -> setInverted(false);
        }

        config.sensorTimeBase = SensorTimeBase::PerSecond;
    }
    
    void linkToSwerve(SwerveModule* LinkSwerve) {
        isLinked = true;           
        linkSwerve = LinkSwerve; 
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
        cancoder -> ConfigMagnetOffset(GetAbsoluteCANCoderPos()); // calibrate to an absolute encoder pos as opposed to a set value - this means they'll all be calibrated properly by themselves. What big boys they are!

        if (isLinked) {
            linkSwerve -> calibrate();
        }
    }
    
    void zero() { /* Works, but not well. Researching alternatives now. */
        /* Old version; slight changes
        if (isLinked){
            linkSwerve -> zero();
        }
        double currentPos = cancoder -> GetAbsolutePosition();
        double target = coterminalShortest(currentPos, 0);
        
        if (cancoder -> HasResetOccurred()) {
            zeroed = false;
        }

        if (!zeroed) { 
            if (currentPos > 180) {
                direction -> SetPercent(-BANGBANG_ERROR_SPEED);
            }
            else if (currentPos < 180) {
                direction -> SetPercent(BANGBANG_ERROR_SPEED);
            }
            else {
                direction -> SetPercent(0);
                zeroed = true;
            }
        }
        
        if (isLinked) {
            linkSwerve -> zero();
        }*/
        /* Edits and fixes by Tyler
        double currentPos = GetAbsoluteCANCoderPos();
        frc::SmartDashboard::PutNumber("tuba", currentPos);

        if (currentPos > 181) {
            direction -> SetPercent(-BANGBANG_ERROR_SPEED);
        }
        else if (currentPos < 179) {
            direction -> SetPercent(BANGBANG_ERROR_SPEED);
        }
        else {
            direction -> SetPercent(0);
            zeroed = true;
        }
        
        if (isLinked) {
            linkSwerve -> zero();
        }*/
        zeroed = true;
        direction -> SetEncoderPosition(GetAbsoluteCANCoderPos()); /* I reconfigured the CANCoders to report their normal 0-4096; we can convert to degrees later 
            Necessary for testing.
        */
    }
    
    void notZeroed() {           // Run when TeleopInit() or DisabledLoop(); this will tell zero() to zero when called again
        zeroed = false;
    }

    void SetDirectionAngle(double targetPos) {
        //double currentPos = GetAbsoluteCANCoderPos();
        
        //targetPos *= positionConversion;      
        
        /*TurnPID.SetPosition(currentPos, targetPos);

        direction -> SetPercent(TurnPID.Update(currentPos, direction -> GetVelocity()));*/
        //direction -> SetPositionPID(targetPos);
        directionController -> SetPosition(targetPos);
        directionController -> Update(GetAbsoluteCANCoderPos());
        if (isLinked){
            linkSwerve -> SetDirectionAngle(targetPos);
        }
    }

    void MovePercent(double spd, bool followLink = true){
        curPercent += spd;
        if (isLinked && followLink){
            linkSwerve -> MovePercent(spd);
        }
    }

    void ApplySpeed(){
        speed -> SetPercent(curPercent);
        curPercent = 0; // Velocity ain't "sticky", this is a safety thing
        if (isLinked){
            linkSwerve -> ApplySpeed();
        }
    }

    void orient(float percent, bool left) {
        if (left) {
            SetDirectionAngle(coterminal(270 - (role * 90)));              // Math time
        }

        else {
            SetDirectionAngle(coterminal(360 - (role * 90)));           
        }
        
        speed -> SetPercent(percent);

        if (isLinked) {
            linkSwerve -> orient(percent, left);
        }
    }  
    
    void brake() {
        if (role == 1 || role == 2) {
            SetDirectionAngle(180);
        }
        
        else {
            SetDirectionAngle(0);
        }
        
        if (isLinked) {
            linkSwerve -> brake();
        }                // This function is not complete

    /*                            Tests                             */        

    long GetAbsoluteCANCoderPos() {
        return smartLoop(cancoder -> GetAbsolutePosition() - encoderOffset);
    }   

    void SetMagOffset(double offset) {
        cancoder -> ConfigMagnetOffset(offset);
    }    

    void Set(float lukeiscool) {
        speed -> SetPercent(lukeiscool);
        direction -> SetPercent(lukeiscool);
    }
    
    bool checkZeroed(){
        return zeroed && (isLinked ? linkSwerve -> checkZeroed() : true);
        /* Ternary operator!
            condition ? valueIfTrue : valueIfFalse
            basically short form of,

            value;
            if (condition){
                value = valueIfTrue;
            }
            else {
                value = valueIfFalse;
            }

            This line works because boolean && true == boolean; true && true is true and false && true is false.
            So really what it's saying is "If I have a link, return the binary AND of my zero state and that link's zero state, otherwise just return my zero state."
        */
    }
};
