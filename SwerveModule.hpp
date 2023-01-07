#include <FRL/SparkMotor.hpp>

#define BANGBANG_ERROR_SPEED 68
#define CANCODER_MARG 15


class SwerveModule {
    SparkMotor* speed;
    SparkMotor* direction;
    CANCoder* cancoder;
    CANCoderConfiguration config;
    
    short role;           // This is determined by what side the module is on, and it does different stuff based off that.
    bool main = false;   // There is a 'main' module that is linked to other modules
    
    SwerveModule link1;
    SwerveModule link2;
    SwerveModule link3;
    
    
    SwerveModule(int speedID, int direcID, int CanCoderID, short roll, bool speedInverted=false, bool direcInverted=false) {
        speed = new SparkMotor {speedID};
        direction = new SparkMotor {direcID};
        cancoder = new CANCoder {CanCoderID};
        role = roll;
        
        if (speedInverted) {
            speed -> setInverted()
        }
        if (direcInverted) {
            direc -> setInverted()
        }
        
        config.sensorCoefficient = 2 * M_PI / 4096.0;      // Convert to radians
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase::PerSecond;
    }
    
    void calibrate() {            // Intented to be ran once, or after the build team destroys the robot. Calibrates the CanCoder 
        cancoder -> ConfigAllSettings(config);
        cancoder -> SetPosition(0);
    }       
    
    void link(SwerveModule* Link1, SwerveModule* Link2, SwerveModule* Link3) {
        main = true;           // Only one module will have main as true; this will be the 'main' module; the one you control with.
        link1 = &Link1;
        link2 = &Link2;
        link3 = &Link3;
        
        
    void zero() {
        if (cancoder -> )
        
        if (cancoder -> GetPosition() > 0) {
            direction -> SetPercent(BANGBANG_ERROR_SPEED);
        }
        else {
            direction -> SetPercent(-BANGBANG_ERROR_SPEED);
        }
        
        if (main) {
            link1.zero();
            link2.zero();
            link3.zero();
        }
    }
    
    void SetDirectionAngle(double angle) {
        double currentPos = cancoder -> GetPosition();
        if ( downMarg < angle) {
            direction -> SetPercent(BANGBANG_ERROR_SPEED);
        }
        else if (upMarg > angle)) {
            direction -> SetPercent(-BANGBANG_ERROR_SPEED);
        }
        
        if (main) {
            link1.SetDirectionAngle(angle);
            link2.SetDirectionAngle(angle);
            link3.SetDirectionAngle(angle);
        }
    } 
};

