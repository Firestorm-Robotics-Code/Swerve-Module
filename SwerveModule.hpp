#include <TalonFXMotor.hpp>

class SwerveModule {
    TalonFXMotor* speed;
    TalonFXMotor* direction;
    short role;           // This is determined by what side the module is on, and it does different stuff based off that.
    bool linked = false;   // Allows to link with other modules (to save time)
    
    SwerveModule link1;
    SwerveModule link2;
    SwerveModule link3;
    
    
    SwerveModule(int speedID, int direcID, short roll, bool speedInverted=false, bool direcInverted=false) {
        speed = new TalonFXMotor {speedID};
        direction = new TalonFXMotor {direcID};
        role = roll;
        
        if (speedInverted) {
            speed -> setInverted()
        }
        if (direcInverted) {
            direc -> setInverted()
        }
    }
    
    void calibrate() {            // Intented to be ran once, or after the build team destroys the robot. Calibrates the CanCoder.
    
    }       // Blank for now
    
    void link(SwerveModule* Link1, SwerveModule* Link2, SwerveModule* Link3) {
        linked = true;           // Only one module will have linked as true; this will be the 'main' module; the one you control with.
        link1 = &Link1;
        link2 = &Link2;
        link3 = &Link3;
        
        
    void zero() {
        direction -> SetPosPID(0);
        
        if (linked) {
            link1.zero();
            link2.zero();
            link3.zero();
        }
    }
    
    void SetDirecAngle(double angle) {
    
    }              // Also blank
};

