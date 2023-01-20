/*
    Keeping this here so code compiles. See the master version in FirestormRoboticsLibrary.
*/

// This is entirely based off the code at https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control, squeezed into a C++ format
#include <BaseMotor.hpp>
#include <frc/Timer.h>


double smartLoop(double pos, double round = 4096){ /* Coterminality function, because modulus don't work too well */
  while (pos > round){
    pos -= round;
  }
  while (pos < 0){
    pos += round;
  }
  return pos;
}


struct PIDConstants {
    double P = 0; // Defaults: configure it yourself if you want it to run
    double I = 0;
    double D = 0;
    double F = 0;
    double iZone = 0;
    double MinOutput = -1;
    double MaxOutput = 1;
};


// Controls any BaseMotor (so modernly compatible with our libraries) and has easily-tuned coefficients + code that makes it normalize frequency
// In other words, a very cool piece of code ;)

class PIDPositionController { // Position only; I don't want to code speed too (although I'm sure most of it would be the same)
    BaseMotor* motor;
    double lastTime;
    float hz;

    double setPoint = 0;

    double previousError = 0;
    double iState = 0;

    double curPos;

    long rotationLength = -1; // -1 = no looping

    double loopize(double set, double cur){
        if (std::abs(set - cur) >= rotationLength/2){
            if (set > cur){
                return -(rotationLength - set + cur);
            }
            else{
                return rotationLength - cur + set;
            }
        }
        else{
            return set - cur;
        }
    }

    double getError(double set, double cur){
        if (rotationLength == -1){
            return set - cur;
        }
        else{
            return loopize(set, cur);
        }
    }

    double DoMath(double FE){
        double error = getError(setPoint, curPos);

        double p = error * constants.P; // This does not need to be adjusted for FE

        if (fabs(error) <= constants.iZone || constants.iZone == 0){ // no clue, I'm basically copy pasting. looks like IZone is a "zone" in which the I coefficient applies.
            iState += (error * constants.I) * FE; // *FE means that, if error * constants.I is 2, it will only actually gain 2 after 1 second/hz is passed. (20 ms by default). This keeps it smooth.
            // This kind of thing is used all throughout platformer; very tested and stable
        }
        else{
            iState = 0;
        }

        double d = (error - previousError); // Not really sure how this can be related to FE; I suppose I'll just leave it how it is and work it out later.
        previousError = error;
        d *= constants.D;

        float f = setPoint * constants.F;

        return p + iState + d + f;
    }

public:
    PIDConstants constants;

    void SetCircumference(long circumference){
        rotationLength = circumference;
    }
    
    PIDPositionController (BaseMotor* m, float frequency = 50){ // Update at 50 hz by default
        motor = m;
        hz = frequency;
    }

    void Update(){ // Call synchronously at any frequency, this uses Math (tm) to adjust for it
        // Pass in a current position argument to Update if you like that, or just let it figure it out from the motor encoder (see: very cool piece of code)
        Update(motor -> GetPosition());
    }

    void Update(double cPos){
        curPos = cPos;
        double secsElapsed = (double)frc::Timer::GetFPGATimestamp() - lastTime;
        double FE = secsElapsed / hz; // This is a trick from my online game. Measures elapsed time and converts it to number of ticks it needs to "draw"!
        // The roborio has at least a few mhz so this will almost never be >1, and will probably hover <0.1 most of the time.
        // We can set up SmartDashboard to track it for performance metrics, if it becomes necessary
        double ret = DoMath(FE);
        if (ret > constants.MaxOutput){
            ret = constants.MaxOutput;
        }
        else if (ret < constants.MinOutput){
            ret = constants.MinOutput;
        }
        motor -> SetPercent(ret);
        lastTime = (double)frc::Timer::GetFPGATimestamp();
    }

    void SetPosition(double pos){
        setPoint = pos;
    }
};