package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Module {
    Motor topMotor, bottomMotor;
    AnalogInput lamprey;
    double lampreyOffset;
    boolean inverted;
    public Module(HardwareMap hwMap, Telemetry telemetry, String topMotorName, String bottomMotorName, String lampreyName, double lampreyOffset, boolean inverted){
        topMotor=new MotorEx(hwMap, topMotorName);
        bottomMotor=new MotorEx(hwMap, bottomMotorName);
        lamprey=hwMap.analogInput.get(lampreyName);
        this.lampreyOffset=lampreyOffset;
        this.inverted=inverted;
    }
    public double getModuleHeading(){
        double before_mod = lamprey.getVoltage()*360/2.189-this.lampreyOffset;
        if (before_mod<0){
            return before_mod+360;
        }
        return before_mod;
    }
    public void setMotors(double topPower, double bottomPower){
        double denom=Math.max(topPower, bottomPower);
        if (denom>1){
            topPower=topPower/denom;
            bottomPower=bottomPower/denom;
        }
        topMotor.set(topPower);
        bottomMotor.set(bottomPower);
    }
    public void setModuleCentricPowers(double forward, double strafe){
        if (inverted){
            forward=-forward;
        }
        setMotors(forward-strafe, -forward-strafe);//diffy stuff
    }
    public void setRobotCentricPowers(double forward, double strafe){
        double moduleHeading=getModuleHeading();
        if (!inverted){
            moduleHeading=moduleHeading+180;
            forward=-forward;
            strafe=-strafe;
        }
        Vector2d powerVector=new Vector2d(forward, strafe).rotateBy(moduleHeading);
        setModuleCentricPowers(powerVector.getX(), powerVector.getY());
    }
}