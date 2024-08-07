package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Module {
    CachedMotorEx topMotor, bottomMotor;
    AnalogInput lamprey;
    double lampreyOffset;
    boolean inverted;
    double moduleHeading=0;
    double prevTopPower, prevBottomPower=0;
    public Module(HardwareMap hwMap, String topMotorName, String bottomMotorName, String lampreyName, double lampreyOffset, boolean inverted){
        topMotor=new CachedMotorEx(hwMap, topMotorName);
        bottomMotor=new CachedMotorEx(hwMap, bottomMotorName);
        lamprey=hwMap.analogInput.get(lampreyName);
        this.lampreyOffset=lampreyOffset;
        this.inverted=inverted;
    }
    private double getModuleHeading(){
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
        topMotor.setPower(topPower);
        bottomMotor.setPower(bottomPower);

    }
    public void setModuleCentricPowers(double forward, double strafe){
        if (inverted){
            forward=-forward;
        }
        setMotors(forward-strafe, -forward-strafe);//diffy stuff
    }
    public void setRobotCentricPowers(double forward, double strafe){
        moduleHeading=getModuleHeading();
        Vector2d powerVector=new Vector2d(forward, strafe).rotateBy(moduleHeading);
        setModuleCentricPowers(powerVector.getX(), powerVector.getY());
    }
    public double getCachedHeading(){
        return moduleHeading;
    }
}