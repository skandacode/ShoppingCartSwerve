package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.CachedMotorEx;

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
        double before_mod = lamprey.getVoltage()*360/2.2-this.lampreyOffset;//2.189
        if (before_mod<0){
            return before_mod+360;
        }
        return before_mod;
    }
    public void setMotors(double topPower, double bottomPower){
        topMotor.setPower(topPower);
        bottomMotor.setPower(bottomPower);

    }
    public double[] setModuleCentricPowers(double forward, double strafe){
        if (inverted){
            forward=-forward;
        }
        return new double[]{forward-strafe, -forward-strafe};//diffy stuff
    }
    public double[] setRobotCentricPowers(double forward, double strafe){
        moduleHeading=getModuleHeading();
        Vector2d powerVector=new Vector2d(forward, strafe).rotateBy(moduleHeading);
        return setModuleCentricPowers(powerVector.getX(), powerVector.getY());
    }
    public double getCachedHeading(){
        return moduleHeading;
    }
}