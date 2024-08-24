package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Module;

public class SwerveDrive {
    Module left, right;
    public SwerveDrive(HardwareMap hwMap){
        left = new Module(hwMap, "leftTop", "leftBottom", "leftLamprey", 13, true);
        right = new Module(hwMap, "rightTop", "rightBottom", "rightLamprey", 322.2, false);
    }
    public void driveRobotCentric(double forwardPower, double strafePower, double turnPower){
        left.setRobotCentricPowers(forwardPower - turnPower, strafePower);
        right.setRobotCentricPowers(forwardPower + turnPower, strafePower);
    }
    public Double[] getModuleCachedRotation(){
        Double[] result=new Double[2];
        result[0]=left.getCachedHeading();
        result[1]= right.getCachedHeading();
        return result;
    }
}
