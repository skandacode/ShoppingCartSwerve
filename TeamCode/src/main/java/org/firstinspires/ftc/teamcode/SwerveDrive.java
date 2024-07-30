package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveDrive {
    Module left, right;
    public SwerveDrive(HardwareMap hwMap){
        left = new Module(hwMap, "leftTop", "leftBottom", "leftLamprey", 13, true);
        right = new Module(hwMap, "rightTop", "rightBottom", "rightLamprey", 255, false);
    }
    public void driveRobotCentric(double forwardPower, double strafePower, double turnPower){
        left.setRobotCentricPowers(forwardPower - turnPower, strafePower);
        right.setRobotCentricPowers(forwardPower + turnPower, strafePower);
    }
}
