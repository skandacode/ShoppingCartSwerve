package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.Module;

public class SwerveDrive {
    Module left, right;
    boolean prevZeroTurn=true;
    PIDFController headingStabilizer;
    IMU imu;
    Telemetry telemetry;
    double cachedHeading;
    public SwerveDrive(HardwareMap hwMap, Telemetry telemetry){
        left = new Module(hwMap, "leftTop", "leftBottom", "leftLamprey", 13, true);
        right = new Module(hwMap, "rightTop", "rightBottom", "rightLamprey", 32, false);
        imu = hwMap.get(IMU.class, "imu");
        headingStabilizer=new PIDFController(0.00001, 0, 0, 0);
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        cachedHeading=0;
        this.telemetry=telemetry;
    }
    public void driveRobotCentric(double forwardPower, double strafePower, double turnPower){
        cachedHeading=getGyroAngle();
        //if (!prevZeroTurn && turnPower==0){
        //    headingStabilizer.setSetPoint(cachedHeading);
        //}
        //if (turnPower==0){
        //    turnPower=headingStabilizer.calculate(cachedHeading);
        //}
        double[] leftPowers=left.setRobotCentricPowers(forwardPower - turnPower, strafePower);
        double[] rightPowers=right.setRobotCentricPowers(forwardPower + turnPower, strafePower);
        double denominator=Math.max(Math.max(Math.max(leftPowers[0], leftPowers[1]), rightPowers[0]), rightPowers[1]);
        left.setMotors(leftPowers[0]/denominator, leftPowers[1]/denominator);
        right.setMotors(rightPowers[0]/denominator, rightPowers[1]/denominator);
        prevZeroTurn=turnPower==0;
        telemetry.addData("left top", leftPowers[0]);
        telemetry.addData("left bottom", leftPowers[1]);
        telemetry.addData("right top", rightPowers[0]);
        telemetry.addData("right bottom", rightPowers[1]);
        telemetry.addData("denominator", denominator);

    }
    public double[] getModuleCachedRotation(){
        return new double[]{left.getCachedHeading(), right.getCachedHeading()};
    }
    public double getGyroAngle(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public double getCachedHeading(){
        return cachedHeading;
    }
}
