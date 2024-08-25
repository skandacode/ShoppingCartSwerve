package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.CachedMotorEx;

import java.util.Arrays;

public class Swerve {
    IMU imu;
    Telemetry telemetry;
    double cachedHeading;
    CachedMotorEx leftTop, rightTop, leftBottom, rightBottom;
    AnalogInput leftLamprey, rightLamprey;
    double [] cachedPositions = new double[]{0, 0};
    public Swerve(HardwareMap hwMap, Telemetry telemetry){
        imu = hwMap.get(IMU.class, "imu");

        leftTop = new CachedMotorEx(hwMap, "leftTop");
        rightTop = new CachedMotorEx(hwMap, "rightTop");
        leftBottom = new CachedMotorEx(hwMap, "leftBottom");
        rightBottom = new CachedMotorEx(hwMap, "rightBottom");

        leftLamprey = hwMap.analogInput.get("leftLamprey");
        rightLamprey = hwMap.analogInput.get("rightLamprey");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        this.telemetry = telemetry;
    }
    public void driveRobotCentric(double forward, double strafe, double turn){
        Vector2d leftVector = new Vector2d(forward - turn, strafe);
        Vector2d rightVector = new Vector2d(forward + turn, strafe);

        cachedPositions = new double[]{getLeftHeading(), getRightHeading()};

        leftVector = leftVector.rotateBy(cachedPositions[0]);
        rightVector = rightVector.rotateBy(cachedPositions[1]);

        double[] motorPowers=new double[]{-leftVector.getX() - leftVector.getY(), -leftVector.getX() + leftVector.getY(),
                rightVector.getX() - rightVector.getY(), rightVector.getX() + rightVector.getY()};

        double denominator=Arrays.stream(motorPowers).max().getAsDouble();

        leftTop.setPower(motorPowers[0]/denominator);
        leftBottom.setPower(motorPowers[1]/denominator);
        rightTop.setPower(motorPowers[2]/denominator);
        rightBottom.setPower(motorPowers[3]/denominator);
    }


    private double getLeftHeading(){
        double x = leftLamprey.getVoltage()*360/2.2-13;
        if (x < 0){
            return x + 360;
        }
        return x;
    }
    private double getRightHeading(){
        double x = rightLamprey.getVoltage()*360/2.2-32;
        if (x < 0){
            return x + 360;
        }
        return x;
    }

    public double[] getCachedPositions() {
        return cachedPositions;
    }
    public double getCachedHeading(){
        return cachedHeading;
    }
    public double getHeading(){
        cachedHeading=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return cachedHeading;
    }
}
