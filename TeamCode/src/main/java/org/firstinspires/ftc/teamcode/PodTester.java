package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class PodTester extends LinearOpMode {
    Module left, right;
    public static double leftForwardPower, leftStrafePower, rightForwardPower, rightStrafePower=0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        left = new Module(hardwareMap, "leftTop", "leftBottom", "leftLamprey", 13, true);
        right = new Module(hardwareMap, "rightTop", "rightBottom", "rightLamprey", 255, false);
        waitForStart();
        while (opModeIsActive()){
            left.setRobotCentricPowers(leftForwardPower, leftStrafePower);
            right.setRobotCentricPowers(rightForwardPower, rightStrafePower);
            telemetry.addData("left pod lamprey", left.getCachedHeading());
            telemetry.addData("right pod lamprey", right.getCachedHeading());
            telemetry.update();
        }
    }
}