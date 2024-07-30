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
    public static double leftTopPower, leftBottomPower, rightTopPower, rightBottomPower=0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        left = new Module(hardwareMap, telemetry, "leftTop", "leftBottom", "leftLamprey", 13);
        right = new Module(hardwareMap, telemetry, "rightTop", "rightBottom", "rightLamprey", 255);
        waitForStart();
        while (opModeIsActive()){
            left.setMotors(leftTopPower, leftBottomPower);
            right.setMotors(rightTopPower, rightBottomPower);
            telemetry.addData("left pod lamprey", left.getModuleHeading());
            telemetry.addData("right pod lamprey", right.getModuleHeading());
            telemetry.update();
        }
    }
}
