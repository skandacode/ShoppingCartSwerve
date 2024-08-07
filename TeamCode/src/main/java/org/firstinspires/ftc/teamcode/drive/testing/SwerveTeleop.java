package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Module;

@TeleOp
public class SwerveTeleop extends LinearOpMode {
    Module left, right;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        left = new Module(hardwareMap, "leftTop", "leftBottom", "leftLamprey", 13, true);
        right = new Module(hardwareMap, "rightTop", "rightBottom", "rightLamprey", 255, false);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.right_bumper){
                left.setRobotCentricPowers(-gamepad1.left_stick_y*0.25+gamepad1.right_stick_x*0.15, -gamepad1.left_stick_x*0.25);
                right.setRobotCentricPowers(-gamepad1.left_stick_y*0.25-gamepad1.right_stick_x*0.15, -gamepad1.left_stick_x*0.25);

            }
            else {
                left.setRobotCentricPowers(-gamepad1.left_stick_y + gamepad1.right_stick_x * 0.5, -gamepad1.left_stick_x);
                right.setRobotCentricPowers(-gamepad1.left_stick_y - gamepad1.right_stick_x * 0.5, -gamepad1.left_stick_x);
            }
            telemetry.addData("left pod lamprey", left.getCachedHeading());
            telemetry.addData("right pod lamprey", right.getCachedHeading());
            telemetry.update();

        }
    }
}