package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class SwerveTeleopUsingClass extends LinearOpMode {
    SwerveDrive swerveDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        swerveDrive=new SwerveDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            swerveDrive.driveRobotCentric(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x/2.0);
        }
    }
}
