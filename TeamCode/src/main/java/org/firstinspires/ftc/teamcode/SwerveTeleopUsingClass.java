package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SwerveDrive;

import java.util.List;

@TeleOp
public class SwerveTeleopUsingClass extends LinearOpMode {
    SwerveDrive swerveDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        swerveDrive=new SwerveDrive(hardwareMap);
        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        long lastlooptime=System.nanoTime();
        while (opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            swerveDrive.driveRobotCentric(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x/2.0);
            long loop=System.nanoTime();
            telemetry.addData("Looptime", 1e9/(loop-lastlooptime));
            telemetry.update();
            lastlooptime=loop;

        }
    }
}
