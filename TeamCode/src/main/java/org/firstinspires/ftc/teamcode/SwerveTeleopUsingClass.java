package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        swerveDrive=new SwerveDrive(hardwareMap);
        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        long lastlooptime=System.nanoTime();
        while (opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            telemetry.addData("turn", gamepad1.right_stick_x/2);
            if (!(gamepad1.right_bumper)){
                swerveDrive.driveRobotCentric(-0.3*gamepad1.left_stick_y, -0.3*gamepad1.left_stick_x, -0.17*gamepad1.right_stick_x);
            }
            else{
                swerveDrive.driveRobotCentric(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -0.5*gamepad1.right_stick_x);
            }
            long loop=System.nanoTime();
            telemetry.addData("Looptime", 1e9/(loop-lastlooptime));
            telemetry.update();
            lastlooptime=loop;

        }
    }
}
