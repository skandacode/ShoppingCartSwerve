package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SwerveDrive;

import java.util.List;

@TeleOp
@Config
public class SwerveTesitng extends LinearOpMode {
    SwerveDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        drive=new SwerveDrive(hardwareMap, telemetry);
        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        long lastlooptime=System.nanoTime();
        while (opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);
            drive.driveRobotCentric(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -0.5*gamepad1.right_stick_x);
            telemetry.addData("heading", drive.getCachedHeading());
            long loop=System.nanoTime();
            telemetry.addData("Looptime", 1e9/(loop-lastlooptime));
            telemetry.update();
            lastlooptime=loop;
        }
    }
}
