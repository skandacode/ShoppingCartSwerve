package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Module {
    Motor topMotor, bottomMotor;
    AnalogInput lamprey;
    double lampreyOffset=0;
    public Module(HardwareMap hwMap, Telemetry telemetry, String topMotorName, String bottomMotorName, String lampreyName, double lampreyOffset){
        topMotor=new MotorEx(hwMap, topMotorName);
        bottomMotor=new MotorEx(hwMap, bottomMotorName);
        lamprey=hwMap.analogInput.get(lampreyName);
        this.lampreyOffset=lampreyOffset;
    }
    public double getModuleHeading(){
        double before_mod = lamprey.getVoltage()*360/2.189-this.lampreyOffset;
        if (before_mod<0){
            return before_mod+360;
        }
        return before_mod;
    }
    public void setMotors(double topPower, double bottomPower){
        topMotor.set(topPower);
        bottomMotor.set(bottomPower);
    }
}
