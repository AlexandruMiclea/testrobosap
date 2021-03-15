package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(group = "drive")
public class TestGyro extends LinearOpMode {

    public ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro.calibrate();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("heading:", getRawExternalHeading());
            telemetry.update();
        }
    }

    double lastAngle, globalAngle;
    public double getRawExternalHeading() {
        double current = gyro.getHeading();

        double delta = current-lastAngle;
        if(delta > 180) {
            delta-=360;
        } else if (delta < -180){
            delta +=360;
        }

        lastAngle = current;
        globalAngle +=delta;
        return globalAngle;
    }
}
