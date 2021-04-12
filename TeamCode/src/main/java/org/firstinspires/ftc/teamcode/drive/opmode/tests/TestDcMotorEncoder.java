package org.firstinspires.ftc.teamcode.drive.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
@TeleOp(group = "driver", name = "Test Tickuri Motor Encoder")
public class TestDcMotorEncoder extends LinearOpMode {

    private DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.right_bumper){
                motor.setPower(0.5);
            } else if (gamepad1.b){
                motor.setPower(-0.5);
            } else {
                motor.setPower(0);
            }

            telemetry.addData("encoder ticks:", motor.getCurrentPosition());
            telemetry.update();
        }

    }
}
