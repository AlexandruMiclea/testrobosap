package org.firstinspires.ftc.teamcode.drive.opmode.driver;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Config
@TeleOp(group = "driver")
public class DriverMode extends OpMode {

    private Robot robot = null;
    private boolean bServoLift = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.bratPivotant.motorBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        //Practic baietii nostri au exact functia noastra de calculat vitezele
        robot.drive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

//        List<Double> powers = MecanumKinematics.robotToWheelVelocities(
//                new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x),
//                1.0,
//                1.0
//
//        );
//
//        telemetry.addData("fl? ", powers.get(0));
//        telemetry.addData("fl power", robot.drive.leftFront.getPower());
//
//        telemetry.addData("bl? ", powers.get(1));
//
//        telemetry.addData("fr? ", powers.get(2));
//        telemetry.addData("br? ", powers.get(3));
//
//        telemetry.update();

        if(gamepad2.a && !bServoLift){
            bServoLift = true;
            robot.bratPivotant.raiseClaw();
        } else if (!gamepad2.a) {
            bServoLift = false;
        }

        if (gamepad2.left_trigger>0){
            robot.bratPivotant.liftArm(gamepad2.left_trigger);
        } else if (gamepad2.right_trigger>0){
            robot.bratPivotant.lowerArm(gamepad2.right_trigger);
        } else robot.bratPivotant.stop();


        telemetry.addData("ticks arm", robot.bratPivotant.motorBrat.getCurrentPosition());
        telemetry.update();
//        if (gamepad1.left_trig0.1){
//            robot.protoAruncare.rotate(gamepad1.left_trigger);
//        }
//        else if (gamepad1.right_trigger > 0.1){
//            robot.protoAruncare.rotate(-gamepad1.right_trigger);
//        }
//        else {
//            robot.protoAruncare.stop();
//        }


    }
}
