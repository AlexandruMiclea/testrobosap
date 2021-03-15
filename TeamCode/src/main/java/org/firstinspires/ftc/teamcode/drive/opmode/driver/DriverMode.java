package org.firstinspires.ftc.teamcode.drive.opmode.driver;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Config
@TeleOp(group = "driver")
public class DriverMode extends OpMode {

    private Robot robot = null;
    private boolean bServoLift = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        //Practic baietii nostri au exact functia noastra de calculat vitezele
        //robot.drive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

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

//        if(gamepad1.a && !bServoLift){
//            bServoLift = true;
//            robot.bratPivotant.raiseClaw();
//        } else if (!gamepad1.a) {
//            bServoLift = false;
//        }
//
//        if (gamepad1.x){
//            robot.bratPivotant.moveBackward(0.5);
//        } else if (gamepad1.y){
//            robot.bratPivotant.moveForward(0.5);
//        } else robot.bratPivotant.stop();

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
