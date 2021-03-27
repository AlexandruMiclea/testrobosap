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
    //private boolean bServoLift = false;
    private boolean bIsAPressed = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        //Practic baietii nostri au exact functia noastra de calculat vitezele
        robot.drive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

        //inchis/deschis gheara wobble goal
        if(gamepad1.a){
            robot.wobbleArm.clawToggle(true);
        }
        if (gamepad1.b){
            robot.wobbleArm.clawToggle(false);
        }

        //oprit sau pornit constraints
        if(gamepad1.x){
            robot.wobbleArm.setConstraints(!robot.wobbleArm.getConstraints());
        }

        //test to position
//        if (gamepad1.right_bumper){
//            robot.wobbleArm.armPositionToggle(false);
//        }
//        else if(gamepad1.left_bumper){
//            robot.wobbleArm.armPositionToggle(true);
//        }
//        //miscat brat wobble goal sus jos
//        else
            if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
            telemetry.addData("Apasam pe triggere", "");
            if(robot.wobbleArm.getMotorMode() == DcMotor.RunMode.RUN_TO_POSITION){
                robot.wobbleArm.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(gamepad1.left_trigger > 0)
                robot.wobbleArm.moveArm(gamepad1.left_trigger);
            else if(gamepad1.right_trigger > 0)
                robot.wobbleArm.moveArm(-gamepad1.right_trigger);
        }
        else if(robot.wobbleArm.getMotorMode() == DcMotor.RunMode.RUN_USING_ENCODER){
            robot.wobbleArm.stop();
        }

        telemetry.addData("pozitie brat: ", robot.wobbleArm.getPosition());
        telemetry.addData("constraints: ", robot.wobbleArm.getConstraints());
        telemetry.addData("motor mode: ", robot.wobbleArm.getMotorMode());
//        telemetry.addData("subsystem mode", robot.wobbleArm.getMode());

        if(robot.wobbleArm.getMotorMode() == DcMotor.RunMode.RUN_TO_POSITION){
            telemetry.addData("target", robot.wobbleArm.getTargetPosition());
        }

//        telemetry.addData("Index: ", robot.localizer.getWheelPositions());
//        telemetry.update();
    }
}
