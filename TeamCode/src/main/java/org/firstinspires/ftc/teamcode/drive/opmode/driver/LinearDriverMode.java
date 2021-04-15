package org.firstinspires.ftc.teamcode.drive.opmode.driver;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Config
@TeleOp(group = "driver")
public class LinearDriverMode extends LinearOpMode {
    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            //Practic baietii nostri au exact functia noastra de calculat vitezele
            robot.drive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

            //SIST WOBBLE GOAL
            //inchis/deschis gheara wobble goal
            if (gamepad1.a) {
                robot.wobbleArm.clawToggle(true);
            } else if (gamepad1.b) {
                robot.wobbleArm.clawToggle(false);
            }

            //oprit sau pornit constraints
            if (gamepad1.x) {
                robot.wobbleArm.setConstraints(!robot.wobbleArm.getConstraints());
            }

            //test to position
            if (gamepad1.right_bumper) {
                robot.wobbleArm.armPositionToggleAsync(false, 0.2);
            } else if (gamepad1.left_bumper) {
                robot.wobbleArm.armPositionToggleAsync(true, 0.2);
            }
            //miscat brat wobble goal sus jos
            else if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                telemetry.addData("Apasam pe triggere", "");
                if (robot.wobbleArm.getMotorMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    robot.wobbleArm.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (gamepad1.left_trigger > 0)
                    robot.wobbleArm.moveArm(gamepad1.left_trigger);
                else if (gamepad1.right_trigger > 0)
                    robot.wobbleArm.moveArm(-gamepad1.right_trigger);
            } else if (gamepad1.dpad_up){
                robot.wobbleArm.armPositionToggle(false, 0.3);
                robot.wobbleArm.clawToggle(false);
                sleep(500);
                robot.wobbleArm.clawToggle(true);
                robot.wobbleArm.armPositionToggle(true, 0.3);
            } else if (robot.wobbleArm.getMotorMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                robot.wobbleArm.stop();
            }

            //SIST ARUNCARE
            if (gamepad2.dpad_up){
                robot.thrower.rotateAtSpeedAsync(2800);
            } else if (gamepad2.dpad_down){
                robot.thrower.stop();
            }

            if(gamepad2.y){
                robot.thrower.pushRing(true);
                robot.thrower.pushRing(false);
            }

            //updates
            if(!Thread.currentThread().isInterrupted()){
                robot.wobbleArm.updateSub();
            }

            //TELEMETRIES
        telemetry.addData("pozitie brat: ", robot.wobbleArm.getPosition());
        telemetry.addData("constraints: ", robot.wobbleArm.getConstraints());
        telemetry.addData("motor mode: ", robot.wobbleArm.getMotorMode());
        telemetry.addData("motor.isBUsy(): ", robot.wobbleArm.getMotorIsBusy());
        telemetry.addData("subsystem mode", robot.wobbleArm.getMode());
        telemetry.addLine();
        telemetry.addData("servo pos", robot.wobbleArm.getServoPosition());
//
//        if(robot.wobbleArm.getMotorMode() == DcMotor.RunMode.RUN_TO_POSITION){
//            telemetry.addData("target", robot.wobbleArm.getTargetPosition());
//        }

//            telemetry.addData("ticks/rev (mid, right): ", robot.localizer.getTicksPerRev());
//            telemetry.addData("ticks: ", robot.localizer.getTicks());

//            telemetry.addData("Thrower current", robot.thrower.getCurrent());
//            telemetry.addData("Thrower speed", robot.thrower.getVelo());
//
        telemetry.addData("ticks/rev (mid, right): ", robot.localizer.getTicksPerRev());
        telemetry.addData("ticks: ", robot.localizer.getTicks());

        telemetry.update();
        }
    }
}
