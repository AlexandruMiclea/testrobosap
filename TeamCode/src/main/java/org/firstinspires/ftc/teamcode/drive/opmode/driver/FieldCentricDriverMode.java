package org.firstinspires.ftc.teamcode.drive.opmode.driver;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Config
@TeleOp(group = "driver")
public class FieldCentricDriverMode extends LinearOpMode {

    private Robot robot = null;

    private boolean bWobbleConstraint = false;
    private boolean bCollectorConstraint = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            PoseStorage.currentPose = robot.drive.getPoseEstimate();

            Vector2d input = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y).rotated(-PoseStorage.currentPose.getHeading());

            robot.drive.setWeightedDrivePower(new Pose2d (input.getX(), input.getY(), -gamepad1.right_stick_x));

            //SIST WOBBLE GOAL

            //inchis/deschis gheara wobble goal
            if (gamepad1.a) {
                robot.wobbleArm.clawToggle(true);
            } else if (gamepad1.b) {
                robot.wobbleArm.clawToggle(false);
            }

            //oprit sau pornit constraints
            if (gamepad1.x && !bWobbleConstraint) {
                robot.wobbleArm.setConstraints(!robot.wobbleArm.getConstraints());
                bWobbleConstraint = true;
            } else if (!gamepad1.x) {
                bWobbleConstraint = false;
            }

            //test to position
            if (gamepad1.right_bumper) {
                robot.wobbleArm.armPositionToggleAsync(false);
            } else if (gamepad1.left_bumper) {
                robot.wobbleArm.armPositionToggleAsync(true);
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
            } else if (gamepad1.dpad_up) {
                robot.wobbleArm.armPositionToggle(false, 0.3);
                robot.wobbleArm.clawToggle(false);
                sleep(500);
                robot.wobbleArm.clawToggle(true);
                robot.wobbleArm.armPositionToggle(true, 0.3);
            } else if (robot.wobbleArm.getMotorMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                robot.wobbleArm.stop();
            }

            //SIST ARUNCARE

            //actionare motor aruncare
            if (gamepad2.left_stick_y != 0) {
                robot.thrower.rotateAsync(gamepad2.left_stick_y);
            } else if (gamepad2.dpad_up) {
                robot.thrower.rotateAtSpeedAsync(2800);
            } else if (gamepad2.x) {
                robot.thrower.rotateAsync(0.9);
            } else {
                robot.thrower.stop();
            }

            //servo
            if (gamepad2.a) {
                robot.thrower.pushRing(true);
            } else if (gamepad2.b) {
                robot.thrower.pushRing(false);
            }
            //this is only for testing purpose
            else if (gamepad2.y) {
                robot.thrower.pushRing();
            }


            if(!Thread.currentThread().isInterrupted()){
                robot.wobbleArm.updateSub();
            }

            //TELEMETRIES
            telemetry.addData("x pos: ", PoseStorage.currentPose.getX());
            telemetry.addData("y pos: ", PoseStorage.currentPose.getY());
            telemetry.addData("heading: ", PoseStorage.currentPose.getHeading());
            telemetry.update();

            robot.drive.update();
        }
    }
}
