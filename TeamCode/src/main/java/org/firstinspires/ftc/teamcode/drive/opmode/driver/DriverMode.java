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

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.bratPivotant.motorBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        //Practic baietii nostri au exact functia noastra de calculat vitezele
        robot.drive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

        //inchis/deschis gheara wobble goal
        if(gamepad2.a){
            robot.bratPivotant.clawToggle();
        }

        //oprit sau pornit constraints
        if(gamepad2.x){
            robot.bratPivotant.setConstraints(!robot.bratPivotant.getConstraints());
        }

        //miscat brat wobble goal sus jos
        if (gamepad2.left_trigger>0){
            robot.bratPivotant.liftArm(gamepad2.left_trigger);
        } else if (gamepad2.right_trigger>0){
            robot.bratPivotant.lowerArm(gamepad2.right_trigger);
        } else robot.bratPivotant.stop();

        //test to position
        if (gamepad2.right_bumper){
            robot.bratPivotant.toPosition(robot.bratPivotant.lowConstraint);
        }
        if(gamepad2.left_bumper){
            robot.bratPivotant.toPosition(robot.bratPivotant.highConstraint);
        }

        /*telemetry.addData("pozitie brat: ", robot.bratPivotant.motorBrat.getCurrentPosition());
        telemetry.update();*/
    }
}
