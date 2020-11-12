package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Config
@TeleOp(group = "driver")
public class DriverMode extends OpMode {

    private Robot robot = null;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        //Practic baietii nostri au exact functia noastra de calculat vitezele
        robot.drive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

        if (gamepad1.a){
            robot.sistColectare.intake();
        }
        else if (gamepad1.b){
            robot.sistColectare.outtake();
        }
        else {
            robot.sistColectare.stop();
        }

        if (gamepad1.x){
            robot.bratPivotant.moveForward();
        }
        else if (gamepad1.y){
            robot.bratPivotant.moveBackward();
        }
        else {
            robot.bratPivotant.stop();
        }


    }
}
