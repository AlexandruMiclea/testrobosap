package org.firstinspires.ftc.teamcode.drive.opmode;

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

        /*if (gamepad1.a){
            robot.sistColectare.intake();
        }
        else if (gamepad1.b){
            robot.sistColectare.outtake();
        }
        else {
            robot.sistColectare.stop();
        }*/

//        if(gamepad1.a && !bServoLift){
//            bServoLift = true;
//            robot.bratPivotant.raiseClaw();
//        } else if (!gamepad1.a) {
//            bServoLift = false;
//        }
//
        if (gamepad1.left_trigger > 0.1){
            robot.protoAruncare.rotate(gamepad1.left_trigger);
        }
        else if (gamepad1.right_trigger > 0.1){
            robot.protoAruncare.rotate(-gamepad1.right_trigger);
        }
        else {
            robot.protoAruncare.stop();
        }

//        if (gamepad1.a) {
//            robot.bratGarou.expandTube();
//        }else if (gamepad1.b) {
//            robot.bratGarou.shrinkTube();
//        }else {
//            robot.bratGarou.stopTube();
//        }
//
//        if(gamepad1.dpad_up){
//            robot.bratGarou.slideUp();
//        } else if(gamepad1.dpad_down) {
//            robot.bratGarou.slideDown();
//        }
//        else {
//            robot.bratGarou.slideStop();
//        }
//
//        if(gamepad1.left_bumper){
//            robot.bratGarou.block();
//        } else if(gamepad1.right_bumper) {
//            robot.bratGarou.unblock();
//        }

    }
}
