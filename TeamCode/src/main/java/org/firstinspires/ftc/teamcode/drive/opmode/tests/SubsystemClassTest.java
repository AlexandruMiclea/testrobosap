package org.firstinspires.ftc.teamcode.drive.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Config
@Autonomous(group = "test", name = "Test Wobble")
public class SubsystemClassTest extends LinearOpMode {

    public Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();
        robot = new Robot(hardwareMap);
        robot.wobbleArm.clawToggle(true);
        robot.wobbleArm.armPositionToggle(true);

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();

        waitForStart();

        robot.wobbleArm.armPositionToggle(false);
        robot.wobbleArm.clawToggle(false);
        robot.drive.turn(Math.toRadians(-90));
        robot.wobbleArm.clawToggle(true);
        robot.wobbleArm.armPositionToggle(true);
        robot.drive.turn(Math.toRadians(90));
        robot.wobbleArm.clawToggle(false);
        robot.wobbleArm.armPositionToggle(false);
        robot.drive.turn(Math.toRadians(-90));
        robot.wobbleArm.armPositionToggle(true);
    }
}
