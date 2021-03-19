package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import android.view.animation.Interpolator;
import android.view.animation.LinearInterpolator;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.localizer.vision.RingStackDeterminationPipeline;
import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Autonomous(group = "drive")
public class AutonomousMain extends LinearOpMode {

    private Robot robot;
    public static int MAX_MILISECONDS = 3500;
    private static double FOAM_TILE_INCH = 23.6;

    private Pose2d startPose;
    private Pose2d wobbleMarker;
    private Vector2d parkingVector;

    public RingStackDeterminationPipeline.RingPosition numberOfRing;

    public void initAutonomous(){
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        robot.bratPivotant.raiseClaw(true);
        robot.bratPivotant.motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bratPivotant.toPosition(robot.bratPivotant.highConstraint);

        robot.openCV.start();

        //aparent avem doar o parte de teren *smiling cowboy face*
        startPose = new Pose2d(-2.6 * FOAM_TILE_INCH, -1 * FOAM_TILE_INCH, Math.toRadians(-90));
        wobbleMarker = new Pose2d(-0.5 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH, Math.toRadians(0));
        parkingVector = new Vector2d(-0.5 * FOAM_TILE_INCH,-2.5 * FOAM_TILE_INCH);

        numberOfRing = RingStackDeterminationPipeline.RingPosition.NONE;

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();
    }

    public void runAutonomous(){

        robot.drive.getLocalizer().setPoseEstimate(startPose);

        robot.timer.reset();

        while (robot.timer.milliseconds() < MAX_MILISECONDS){
            numberOfRing = robot.openCV.getRingPosition();
        }

        if(numberOfRing == RingStackDeterminationPipeline.RingPosition.FOUR){
            wobbleMarker = new Pose2d(1.5 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH, Math.toRadians(-90));
        } else if(numberOfRing == RingStackDeterminationPipeline.RingPosition.ONE){
            wobbleMarker = new Pose2d(0.5 * FOAM_TILE_INCH, -1.5 * FOAM_TILE_INCH, Math.toRadians(-90));
        } else {
            wobbleMarker = new Pose2d(0.5 * FOAM_TILE_INCH, -1.5 * FOAM_TILE_INCH, Math.toRadians(180));
        }

        try {
            robot.openCV.finalize();
        } catch (Throwable throwable) {
            throwable.printStackTrace();
        }

        //move away from the wall so we don't hit it, might not need it if we don't rotate at beginning of spline
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(new Pose2d(robot.drive.getLocalizer().getPoseEstimate().getX(), robot.drive.getLocalizer().getPoseEstimate().getY(), robot.drive.getLocalizer().getPoseEstimate().getHeading())).strafeLeft(0.2*FOAM_TILE_INCH).build());

        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), Math.toRadians(40)).splineToLinearHeading(wobbleMarker, Math.toRadians(0)).build());

        robot.bratPivotant.toPosition(robot.bratPivotant.lowConstraint);

        while (robot.bratPivotant.motorBrat.isBusy()) idle();
        robot.bratPivotant.raiseClaw(false);
        robot.bratPivotant.toPosition(robot.bratPivotant.highConstraint);
        while (robot.bratPivotant.motorBrat.isBusy()) idle();

        if (numberOfRing == RingStackDeterminationPipeline.RingPosition.FOUR){
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeTo(parkingVector).build());
        }
    }

    @Override
    public void runOpMode(){
        initAutonomous();
        waitForStart();
        runAutonomous();
        try {
            robot.openCV.finalize();
        } catch (Throwable throwable) {
            throwable.printStackTrace();
        }
    }
}
