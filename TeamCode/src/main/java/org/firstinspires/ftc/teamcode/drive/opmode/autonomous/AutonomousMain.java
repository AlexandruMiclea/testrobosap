package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import android.view.animation.Interpolator;
import android.view.animation.LinearInterpolator;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.localizer.vision.RingStackDeterminationPipeline;
import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Autonomous(group = "drive")
public class AutonomousMain extends LinearOpMode {

    private Robot robot;
    public static int MAX_MILISECONDS = 3500;
    private static double FOAM_TILE_INCH = 23.6;

    private Pose2d startPose;
    private Pose2d wobbleMarker;
    private double targetAngle;
    private Vector2d testpose;
    // private Vector2d parkingVector;
    private Vector2d parkingVector;

    public RingStackDeterminationPipeline.RingPosition numberOfRing;

    public void initAutonomous(){
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        robot.bratPivotant.raiseClaw(true);

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();

        //aparent avem doar o parte de teren *smiling cowboy face*
        startPose = new Pose2d(-2.6 * FOAM_TILE_INCH, -1 * FOAM_TILE_INCH, Math.toRadians(-90));
        testpose = new Vector2d(-0.5 * FOAM_TILE_INCH, -2 * FOAM_TILE_INCH);
        wobbleMarker = new Pose2d(-0.5 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH, Math.toRadians(0));
        // parkingVector = new Vector2d(1.5 * FOAM_TILE_INCH,0.5 * FOAM_TILE_INCH);
        parkingVector = new Vector2d(0.5 * FOAM_TILE_INCH,-1.5 * FOAM_TILE_INCH);
        numberOfRing = RingStackDeterminationPipeline.RingPosition.NONE;
    }

    public void runAutonomous(){

        robot.drive.getLocalizer().setPoseEstimate(startPose);

        robot.timer.startTime();
        robot.timer.reset();
        robot.timer.startTime();

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

        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(new Pose2d(robot.drive.getLocalizer().getPoseEstimate().getX(), robot.drive.getLocalizer().getPoseEstimate().getY(), robot.drive.getLocalizer().getPoseEstimate().getHeading())).strafeLeft(0.2*FOAM_TILE_INCH).build());

        Pose2d current = new Pose2d(robot.drive.getLocalizer().getPoseEstimate().getX(),  robot.drive.getLocalizer().getPoseEstimate().getY(), Math.toRadians(40));
//        Pose2d current = new Pose2d(robot.drive.getLocalizer().getPoseEstimate().getX(), robot.drive.getLocalizer().getPoseEstimate().getY(),  robot.drive.getLocalizer().getPoseEstimate().getHeading());
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(current).splineToLinearHeading(wobbleMarker, Math.toRadians(0)).build());

        robot.timer.reset();
        robot.timer.startTime();
        robot.bratPivotant.moveBackward(0.4);
        while (robot.timer.milliseconds() < 700){
            idle();
        }

        robot.bratPivotant.raiseClaw(false);

        robot.timer.reset();
        robot.timer.startTime();
        robot.bratPivotant.moveForward(0.4);
        while (robot.timer.milliseconds() < 700){
            idle();
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
