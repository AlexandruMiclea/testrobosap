package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.localization.vision.RingStackDeterminationPipeline;
import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Autonomous(name="Autonomie Standard",group = "autonomous")
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
        robot.wobbleArm.clawToggle(true);

        robot.wobbleArm.armPositionToggle(true);

        robot.openCV.start();

        //aparent avem doar o parte de teren *smiling cowboy face*
        startPose = new Pose2d(-2.6 * FOAM_TILE_INCH, -1 * FOAM_TILE_INCH, Math.toRadians(-90));
        wobbleMarker = new Pose2d(0.5 * FOAM_TILE_INCH, -1.5 * FOAM_TILE_INCH, Math.toRadians(180));
        parkingVector = new Vector2d(0.5 * FOAM_TILE_INCH,-2.5 * FOAM_TILE_INCH);

        numberOfRing = RingStackDeterminationPipeline.RingPosition.NONE;

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();
    }

    public void runAutonomous(){

        robot.drive.getLocalizer().setPoseEstimate(startPose);

        //Wait a few seconds to identify the the number of rings
        robot.timer.reset();
        while (robot.timer.milliseconds() < MAX_MILISECONDS){
            numberOfRing = robot.openCV.getRingPosition();
        }

        //Set a target position on the field depending on the numbe of rings identified
        if(numberOfRing == RingStackDeterminationPipeline.RingPosition.FOUR){
            wobbleMarker = new Pose2d(1.5 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH, Math.toRadians(-90));
        } else if(numberOfRing == RingStackDeterminationPipeline.RingPosition.ONE){
            wobbleMarker = new Pose2d(0.5 * FOAM_TILE_INCH, -1.5 * FOAM_TILE_INCH, Math.toRadians(-90));
        } else {
            wobbleMarker = new Pose2d(0.5 * FOAM_TILE_INCH, -1.5 * FOAM_TILE_INCH, Math.toRadians(-180));
        }

        //Close OpenCV and thread as they are not used any longer
        try {
            robot.openCV.finalize();
        } catch (Throwable throwable) {
            throwable.printStackTrace();
        }

        //move away from the wall so we don't hit it, might not need it if we don't rotate at beginning of spline
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeLeft(0.2*FOAM_TILE_INCH).build());
        robot.drive.turn(Math.toRadians(90));

        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), Math.toRadians(40)).splineToLinearHeading(wobbleMarker, Math.toRadians(0)).build());

        robot.wobbleArm.armPositionToggle(false);

        robot.wobbleArm.clawToggle(false);

        robot.wobbleArm.armPositionToggle(true);

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
