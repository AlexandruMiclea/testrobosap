package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.localization.vision.RingStackDeterminationPipeline;
import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Autonomous(name="Autonomie Standard",group = "autonomous")
public class AutonomousMain extends LinearOpMode {

    private Robot robot;
    public static int MAX_MILISECONDS = 3500;
    private static double FOAM_TILE_INCH = 23.6;

    private final Pose2d startPose = new Pose2d(-2.6 * FOAM_TILE_INCH, -1 * FOAM_TILE_INCH, Math.toRadians(-90));
    private final Vector2d parkingVector = new Vector2d(0.5 * FOAM_TILE_INCH,-2.5 * FOAM_TILE_INCH);
    //TODO: adjust depending on arm position
    private final Pose2d secondWobble = new Pose2d(-2.3 * FOAM_TILE_INCH, -1 * FOAM_TILE_INCH, Math.toRadians(-180));

    private Pose2d wobbleDropPose = new Pose2d(0.5 * FOAM_TILE_INCH, -1.5 * FOAM_TILE_INCH, Math.toRadians(-180));

    public RingStackDeterminationPipeline.RingPosition numberOfRing = RingStackDeterminationPipeline.RingPosition.NONE;

    public void initAutonomous(){
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        robot.wobbleArm.clawToggle(true);

        robot.wobbleArm.armPositionToggle(true);

//        robot.openCV.start();

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode(){
        initAutonomous();

        waitForStart();


        robot.drive.getLocalizer().setPoseEstimate(startPose);

        //Wait a few seconds to identify the the number of rings
//        robot.timer.reset();
//        while (robot.timer.milliseconds() < MAX_MILISECONDS){
//            numberOfRing = robot.openCV.getRingPosition();
//        }

        //Set a target position on the field depending on the numbe of rings identified
        //TODO: adjust based on wobble arm position cause we need some clearance
//        if(numberOfRing == RingStackDeterminationPipeline.RingPosition.FOUR){
//            wobbleDropPose = new Pose2d(1.5 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH, Math.toRadians(-90));
//        } else if(numberOfRing == RingStackDeterminationPipeline.RingPosition.ONE){
//            wobbleDropPose = new Pose2d(0.5 * FOAM_TILE_INCH, -1.5 * FOAM_TILE_INCH, Math.toRadians(-90));
//        } else {
//            wobbleDropPose = new Pose2d(0.5 * FOAM_TILE_INCH, -1.5 * FOAM_TILE_INCH, Math.toRadians(-180));
//        }
//
//        //Close OpenCV and thread as they are not used any longer
//        try {
//            robot.openCV.finalize();
//        } catch (Throwable throwable) {
//            throwable.printStackTrace();
//        }

        //move away from the wall so we don't hit it and rotate so that we dont strafe spline
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeLeft(0.2*FOAM_TILE_INCH).build());
        robot.drive.turn(Math.toRadians(90));

        //drive to where we drop the wobble goal
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), Math.toRadians(40)).splineToLinearHeading(wobbleDropPose, Math.toRadians(0)).build());

        //drop wobble goal and lift arm back up
        robot.wobbleArm.armPositionToggle(false);
        robot.wobbleArm.clawToggle(false);
        robot.wobbleArm.armPositionToggle(true);

        //move to grab second wobble
//        //TODO set values of tangents
//        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), Math.toRadians(60)).splineToLinearHeading(secondWobble, Math.toRadians(150)).build());
//
//        //take wobble goal
//        robot.wobbleArm.armPositionToggle(false);
//        robot.wobbleArm.clawToggle(true);
//        robot.wobbleArm.armPositionToggle(true);
//
//        //strafe to drop zone again
//        robot.drive.turn(-180);
//        //TODO set values of tangents
//        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), Math.toRadians(40)).splineToLinearHeading(wobbleDropPose, Math.toRadians(0)).build());
//
//        //drop wobble
//        robot.wobbleArm.armPositionToggle(false);
//        robot.wobbleArm.clawToggle(false);
//        robot.wobbleArm.armPositionToggle(true);

        //park
        if (numberOfRing == RingStackDeterminationPipeline.RingPosition.FOUR){
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeTo(parkingVector).build());
        }

    }
}
