package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Subsystem;
import org.firstinspires.ftc.teamcode.drive.localization.vision.RingStackDeterminationPipeline;
import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Autonomous(name="Autonomie Standard",group = "autonomous")
public class AutonomousMain extends LinearOpMode {

    private Robot robot;
    public static int MAX_MILISECONDS = 2500;
    private static double FOAM_TILE_INCH = 23.6;

    private final Pose2d startPose = new Pose2d(-2.6 * FOAM_TILE_INCH, -1 * FOAM_TILE_INCH, Math.toRadians(-90));
    private final Vector2d parkingVector = new Vector2d(0.5 * FOAM_TILE_INCH,-2.3 * FOAM_TILE_INCH);
    //TODO: adjust depending on arm position
    private final Vector2d secondWobble = new Vector2d(-1.2 * FOAM_TILE_INCH, -2.4 * FOAM_TILE_INCH);
    private Vector2d wobbleDropPose = new Vector2d(0.3 * FOAM_TILE_INCH, -1 * FOAM_TILE_INCH);
    private double endTargetTangent = Math.toRadians(-180);

    private RingStackDeterminationPipeline.RingPosition numberOfRing = RingStackDeterminationPipeline.RingPosition.NONE;

    public void initAutonomous(){
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        robot.wobbleArm.clawToggle(true);

        robot.wobbleArm.armPositionToggle(true);

        robot.openCV.start();

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

        robot.timer.reset();
        while (robot.timer.milliseconds() < MAX_MILISECONDS) {
            numberOfRing = robot.openCV.getRingPosition();
            telemetry.addData("number of rings: ", robot.openCV.getAnalysis());
        }

        //Set a target position on the field depending on the number of rings identified
        //TODO: adjust based on wobble arm position cause we need some clearance
        if(numberOfRing == RingStackDeterminationPipeline.RingPosition.FOUR){
            wobbleDropPose = new Pose2d(1.8 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH,Math.toRadians(-90)).vec();
            endTargetTangent = Math.toRadians(-120);
        } else if(numberOfRing == RingStackDeterminationPipeline.RingPosition.ONE){
            wobbleDropPose = new Pose2d(0.8 * FOAM_TILE_INCH, -1.7 * FOAM_TILE_INCH, Math.toRadians(-90)).vec();
            endTargetTangent = Math.toRadians(-90);
        } else {
            wobbleDropPose = new Pose2d(0.8 * FOAM_TILE_INCH, -1.6 * FOAM_TILE_INCH, Math.toRadians(-180)).vec();
            endTargetTangent = Math.toRadians(-90);
        }

        //Close OpenCV and thread as they are not used any longer
        try {
            robot.openCV.finalize();
        } catch (Throwable throwable) {
            throwable.printStackTrace();
        }

        //move away from the wall so we don't hit it and rotate so that we dont strafe spline
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeLeft(0.2*FOAM_TILE_INCH).build());
        robot.drive.turn(Math.toRadians(150));

        //drive to where we drop the wobble goal
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), Math.toRadians(60)).splineTo(wobbleDropPose, endTargetTangent).build());

        robot.wobbleArm.armPositionToggleAsync(false);

        if(numberOfRing == RingStackDeterminationPipeline.RingPosition.NONE)
            robot.drive.turn(Math.toRadians(-90));

        while(opModeIsActive() && robot.wobbleArm.isSubBusy()){
            robot.wobbleArm.updateSub();
        }

        //drop wobble goal and lift arm back up
        robot.wobbleArm.clawToggle(false);
        robot.wobbleArm.armPositionToggle(true);

        //move to grab second wobble
        if(numberOfRing != RingStackDeterminationPipeline.RingPosition.FOUR){
            if (numberOfRing == RingStackDeterminationPipeline.RingPosition.ONE)
                robot.drive.turn(Math.toRadians(-60));
            else if (numberOfRing ==  RingStackDeterminationPipeline.RingPosition.NONE)
                robot.drive.turn(Math.toRadians(30));

            //TODO set values of tangents
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).splineTo(secondWobble, Math.toRadians(-180)).build());

            robot.drive.turn(Math.toRadians(-125));

            //take wobble goal
            robot.wobbleArm.armPositionToggle(false);
            robot.wobbleArm.clawToggle(true);
            sleep(600);
            robot.wobbleArm.armPositionToggle(true);

            //strafe to drop zone again

            /*robot.drive.turn(Math.toRadians(-180));
            //TODO set values of tangents
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), Math.toRadians(90)).splineTo(wobbleDropPose, endTargetTangent).build());

            //drop wobble
            robot.wobbleArm.armPositionToggle(false);
            robot.wobbleArm.clawToggle(false);
            robot.wobbleArm.armPositionToggle(true);*/
        }

        //park
        if (numberOfRing == RingStackDeterminationPipeline.RingPosition.FOUR){
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeTo(parkingVector).build());
        }

    }
}
