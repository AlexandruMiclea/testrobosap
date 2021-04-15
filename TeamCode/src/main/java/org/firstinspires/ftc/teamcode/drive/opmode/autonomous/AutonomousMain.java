package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.Subsystem;
import org.firstinspires.ftc.teamcode.drive.localization.vision.RingStackDeterminationPipeline;
import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Autonomous(name="Autonomie Standard", group = "autonomous")
public class AutonomousMain extends LinearOpMode {

    private Robot robot;
    public static int MAX_MILISECONDS = 2500;
    private static final double FOAM_TILE_INCH = 23.6;
    private double startAngle, wobbleSpeed;

    private final Pose2d startPose = new Pose2d(-2.6 * FOAM_TILE_INCH, -1 * FOAM_TILE_INCH, Math.toRadians(-90));
    private final Vector2d parkingVector = new Vector2d(0.5 * FOAM_TILE_INCH,-2.3 * FOAM_TILE_INCH);
    private final Vector2d secondWobble = new Vector2d(-1.3 * FOAM_TILE_INCH, -2.4 * FOAM_TILE_INCH);
    private Vector2d wobbleDropPose = new Vector2d(0.3 * FOAM_TILE_INCH, -1 * FOAM_TILE_INCH);
    private static double endTargetTangent = Math.toRadians(-180);
    private static double startTargetTangent = Math.toRadians(60);

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
        if (numberOfRing == RingStackDeterminationPipeline.RingPosition.FOUR){
            wobbleDropPose = new Pose2d(1.8 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH,Math.toRadians(-90)).vec();
            endTargetTangent = Math.toRadians(-120);
            startAngle = Math.toRadians(60);
            wobbleSpeed = 0.1;
        } else if (numberOfRing == RingStackDeterminationPipeline.RingPosition.ONE){
            wobbleDropPose = new Pose2d(0.5 * FOAM_TILE_INCH, -1.7 * FOAM_TILE_INCH, Math.toRadians(-90)).vec();
            endTargetTangent = Math.toRadians(-90);
            startAngle = Math.toRadians(60);
            wobbleSpeed = 0.12;
        } else {
            wobbleDropPose = new Pose2d(0.5 * FOAM_TILE_INCH, -1.6 * FOAM_TILE_INCH, Math.toRadians(-180)).vec();
            startTargetTangent = Math.toRadians(0);
            endTargetTangent = Math.toRadians(0);
            startAngle = Math.toRadians(-180);
            wobbleSpeed = 0.2;
        }

        //Close OpenCV and thread as they are not used any longer
        try {
            robot.openCV.finalize();
        } catch (Throwable throwable) {
            throwable.printStackTrace();
        }

        //move ewey from wall
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), 0).splineToLinearHeading(new Pose2d(-2*FOAM_TILE_INCH, -1*FOAM_TILE_INCH, 0), 0).build());

        //try to throw rings
        robot.thrower.rotateAtSpeedAsync(2500);
        robot.thrower.pushRing();
        robot.thrower.pushRing();
        robot.thrower.pushRing();
        robot.thrower.stop();

        robot.drive.turn(startAngle);

        robot.wobbleArm.armPositionToggleAsync(false, wobbleSpeed);

        //drive to where we drop the wobble goal
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), startTargetTangent).splineTo(wobbleDropPose, endTargetTangent).build());

        while(opModeIsActive() && robot.wobbleArm.isSubBusy()){
            robot.wobbleArm.updateSub();
        }

        //drop wobble goal and lift arm back up
        robot.wobbleArm.clawToggle(false);

        //move to grab second wobble
        robot.wobbleArm.armPositionToggleAsync(true, 0.4);

        if (numberOfRing == RingStackDeterminationPipeline.RingPosition.ONE) {
            robot.drive.turn(Math.toRadians(-60));
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).splineTo(secondWobble, Math.toRadians(-180)).build());
        } else if (numberOfRing ==  RingStackDeterminationPipeline.RingPosition.NONE) {
            robot.drive.turn(Math.toRadians(30));
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).splineTo(secondWobble, Math.toRadians(-180)).build());
        } else {
            robot.drive.turn(Math.toRadians(120));
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), Math.toRadians(-180)).splineTo(secondWobble, Math.toRadians(-180)).build());
        }

        //Just in case the motor hasn't finished rotating :)
        while(opModeIsActive() && robot.wobbleArm.isSubBusy()){
            robot.wobbleArm.updateSub();
        }

        if(numberOfRing == RingStackDeterminationPipeline.RingPosition.FOUR){
            robot.wobbleArm.armPositionToggleAsync(false, 0.2);
            robot.drive.turn(Math.toRadians(45));
        } else if (numberOfRing == RingStackDeterminationPipeline.RingPosition.ONE){
            robot.wobbleArm.armPositionToggleAsync(false, 0.2);
            robot.drive.turn(Math.toRadians(-135));
        } else {
            robot.wobbleArm.armPositionToggleAsync(false, 0.2);
            robot.drive.turn(Math.toRadians(-135));
        }

        while(opModeIsActive() && robot.wobbleArm.isSubBusy()){
            robot.wobbleArm.updateSub();
        }

        //take wobble goal
        robot.wobbleArm.clawToggle(true);
        sleep(600);
        robot.wobbleArm.armPositionToggleAsync(true);

        //strafe to drop zone again
        if (numberOfRing == RingStackDeterminationPipeline.RingPosition.ONE){
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), Math.toRadians(-10)).splineToLinearHeading(new Pose2d(wobbleDropPose, Math.toRadians(-90)), 0).build());
        } else if (numberOfRing == RingStackDeterminationPipeline.RingPosition.NONE) {
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), Math.toRadians(-10)).splineToLinearHeading(new Pose2d(wobbleDropPose, Math.toRadians(-180)), 0).build());
        } else{
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), Math.toRadians(-10)).splineToLinearHeading(new Pose2d(1.8*FOAM_TILE_INCH, -2.5*FOAM_TILE_INCH, Math.toRadians(-90)), -15).build());
        }

        while (opModeIsActive() && robot.wobbleArm.isSubBusy()) {
            robot.wobbleArm.updateSub();
        }

        //drop wobble
        robot.wobbleArm.armPositionToggle(false, 0.6);
        robot.wobbleArm.clawToggle(false);
        robot.wobbleArm.armPositionToggle(true, 0.6);

        //park
        if (numberOfRing == RingStackDeterminationPipeline.RingPosition.FOUR) {
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeTo(parkingVector).build());
        }

        // Transfer the final pose to the PoseStorage class so we can use it in TeleOp
        PoseStorage.currentPose = robot.drive.getPoseEstimate();
    }
}
