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
    public static int MAX_MILISECONDS = 3000;
    private static double FOAM_TILE_INCH = 23.622;

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

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();

        //aparent avem doar o parte de teren *smiling cowboy face*
        startPose = new Pose2d(-2.5 * FOAM_TILE_INCH, -1 * FOAM_TILE_INCH, Math.toRadians(-90));
        testpose = new Vector2d(-0.5 * FOAM_TILE_INCH, -2 * FOAM_TILE_INCH);
        wobbleMarker = new Pose2d(1.5 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH, Math.toRadians(90));
        // parkingVector = new Vector2d(1.5 * FOAM_TILE_INCH,0.5 * FOAM_TILE_INCH);
        parkingVector = new Vector2d(0.5 * FOAM_TILE_INCH,-1.5 * FOAM_TILE_INCH);
        numberOfRing = RingStackDeterminationPipeline.RingPosition.NONE;
    }

    public void runAutonomous(){

        robot.drive.getLocalizer().setPoseEstimate(startPose);

        robot.timer.startTime();

        //vezi ce se intampla aici si daca iti trebuie idle
        while (robot.timer.milliseconds() < MAX_MILISECONDS){
            numberOfRing = robot.openCV.getRingPosition();
            switch(numberOfRing){
                case FOUR:
//                    wobbleMarker = new Pose2d(1.5 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH, Math.toRadians(-90));
                    testpose = new Vector2d(-1*FOAM_TILE_INCH, -1*FOAM_TILE_INCH);
                    break;
                case ONE:
//                    wobbleMarker = new Pose2d(1.5 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH, Math.toRadians(0));
                    testpose = new Vector2d(0.5*FOAM_TILE_INCH, -1*FOAM_TILE_INCH);
                    break;
                default:
//                    wobbleMarker = new Pose2d(1.5 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH, Math.toRadians(90));
                    testpose = new Vector2d(2*FOAM_TILE_INCH, -1*FOAM_TILE_INCH);
                    break;
            }
//            idle();
        }

        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getLocalizer().getPoseEstimate()).lineTo(testpose).build());

//        Pose2d current = new Pose2d(robot.drive.getLocalizer().getPoseEstimate().getX(), robot.drive.getLocalizer().getPoseEstimate().getY(), Math.toRadians(30));

//        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(current).splineToLinearHeading(wobbleMarker, Math.toRadians(-90)).build());

        // da drumul la wobble goal aka
        // 1) coboara brat
        // 2) unclamp
        // 3) ridica brat so it not in the way

        // TODO: de vazut daca incurca sau nu wobble goalul mergand pe diagonala
        // DONE: incurca intr adevar wobble goal ul mergand pe diagonala, trebuie sa facem niste pathuri diferite
//        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getLocalizer().getPoseEstimate()).strafeTo(parkingVector).build());

        //TODO testeaza cum ar merge
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
