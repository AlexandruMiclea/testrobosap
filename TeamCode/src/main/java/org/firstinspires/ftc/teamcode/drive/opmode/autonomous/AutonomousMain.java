package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import android.view.animation.Interpolator;
import android.view.animation.LinearInterpolator;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Autonomous(group = "drive")
public class AutonomousMain extends LinearOpMode {

    private Robot robot;
    public static int MAX_MILISECONDS = 3000;
    private static double FOAM_TILE_INCH = 23.622;

    private Pose2d startPose;
    private Pose2d wobbleMarker;
    private double targetAngle;
    private Vector2d seeRings;
    private Vector2d testpose;
    private Vector2d testpose2;
    // private Vector2d parkingVector;
    private Vector2d parkingVector;

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
        startPose = new Pose2d(-2.5 * FOAM_TILE_INCH, -2 * FOAM_TILE_INCH, Math.toRadians(0));
        testpose = new Vector2d(-0.5 * FOAM_TILE_INCH, -2 * FOAM_TILE_INCH);
        testpose2 = new Vector2d(-1.5 * FOAM_TILE_INCH, -2 * FOAM_TILE_INCH);
//        seeRings = new Vector2d (-2.3 * FOAM_TILE_INCH, 2 * FOAM_TILE_INCH);
        wobbleMarker = new Pose2d(1.5 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH);
        // parkingVector = new Vector2d(1.5 * FOAM_TILE_INCH,0.5 * FOAM_TILE_INCH);
        parkingVector = new Vector2d(0.5 * FOAM_TILE_INCH,-1.5 * FOAM_TILE_INCH);
    }

    public void runAutonomous(){

        robot.drive.getLocalizer().setPoseEstimate(startPose);

        



        // fa initial movements ca sa vezi inelele

//        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getLocalizer().getPoseEstimate()).lineTo(seeRings).build());
//        robot.drive.turn(Math.toRadians(35));

//        robot.timer.startTime();

        //vezi ce se intampla aici si daca iti trebuie idle
//        while (robot.timer.milliseconds() < MAX_MILISECONDS){
//            switch(robot.openCV.getRingPosition()){
//                case FOUR:
//                    targetAngle = Math.toRadians(0);
//                    break;
//                case ONE:
//                    targetAngle = Math.toRadians(90);
//                    break;
//                default:
//                    targetAngle = Math.toRadians(180);
//                    break;
//            }
//            idle();
//        }
        targetAngle = Math.toRadians(0);

        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getLocalizer().getPoseEstimate()).splineToSplineHeading(wobbleMarker, targetAngle).build());

        // da drumul la wobble goal aka
        // 1) coboara brat
        // 2) unclamp
        // 3) ridica brat so it not in the way

        // de vazut daca incurca sau nu wobble goalul mergand pe diagonala
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getLocalizer().getPoseEstimate()).strafeTo(parkingVector).build());

        //TODO testeaza cum ar merge



    }

    @Override
    public void runOpMode(){
        initAutonomous();
        waitForStart();
        runAutonomous();
    }

}
