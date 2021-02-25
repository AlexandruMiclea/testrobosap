package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.tank.Robot;

@Autonomous(group = "drive")
public abstract class AutonomousMain extends LinearOpMode {

    private Robot robot;
    public static int MAX_MILISECONDS = 3000;
    private static double FOAM_TILE_INCH = 23.622;

    private Pose2d startPose;
    private Pose2d wobbleMarker;
    // private Vector2d parkingVector;
    private Pose2d parkingVector;

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
        startPose = new Pose2d(2 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH, Math.toRadians(180));
        wobbleMarker = new Pose2d(2.5 * FOAM_TILE_INCH, 1.5 * FOAM_TILE_INCH);
        // parkingVector = new Vector2d(1.5 * FOAM_TILE_INCH,0.5 * FOAM_TILE_INCH);
        parkingVector = new Pose2d(1.5 * FOAM_TILE_INCH,0.5 * FOAM_TILE_INCH);
    }

    public void runAutonomous(){
        robot.drive.getLocalizer().setPoseEstimate(startPose);

        robot.drive.turn(Math.toRadians(35));
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(wobbleMarker).build());

//        if(/*zero ringuri*/) {
//            robot.drive.turn(Math.toRadians(360));
//        }
//        if(/*un ring*/) {
//            robot.drive.turn(Math.toRadians(270));
//        }
//        if(/*patru ringuri*/) {
//            robot.drive.turn(Math.toRadians(180));
//        }
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(parkingVector).build());

        //rotatia ca sa priviti inelele
        //lineTo wobbleMarker
        //rotatia 90 (inspre ce tile vrem sa lasam wobble goal-ul)
        //lineTo parkingVector

    }

}
