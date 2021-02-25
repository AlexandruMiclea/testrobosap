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
        startPose = new Pose2d(2 * FOAM_TILE_INCH, -2.5 * FOAM_TILE_INCH, Math.toRadians(180));
        wobbleMarker = new Pose2d(2.5 * FOAM_TILE_INCH, 1.5 * FOAM_TILE_INCH);
        parkingVector = new Vector2d(1.5 * FOAM_TILE_INCH,0.5 * FOAM_TILE_INCH);
    }

    public void runAutonomous(){
        robot.drive.getLocalizer().setPoseEstimate(startPose);

        //rotatia ca sa priviti inelele
        //lineTo wobbleMarker
        //rotatia 90 (inspre ce tile vrem sa lasam wobble goal-ul)
        //lineTo parkingVector

    }

}
