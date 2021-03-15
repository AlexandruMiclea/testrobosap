/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDriveChassis;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;
import org.firstinspires.ftc.teamcode.util.RegressionUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rpmToVelocity;

@TeleOp(name = "OpenCV Test", group = "Concept")
public class EasyOpenCVExample extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    RingStackDeterminationPipeline pipeline;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingStackDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class RingStackDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the ring stack number
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(100,50);

        static final int REGION_WIDTH = 150;
        static final int REGION_HEIGHT = 150;

        final int FOUR_RING_THRESHOLD = 160;
        final int ONE_RING_THRESHOLD = 150;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    /*
     * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
     * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
     * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
     * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
     * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
     * you've successfully connected, start the program, and your robot will begin moving forward and
     * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
     * your follower PID coefficients such that you follow the target position as accurately as possible.
     * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
     * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
     * These coefficients can be tuned live in dashboard.
     *
     * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
     * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
     */
    @Config
    @Autonomous(group = "drive")
    public static class BackAndForth extends LinearOpMode {

        public static double DISTANCE = 50;

        @Override
        public void runOpMode() throws InterruptedException {
            MecanumDriveChassis drive = new MecanumDriveChassis(hardwareMap);

            Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                    .forward(DISTANCE)
                    .build();

            Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                    .back(DISTANCE)
                    .build();

            waitForStart();

            while (opModeIsActive() && !isStopRequested()) {
                drive.followTrajectory(trajectoryForward);
                drive.followTrajectory(trajectoryBackward);
            }
        }
    }

    /*
     * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
     * outline of the procedure:
     *   1. Slowly ramp the motor power and record encoder values along the way.
     *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
     *      and an optional intercept (kStatic).
     *   3. Accelerate the robot (apply constant power) and record the encoder counts.
     *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
     *      regression.
     */
    @Config
    @Autonomous(group = "drive")
    public static class AutomaticFeedforwardTuner extends LinearOpMode {
        public static double MAX_POWER = 0.7;
        public static double DISTANCE = 100; // in

        @Override
        public void runOpMode() throws InterruptedException {
            if (RUN_USING_ENCODER) {
                RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                        "when using the built-in drive motor velocity PID.");
            }

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            MecanumDriveChassis drive = new MecanumDriveChassis(hardwareMap);

            NanoClock clock = NanoClock.system();

            telemetry.addLine("Press play to begin the feedforward tuning routine");
            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;

            telemetry.clearAll();
            telemetry.addLine("Would you like to fit kStatic?");
            telemetry.addLine("Press (A) for yes, (B) for no");
            telemetry.update();

            boolean fitIntercept = false;
            while (!isStopRequested()) {
                if (gamepad1.a) {
                    fitIntercept = true;
                    while (!isStopRequested() && gamepad1.a) {
                        idle();
                    }
                    break;
                } else if (gamepad1.b) {
                    while (!isStopRequested() && gamepad1.b) {
                        idle();
                    }
                    break;
                }
                idle();
            }

            telemetry.clearAll();
            telemetry.addLine(Misc.formatInvariant(
                    "Place your robot on the field with at least %.2f in of room in front", DISTANCE));
            telemetry.addLine("Press (A) to begin");
            telemetry.update();

            while (!isStopRequested() && !gamepad1.a) {
                idle();
            }
            while (!isStopRequested() && gamepad1.a) {
                idle();
            }

            telemetry.clearAll();
            telemetry.addLine("Running...");
            telemetry.update();

            double maxVel = rpmToVelocity(MAX_RPM);
            double finalVel = MAX_POWER * maxVel;
            double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
            double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

            List<Double> timeSamples = new ArrayList<>();
            List<Double> positionSamples = new ArrayList<>();
            List<Double> powerSamples = new ArrayList<>();

            drive.setPoseEstimate(new Pose2d());

            double startTime = clock.seconds();
            while (!isStopRequested()) {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > rampTime) {
                    break;
                }
                double vel = accel * elapsedTime;
                double power = vel / maxVel;

                timeSamples.add(elapsedTime);
                positionSamples.add(drive.getPoseEstimate().getX());
                powerSamples.add(power);

                drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
                drive.updatePoseEstimate();
            }
            drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

            RegressionUtil.RampResult rampResult = RegressionUtil.fitRampData(
                    timeSamples, positionSamples, powerSamples, fitIntercept,
                    LoggingUtil.getLogFile(Misc.formatInvariant(
                            "DriveRampRegression-%d.csv", System.currentTimeMillis())));

            telemetry.clearAll();
            telemetry.addLine("Quasi-static ramp up test complete");
            if (fitIntercept) {
                telemetry.addLine(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                        rampResult.kV, rampResult.kStatic, rampResult.rSquare));
            } else {
                telemetry.addLine(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
                        rampResult.kStatic, rampResult.rSquare));
            }
            telemetry.addLine("Would you like to fit kA?");
            telemetry.addLine("Press (A) for yes, (B) for no");
            telemetry.update();

            boolean fitAccelFF = false;
            while (!isStopRequested()) {
                if (gamepad1.a) {
                    fitAccelFF = true;
                    while (!isStopRequested() && gamepad1.a) {
                        idle();
                    }
                    break;
                } else if (gamepad1.b) {
                    while (!isStopRequested() && gamepad1.b) {
                        idle();
                    }
                    break;
                }
                idle();
            }

            if (fitAccelFF) {
                telemetry.clearAll();
                telemetry.addLine("Place the robot back in its starting position");
                telemetry.addLine("Press (A) to continue");
                telemetry.update();

                while (!isStopRequested() && !gamepad1.a) {
                    idle();
                }
                while (!isStopRequested() && gamepad1.a) {
                    idle();
                }

                telemetry.clearAll();
                telemetry.addLine("Running...");
                telemetry.update();

                double maxPowerTime = DISTANCE / maxVel;

                timeSamples.clear();
                positionSamples.clear();
                powerSamples.clear();

                drive.setPoseEstimate(new Pose2d());
                drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));

                startTime = clock.seconds();
                while (!isStopRequested()) {
                    double elapsedTime = clock.seconds() - startTime;
                    if (elapsedTime > maxPowerTime) {
                        break;
                    }

                    timeSamples.add(elapsedTime);
                    positionSamples.add(drive.getPoseEstimate().getX());
                    powerSamples.add(MAX_POWER);

                    drive.updatePoseEstimate();
                }
                drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

                RegressionUtil.AccelResult accelResult = RegressionUtil.fitAccelData(
                        timeSamples, positionSamples, powerSamples, rampResult,
                        LoggingUtil.getLogFile(Misc.formatInvariant(
                                "DriveAccelRegression-%d.csv", System.currentTimeMillis())));

                telemetry.clearAll();
                telemetry.addLine("Constant power test complete");
                telemetry.addLine(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
                        accelResult.kA, accelResult.rSquare));
                telemetry.update();
            }

            while (!isStopRequested()) {
                idle();
            }
        }
    }
}