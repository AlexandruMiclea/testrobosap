package org.firstinspires.ftc.teamcode.drive.localizer;

//import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class BrokeEncoderLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4175; //TODO
    public static double WHEEL_RADIUS = 2; // inch
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double ENCODER_RATIO = 1.0; // ratio between left encoder ticks per rev and right encoder ticks per rev

    public static double LATERAL_DISTANCE = 13; // inch; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // inch; offset of the lateral wheel

    private DcMotor rightEncoder, middleEncoder;
    BNO055IMU imu;

    public BrokeEncoderLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        rightEncoder = hardwareMap.dcMotor.get("LeftEncoder");
        middleEncoder = hardwareMap.dcMotor.get("MiddleEncoder");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    //@NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(middleEncoder.getCurrentPosition()) * ENCODER_RATIO //TODO: alternative solution to shitty encoders
        );
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}

