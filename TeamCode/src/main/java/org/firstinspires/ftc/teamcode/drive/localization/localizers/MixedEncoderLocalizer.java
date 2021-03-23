package org.firstinspires.ftc.teamcode.drive.localization.localizers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AbsoluteEncoder;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class MixedEncoderLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4175; //TODO
    public static double WHEEL_RADIUS = 2; // inch
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14; // inch; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // inch; offset of the lateral wheel

    public static double MAX_VOLTAGE = 3.27;

    private DcMotor middleEncoder;
    private AbsoluteEncoder rightEncoder = new AbsoluteEncoder();
    BNO055IMU imu;

    public MixedEncoderLocalizer(HardwareMap hardwareMap){
        super(Arrays.asList(
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), //right wheel distance from center in inch
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) //front wheel distance from center in inch
        ));

        middleEncoder= hardwareMap.dcMotor.get("middleEncoder");
        rightEncoder.encoder = hardwareMap.get(AnalogInput.class, "rightEncoder");

        rightEncoder.setInitVolt(rightEncoder.encoder.getVoltage());

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double voltageToInches(double pos) {
        return ((pos * WHEEL_RADIUS * 2* Math.PI) / MAX_VOLTAGE);
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(middleEncoder.getCurrentPosition()),
                voltageToInches(rightEncoder.getVoltageWithIndex())
        );
    }
}
