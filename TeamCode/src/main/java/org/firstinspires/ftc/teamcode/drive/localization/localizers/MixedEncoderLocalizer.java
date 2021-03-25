package org.firstinspires.ftc.teamcode.drive.localization.localizers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class MixedEncoderLocalizer extends TwoTrackingWheelLocalizer {
    private static double TICKS_PER_REV = 1024; //TODO
    private static double WHEEL_RADIUS = 2; // inch

    private static double LATERAL_DISTANCE = 14; // inch; distance between the left and right wheels
    private static double FORWARD_OFFSET = 0; // inch; offset of the lateral wheel

    private Encoder middleEncoder;
    private AbsoluteEncoder rightEncoder;
    BNO055IMU imu;

    public MixedEncoderLocalizer(HardwareMap hardwareMap){
        super(Arrays.asList(
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), //right wheel distance from center in inch
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) //front wheel distance from center in inch
        ));

        middleEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "middleEncoder"));
        rightEncoder = new AbsoluteEncoder(hardwareMap.get(AnalogInput.class, "rightEncoder"));

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public double encoderTicksToInches(Encoder encoder) {
        return WHEEL_RADIUS * 2 * Math.PI * encoder.getCurrentPosition() / encoder.getTicksPerRev();
    }

    public double voltageToInches(AbsoluteEncoder encoder) {
        return ((encoder.getVoltageWithIndex() * WHEEL_RADIUS * 2 * Math.PI) / encoder.getMaxVoltage());
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(middleEncoder),
                voltageToInches(rightEncoder)
        );
    }
}
