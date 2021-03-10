package org.firstinspires.ftc.teamcode.drive.localizer;

//import android.support.annotation.NonNull;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.List;

import static java.lang.Math.abs;

public class AnalogEncoderLocalizer extends TwoTrackingWheelLocalizer {
    public static double WHEEL_RADIUS = 4; // inch, care e defapt diametrul but oh well
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double MAX_VOLTAGE = 3.27;

    public static double ENCODER_RATIO = 1.0; // ratio between left encoder ticks per rev and right encoder ticks per rev

    public static double LATERAL_DISTANCE = 14 ; // inch; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // inch; offset of the lateral wheel

    private absoluteEncoder middleEncoder = new absoluteEncoder();
    private absoluteEncoder rightEncoder = new absoluteEncoder();

    BNO055IMU imu;

    public AnalogEncoderLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));


        middleEncoder.encoder = hardwareMap.get(AnalogInput.class, "middleEncoder");
        rightEncoder.encoder = hardwareMap.get(AnalogInput.class, "rightEncoder");

        middleEncoder.setInitVolt(middleEncoder.encoder.getVoltage());
        rightEncoder.setInitVolt(rightEncoder.encoder.getVoltage());


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public static double voltageToInches(double pos) {
        return ((pos * WHEEL_RADIUS * Math.PI) / MAX_VOLTAGE);
//        return toThreeDec(((pos * WHEEL_RADIUS * Math.PI) / MAX_VOLTAGE));
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                voltageToInches(rightEncoder.getVoltageWithIndex()),
                voltageToInches(middleEncoder.getVoltageWithIndex())
        );
    }

    public List<Double> getVoltages() {
        return Arrays.asList(
                rightEncoder.toThreeDec(rightEncoder.readVoltage()),
                middleEncoder.toThreeDec(middleEncoder.readVoltage())
        );
    }

    public List <Double> getDerivatives(){
        return Arrays.asList(
                rightEncoder.getDerivative(),
                middleEncoder.getDerivative()
        );
    }

    public List <Double> getTotalVoltages(){
        return Arrays.asList(
                rightEncoder.getTotalVoltage(),
                middleEncoder.getTotalVoltage()
        );
    }

    public List <Double> getTotalVoltagesWithIndex(){
        return Arrays.asList(
                rightEncoder.getVoltageWithIndex(),
                middleEncoder.getVoltageWithIndex()
        );
    }

    public List <Double> getIndex(){
        return Arrays.asList(
                rightEncoder.getTurnIndex(),
                middleEncoder.getTurnIndex()
        );
    }

    public static double toThreeDec(double num) {
        DecimalFormat numberFormat = new DecimalFormat("#.000");
        return Double.parseDouble(numberFormat.format(num));
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}

class absoluteEncoder{
    public AnalogInput encoder;
    public double lastVoltage;
    public double totalVoltage;

    public double lastIndexVoltage=0;
    public double turnIndex = 0;
    public double voltageWithIndex;

    public static double MAX_VOLTAGE =3.25;

    public double initVolt;

//    absoluteEncoder (AnalogSensor encoder, double lastVoltage, double totalVoltage){
//        this.encoder = encoder;
//        this.lastVoltage = lastVoltage;
//        this.totalVoltage = totalVoltage;
//    }

    public void setInitVolt(double initVoltage){
        this.initVolt = initVoltage;
    }

    public double getDerivative(){
        double current = toThreeDec(encoder.getVoltage());

        double eps = 2 * 1e-3;

        double derivative = toThreeDec(current - lastVoltage);

        if (abs(derivative) <= eps) derivative = 0;

        if ((derivative < -MAX_VOLTAGE / 2) && (derivative != 0)){
            derivative += MAX_VOLTAGE;
        } else if ((derivative > MAX_VOLTAGE / 2) && (derivative != 0)){
            derivative -= MAX_VOLTAGE;
        }

        lastVoltage = current;

        return derivative;
    }

    public double getTotalVoltage(){
        double derivative = getDerivative();

        totalVoltage += derivative;

        return totalVoltage;
    }


    public double readVoltage() {
        return encoder.getVoltage();
    }

    public static double toThreeDec(double num) {
        DecimalFormat numberFormat = new DecimalFormat("#.000");
        return Double.parseDouble(numberFormat.format(num));
    }

    public double getVoltageWithIndex(){ //TODO cleanup, momentan functioneaza ca un get index
        double current1 = encoder.getVoltage();

        double derivative1 = current1 - lastIndexVoltage;

        if ((derivative1 < -MAX_VOLTAGE / 2) && (derivative1 != 0)){
            turnIndex ++;
        } else if ((derivative1 > MAX_VOLTAGE / 2) && (derivative1 != 0)){
            turnIndex --;
        }

        lastIndexVoltage = current1;

        voltageWithIndex = current1 + (turnIndex * MAX_VOLTAGE) - initVolt;

        return voltageWithIndex;
    }

    public double getTurnIndex(){
        return turnIndex;
    }
}

