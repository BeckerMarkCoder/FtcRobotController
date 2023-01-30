package org.firstinspires.ftc.teamcode.season2022;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import android.graphics.Color;

@TeleOp
//@Disabled
public class TestOp  extends OpMode{
    BNO055IMU imu = null;
    
    ColorSensor colorsensor;
    DcMotor frontleft;
    DcMotor lift;
    // todo: write your code here
    public void init(){
        frontleft= hardwareMap.get(DcMotor.class, "fl");
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        colorsensor= hardwareMap.get(ColorSensor.class,"cs");
        
    lift= hardwareMap.get(DcMotor.class, "up");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void loop(){
        telemetry.addData("ticks",lift.getCurrentPosition());
        telemetry.addData("Hue",getHue());
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        telemetry.addData("firstAngle", angles.firstAngle);
        telemetry.addData("secondAngle", angles.secondAngle );
        telemetry.addData("thirdAngle", angles.thirdAngle );
        
        
        if (gamepad1.y){
            frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } 
        
        double power = gamepad1.left_stick_y;
        lift.setPower(power);
        
        
    }
    public double getHue(){
        
    float hsvValues[] = {0F, 0F, 0F};
    
    final double SCALE_FACTOR = 255;
        Color.RGBToHSV((int)(colorsensor.red() * SCALE_FACTOR),
            (int) (colorsensor.green() * SCALE_FACTOR),
            (int) (colorsensor.blue() * SCALE_FACTOR),
            hsvValues);
            return hsvValues[0];
    }
}