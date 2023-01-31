package org.firstinspires.ftc.teamcode.season2023;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
//import android.graphics.Color;

@TeleOp
@Disabled
public class TestOp_MAB  extends OpMode{
    BNO055IMU imu = null;
    DcMotor frontleft;
    DcMotor backleft;
    DcMotor frontright;
    DcMotor backright;
    // todo: write your code here
    public void init(){
        frontleft= hardwareMap.get(DcMotor.class, "fl");
        backleft= hardwareMap.get(DcMotor.class, "bl");
        frontright= hardwareMap.get(DcMotor.class, "fr");
        backright= hardwareMap.get(DcMotor.class, "br");
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void loop(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        telemetry.addData("firstAngle", angles.firstAngle);
        telemetry.addData("secondAngle", angles.secondAngle );
        telemetry.addData("thirdAngle", angles.thirdAngle );
        
        if (gamepad1.y){
            frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } 
        if (gamepad1.x){
            backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } 
        if (gamepad1.a){
            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } 
        if (gamepad1.b){
            backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } 
    }
}
