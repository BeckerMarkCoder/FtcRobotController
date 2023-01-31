

package org.firstinspires.ftc.teamcode.season2023;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Basic: Mechanum ", group = "Iterative Opmode")
@Disabled
public class Mechanum extends OpMode {

    boolean EnableFieldCentric = false;

    double desiredAngle = 0;

    double armspeed = 1;
    double intakespeed = 1.0;
    double dpadspeed = 0.35;
    double liftspeed=1; 
    double liftdownspeed=-0.6; 
    double outspeed=0.45; 


    DcMotor frontleftmotor = null;
    DcMotor frontrightmotor = null;
    DcMotor backleftmotor = null;
    DcMotor backrightmotor = null;
    DcMotor leftpickup = null;
    DcMotor rightpickup = null;
    Servo leftfoundationservo= null; 
    Servo rightfoundationservo=null; 
    DcMotor lift=null; 
    DcMotor out=null; 
    Servo frontgrab=null; 
    Servo backgrab=null; 
    

    BNO055IMU imu = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        frontleftmotor = hardwareMap.get(DcMotor.class, "flm");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frm");
        backleftmotor = hardwareMap.get(DcMotor.class, "blm");
        backrightmotor = hardwareMap.get(DcMotor.class, "brm");
        leftpickup = hardwareMap.get(DcMotor.class, "lpm");
        rightpickup = hardwareMap.get(DcMotor.class, "rpm");
        leftfoundationservo= hardwareMap.get(Servo.class, "lfs"); 
        rightfoundationservo= hardwareMap.get(Servo.class, "rfs"); 
        lift=hardwareMap.get(DcMotor.class, "lm");
        out=hardwareMap.get(DcMotor.class, "o"); 
        frontgrab=hardwareMap.get(Servo.class, "fg"); 
        backgrab=hardwareMap.get(Servo.class, "bg");
        


        frontrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightpickup.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        desiredAngle = 0;
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double a = 0;
        telemetry.addData("LT1","%f",gamepad1.left_trigger);
        if (gamepad2.left_bumper) {
            leftpickup.setPower(a);
            rightpickup.setPower(a);
            a = intakespeed;
        }

        if (gamepad2.right_bumper) {
            leftpickup.setPower(a);
            rightpickup.setPower(a);
            a = -intakespeed;
        }
        
        
        if (gamepad2.left_trigger>0.8){
            leftfoundationservo.setPosition(1); 
            rightfoundationservo.setPosition(0); 
        }
        if (gamepad2.right_trigger>0.8){
            leftfoundationservo.setPosition(0); 
            rightfoundationservo.setPosition(1);
        }
        
        
        
        if (gamepad2.dpad_up){
            lift.setPower(liftspeed); 
        }
        
        else if (gamepad2.dpad_down){
            lift.setPower(liftdownspeed); 
        }
        else{
            lift.setPower(0); 
        }
        
        
        
        
        
        
        if (gamepad2.dpad_right /*&& out.getCurrentPosition() > -2550*/){
            out.setPower(outspeed); 
        }
        else if (gamepad2.dpad_left /*&& out.getCurrentPosition() < 10*/){
            out.setPower(-outspeed); 
        }
        else{
            out.setPower(0);
            
        }
        
        
        telemetry.addData("Lift Position", String.format("%d", lift.getCurrentPosition()));
        telemetry.addData("Out Position", String.format("%d", out.getCurrentPosition()));
        
        
        
        
        if (gamepad2.x){
        frontgrab.setPosition(0.9);
        backgrab.setPosition(0.1);
        }
        if (gamepad2.y){
        frontgrab.setPosition(0.9);
        backgrab.setPosition(0.7);
         }
        if (gamepad2.b){
        frontgrab.setPosition(0.1); 
        backgrab.setPosition(0.75); 
        }
    
        
        
        
        rightpickup.setPower(a);
        leftpickup.setPower(a);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;
        telemetry.addData("currentAngle", "%.2f", currentAngle);
        double x = 0;
        double y = 0;
        double rotation = 0;

        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y * -1;
        
        



        
        x = Math.pow(x, 3);
        y = Math.pow(y, 3);
        
        if(gamepad1.dpad_down){
            y -= dpadspeed;
        }
        if(gamepad1.dpad_up){
            y += dpadspeed;
        }

        if(gamepad1.dpad_right){
            x += dpadspeed;
        }

        if(gamepad1.dpad_left){
            x -= dpadspeed;
        }

        if (EnableFieldCentric) {
            double Radians = Math.toRadians(currentAngle * -1);
            double NewX = x * Math.cos(Radians) - y * Math.sin(Radians);
            double NewY = x * Math.sin(Radians) + y * Math.cos(Radians);
            x = NewX;
            y = NewY;

        }

        double frontleftmotorposition = frontleftmotor.getCurrentPosition();
        telemetry.addData("frontleftmotor", "%.2f", frontleftmotorposition);

        
        rotation = gamepad1.right_stick_x*0.5;
        
                
        if(gamepad1.left_bumper || gamepad1.right_bumper){
            x *= 0.5;
            y *= 0.5;
            rotation *= 0.5;
        }

        double frontleftpower = 0;
        double frontrightpower = 0;
        double backleftpower = 0;
        double backrightpower = 0;


        frontleftpower = frontleftpower + x;
        frontrightpower = frontrightpower - x;
        backleftpower = backleftpower - x;
        backrightpower = backrightpower + x;

        frontleftpower = frontleftpower + y;
        frontrightpower = frontrightpower + y;
        backleftpower = backleftpower + y;
        backrightpower = backrightpower + y;


        frontleftpower = frontleftpower + rotation;
        frontrightpower = frontrightpower - rotation;
        backleftpower = backleftpower + rotation;
        backrightpower = backrightpower - rotation;

        

      



        double max = 0;
        max = Math.max(Math.max(Math.abs(frontleftpower), Math.abs(frontrightpower)),
                Math.max(Math.abs(backleftpower), Math.abs(backrightpower)));

        if (max > 1) {
            frontleftpower = frontleftpower / max;
            frontrightpower = frontrightpower / max;
            backleftpower = frontleftpower / max;
            backrightpower = backrightpower / max;
        }

        frontleftmotor.setPower(frontleftpower);
        frontrightmotor.setPower(frontrightpower);
        backleftmotor.setPower(backleftpower);
        backrightmotor.setPower(backrightpower);


    }


    public double calculateAngleDifference(double targetAngle, double currentAngle) {
        double d = Math.abs(targetAngle - currentAngle) % 360;
        double r = d > 180 ? -d : d;

        int sign = (targetAngle - currentAngle >= 0 && targetAngle - currentAngle <= 180) || (targetAngle - currentAngle <= -180 && targetAngle - currentAngle >= -360) ? 1 : -1;
        r *= sign;
        return r;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }


}

