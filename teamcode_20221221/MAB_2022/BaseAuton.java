/*
Copyright 2022 FIRST Tech Challenge Team 10279

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.MAB_2022;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */

public class BaseAuton extends LinearOpMode {
DcMotor frontleft;
DcMotor frontright;
DcMotor backleft;
DcMotor backright;
BNO055IMU imu = null;
    
    @Override
    public void runOpMode() {
    frontleft= hardwareMap.get(DcMotor.class, "fl");    
    frontright= hardwareMap.get(DcMotor.class, "fr");
    backleft= hardwareMap.get(DcMotor.class, "bl");
    backright= hardwareMap.get(DcMotor.class, "br");
    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    backleft.setDirection(DcMotorSimple.Direction.REVERSE);

     BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

 
    }
    
    public void DrivexFeet(double x){
        DrivexFeet(x,0.5);
    }
    public void DrivexFeet(double x,double speed){
        double ticks=x*877;
        
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            
            if(x>0){
                while(frontleft.getCurrentPosition()<ticks && opModeIsActive()){
                    frontleft.setPower(speed);
                    frontright.setPower(speed); 
                    backright.setPower(speed);
                    backleft.setPower(speed);
                } 
                
            }
            else{
                while(frontleft.getCurrentPosition()>ticks && opModeIsActive()){
                    frontleft.setPower(-speed);
                    frontright.setPower(-speed); 
                    backright.setPower(-speed);
                    backleft.setPower(-speed);
                } 
            }
            
            
            frontleft.setPower(0);
                frontright.setPower(0); 
                backright.setPower(0);
                backleft.setPower(0);
            
            
    } // last curly brace of DrivexFeet
    
    public void StrafexFeet(double x){
        double ticks=x*877;
        
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            
            if(x>0){
                while(frontleft.getCurrentPosition()<ticks && opModeIsActive()){
                    frontleft.setPower(0.5);
                    frontright.setPower(-0.5); 
                    backright.setPower(0.5);
                    backleft.setPower(-0.5);
                } 
                
            }
            else{
                while(frontleft.getCurrentPosition()>ticks && opModeIsActive()){
                    frontleft.setPower(-0.5);
                    frontright.setPower(0.5); 
                    backright.setPower(-0.5);
                    backleft.setPower(0.5);
                } 
            }
            
            
            frontleft.setPower(0);
                frontright.setPower(0); 
                backright.setPower(0);
                backleft.setPower(0);
            
            
    }
    
    public double getcurrentAngle(){          
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
        
    }

    public double BearingDiff(double from_a,double to_a ){
        double R;
        R=(to_a-from_a)%360.0;
        if(R>=100){
            R=R-360;
        }
        return R;
    
    }
    
    public double NormalizeBearing(double a){
        while(a>180){
            a=a-360;
        }
        while(a<-180){
            a=a+360;
        }
        return a;
    }
    
    
    public void turnnDegreesAbsoute(double degrees){
        degrees=NormalizeBearing(degrees);
        double error;
        double speed = 0.25;
        do{
        error=BearingDiff(getcurrentAngle(),degrees);
        if (error>0){
            frontleft.setPower(-speed);
                    frontright.setPower(speed); 
                    backright.setPower(speed);
                    backleft.setPower(-speed);
        }
        else{
            frontleft.setPower(speed);
                    frontright.setPower(-speed); 
                    backright.setPower(-speed);
                    backleft.setPower(speed);
        }
        } while(Math.abs(error)>2 && opModeIsActive());
        
        frontleft.setPower(0);
                    frontright.setPower(0); 
                    backright.setPower(0);
                    backleft.setPower(0);
        
    }
    
    

    public void waitnseconds(double n){
        ElapsedTime runtime=new ElapsedTime();
        runtime.reset();
        while(runtime.time()< n){
            idle();
        }
    }


 
    
}

    
    
    

