package org.firstinspires.ftc.teamcode.season2022;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Simple Mecanum")
public class SimpleMecanum extends OpMode{

DcMotor frontleft;
DcMotor frontright;
DcMotor backleft;
DcMotor backright;
Servo righthand;
Servo lefthand;
DcMotor lift;
Servo wrist;



    @Override
    public void init(){
    frontleft= hardwareMap.get(DcMotor.class, "fl");    
    frontright= hardwareMap.get(DcMotor.class, "fr");
    backleft= hardwareMap.get(DcMotor.class, "bl");
    backright= hardwareMap.get(DcMotor.class, "br");
    righthand= hardwareMap.get(Servo.class, "rh");
    lefthand = hardwareMap.get(Servo.class, "lh");
    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    backleft.setDirection(DcMotorSimple.Direction.REVERSE);
    lift=hardwareMap.get(DcMotor.class, "up");
    wrist=hardwareMap.get(Servo.class, "w");
    }
    
    @Override
    public void loop(){
    double x;
    double y;
    double rotate;
    x=gamepad1.left_stick_x;
    y=gamepad1.left_stick_y;
    rotate=gamepad1.right_stick_x;
    
    y=y*-1;
    x=Math.pow(x,3);
    y=Math.pow(y,3);
    rotate=Math.pow(rotate,3);
    if(!gamepad1.right_bumper){
        x=x*0.25;
        y=y*0.25;
        rotate=rotate*0.25;
    }
    
    
    
    double flpower;
    double frpower;
    double blpower;
    double brpower;
    flpower=x+y+rotate;
    frpower=-x+y-rotate;
    blpower=-x+y+rotate;
    brpower=x+y-rotate;
    frontleft.setPower(flpower);
    frontright.setPower(frpower);
    backleft.setPower(blpower);
    backright.setPower(brpower);
    if(gamepad2.dpad_up || gamepad2.left_bumper){
        lefthand.setPosition(0.7);
        righthand.setPosition(0.45);
    }if(gamepad2.dpad_down){
        lefthand.setPosition(0);
        righthand.setPosition(0.8);
    }
     
    double uppower;
    uppower=gamepad2.left_stick_y;
    uppower=uppower*-0.6;
    if (gamepad2.right_bumper){
        uppower=uppower*2;
    }
   lift.setPower(uppower);  
   
   
    
    double wristpower;
    wristpower=gamepad2.right_stick_y;
    wristpower=-0.5*wristpower+0.5;
    wrist.setPosition(wristpower);
    
    
    
    
    }
}