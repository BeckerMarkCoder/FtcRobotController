package org.firstinspires.ftc.teamcode.season2023;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp//(name = "Simple Mecanum")
public class SimpleMecanum_MAB extends OpMode{

DcMotor frontleft;
DcMotor frontright;
DcMotor backleft;
DcMotor backright;

    @Override
    public void init(){
    frontleft= hardwareMap.get(DcMotor.class, "fl");    
    frontright= hardwareMap.get(DcMotor.class, "fr");
    backleft= hardwareMap.get(DcMotor.class, "bl");
    backright= hardwareMap.get(DcMotor.class, "br");
    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    backleft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    @Override
    public void loop(){
    double x;
    double y;
    double rotate;
    double wheelPower = 0.7;
    x=gamepad1.left_stick_x;
    y=gamepad1.left_stick_y;
    rotate=gamepad1.right_stick_x;
    
    y=y*-1;
    x=Math.pow(x,3);
    y=Math.pow(y,3);
    rotate=Math.pow(rotate,3);
    if(!gamepad1.right_bumper){
        x=x*wheelPower;
        y=y*wheelPower;
        rotate=rotate*wheelPower;
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

    }
}
