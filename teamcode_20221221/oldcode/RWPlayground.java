package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
@Disabled
public class RWPlayground extends OpMode{

    DcMotor armMotor;
    // todo: write your code here
    public void init(){
        armMotor=hardwareMap.get(DcMotor.class,"Bob" );
        armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
    }
    
    public void loop(){
        if(gamepad1.right_bumper){
            armMotor.setPower(0.5);
            
        }
        else{
            armMotor.setPower(0);
            
        }
        if(gamepad1.a){
            armMotor.setTargetPosition(240);
            
        }
        else{
            armMotor.setTargetPosition(0);
            
        }
        
          double position= armMotor.getCurrentPosition();
          telemetry.addData("position", "%.2f ticks", position);
          double desiredPosition = armMotor.getTargetPosition();
          telemetry.addData("desiredPosition", "%.2f ticks", desiredPosition);
          
    }
}
