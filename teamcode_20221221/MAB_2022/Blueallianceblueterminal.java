package org.firstinspires.ftc.teamcode.MAB_2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class Blueallianceblueterminal extends BaseAuton{

    // todo: write your code here

    public void runOpMode(){
        super.runOpMode();
        closedhand();
        DrivexFeet(14.5/12,0.3);
        StrafexFeet(5/12.0);
        waitnseconds(1);
        int zone=readzone();
        //int zone = 3;
        telemetry.addData("zone",zone);
        telemetry.update();
        if(zone==1){
            StrafexFeet(-5.0/12);
            DrivexFeet(0.9);
        StrafexFeet(-2.2);
        turnnDegreesAbsoute(0);
        DrivexFeet(0.2);      
        

        }
        if(zone==2){
            StrafexFeet(5/-12.0);
            DrivexFeet(27/12.0);
        }
        if(zone==3){
            DrivexFeet(10.5/12);
            StrafexFeet(19/12.0);
            DrivexFeet(3.3/12);
            turnnDegreesAbsoute(0);
            DrivexFeet(3.3/12);
            
        }
        
    }
}
