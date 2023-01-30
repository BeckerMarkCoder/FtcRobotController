package org.firstinspires.ftc.teamcode.season2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class Redallianceblueterminal extends BaseAuton{

    // todo: write your code here

    public void runOpMode(){
        super.runOpMode();
    DrivexFeet(0.7,0.2);
    waitnseconds(1);
    DrivexFeet(-0.7,0.3);
    turnnDegreesAbsoute(135);
    DrivexFeet(1.5);
    
    }
}