package org.firstinspires.ftc.teamcode.MAB_2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class Redallianceredterminal extends BaseAuton{

    // todo: write your code here

   public void runOpMode(){
        super.runOpMode();
    DrivexFeet(0.5,0.3);
    DrivexFeet(-0.5,0.3);
    turnnDegreesAbsoute(-135);
    DrivexFeet(-1.5);
    
    } 
}
