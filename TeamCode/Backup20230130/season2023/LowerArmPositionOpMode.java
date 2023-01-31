/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.season2023;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Lower Arm Position", group="Iterative Opmode")

public class LowerArmPositionOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lowerArm = null;

    // Setup global variables for lower arm:
    double lowerArmMaxPower = 0.3;
    double lowerArmPower = 0.0;
    double lowerArmTarget = 0;  //convert to degrees in upgrade
    double lowerArmGain = 0.002;
    double deltaPower = 0;
    double lowerArmError;
    int currArmPosition;
    double totalError = 0;
    double kI = .0000001;
    double lowerArmMoveLimit = 4.0;  //Amount per loop to change position.
    int  lowerArmLowerLimit = 0;
    int  lowerArmUpperLimit = 1100;  //Measured
    double lowerArmMinInput = 0.1;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lowerArm  = hardwareMap.get(DcMotor.class, "la");

        //Change to REVERSE if needed:
        lowerArm.setDirection(DcMotor.Direction.REVERSE);
        lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        runtime.reset();
        lowerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Call all process functions:
        processLowerArm();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
    //User functions

    /**
     * Process lower arm control
     */
    public void processLowerArm() {
        // 
        //lowerArmTarget = 600;
        //use gamepad1.left_stick_y to change lowerArmTarget:
        //double lowerArmUserInput = gamepad1.left_stick_y;
        double lowerArmUserInput = readLeftBumperTrigger();
        //Check whether the absolute value of lowerArmUserInput is above lowerArmMinInput:
        if (Math.abs(lowerArmUserInput) > 0.0) {
            //user is requesting that the lower arm move.
            lowerArmTarget += lowerArmMoveLimit * lowerArmUserInput;
            //limit the lower arm from moving past its upper limit:
            if (lowerArmTarget > lowerArmUpperLimit) {
               lowerArmTarget = lowerArmUpperLimit;
            }
            //limit the lower arm from moving past its lower limit:
            if (lowerArmTarget < lowerArmLowerLimit) {
               lowerArmTarget = lowerArmLowerLimit;
            }
            //clear total error:
            totalError = 0;
        }
        updateLowerArmPower();
        telemetry.addData("Motors", "lowerArmUserInput (%.2f)", lowerArmUserInput);
        telemetry.addData("Motors", "lower arm current pos (%d)", currArmPosition);
        telemetry.addData("Motors", "lower arm target (%.3f)", lowerArmTarget);
        telemetry.addData("Motors", "lower arm error (%.3f)", lowerArmError);
        telemetry.addData("Motors", "lower arm power (%.5f)", lowerArmPower);
    }
    
    /**
     * readLeftBumperTrigger
     */
    public double readLeftBumperTrigger() {
        // The left trigger controls the lower arm movement
        // while the left bumber changes direction of the movement.
        double triggerValue = gamepad1.left_trigger;
        //Check whether the absolute value of stick_Y input is above lowerArmMinInput:
        if (Math.abs(triggerValue) > lowerArmMinInput) {
            //user is requesting that the lower arm move.
            if (gamepad1.left_bumper) {
                //left bumper is pressed, trigger value is negative:
                triggerValue = -1 * triggerValue;
            }
        } else {
            triggerValue = 0.0;
        }
        return triggerValue;
    }

    /**
     * Update lowerArm power based on setPoint difference
     */
    public void updateLowerArmPower() {
        // 
        //Read the lower arm position:
        currArmPosition = lowerArm.getCurrentPosition();
        //calculate difference from target:
        lowerArmError = lowerArmTarget - currArmPosition;
        totalError += lowerArmError;

        //calculate PI control:
        lowerArmPower = lowerArmGain * lowerArmError + kI * totalError;
        if (lowerArmPower > lowerArmMaxPower) {
            lowerArmPower = lowerArmMaxPower;
        }
        RobotLog.d("MAB -  %d %.3f %.3f %.2f %.5f %.5f", 
            currArmPosition, 
            lowerArmTarget,
            lowerArmError, 
            totalError, 
            lowerArmGain * lowerArmError, 
            kI * totalError, 
            lowerArmPower);

        // Send calculated power to arm motor
        lowerArm.setPower(lowerArmPower);
    }
    

}
