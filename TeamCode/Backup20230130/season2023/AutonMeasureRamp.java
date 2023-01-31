/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Iterator;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together 
 *  in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: 
 *  RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: 
 *  RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and 
 *  possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a 
 *  flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : 
 *  left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and 
 *  causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  
 *  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode 
 *  for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading 
 *      and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Measure Straight Ramp", group="Robot")

public class AutonMeasureRamp extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         backLeftDrive   = null;
    private DcMotor         backRightDrive  = null;
    private DcMotor         frontLeftDrive   = null;
    private DcMotor         frontRightDrive  = null;
    private BNO055IMU       imu         = null;      // Control/Expansion Hub IMU

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  backLeftSpeed     = 0;
    private double  backRightSpeed    = 0;
    private int     backLeftTarget    = 0;
    private int     backRightTarget   = 0;
    private double  frontLeftSpeed     = 0;
    private double  frontRightSpeed    = 0;
    private int     frontLeftTarget    = 0;
    private int     frontRightTarget   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. 
    // Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // Orig: 537.7 eg: 
                                                                //GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // Orig: 1.0 No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.8 ;     // 96mm = 3.8 Orig: 4.0 For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     MAX_DRIVE_SPEED         = 1.0;     // Orig: 0.4 Max driving speed 
                                                                //for better distance accuracy.
    static final double     MIN_DRIVE_SPEED         = 0.1;     // Min driving speed: keep from 
                                                                //slowing down too much in accel or deccel.
    static final double     MAX_ACCEL_RATE         = 0.0006;   // Units:  
                                            
    static final double     MAX_DECCEL_RATE        = 0.0005;   // Units:  
                                            
    static final double     MAX_TURN_SPEED          = 0.4;     // Orig: 0.2 Max Turn speed to 
                                                                //limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // Orig: 1.0 How close must the 
                                                               //heading get to the target before 
                                                               //moving to next step.
                                                               // Requiring more accuracy (a smaller 
                                                               //number) will often make the turn 
                                                               //take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when 
    //Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough 
    //(eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value 
    //(eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;      // Orig: 0.02 Larger is more responsive, 
                                                                // but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;      // Orig: 0.03 Larger is more responsive, 
                                                                //but also less stable


    @Override
    public void runOpMode() {

        RobotLog.d("MAB - entered runOpMode()");
        // Initialize the drive system variables.
        backLeftDrive  = hardwareMap.get(DcMotorEx.class, "bl");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "br");
        frontLeftDrive  = hardwareMap.get(DcMotorEx.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fr");

        // Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

         //  Set all Expansion hubs to use the AUTO Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

       // To drive forward, most robots need the motor on one side to be reversed, 
        // because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. 
        // So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  
        // Gear Reduction or 90 Deg drives may require direction flips
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();
        
        // Define robot path here:
        // Step through each leg of the path:
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(seconds) after any step to keep the telemetry data 
        //          visible for review.  Comment out for competition.

        driveStraightRamp(144.0, 0.0);    // Drive Forward 144"
        //driveStraight(MAX_DRIVE_SPEED, 144.0, 0.0);    // Drive Forward 144"

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);                              // Pause to display last telemetry message.
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Method to drive in a straight line while not exceeding MAX_ACCEL_RATE.
     * Robot is stopped going into this method.  
     * The first portion of the travel is the accelerate phase where the robot 
     * accelerates to MAX_DRIVE_SPEED.  
     * The second portion is where the robots travels at MAX_DRIVE_SPEED.
     * The last portion is the deccelerate portion where the robot slows to a stop
     * without exceeding -MAX_ACCEL_RATE or going below MIN_DRIVE_SPEED.
     * 
     * MAX_ACCEL_RATE is in units of delta drive speed per delta ticks.???
     * 
     * It is possible that the distance to travel is low enough to not allow the robot
     * to reach MAX_DRIVE_SPEED.  
    */
    public void driveStraightRamp(double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //Setup:
            
            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            RobotLog.d("MAB - driveStraightRamp() - COUNTS_PER_INCH: %8.4f", COUNTS_PER_INCH);
            RobotLog.d("MAB - driveStraightRamp() - moveCounts:                                       %d", moveCounts);
            backLeftTarget = backLeftDrive.getCurrentPosition() + moveCounts;
            backRightTarget = backRightDrive.getCurrentPosition() + moveCounts;
            frontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
            RobotLog.d("MAB - driveStraightRamp() - backLeftTarget: %d", backLeftTarget);
            RobotLog.d("MAB - driveStraightRamp() - backRightTarget: %d", backRightTarget);
            RobotLog.d("MAB - driveStraightRamp() - frontLeftTarget: %d", frontLeftTarget);
            RobotLog.d("MAB - driveStraightRamp() - frontRightTarget: %d", frontRightTarget);

            // Set Target FIRST, then turn on RUN_TO_POSITION
            backLeftDrive.setTargetPosition(backLeftTarget);
            backRightDrive.setTargetPosition(backRightTarget);
            frontLeftDrive.setTargetPosition(frontLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);

            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);  
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            //maxDriveSpeed = Math.abs(maxDriveSpeed);
            // Start driving straight, and then immediately enter the control loop
            RobotLog.d("MAB - driveStraightRamp() - Calling moveRobot with MIN_DRIVE_SPEED");
            moveRobot(MIN_DRIVE_SPEED, 0);  //start at MIN_DRIVE_SPEED 
            //Set up drive speed variable to be used inside control loop:
            //double currDriveSpeed = 0;
            //Set up old drive speed variable to be used inside control loop:
            double oldDriveSpeed = 0;
            //Set up current position variable to be used inside control loop:
            int currPos = 0;
            //Set up old position variable to be used inside control loop:
            int oldPos = 0;
            //Set up delta position variable to be used inside control loop:
            int deltaPos = 0;
            //Set up counter variable to be used to count control loops:
            int nLoops = 0;
            //Set up newDriveSpeed variable to be used inside control loop:
            double newDriveSpeed = 0.0;
            double targetStartDeccelTicks = 0.0;
            double ticksInAccelPhase = 0.0;
            double ticksInDeccelPhase = 0.0;
            double ticksInConstPhase = 0.0;
            double deltaTicks = 0.0;
            
            //double maxPossibleSpeed = moveCounts / MAX_ACCEL_RATE;  //bad measure, clean up
            //RobotLog.d("MAB - driveStraightRamp() - maxPossibleSpeed: %8.4f", maxPossibleSpeed);
            double maxTicksInAccel = (MAX_DRIVE_SPEED - MIN_DRIVE_SPEED) / MAX_ACCEL_RATE;
            double maxTicksInDeccel = (MAX_DRIVE_SPEED - MIN_DRIVE_SPEED) / MAX_DECCEL_RATE;
            boolean constSpeedPortion = (moveCounts > (maxTicksInAccel + maxTicksInDeccel));

            RobotLog.d("MAB - driveStraightRamp() - MAX_DRIVE_SPEED: %8.4f", MAX_DRIVE_SPEED);
            RobotLog.d("MAB - driveStraightRamp() - MIN_DRIVE_SPEED: %8.4f", MIN_DRIVE_SPEED);
            RobotLog.d("MAB - driveStraightRamp() - MAX_ACCEL_RATE: %8.4f", MAX_ACCEL_RATE);
            RobotLog.d("MAB - driveStraightRamp() - MAX_DECCEL_RATE: %8.4f", MAX_DECCEL_RATE);
            RobotLog.d("MAB - driveStraightRamp() - maxTicksInAccel: %8.4f", maxTicksInAccel);
            RobotLog.d("MAB - driveStraightRamp() - maxTicksInDeccel: %8.4f", maxTicksInDeccel);
            RobotLog.d("MAB - driveStraightRamp() - constSpeedPortion: %b", constSpeedPortion);
            if (constSpeedPortion) {
                // there will be a constant speed portion.
                ticksInAccelPhase = maxTicksInAccel;
                RobotLog.d("MAB - driveStraightRamp()CSP - ticksInAccelPhase: %8.4f", ticksInAccelPhase);
                ticksInDeccelPhase = maxTicksInDeccel;
                RobotLog.d("MAB - driveStraightRamp()CSP - ticksInDeccelPhase: %8.4f", ticksInDeccelPhase);
                ticksInConstPhase = moveCounts - ticksInAccelPhase - ticksInDeccelPhase;
                RobotLog.d("MAB - driveStraightRamp()CSP - ticksInConstPhase: %8.4f", ticksInConstPhase);
                targetStartDeccelTicks = moveCounts - ticksInDeccelPhase;
                RobotLog.d("MAB - driveStraightRamp()CSP - targetStartDeccelTicks: %8.4f", targetStartDeccelTicks);
            } else {
                // there will not be a constant speed portion, only an Accel phase and a deccel phase.
                /*
                    to simplify, ticksInAccelPhase is the ticks in accel phase, 
                    ticksInDeccelPhase is the ticks in decel.
                    ticksInAccelPhase / ticksInDeccelPhase ~= (MAX_DECCEL_RATE / MAX_ACCEL_RATE) = ratio  
                    and
                    ticksInAccelPhase + ticksInDeccelPhase = moveCounts
                    ticksInAccelPhase in terms of ticksInDeccelPhase:  ticksInAccelPhase = ratio / ticksInDeccelPhase
                    Eq2 in terms of ticksInDeccelPhase:  ratio / ticksInDeccelPhase + ticksInDeccelPhase = dist.
                    rearrange:
                    ticksInDeccelPhase = moveCounts / (1 + 1 / ratio)
                    Substitute value of ticksInDeccelPhase in eq. 2:  ticksInAccelPhase = moveCounts - ticksInDeccelPhase
                */
                double ratio = MAX_DECCEL_RATE / MAX_ACCEL_RATE;
                RobotLog.d("MAB - driveStraightRamp()nCSP - ratio: %8.4f", ratio);
                ticksInDeccelPhase = moveCounts / (1.0 + ratio);
                RobotLog.d("MAB - driveStraightRamp()nCSP - ticksInDeccelPhase: %8.4f", ticksInDeccelPhase);
                ticksInAccelPhase = moveCounts - ticksInDeccelPhase;
                RobotLog.d("MAB - driveStraightRamp()nCSP - ticksInAccelPhase: %8.4f", ticksInAccelPhase);
                ticksInConstPhase = 0.0;
                RobotLog.d("MAB - driveStraightRamp()nCSP - ticksInConstPhase: %8.4f", ticksInConstPhase);
                targetStartDeccelTicks = moveCounts - ticksInAccelPhase;
                RobotLog.d("MAB - driveStraightRamp()nCSP - targetStartDeccelTicks: %8.4f", targetStartDeccelTicks);
            }
            //Start loop:
            RobotLog.d("MAB - driveStraightRamp() - Starting loop initially");
            RobotLog.d("MAB - driveStraightRamp() - opModeIsActive(): %b", opModeIsActive());
            RobotLog.d("MAB - driveStraightRamp() - backLeftDrive.isBusy(): %b", backLeftDrive.isBusy());
            RobotLog.d("MAB - driveStraightRamp() - backRightDrive.isBusy(): %b", backRightDrive.isBusy());
            RobotLog.d("MAB - driveStraightRamp() - frontLeftDrive.isBusy(): %b", frontLeftDrive.isBusy());
            RobotLog.d("MAB - driveStraightRamp() - frontRightDrive.isBusy(): %b", frontRightDrive.isBusy());
            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&              //Check that opMode is Active(user did not hit stop)
                   (backLeftDrive.isBusy() &&       //
                    backRightDrive.isBusy() && 
                    frontLeftDrive.isBusy() && 
                    frontRightDrive.isBusy())) {

                //Now inside loop.
                //RobotLog.d("MAB - driveStraightRamp() - Inside loop");
                
                // Determine required steering correction to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;
                //RobotLog.d("MAB - driveStraightRamp() - turnSpeed: %8.4f", turnSpeed);
                
                //Determine current portion (accel, const or deccel):
                currPos = backLeftDrive.getCurrentPosition();  //pick a motor.
                deltaPos = currPos - oldPos;
                nLoops += 1;
                //RobotLog.d("MAB - driveStraightRamp() - currPos(%d):                           %d diff: %d", nLoops, currPos, deltaPos);
                //test:
                if (currPos > targetStartDeccelTicks) {
                    //in deccel phase:  lower oldDriveSpeed by MAX_ACCEL_RATE:
                    //deltaPos = currPos - oldPos;
                    //RobotLog.d("MAB - driveStraightRamp()decelPhase - deltaPos: %d", deltaPos);
                    //Reduce MAX_DRIVE_SPEED by using the distance traveled past the targetStartDeccelTicks.
                    newDriveSpeed = MAX_DRIVE_SPEED - ( currPos - targetStartDeccelTicks) * MAX_DECCEL_RATE;
                    //currDriveSpeed = oldDriveSpeed + newDriveSpeed;
                    //RobotLog.d("MAB - driveStraightRamp()decel - currDriveSpeed: %8.4f", currDriveSpeed);
                    if (newDriveSpeed < MIN_DRIVE_SPEED) {
                       newDriveSpeed = MIN_DRIVE_SPEED; 
                    }
                    //RobotLog.d("MAB - driveStraightRamp()decelPhase - newDriveSpeed: %8.4f", newDriveSpeed);
                } else if (currPos < ticksInAccelPhase) {
                    //in accel phase:  raise currDriveSpeed by MAX_ACCEL_RATE:
                    //deltaPos = oldPos - currPos;
                    //RobotLog.d("MAB - driveStraightRamp()accelPhase - deltaPos: %d", deltaPos);
                    //Increase MIN_DRIVE_SPEED by using the distance travelled from the start
                    newDriveSpeed = MIN_DRIVE_SPEED + currPos * MAX_ACCEL_RATE;
                    //RobotLog.d("MAB - driveStraightRamp()accel - newDriveSpeed: %8.4f", newDriveSpeed);
                    //currDriveSpeed = MIN_DRIVE_SPEED + newDriveSpeed;
                    //RobotLog.d("MAB - driveStraightRamp()accelPhase - newDriveSpeed: %8.4f", newDriveSpeed);
                } else {
                    //in const portion.
                    newDriveSpeed = MAX_DRIVE_SPEED;
                    //RobotLog.d("MAB - driveStraightRamp()constPhase - newDriveSpeed: %8.4f", newDriveSpeed);
                }

                // Apply the turning correction to the current driving speed.
                //RobotLog.d("MAB - driveStraightRamp() - Calling moveRobot with newDriveSpeed: %8.4f and turnSpeed: %8.4f", newDriveSpeed, turnSpeed);
                moveRobot(newDriveSpeed, turnSpeed);
                RobotLog.d("MAB - DSR() - %5d %8.4f %5d %5d %8.4f", nLoops, turnSpeed, currPos, deltaPos, newDriveSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
                
                //update:
                oldPos = currPos;
                //oldDriveSpeed = currDriveSpeed;
            }
            RobotLog.d("MAB - driveStraightRamp() - Loop complete");

            // Stop all motion & Turn off RUN_TO_POSITION
            stopRobot();
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  
    *                   Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            RobotLog.d("MAB - driveStraight() - moveCounts: %d", moveCounts);
            backLeftTarget = backLeftDrive.getCurrentPosition() + moveCounts;
            backRightTarget = backRightDrive.getCurrentPosition() + moveCounts;
            frontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
            RobotLog.d("MAB - driveStraight() - backLeftTarget: %d", backLeftTarget);
            RobotLog.d("MAB - driveStraight() - backRightTarget: %d", backRightTarget);
            RobotLog.d("MAB - driveStraight() - frontLeftTarget: %d", frontLeftTarget);
            RobotLog.d("MAB - driveStraight() - frontRightTarget: %d", frontRightTarget);

            // Set Target FIRST, then turn on RUN_TO_POSITION
            backLeftDrive.setTargetPosition(backLeftTarget);
            backRightDrive.setTargetPosition(backRightTarget);
            frontLeftDrive.setTargetPosition(frontLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);

            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                   (backLeftDrive.isBusy() && 
                    backRightDrive.isBusy() && 
                    frontLeftDrive.isBusy() && 
                    frontRightDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                RobotLog.d("MAB - driveStraight() - turnSpeed: %8.4f", turnSpeed);
                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(maxDriveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopRobot();
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        stopRobot();
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        stopRobot();
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction.  
        // Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method stops the robot movement.
     */
    public void stopRobot() {
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
    }
    
    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive_speed, double turn_speed) {
        driveSpeed = drive_speed;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn_speed;      // save this value as a class member so it can be used by telemetry.
        /*
        RobotLog.d("MAB - Start of moveRobot() - drive_speed: %6.4f", drive_speed);
        RobotLog.d("MAB - Start of moveRobot() - turn_speed: %6.4f", turn_speed);
        RobotLog.d("MAB - moveRobot() - initial backLeftSpeed: %6.4f", backLeftSpeed);
        RobotLog.d("MAB - moveRobot() - initial backRightSpeed: %6.4f", backRightSpeed);
        RobotLog.d("MAB - moveRobot() - initial frontLeftSpeed: %6.4f", frontLeftSpeed);
        RobotLog.d("MAB - moveRobot() - initial frontRightSpeed: %6.4f", frontRightSpeed);
        */
        backLeftSpeed  = drive_speed - turn_speed;
        backRightSpeed = drive_speed + turn_speed;
        frontLeftSpeed  = drive_speed - turn_speed;
        frontRightSpeed = drive_speed + turn_speed;
        /*
        RobotLog.d("MAB - moveRobot() - after turnspeed backLeftSpeed: %6.4f", backLeftSpeed);
        RobotLog.d("MAB - moveRobot() - after turnspeed backRightSpeed: %6.4f", backRightSpeed);
        RobotLog.d("MAB - moveRobot() - after turnspeed frontLeftSpeed: %6.4f", frontLeftSpeed);
        RobotLog.d("MAB - moveRobot() - after turnspeed frontRightSpeed: %6.4f", frontRightSpeed);
        */
        // Scale speeds down if any one exceeds +/- drive_speed;
        double max = Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed));
        max = Math.max(max, Math.abs(frontLeftSpeed));
        max = Math.max(max, Math.abs(frontRightSpeed));
        //RobotLog.d("MAB - moveRobot() - max: %6.4f", max);
        if (max > MAX_DRIVE_SPEED) {
            //Ratio max with drive_speed:
            double corr = drive_speed / max;
            //RobotLog.d("MAB - moveRobot() - corr: %6.4f", corr);
            backLeftSpeed /= corr;
            backRightSpeed /= corr;
            frontLeftSpeed /= corr;
            frontRightSpeed /= corr;
        }
        /*
        RobotLog.d("MAB - moveRobot() - after max backLeftSpeed: %6.4f", backLeftSpeed);
        RobotLog.d("MAB - moveRobot() - after max backRightSpeed: %6.4f", backRightSpeed);
        RobotLog.d("MAB - moveRobot() - after max frontLeftSpeed: %6.4f", frontLeftSpeed);
        RobotLog.d("MAB - moveRobot() - after max frontRightSpeed: %6.4f", frontRightSpeed);
        RobotLog.d("MAB - moveRobot() - setPower to these values and exit moveRobot()");
        */
        backLeftDrive.setPower(backLeftSpeed);
        backRightDrive.setPower(backRightSpeed);
        frontLeftDrive.setPower(frontLeftSpeed);
        frontRightDrive.setPower(frontRightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder 
     * positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Back Target Pos L:R",  "%7d:%7d",      
                    backLeftTarget,  backRightTarget);
            telemetry.addData("Front Target Pos bL:bR",  "%7d:%7d",      
                    frontLeftTarget,  frontRightTarget);
            telemetry.addData("Actual Back Pos L:R",  "%7d:%7d",      
                    backLeftDrive.getCurrentPosition(),
                    backRightDrive.getCurrentPosition());
            telemetry.addData("Actual Front Pos L:R",  "%7d:%7d",      
                    frontLeftDrive.getCurrentPosition(),
                    frontRightDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds bL:bR:fL:fR.", "%5.2f : %5.2f : %5.2f : %5.2f", 
                          backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
        //telemetry.addData("Front Wheel Speeds L:R.", "%5.2f : %5.2f", frontLeftSpeed, frontRightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(
                AxesReference.INTRINSIC, 
                AxesOrder.ZYX, 
                AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
