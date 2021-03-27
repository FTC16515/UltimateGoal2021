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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Official TeleOp", group="Linear Opmode")
//@Disabled
public class OPModeBlue extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor zeroDrive = null;
    private DcMotor twoDrive = null;
    private DcMotor oneDrive = null;
    private DcMotor threeDrive = null;
    private DcMotor rightPull  = null;
    private DcMotor leftPull   = null;
    private DcMotor complacentLeftDrive = null;
    private DcMotor complacentRightDrive = null;

    //added later
    private Servo left_hand = null;
    private Servo right_hand = null;

    //static final double MAX_POSITION = 0.5;
    // static final double MIN_POSITION = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        zeroDrive = hardwareMap.get(DcMotor.class, "zeroDrive");
        twoDrive = hardwareMap.get(DcMotor.class, "twoDrive");
        oneDrive  = hardwareMap.get(DcMotor.class, "oneDrive");
        threeDrive = hardwareMap.get(DcMotor.class, "threeDrive");
        leftPull = hardwareMap.get(DcMotor.class, "leftPull");
        rightPull = hardwareMap.get(DcMotor.class,"rightPull");
        complacentLeftDrive = hardwareMap.get(DcMotor.class, "complacentLeftDrive");
        complacentRightDrive = hardwareMap.get(DcMotor.class, "complacentRightDrive");
        right_hand = hardwareMap.get(Servo.class, "right_hand");
        left_hand = hardwareMap.get(Servo.class, "left_hand");
        //WHOEVER IS SETTING UP THE ARM, LOOK HERE.
        //First, plug in the RC to this computer with the data transfer cord. (white)  Then plug the RC into the robot, plug in the battery, and flip the wired switch to the I position.
        //Next get the controllers.  Hit start & a to designate a controller as Controller1- it controls the movement- and then start & a on the other controller to designate Controller2
        //which controls the arm movement.  Hit the three dots at the top right of one of the phones, then hit configure robot.  Find the port (DO NOT REPLACE THE DC MOTORS oneDrive thru threeDrive!)
        //and name the motors armBaseDrive for the motor that moves the arm, and armRPDrive for the motor that moves the rack and pinion.

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        zeroDrive.setDirection(DcMotor.Direction.REVERSE);
        twoDrive.setDirection(DcMotor.Direction.REVERSE);
        oneDrive.setDirection(DcMotor.Direction.REVERSE);
        threeDrive.setDirection(DcMotor.Direction.REVERSE);
        complacentLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        complacentRightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftPull.setDirection(DcMotor.Direction.FORWARD);
        rightPull.setDirection(DcMotor.Direction.FORWARD);

        //no servos

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)


        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double zeroPower;
            double twoPower;
            double onePower;
            double threePower;
            double leftPullPower;
            double rightPullPower;
            double complacentLeftPower;
            double complacentRightPower;

            if(gamepad2.a == true) {
                right_hand.setPosition(1);
                left_hand.setPosition(0.1);
            }
            else{

                idle();
            }

            if(gamepad2.b == true){
                    right_hand.setPosition(0.1);
                    left_hand.setPosition(1);
            }

            else{
                idle();
            }


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            double Xdrive = gamepad1.right_stick_x;
            double Ydrive = -gamepad1.right_stick_y;
            double turn  =  gamepad1.left_stick_x;
            double rightWheel = 0;
            double leftWheel = 0;
            double pullLeft = 0;
            double pullRight = 0;

            if(gamepad2.left_bumper == true){
                leftWheel = 0.25;
                rightWheel = -0.25;

            }

            if(gamepad2.right_bumper == true){
                rightWheel = 0.5;
                leftWheel = -0.5;
            }

            if(gamepad2.dpad_up == true){
                pullLeft = -1.0;
                pullRight = 1.0;
            }
            if(gamepad2.dpad_down == true){
                pullLeft = 1.0;
                pullRight = -1.0;
            }


            zeroPower    = Range.clip(Xdrive - Ydrive - turn, -1.0, 1.0) ;
            twoPower   = Range.clip(-Xdrive - Ydrive - turn, -1.0, 1.0) ;
            onePower    = Range.clip(Xdrive + Ydrive - turn, -1.0, 1.0) ;
            threePower   = Range.clip(-Xdrive + Ydrive - turn, -1.0, 1.0) ;
            leftPullPower = Range.clip(pullLeft + 0, -1.0, 1.0);
            rightPullPower = Range.clip(pullRight + 0, -1.0, 1.0);
            //min and max for speed
            complacentRightPower = Range.clip( rightWheel + 0, -1, 1) ;
            complacentLeftPower = Range.clip( leftWheel + 0, -1, 1) ;

            // Send calculated power to wheels
            zeroDrive.setPower(zeroPower);
            twoDrive.setPower(twoPower);
            oneDrive.setPower(onePower);
            threeDrive.setPower(threePower);
            leftPull.setPower(leftPullPower);
            rightPull.setPower(rightPullPower);
            complacentRightDrive.setPower(complacentRightPower);
            complacentLeftDrive.setPower(complacentLeftPower);




            //complacentLeftPower = 0;
            //complacentRightPower = 0;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "zero (%.2f), two (%.2f), one (%.2f), three (%.2f)", zeroPower, twoPower, onePower, threePower);
            telemetry.update();
        }
    }
}
