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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue_B_Target_2_Shoot", group="Pushbot")
//@Disabled
public class Blue_B_Target_2_Shoot extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.75;
    static final double SHOOT_SPEED = 1.0;
    static final double TURN_SPEED = 0.4;

    private DcMotor zeroDrive = null;
    private DcMotor twoDrive = null;
    private DcMotor oneDrive = null;
    private DcMotor threeDrive = null;
    private DcMotor complacentLeftDrive = null;
    private DcMotor complacentRightDrive = null;
    private Servo right_hand = null;
    private Servo left_hand = null;

    double circ = 2 * 3.14 * 1.85;
    //720 = 1440 ticks/2
    double ticksPerInch = circ/720;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way


        //    private DcMotor complacentLeftDrive = null;
        //    private DcMotor complacentRightDrive = null;

        // step (using the FTC Robot Controller app on the phone).
        zeroDrive = hardwareMap.get(DcMotor.class, "zeroDrive");
        twoDrive = hardwareMap.get(DcMotor.class, "twoDrive");
        oneDrive = hardwareMap.get(DcMotor.class, "oneDrive");
        threeDrive = hardwareMap.get(DcMotor.class, "threeDrive");
        complacentLeftDrive = hardwareMap.get(DcMotor.class, "complacentLeftDrive");
        complacentRightDrive = hardwareMap.get(DcMotor.class, "complacentRightDrive");


        zeroDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        oneDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        twoDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //set one and three to reverse
        zeroDrive.setDirection(DcMotor.Direction.FORWARD);
        twoDrive.setDirection(DcMotor.Direction.FORWARD);
        oneDrive.setDirection(DcMotor.Direction.REVERSE);
        threeDrive.setDirection(DcMotor.Direction.REVERSE);


        //resets to 0
        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Forwards
        forwardsMove(42);

        robot.zeroDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.oneDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.twoDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.threeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(zeroDrive.isBusy() || oneDrive.isBusy()) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.zeroDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.threeDrive.setPower(0);

        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        //Forwards
        forwardsMove(15);

        robot.zeroDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.oneDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.twoDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.threeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(zeroDrive.isBusy() || oneDrive.isBusy()) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.zeroDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.threeDrive.setPower(0);

        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        leftTurn();
        robot.zeroDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.oneDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.twoDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.threeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(zeroDrive.isBusy() || oneDrive.isBusy()) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.zeroDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.threeDrive.setPower(0);

        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        forwardsMove(10);
        robot.zeroDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.oneDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.twoDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.threeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(zeroDrive.isBusy() || oneDrive.isBusy()) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.zeroDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.threeDrive.setPower(0);

        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        backwards(12);
        robot.zeroDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.oneDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.twoDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.threeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(zeroDrive.isBusy() || oneDrive.isBusy()) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.zeroDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.threeDrive.setPower(0);

        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        leftTurn();
        robot.zeroDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.oneDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.twoDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.threeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(zeroDrive.isBusy() && oneDrive.isBusy()) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.zeroDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.threeDrive.setPower(0);

        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        forwardsMove(10);
        robot.zeroDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.oneDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.twoDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.threeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(zeroDrive.isBusy() && oneDrive.isBusy()) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.zeroDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.threeDrive.setPower(0);

        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        //New code area
        rightTurn_V2();
        while(zeroDrive.isBusy() && oneDrive.isBusy()) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.zeroDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.threeDrive.setPower(0);

        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        forwardsMove(20);
        robot.zeroDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.oneDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.twoDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.threeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(zeroDrive.isBusy() && oneDrive.isBusy()) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.zeroDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.threeDrive.setPower(0);

        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        rightTurn_V2();
        while(zeroDrive.isBusy() && oneDrive.isBusy()) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.zeroDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.threeDrive.setPower(0);

        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        //adjust to the shooting
        backwards(10);
        robot.zeroDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.oneDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.twoDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.threeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(zeroDrive.isBusy() && oneDrive.isBusy()) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.zeroDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.threeDrive.setPower(0);

        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);

        /*Check if works with other mechanisms; find one loop
        Shooting_Mech(5);


        robot.zeroDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.oneDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.twoDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.threeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         */


    }

    public void leftTurn()
    {
        zeroDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        oneDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        twoDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        zeroDrive.setDirection(DcMotor.Direction.FORWARD);
        twoDrive.setDirection(DcMotor.Direction.FORWARD);
        oneDrive.setDirection(DcMotor.Direction.REVERSE);
        threeDrive.setDirection(DcMotor.Direction.REVERSE);

        complacentLeftDrive.setPower(0);
        complacentRightDrive.setPower(0);
        robot.zeroDrive.setPower(FORWARD_SPEED);
        robot.twoDrive.setPower(FORWARD_SPEED);
        robot.oneDrive.setPower(-FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);

        int rotation30 = (int)(8.7/ticksPerInch);

        robot.zeroDrive.setTargetPosition(-rotation30);
        robot.oneDrive.setTargetPosition(rotation30);
        robot.twoDrive.setTargetPosition(-rotation30);
        robot.threeDrive.setTargetPosition(rotation30);


    }

    public void forwardsMove(double inches)
    {
        zeroDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        oneDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        twoDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        zeroDrive.setDirection(DcMotor.Direction.FORWARD);
        twoDrive.setDirection(DcMotor.Direction.FORWARD);
        oneDrive.setDirection(DcMotor.Direction.REVERSE);
        threeDrive.setDirection(DcMotor.Direction.REVERSE);

        complacentLeftDrive.setPower(0);
        complacentRightDrive.setPower(0);
        robot.zeroDrive.setPower(FORWARD_SPEED);
        robot.twoDrive.setPower(FORWARD_SPEED);
        robot.oneDrive.setPower(-FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);

        int movement = (int)(inches/ticksPerInch);

        robot.zeroDrive.setTargetPosition(movement);
        robot.oneDrive.setTargetPosition(movement);
        robot.twoDrive.setTargetPosition(movement);
        robot.threeDrive.setTargetPosition(movement);

    }

    //encoders
    public void rightTurn_V1()
    {
        zeroDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        oneDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        twoDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        zeroDrive.setDirection(DcMotor.Direction.FORWARD);
        twoDrive.setDirection(DcMotor.Direction.FORWARD);
        oneDrive.setDirection(DcMotor.Direction.FORWARD);
        threeDrive.setDirection(DcMotor.Direction.FORWARD);

        int rotation30 = (int)(8.7/ticksPerInch);

        robot.zeroDrive.setTargetPosition(rotation30);
        robot.oneDrive.setTargetPosition(rotation30);
        robot.twoDrive.setTargetPosition(rotation30);
        robot.threeDrive.setTargetPosition(rotation30);

    }


    //timed
    public void rightTurn_V2()
    {
        zeroDrive.setDirection(DcMotor.Direction.FORWARD);
        twoDrive.setDirection(DcMotor.Direction.FORWARD);
        oneDrive.setDirection(DcMotor.Direction.FORWARD);
        threeDrive.setDirection(DcMotor.Direction.FORWARD);

        zeroDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oneDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        twoDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        threeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.zeroDrive.setPower(FORWARD_SPEED);
        robot.twoDrive.setPower(FORWARD_SPEED);
        robot.oneDrive.setPower(FORWARD_SPEED);
        robot.threeDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    }

    public void backwards(double inches)
    {
        zeroDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        oneDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        twoDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        zeroDrive.setDirection(DcMotor.Direction.REVERSE);
        twoDrive.setDirection(DcMotor.Direction.REVERSE);
        oneDrive.setDirection(DcMotor.Direction.FORWARD);
        threeDrive.setDirection(DcMotor.Direction.FORWARD);

        robot.zeroDrive.setPower(-FORWARD_SPEED);
        robot.twoDrive.setPower(-FORWARD_SPEED);
        robot.oneDrive.setPower(FORWARD_SPEED);
        robot.threeDrive.setPower(FORWARD_SPEED);

        int movement = (int)(inches/ticksPerInch);

        robot.zeroDrive.setTargetPosition(movement);
        robot.oneDrive.setTargetPosition(movement);
        robot.twoDrive.setTargetPosition(movement);
        robot.threeDrive.setTargetPosition(movement);


    }

    public void Shooting_Mech(double seconds)
    {
        complacentLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        complacentRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        complacentLeftDrive.setPower(SHOOT_SPEED);
        complacentRightDrive.setPower(SHOOT_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        complacentLeftDrive.setPower(0);
        complacentRightDrive.setPower(0);

    }
}