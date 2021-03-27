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

@Autonomous(name="Red_Place_Park", group="Pushbot")
//@Disabled
public class Red_Place_Park extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.75;
    static final double TURN_SPEED = 0.4;

    private DcMotor zeroDrive = null;
    private DcMotor twoDrive = null;
    private DcMotor oneDrive = null;
    private DcMotor threeDrive = null;
    private DcMotor complacentLeftDrive = null;
    private DcMotor complacentRightDrive = null;
    private DcMotor leftPull = null;
    private DcMotor rightPull = null;

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
        leftPull = hardwareMap.get(DcMotor.class, "leftPull");
        rightPull = hardwareMap.get(DcMotor.class,"rightPull");

        zeroDrive.setDirection(DcMotor.Direction.FORWARD);
        twoDrive.setDirection(DcMotor.Direction.FORWARD);
        oneDrive.setDirection(DcMotor.Direction.FORWARD);
        threeDrive.setDirection(DcMotor.Direction.FORWARD);
        complacentLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        complacentRightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftPull.setDirection(DcMotor.Direction.FORWARD);
        rightPull.setDirection(DcMotor.Direction.FORWARD);

        //Move LEFT
        robot.zeroDrive.setPower(-(FORWARD_SPEED-0.1));
        robot.twoDrive.setPower(FORWARD_SPEED-0.1);
        robot.oneDrive.setPower(-(FORWARD_SPEED-0.1));
        robot.threeDrive.setPower(FORWARD_SPEED-0.1);
        complacentLeftDrive.setPower(0);
        complacentRightDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.7)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Go FORWARD w/ SPINNING WHEELS

        robot.zeroDrive.setPower(FORWARD_SPEED/3);
        robot.twoDrive.setPower(FORWARD_SPEED/3);
        robot.oneDrive.setPower(-FORWARD_SPEED/3);
        robot.threeDrive.setPower(-FORWARD_SPEED/3);
        complacentLeftDrive.setPower(0.75);
        complacentRightDrive.setPower(0.75);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Right TURN
        complacentLeftDrive.setPower(0);
        complacentRightDrive.setPower(0);
        robot.zeroDrive.setPower(-FORWARD_SPEED);
        robot.twoDrive.setPower(-FORWARD_SPEED);
        robot.oneDrive.setPower(-FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Move FORWARD
        complacentLeftDrive.setPower(0);
        complacentRightDrive.setPower(0);
        robot.zeroDrive.setPower(FORWARD_SPEED);
        robot.twoDrive.setPower(FORWARD_SPEED);
        robot.oneDrive.setPower(-FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }



        // RIGHT TURN
        complacentLeftDrive.setPower(0);
        complacentRightDrive.setPower(0);
        robot.zeroDrive.setPower(-FORWARD_SPEED);
        robot.twoDrive.setPower(-FORWARD_SPEED);
        robot.oneDrive.setPower(-FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Forward
        robot.zeroDrive.setPower(FORWARD_SPEED);
        robot.twoDrive.setPower(FORWARD_SPEED);
        robot.oneDrive.setPower(-FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.4)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Right TURN
        robot.zeroDrive.setPower(-FORWARD_SPEED);
        robot.twoDrive.setPower(-FORWARD_SPEED);
        robot.oneDrive.setPower(-FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Forward
        robot.zeroDrive.setPower(FORWARD_SPEED);
        robot.twoDrive.setPower(FORWARD_SPEED);
        robot.oneDrive.setPower(-FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }



        //Lift UP - not far?
        leftPull.setPower(-0.75);
        rightPull.setPower(0.75);
        complacentLeftDrive.setPower(0);
        complacentRightDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Run wheels

        complacentLeftDrive.setPower(-0.25);
        complacentRightDrive.setPower(-0.25);
        leftPull.setPower(0);
        rightPull.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5 * .25)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //STOP- except compliant

        robot.zeroDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.threeDrive.setPower(0);

        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }

        // Move BACKWARDS
        robot.zeroDrive.setPower(-FORWARD_SPEED);
        robot.twoDrive.setPower(-FORWARD_SPEED);
        robot.oneDrive.setPower(FORWARD_SPEED);
        robot.threeDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Lift Down
        leftPull.setPower(0.75);
        rightPull.setPower(-0.75);
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Move Right
        robot.zeroDrive.setPower(FORWARD_SPEED);
        robot.twoDrive.setPower(-FORWARD_SPEED);
        robot.oneDrive.setPower(FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //STOP

        complacentLeftDrive.setPower(0);
        complacentRightDrive.setPower(0);
        robot.zeroDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.threeDrive.setPower(0);

        while (opModeIsActive() && (runtime.seconds() < 1.3 * .25)) {
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }

       /* // Step 1:  RIGHT TURN
        robot.zeroDrive.setPower(FORWARD_SPEED);
        robot.twoDrive.setPower(FORWARD_SPEED);
        robot.oneDrive.setPower(FORWARD_SPEED);
        robot.threeDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 2: LEFT TURN
        robot.zeroDrive.setPower(-FORWARD_SPEED);
        robot.twoDrive.setPower(-FORWARD_SPEED);
        robot.oneDrive.setPower(-FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 3: Move FORWARD
        robot.zeroDrive.setPower(FORWARD_SPEED);
        robot.twoDrive.setPower(FORWARD_SPEED);
        robot.oneDrive.setPower(-FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 4: Move BACKWARDS
        robot.zeroDrive.setPower(-FORWARD_SPEED);
        robot.twoDrive.setPower(-FORWARD_SPEED);
        robot.oneDrive.setPower(FORWARD_SPEED);
        robot.threeDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 5: Move LEFT
        robot.zeroDrive.setPower(-FORWARD_SPEED);
        robot.twoDrive.setPower(FORWARD_SPEED);
        robot.oneDrive.setPower(-FORWARD_SPEED);
        robot.threeDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 6: Move RIGHT
        robot.zeroDrive.setPower(FORWARD_SPEED);
        robot.twoDrive.setPower(-FORWARD_SPEED);
        robot.oneDrive.setPower(FORWARD_SPEED);
        robot.threeDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.zeroDrive.setPower(0);
        robot.twoDrive.setPower(0);
        robot.oneDrive.setPower(0);
        robot.threeDrive.setPower(0);
       // robot.leftClaw.setPosition(1.0);
       // robot.rightClaw.setPosition(0.0);
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
        telemetry.addData("Path", "Complete");
        telemetry.update();

        }
*/

    }
}