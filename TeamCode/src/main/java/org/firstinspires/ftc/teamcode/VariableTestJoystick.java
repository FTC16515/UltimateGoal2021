package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="VariableTestJoystick", group="Pushbot")
//@Disabled

public class VariableTestJoystick extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.3;
    static final double TURN_SPEED = 0.4;

    private DcMotor zeroDrive = null;
    private DcMotor twoDrive = null;
    private DcMotor oneDrive = null;
    private DcMotor threeDrive = null;
    private DcMotor complacentLeftDrive = null;
    private DcMotor complacentRightDrive = null;
    private Servo right_hand = null;
    private Servo left_hand = null;

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

        zeroDrive = hardwareMap.get(DcMotor.class, "zeroDrive");
        twoDrive = hardwareMap.get(DcMotor.class, "twoDrive");
        oneDrive = hardwareMap.get(DcMotor.class, "oneDrive");
        threeDrive = hardwareMap.get(DcMotor.class, "threeDrive");
        complacentLeftDrive = hardwareMap.get(DcMotor.class, "complacentLeftDrive");
        complacentRightDrive = hardwareMap.get(DcMotor.class, "complacentRightDrive");
        right_hand = hardwareMap.get(Servo.class, "right_hand");
        left_hand = hardwareMap.get(Servo.class, "left_hand");


        zeroDrive.setDirection(DcMotor.Direction.REVERSE);
        twoDrive.setDirection(DcMotor.Direction.REVERSE);
        oneDrive.setDirection(DcMotor.Direction.REVERSE);
        threeDrive.setDirection(DcMotor.Direction.REVERSE);
        complacentLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        complacentRightDrive.setDirection(DcMotor.Direction.FORWARD);


        // Step 1:  RIGHT TURN
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
    }
}
