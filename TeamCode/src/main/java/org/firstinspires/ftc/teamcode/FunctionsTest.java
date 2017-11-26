package org.firstinspires.ftc.teamcode;

import android.widget.BaseExpandableListAdapter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Scanner;
import java.util.StringTokenizer;

/**
 * Created by arulgupta on 9/26/17.
 */
@Autonomous(name="Functions test", group="Main Blue")

public class FunctionsTest extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    //CONSTANTS FOR THE MOTOR ENCODER CONVERSIONS For robot

    public static final double COUNTS_PER_MOTOR_REV = 1680;    // eg: NeverRest  Encoder
    public static final double COUNTS_MOTOR_REG = 1440;
    public static final double COUNTS_PER_FLICKER = 2 * COUNTS_PER_MOTOR_REV;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);                   // Conversion: Encoder Count to Inches
    public static final double COUNTS_PER_INCHTETRIX = (COUNTS_MOTOR_REG * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //                      SENSOR CALIBRATIONS
    public static final int blueValue = 1;
    movements[] allMovements = {movements.forward, movements.backward, movements.right, movements.left, movements.tr, movements.tl, movements.br, movements.bl, movements.ccw, movements.cw};

    //                             MOTOR/SERVO DECLARATIONS

    //Drivetrains:
    public static DcMotor motorFR;
    public static DcMotor motorFL;
    public static DcMotor motorBR;
    public static DcMotor motorBL;

    public static String motorFRS = "motorFR_green1";
    public static String motorFLS = "motorFL_red2";
    public static String motorBRS = "motorBR_green0";
    public static String motorBLS = "motorBL_red1";


    //Jewel Systems:
    public static Servo jewelDown;
    public static Servo jewelFlick;
    public static ColorSensor jewelSensor;

    public static String jewelDownS = "jewelDown_green5";
    public static String jewelFlickS = "jewelFlick_red5";
    public static String jewelSensorS = "colorSensor_red1";
    //Relic Systems:

    // Empty rn

    //Glyph System:

    public static DcMotor pivot;
    public static DcMotor rightTread;
    public static DcMotor leftTread;

    public static String pivotS = "pivotMotor_green2";
    public static String rightTreadS = "rightTread_green3";
    public static String leftTreadS = "leftTread_red3";

    public double rightArmPosition = 0.4;
    public double leftArmPosition = 0.4;

//---------------SERVO SETUP--------------------------------

    //starting positions
    private double minPositionDown = 0; //CHECK AND CHOOSE POSITION
    private double minPositionFlick = 0; //CHECK AND CHOOSE POSITION

    //end positions
    private double maxPositionDown = 1; //CHECK AND CHOOSE POSITION
    private double maxPositionFlick = 1; //CHECK AND CHOOSE POSITION

    private double jewelDownPosition = 0.75;
    private double jewelFlickPosition = 0.5;


    //BALANCE BOARD SYSTEMS:


    DcMotor[] drivetrain = {motorFR, motorFL, motorBR, motorBL};
    DcMotorSimple.Direction[] driveDirections = {DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD};
    String[] driveConfigStrings = {motorFRS, motorFLS, motorBRS, motorBLS};

    Servo[] jewelServos = {jewelDown, jewelFlick};
    Servo.Direction[] jewelDirections = {Servo.Direction.FORWARD, Servo.Direction.FORWARD};
    String[] jewelConfigStrings = {jewelDownS, jewelFlickS};
    double[] jewelMinPositions = {minPositionDown, minPositionFlick};
    double [] jewelMaxPositions = {maxPositionDown, maxPositionFlick};
    double[] jewelStartPositions = {jewelDownPosition, jewelFlickPosition};

    DcMotor[] glyphMotors = {pivot, leftTread, rightTread};
    String[] glyphConfigStrings = {pivotS, leftTreadS, rightTreadS};
    DcMotorSimple.Direction[] glyphDirections = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD};


    public enum movements{

        forward(1, 1, 1, 1),
        backward(-1, -1, -1, -1),
        right(-1, -1, 1, 1),
        left(1, 1, -1, -1),
        tr(0.3, 1, 1, 0.3),
        tl(1, 0.3, 0.3, 1),
        br(-1, -0.3, -0.3, -1),
        bl(-0.3, -1, -1 -0.3),
        cw(-1, 1, -1, 1),
        ccw(1, -1, 1, -1),
        glyphUp(),
        glyphDown(),
        treadUp(),
        treadDown();

        private final double[] directions;

        movements(double... signs){
            this.directions = signs;
        }

        private double[] getSigns(){
            return directions;
        }
    }
    public enum treadPivotSettings{
        lift, drop, center;
    }
    public enum setupType{
        all, glyph, jewel, relic, drive, teleop, pivot;
    }
    public enum team{
        red1, red2, blue1, blue2;
    }


    //------------------------------------------------------------------------------------------------------------------------

    public FunctionsTest(){

    }
    public void runOpMode() throws InterruptedException{
        Setup(setupType.all);

    }

    public void Setup(setupType setup) throws InterruptedException{
        switch (setup){
            case all:
                //DRIVETRAIN SETUP
                driveTrainSetup();

                //GLYPH SETUP
                glyphMotorSetup();

                //JEWEL SETUP
                jewelServoSetup();
                jewelSensor = colorSensor(jewelSensor, hardwareMap, jewelSensorS, true);

                //ENCODER-BASED MOTORS
                motorEncoderMode(motorFR, motorFL, motorBR, motorBL, pivot);
                break;
            case relic:



            case jewel:
                jewelServoSetup();

                jewelSensor = colorSensor(jewelSensor, hardwareMap, jewelSensorS, true);
                sleep(500);
            case drive:
                driveTrainSetup();

                motorEncoderMode(motorFR, motorFL, motorBR, motorBL);
            case teleop:
                //DRIVETRAIN SETUP
                driveTrainSetup();

                //GLYPH SETUP
                glyphMotorSetup();

                //RELIC SETUP

                //JEWEL SETUP
                jewelServoSetup();

                jewelSensor = colorSensor(jewelSensor, hardwareMap, jewelSensorS, true);

                //ENCODER-BASED MOTORS
                motorEncoderMode(motorFR, motorFL, motorBR, motorBL, pivot);
                break;
            case glyph:
                glyphMotorSetup();

                motorEncoderMode(pivot, rightTread, leftTread);

        }
        telemetry.addLine("SETUP COMPLETE");
        telemetry.addLine("READY!");
        telemetry.update();


    }

    //ENCODER MOVEMENTS
    public void Forward(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void Backward(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void Right(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void Left(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }

    //CAN BE ALTERED FOR 45˚ MOVEMENT
    public void TR(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * 0.2);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed) * 0.2);
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed) * 0.2);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void TL(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * 0.2);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed) * 0.2);
            motorBR.setPower(Math.abs(speed) * 0.2);
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void BR(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed) * 0.2);
            motorBR.setPower(Math.abs(speed) * 0.2);
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void BL(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed) * 0.2);
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed) * 0.2);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }

    public void CW(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void CCW(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }

    public void pivot(double speed, double distance, double timeoutS, long waitAfter){

    }

    //GLYPH FUNCTIONS


    //JEWEL FUNCTIONS
    public void flick(team side) throws InterruptedException{
        for (double p = jewelDown.getPosition(); jewelDown.getPosition() > 0.1; p-=0.01){
            jewelDown.setPosition(p);
            sleep(50);
        }
        jewelSensor.enableLed(true);
        sleep(1500);
        telemetry.addData("Blue Value: ", jewelSensor.blue());
        telemetry.update();
        switch (side){
            case red1:
            case red2:

                if (jewelSensor.blue() >= blueValue) { //FLICK REG
                    jewelFlick.setPosition(0);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();

                }
                else {                               //FLICK OPPOSITE
                    jewelFlick.setPosition(1);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();
                }

                break;
            case blue1:
            case blue2:
                if (jewelSensor.blue() >= blueValue) { //FLICK OPPOSITE
                    jewelFlick.setPosition(1);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();

                }
                else {                              //FLICK REGULAR
                    jewelFlick.setPosition(0);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();
                }
                break;
        }
        sleep(1000);
        jewelFlick.setPosition(0.6);
        jewelDown.setPosition(0.6);
        sleep(50);

    }

    //RELIC FUNCTIONS

    //FUNCTIONS TO BE USED IN CHILD CLASSES FOR SETUP
    public void setRuntime(ElapsedTime time) throws InterruptedException {
        runtime = time;
    }

    //TEST FUNCTIONS
    public void MecanumTest()throws InterruptedException {
        Forward(0.2, 10, 8, 2000);
        Backward(0.2, 10, 8, 2000);
        Right(0.2, 10, 8, 2000);
        Left(0.2, 10, 8, 2000);
        TR(0.2, 10, 8, 2000);
        TL(0.2, 10, 8, 2000);
        BR(0.2, 10, 8, 2000);
        BL(0.2, 10, 8, 2000);
        CW(0.2, 10, 7, 2000);
        CCW(0.2, 10, 7, 2000);
    }
    public void newEncodersStyleTest(double speed, double distance, double timeOutS, long waitAfter) throws InterruptedException{
        for (movements movement: allMovements){
            driveTrainEncoderMovement(speed, distance, timeOutS, waitAfter, movement);
        }
    }
    public void JewelTest() throws InterruptedException{
        flick(team.blue1);
        sleep(3000);
        flick(team.red1);
    }
    public void RelicTest() throws InterruptedException{

    }

    //POSTIONING
    public void setServoPosition(double degrees, Servo servo, double maxDegrees) throws InterruptedException{
        servo.setPosition(degrees/maxDegrees);
    }
    /*public void setPivot(treadPivotSettings position) throws InterruptedException{
        switch (position){
            case lift:
                pivot.setPosition(0);
                break;
            case center:
                pivot.setPosition(0.5);
                break;
            case drop:
                pivot.setPosition(1);
                break;
        }
    }*/

    //HARDWARE SETUP FUNCTIONS
    public void driveTrainSetup() throws InterruptedException{
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor = motor(motor, hardwareMap, driveConfigStrings[x], driveDirections[x]);
        }
    }
    public void jewelServoSetup() throws InterruptedException{
        for (Servo servo: jewelServos){
            int x = Arrays.asList(jewelServos).indexOf(servo);
            servo = servo(servo, hardwareMap, jewelConfigStrings[x], jewelDirections[x], jewelMinPositions[x], jewelMaxPositions[x], jewelStartPositions[x]);
        }
    }
    public void glyphMotorSetup() throws InterruptedException{
        for (DcMotor motor: glyphMotors) {
            int x = Arrays.asList(glyphMotors).indexOf(motor);
            motor = motor(motor, hardwareMap, glyphConfigStrings[x], glyphDirections[x]);
        }
    }
    public void motorEncoderMode(DcMotor... motor) throws InterruptedException{
        for (DcMotor i: motor){
            i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        idle();
        for (DcMotor i: motor){
            i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public DcMotor motor(DcMotor motor, HardwareMap hardwareMap, String name, DcMotor.Direction direction) throws InterruptedException{
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        return motor;
    }
    public Servo servo(Servo servo, HardwareMap hardwareMap, String name, Servo.Direction direction, double min, double max, double start) throws InterruptedException{
        servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }
    public ColorSensor colorSensor(ColorSensor sensor, HardwareMap hardwareMap, String name, boolean ledOn) throws InterruptedException{
        sensor = hardwareMap.colorSensor.get(name);
        sensor.enableLed(ledOn);

        telemetry.addData("Beacon Red Value: ", sensor.red());
        telemetry.update();

        return sensor;
    }

    public void driveTrainMovement(double speed, movements movement) throws InterruptedException{
        double[] signs = movement.getSigns();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x]* speed);

        }
    }
    public void stopDrivetrain() throws InterruptedException{
        for (DcMotor motor: drivetrain){
            motor.setPower(0);
        }
    }
    //TO BE TESTED: REVOLUTIONARY FUNCTIONS
    public void driveTrainEncoderMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement) throws  InterruptedException{

        int[] targets = new int[drivetrain.length];
        double[] signs = movement.getSigns();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH);
            }
            for (DcMotor motor: drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:drivetrain){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
                for (DcMotor motor: drivetrain){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            for (DcMotor motor: drivetrain){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            sleep(waitAfter);


        }
    }
    public void encoderMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement, DcMotor... motors) throws  InterruptedException{

        int[] targets = new int[motors.length];
        double[] signs = movement.getSigns();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            sleep(waitAfter);


        }
    }


}


