package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name = "Functions New", group = "Main")

public class FunctionsNew extends LinearOpMode{

    //----------------------------------------------------------------------------------------------------------------------------------------------------------

    private ElapsedTime runtime = new ElapsedTime();

    //                              ENCODERS


        public static final double COUNTS_PER_MOTOR_REV = 1680; // For Neverest 60 Motors
        public static final double COUNTS_MOTOR_REG = 1440;     // For Tetrix Motors
        public static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);                   // Conversion: Encoder Count to Inches
        public static final double COUNTS_PER_TETRIX_INCH = (COUNTS_MOTOR_REG * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);               // Conversion: Encoder Counts Motor Tetrix to Inches

        public static final int BLUE_VALUE = 1;

        public static final double PULL_SERVO_END_VAL = 1;
        public static final double PULL_SERVO_END_INCREMENT = 0.02;



    //Enumerations
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
            all, glyph, jewel, relic, drive, teleop;
        }
        public enum team{
        red1, red2, blue1, blue2;
    }

    //Motors/Sensors

        //Drivetrain
        public DcMotor motorFR;
        public DcMotor motorFL;
        public DcMotor motorBR;
        public DcMotor motorBL;

        public static final String motorFRS = "motorFR_red3"; // updated //Configured
        public static final String motorFLS = "motorFL_green2"; // updated //Configured
        public static final String motorBRS = "motorBR_red2";// updated //Configured
        public static final String motorBLS = "motorBL_green1"; // updated //Configured

        //Jewel Systems
        public Servo jewelDown;
        public Servo jewelFlick;
        public ColorSensor jewelSensor;

        public static final String jewelDownS = "jewelDown_red4"; // updated //Configured
        public static final String jewelFlickS = "jewelFlick_red5"; // updated //Configured
        public static final String jewelSensorS = "colorSensor_red1"; // updated //Configured

        //Glyph System
        public Servo pullServo;
        public DcMotor rightTread;
        public DcMotor leftTread;
        public TouchSensor glyphButton;

        public static final String pullServoS = "pullServo_green0"; // updated //Configured
        public static final String rightTreadS = "rightTread_red1"; // updated //Configured
        public static final String leftTreadS = "leftTread_green3"; // updated //Configured
        public static final String glyphButtonS = "glyphButton_red2";

        //Relic Systems

        public DcMotor relicMotor;
        public Servo angleServo;
        public Servo rightClaw;
        public Servo leftClaw;

        public static final String relicMotorS = "relicMotor_green0"; // updated //Configured
        public static final String angleServoS = "angleServo_red4";
        public static final String rightClawS = "rightClaw_red3";
        public static final String leftClawS = "leftClaw_red2";

    //                      Initializations

        //  Jewel Servos

            //  Starting Positions
                public static final double MIN_POSITION_DOWN = 0;
                public static final double MIN_POSITION_FLICK = 0;

            //  End Positions
                public static final double MAX_POSITION_DOWN = 1;
                public static final double MAX_POSITION_FLICK = 1;

            // Initial Positions
                public static final double JEWEL_DOWN_INITIAL_POSITION = 0.65;
                public static final double JEWEL_FLICK_INITIAL_POSITION = 0.55;

            // Important Positions
                public static final double JEWEL_DOWN_LOW_POSITION = 0.1;
                public static final double JEWEL_FLICK_HIT_POSITION_LEFT = 0;
                public static final double JEWEL_FLICK_HIT_POSITION_RIGHT = 1;

            // Increments
                public static final double JEWEL_DOWN_INCREMENT = 0.01;




    //Relic Servos
            //  Starting Positions
                public static final double MIN_POSITION_ANGLE = 0;
                public static final double MIN_POSITION_RIGHT_CLAW = 0;
                public static final double MIN_POSITION_LEFT_CLAW = 0;

            //  End Positions
                public static final double MAX_POSITION_ANGLE = 1;
                public static final double MAX_POSITION_RIGHT_CLAW = 1;
                public static final double MAX_POSITION_LEFT_CLAW = 1;

            // Initial Positions
                public static final double ANGLE_SERVO_INITIAL_POSITION = 0;
                public static final double RIGHT_CLAW_INITIAL_POSITION = 0;
                public static final double LEFT_CLAW_INITIAL_POSITION = 0;


        //  Glyph Servos

            //  Starting Positions
                public static final double MIN_POSITION_PULL_SERVO = 0;

            // End Position
                public static final double MAX_POSITION_PULL_SERVO = 1;

            //  Initial Positions
                public static final double PULL_SERVO_INITIAL_POSITION = 0;

    // Arrays
    public movements[] allMovements = {movements.forward, movements.backward, movements.right, movements.left, movements.tr, movements.tl, movements.br, movements.bl, movements.cw, movements.ccw};
    public DcMotor[] drivetrain = new DcMotor[4];


    //----------------------------------------------------------------------------------------------------------------------------------------------------------

    public FunctionsNew(){};

    public void runOpMode() throws InterruptedException{

    }

    public void Setup(setupType setup) throws InterruptedException{
        /*
        Sample for motor with encoder on drivetrain:
        motorFL = motor(motorFL, hardwareMap, motorFLS, DcMotor.Direction.FORWARD);

        motorEncoderMode(motorFL);

        drivetrain[0] = motorFL;*/

        switch (setup){
            case all:
                //DRIVETRAIN INITIALIZATION
                motorFR = motor(motorFR, hardwareMap, motorFRS, DcMotorSimple.Direction.FORWARD);
                motorFL = motor(motorFL, hardwareMap, motorFLS, DcMotorSimple.Direction.FORWARD);
                motorBR = motor(motorBR, hardwareMap, motorBRS, DcMotorSimple.Direction.FORWARD);
                motorBL = motor(motorBL, hardwareMap, motorBLS, DcMotorSimple.Direction.FORWARD);

                //GLYPH SETUP
                rightTread = motor(rightTread, hardwareMap, rightTreadS, DcMotorSimple.Direction.FORWARD);
                leftTread = motor(leftTread, hardwareMap, leftTreadS, DcMotorSimple.Direction.FORWARD);
                pullServo = servo(pullServo, hardwareMap, pullServoS,
                        Servo.Direction.FORWARD, MIN_POSITION_PULL_SERVO, MAX_POSITION_PULL_SERVO, PULL_SERVO_INITIAL_POSITION);

                //JEWEL SETUP
                jewelDown = servo(jewelDown, hardwareMap, jewelDownS
                        , Servo.Direction.FORWARD, MIN_POSITION_DOWN, MAX_POSITION_DOWN, JEWEL_DOWN_INITIAL_POSITION);
                jewelFlick = servo(jewelFlick, hardwareMap, jewelFlickS,
                        Servo.Direction.FORWARD, MIN_POSITION_FLICK, MAX_POSITION_FLICK, JEWEL_FLICK_INITIAL_POSITION);

                //RELIC SETUP

                /*
                relicMotor = motor(relicMotor, hardwareMap, relicMotorS, DcMotorSimple.Direction.FORWARD);


                angleServo = servo(angleServo, hardwareMap, angleServoS, Servo.Direction.FORWARD, MIN_POSITION_ANGLE, MAX_POSITION_ANGLE, ANGLE_SERVO_INITIAL_POSITION);
                rightClaw = servo(rightClaw, hardwareMap, rightClawS, Servo.Direction.FORWARD, MIN_POSITION_RIGHT_CLAW, MAX_POSITION_RIGHT_CLAW, RIGHT_CLAW_INITIAL_POSITION);
                leftClaw = servo(leftClaw, hardwareMap, leftClawS, Servo.Direction.FORWARD, MIN_POSITION_LEFT_CLAW, MAX_POSITION_LEFT_CLAW, LEFT_CLAW_INITIAL_POSITION);
                */

                //ENCODER SETUP
                motorEncoderMode(motorFR, motorFL, motorBR, motorBL/*, relicMotor*/);

                //DRIVETRAIN SETUP
                drivetrain[0] = motorFR;
                drivetrain[1] = motorFL;
                drivetrain[2] = motorBR;
                drivetrain[3] = motorBL;
                break;
            case drive:
                //DRIVETRAIN INITIALIZATION
                motorFR = motor(motorFR, hardwareMap, motorFRS, DcMotorSimple.Direction.FORWARD);
                motorFL = motor(motorFL, hardwareMap, motorFLS, DcMotorSimple.Direction.FORWARD);
                motorBR = motor(motorBR, hardwareMap, motorBRS, DcMotorSimple.Direction.FORWARD);
                motorBL = motor(motorBL, hardwareMap, motorBLS, DcMotorSimple.Direction.FORWARD);

                //ENCODER MODE
                motorEncoderMode(motorFR, motorFL, motorBR, motorBL);

                //DRIVETRAIN SETUP
                drivetrain[0] = motorFR;
                drivetrain[1] = motorFL;
                drivetrain[2] = motorBR;
                drivetrain[3] = motorBL;
                break;
            case glyph:
                //GLYPH SETUP
                rightTread = motor(rightTread, hardwareMap, rightTreadS, DcMotorSimple.Direction.FORWARD);
                leftTread = motor(leftTread, hardwareMap, leftTreadS, DcMotorSimple.Direction.FORWARD);
                pullServo = servo(pullServo, hardwareMap, pullServoS,
                        Servo.Direction.FORWARD, MIN_POSITION_ANGLE, MAX_POSITION_ANGLE, PULL_SERVO_INITIAL_POSITION);

                break;
            case jewel:
                //JEWEL SETUP
                jewelDown = servo(jewelDown, hardwareMap, jewelDownS,
                        Servo.Direction.FORWARD, MIN_POSITION_DOWN, MAX_POSITION_DOWN, JEWEL_DOWN_INITIAL_POSITION);
                jewelFlick = servo(jewelFlick, hardwareMap, jewelFlickS,
                        Servo.Direction.FORWARD, MIN_POSITION_FLICK, MAX_POSITION_FLICK, JEWEL_FLICK_INITIAL_POSITION);
                break;
            case relic:
                //RELIC SETUP

                /*
                relicMotor = motor(relicMotor, hardwareMap, relicMotorS, DcMotorSimple.Direction.FORWARD);


                angleServo = servo(angleServo, hardwareMap, angleServoS, Servo.Direction.FORWARD, MIN_POSITION_ANGLE, MAX_POSITION_ANGLE, ANGLE_SERVO_INITIAL_POSITION);
                rightClaw = servo(rightClaw, hardwareMap, rightClawS, Servo.Direction.FORWARD, MIN_POSITION_RIGHT_CLAW, MAX_POSITION_RIGHT_CLAW, RIGHT_CLAW_INITIAL_POSITION);
                leftClaw = servo(leftClaw, hardwareMap, leftClawS, Servo.Direction.FORWARD, MIN_POSITION_LEFT_CLAW, MAX_POSITION_LEFT_CLAW, LEFT_CLAW_INITIAL_POSITION);
                */
                break;
            case teleop:
                //DRIVETRAIN INITIALIZATION
                motorFR = motor(motorFR, hardwareMap, motorFRS, DcMotorSimple.Direction.FORWARD);
                motorFL = motor(motorFL, hardwareMap, motorFLS, DcMotorSimple.Direction.FORWARD);
                motorBR = motor(motorBR, hardwareMap, motorBRS, DcMotorSimple.Direction.FORWARD);
                motorBL = motor(motorBL, hardwareMap, motorBLS, DcMotorSimple.Direction.FORWARD);

                //GLYPH SETUP
                rightTread = motor(rightTread, hardwareMap, rightTreadS, DcMotorSimple.Direction.FORWARD);
                leftTread = motor(leftTread, hardwareMap, leftTreadS, DcMotorSimple.Direction.FORWARD);
                pullServo = servo(pullServo, hardwareMap, pullServoS,
                        Servo.Direction.FORWARD, MIN_POSITION_ANGLE, MAX_POSITION_ANGLE, PULL_SERVO_INITIAL_POSITION);

                //JEWEL SETUP
                jewelDown = servo(jewelDown, hardwareMap, jewelDownS
                        , Servo.Direction.FORWARD, MIN_POSITION_DOWN, MAX_POSITION_DOWN, JEWEL_DOWN_INITIAL_POSITION);
                jewelFlick = servo(jewelFlick, hardwareMap, jewelFlickS,
                        Servo.Direction.FORWARD, MIN_POSITION_FLICK, MAX_POSITION_FLICK, JEWEL_FLICK_INITIAL_POSITION);

                //RELIC SETUP

                /*
                relicMotor = motor(relicMotor, hardwareMap, relicMotorS, DcMotorSimple.Direction.FORWARD);


                angleServo = servo(angleServo, hardwareMap, angleServoS, Servo.Direction.FORWARD, MIN_POSITION_ANGLE, MAX_POSITION_ANGLE, ANGLE_SERVO_INITIAL_POSITION);
                rightClaw = servo(rightClaw, hardwareMap, rightClawS, Servo.Direction.FORWARD, MIN_POSITION_RIGHT_CLAW, MAX_POSITION_RIGHT_CLAW, RIGHT_CLAW_INITIAL_POSITION);
                leftClaw = servo(leftClaw, hardwareMap, leftClawS, Servo.Direction.FORWARD, MIN_POSITION_LEFT_CLAW, MAX_POSITION_LEFT_CLAW, LEFT_CLAW_INITIAL_POSITION);
                */

                //ENCODER SETUP
                motorEncoderMode(motorFR, motorFL, motorBR, motorBL/*, relicMotor*/);

                //DRIVETRAIN SETUP
                drivetrain[0] = motorFR;
                drivetrain[1] = motorFL;
                drivetrain[2] = motorBR;
                drivetrain[3] = motorBL;
                break;
        }

    }

    //------------------ENCODERS MOVEMENTS------------------------------------------------------------------------
    public void Forward(double speed, double distance,  /*In Revolution*/
                        double timeoutS, long waitAfter) throws   InterruptedException {

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
    public void Backward(double speed, double distance,  /*In Revolution*/
                         double timeoutS, long waitAfter) throws   InterruptedException {

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
    public void Right(double speed, double distance,  /*In Revolution*/
                      double timeoutS, long waitAfter) throws   InterruptedException {

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
    public void Left(double speed, double distance,  /*In Revolution*/
                     double timeoutS, long waitAfter) throws   InterruptedException {

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
    public void TR(double speed, double distance,  /*In Revolution*/
                   double timeoutS, long waitAfter) throws   InterruptedException {

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
    public void TL(double speed, double distance,  /*In Revolution*/
                   double timeoutS, long waitAfter) throws   InterruptedException {

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
    public void BR(double speed, double distance,  /*In Revolution*/
                   double timeoutS, long waitAfter) throws   InterruptedException {

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
    public void BL(double speed, double distance,  /*In Revolution*/
                   double timeoutS, long waitAfter) throws   InterruptedException {

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

    public void CW(double speed, double distance,  /*In Revolution*/
                   double timeoutS, long waitAfter) throws   InterruptedException {

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
    public void CCW(double speed, double distance,  /*In Revolution*/
                    double timeoutS, long waitAfter) throws   InterruptedException {

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

    //------------------TEST FUNCTIONS------------------------------------------------------------------------
    public void MecanumTest(double speed, double distance) throws InterruptedException{
        telemetry.addLine("FORWARD");
        telemetry.update();
        Forward(speed, distance, 8, 2000);
        telemetry.addLine("Backward");
        telemetry.update();
        Backward(speed, distance, 8, 2000);
        telemetry.addLine("right");
        telemetry.update();
        Right(speed, distance, 8, 2000);
        telemetry.addLine("left");
        telemetry.update();
        Left(speed, distance, 8, 2000);
        telemetry.addLine("top right");
        telemetry.update();
        TR(speed, distance, 8, 2000);
        telemetry.addLine("top left");
        telemetry.update();
        TL(speed, distance, 8, 2000);
        telemetry.addLine("back right");
        telemetry.update();
        BR(speed, distance, 8, 2000);
        telemetry.addLine("back left");
        telemetry.update();
        BL(speed, distance, 8, 2000);
        telemetry.addLine("cw");
        telemetry.update();
        CW(speed, distance, 8, 2000);
        telemetry.addLine("ccw");
        telemetry.update();
        CCW(speed, distance, 8, 2000);
        telemetry.addLine("Done.");
        telemetry.update();

    }
    //------------------JEWEL FUNCTIONS------------------------------------------------------------------------
    public void flick(team side) throws InterruptedException{
        for (double p = jewelDown.getPosition(); jewelDown.getPosition() > JEWEL_DOWN_LOW_POSITION; p-= JEWEL_DOWN_INCREMENT){
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

                if (jewelSensor.blue() >= BLUE_VALUE) { //FLICK REG
                    jewelFlick.setPosition(JEWEL_FLICK_HIT_POSITION_LEFT);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();

                }
                else {                               //FLICK OPPOSITE
                    jewelFlick.setPosition(JEWEL_FLICK_HIT_POSITION_RIGHT);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();
                }

                break;
            case blue1:
            case blue2:
                if (jewelSensor.blue() >= BLUE_VALUE) { //FLICK OPPOSITE
                    jewelFlick.setPosition(JEWEL_FLICK_HIT_POSITION_RIGHT);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();

                }
                else {                              //FLICK REGULAR
                    jewelFlick.setPosition(JEWEL_FLICK_HIT_POSITION_LEFT);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();
                }
                break;
        }
        sleep(1000);
        jewelFlick.setPosition(JEWEL_FLICK_INITIAL_POSITION);
        jewelDown.setPosition(JEWEL_DOWN_INITIAL_POSITION);
        sleep(50);
    }

    //------------------RELIC FUNCTIONS------------------------------------------------------------------------
    //none right now

    //------------------GLYPH FUNCTIONS------------------------------------------------------------------------
    //none right now

    //------------------SET FUNCTIONS------------------------------------------------------------------------
    public void setRuntime(ElapsedTime time) throws InterruptedException{
        runtime = time;
    }
    //DRIVETRAIN SET

    //------------------SERVO FUNCTIONS------------------------------------------------------------------------
    //none right now

    //------------------HARDWARE SETUP FUNCTIONS------------------------------------------------------------------------
    public DcMotor motor(DcMotor motor, HardwareMap hardwareMap,
                         String name, DcMotor.Direction direction) throws InterruptedException{
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        return motor;
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
    public Servo servo(Servo servo, HardwareMap hardwareMap,
                       String name, Servo.Direction direction, double min, double max, double start) throws InterruptedException{
        servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }
    public ColorSensor colorSensor(ColorSensor sensor, HardwareMap hardwareMap,
                                   String name, boolean ledOn) throws InterruptedException{
        sensor = hardwareMap.colorSensor.get(name);
        sensor.enableLed(ledOn);

        telemetry.addData("Beacon Red Value: ", sensor.red());
        telemetry.update();

        return sensor;
    }

    //------------------DRIVETRAIN FUNCTIONS------------------------------------------------------------------------
    public void driveTrainMovement(double speed, FunctionsNew.movements movement) throws InterruptedException{
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


    //------------------ALTERED ENCODER FUNCTIONS------------------------------------------------------------------------
    public void driveTrainEncoderMovement(double speed, double distance,
                                          double timeoutS, long waitAfter, FunctionsNew.movements movement) throws  InterruptedException{

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
    public void encoderMovement(double speed, double distance,
                                double timeoutS, long waitAfter, FunctionsNew.movements movement, DcMotor... motors) throws  InterruptedException{

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
