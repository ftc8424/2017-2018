package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.AUTOTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.COLORTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLTELEOP;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.TROLLBOT;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.TROLLBOTMANIP;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.TROLLBOT_SERVOTEST;

/**
 * Created by FTC8424 on 10/13/2016.
 *
 * This is the initial helper class for the hardware components of E-Cubed (FTC8424) robot.
 * It is NOT an OpMode or any of the others, it's a helper class that has the hardware
 * map and some of the helper methods that are needed for all OpModes.
 *
 * This class assumes that the following hardware components have been given the following
 * names in the configuration file.  If they have not, then bad things will happen.
 *
 */

public class HardwareHelper {

    /* Public OpMode members, things they can use */
    public DcMotor     leftMidDrive = null;   private static final String cfgLMidDrive   = "L Mid";
    public DcMotor     rightMidDrive = null;  private static final String cfgRMidDrive   = "R Mid";
    public DcMotor     leftBackDrive = null;  private static final String cfgLBckDrive   = "L Back";
    public DcMotor     rightBackDrive = null; private static final String cfgRtBckDrive  = "R Back";
    public DcMotor       rightManip = null; private static final String  cfgrightManip = "R Manip";
    public DcMotor       leftManip = null; private static final String  cfgleftManip = "L Manip";
    public Servo        colorArm = null; private static final String cfgcolorArm = "C Arm";
    public DcMotor     lift = null; private static final String  cfgLift = "Lift";
    public ColorSensor color = null; private static final String cfgrpColorSensor = "Color Sensor";
    public ModernRoboticsI2cGyro gyro = null; private static final String cfgGyro = "Gyro";
    public Servo       servotest = null; private static final String cfgServoTest = "servo";

    /* Servo positions, adjust as necessary. */
    public static final double cArmStart = 0;
    public static final double cArmDeploy = 0.525;



    /* Use this when creating the constructor, to state the type of robot we're using. */
    public enum RobotType {
        FULLTELEOP, FULLAUTO, LAUNCHTEST, COLORTEST, AUTOTEST, TROLLBOT,TROLLBOTMANIP,
        TROLLBOT_SERVOTEST
    }

    /*
     * Private instance variables
     */

    /* Wheel ratio values for the encoders (see end of this file for calculations). */
    private static final int   COUNTS_PER_SECOND_MAX = 2800;  // AndyMark NeveRest 40:1/20:1
    private static final double COUNTS_PER_MOTOR_REV = 1120;  // AndyMark NeveRest 40:1 CPR
    private static final double DRIVE_GEAR_REDUCTION = 1.0;   // No gears, just motor shafts
    private static final double WHEEL_DIAMETER_INCHES= 4.0;   // 4" Omni wheels and 4" Stealth

    private static final int    COUNTS_PER_LAUNCHER  = 3600; // AndyMark NeveRest 3.7:1, ideal

    private static final double encoderInch = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    /* Other privates for things such as the runtime, the hardware Map, etc. */
    private RobotType robotType;
    private HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();



    /**
     * Constructor for HardwareHelper, pass in the enumerated type RobotType based on the type of
     * OpMode we are running (e.g., a Trollbot; TeleOp for the full robot with all sensors, servos,
     * motors, etc.; Testing autonomous with only partial motor configurations, etc.  This is then
     * used throughout the rest of the HardwareHelper methods to make them dynamically figure out
     * what to do (e.g., if encoderDrive() is called and we have four motors based on the
     * enumerated type, then send power to all motors, otherwise just those that are on the
     * minimum robot).
     *
     * @param type    The enumerated type of the robot
     */
    public HardwareHelper(RobotType type) {
        robotType = type;

    }

    /**
     * This is the initialization routine for every OpMode in the system.  It uses the RobotType
     * as passed in the contrustor to determine which elements of the physical robot to go
     * and get from the hardware map.  It also expects to be sent the hardware map that is used
     * by the specific OpMode (not sure if there is only a single HardwareMap per the FTC
     * Robot Controller, or if each OpMode has its own, so just pass in whatever your OpMode
     * has and robot_init() will take care of it).
     *
     * This method goes and instantiates every element in the hardware map that is appropriate
     * for the type of robot we are and then sets the initial configuration options for them.
     *
     * @param hwMap    The hardware map entry from the OpMode.
     */
    public void robot_init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        initMotor();
        initServo();
        initSensor();
    }

    /**
     * This is the method used for 180 degree servos.  It gets their current position and sets
     * it to their other position.
     * @param servo
     */
    public void deploy(Servo servo) {
        double targetPosition = 0;
        if (servo.equals(colorArm)) targetPosition = servo.getPosition()== cArmStart ? cArmDeploy : cArmStart;
        else return;
        servo.setPosition(targetPosition);
    }

    /**
     * This method is used to initialize all motors and set them with directions and other
     * parameters as necessary basd on the robot type.
     */
    private void initMotor() {
         /* Set the drive motors in the map */
        if ( robotType == TROLLBOT || robotType == FULLTELEOP || robotType == FULLAUTO ||
                robotType == AUTOTEST || robotType == TROLLBOTMANIP || robotType == COLORTEST ) {
            leftBackDrive = hwMap.dcMotor.get(cfgLBckDrive);
            rightBackDrive = hwMap.dcMotor.get(cfgRtBckDrive);

            //rpCenter = hwMap.dcMotor.get(cfgrpCenter);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            if ( robotType == FULLAUTO || robotType == FULLTELEOP || robotType == TROLLBOT || robotType == TROLLBOTMANIP) {
                lift = hwMap.dcMotor.get(cfgLift);
               if (robotType == FULLTELEOP) {
                   leftManip = hwMap.dcMotor.get(cfgleftManip);
                   rightManip = hwMap.dcMotor.get(cfgrightManip);
                   rightManip.setDirection(DcMotor.Direction.REVERSE);
               } else {
                   leftMidDrive = hwMap.dcMotor.get(cfgLMidDrive);
                   rightMidDrive = hwMap.dcMotor.get(cfgRMidDrive);
                   rightMidDrive.setDirection(DcMotor.Direction.REVERSE);

               }

            }

            /*
             * If autonomous, then reset encoders and set mode to be with encoders
             * NOTE:  This should really throw an exception or something, but all it does
             * is silently ignore if the resetting of the encoders didn't work and blindly
             * sets the mode to be RUN_USING_ENCODER.  Can't really call telemetry because
             * don't have a caller to know which should build the telemetry and send.  Badness
             * will ensue when the actual autonomous runs and hopefully this note will help
             * folks figure out the failed reset might be at fault.
             */

            boolean resetOk = false;
            if ( robotType == AUTOTEST || robotType == FULLAUTO || robotType == COLORTEST ) {
                resetOk = waitForReset(leftBackDrive, rightBackDrive, 2000);
                if ( robotType == FULLAUTO )
                    resetOk = resetOk && waitForReset(leftMidDrive, rightMidDrive, 2000);
                if (resetOk) {
                    leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if ( robotType == FULLAUTO ) {
                        leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
            }
        }

         /* Now that hardware is mapped, set to initial positions/settings. */
        if ( robotType != TROLLBOT_SERVOTEST ) {
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            if ( robotType == FULLTELEOP || robotType == FULLAUTO || robotType == TROLLBOT ) {
                leftMidDrive.setPower(0);
                rightMidDrive.setPower(0);

            }
        }
    }

    /**
     * This method is used to initialize servos and set their positions.
     */
    private void initServo() {
        if ( robotType == TROLLBOT_SERVOTEST ) {
            servotest = hwMap.servo.get(cfgServoTest);
        }

        if ( robotType == COLORTEST || robotType == FULLTELEOP ) {
            colorArm = hwMap.servo.get(cfgcolorArm);
            colorArm.setPosition(cArmStart);
        }
    }

    /**
     * This method is used to initialize the sensors based on the type of sensor and robot.
     */
    private void initSensor() {

        /* Set the sensors based on type */
        if ( robotType == AUTOTEST || robotType == COLORTEST || robotType == FULLAUTO || robotType == FULLTELEOP ) {
            color = hwMap.colorSensor.get(cfgrpColorSensor);
        }

        /* Set the sensors based on type */


        /* Get the Gyro */
        if ( robotType == AUTOTEST || robotType == FULLAUTO || robotType == COLORTEST ) {
           gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get(cfgGyro);

        }

    }

    /**
     * TODO THIS ROUTINE NEEDS TO BE REPLACED
     *
     * The code shuld come from the PushbotAutoDriveByGyro.java file as well as the helper
     * methods such as onHeading, gyroHold, etc., but it needs to take into account the robotType
     * so that it only sends power to the appropriate motors.
     *
     * @param caller
     * @param heading
     * @param timeoutS
     * @return
     * @throws InterruptedException
     */
    public boolean gyroTurn(LinearOpMode caller,
                            int heading,
                            double timeoutS) throws InterruptedException {
        int zValue;
        int gHeading;
        int heading360;
        int absHeading;
        int deltaHeading;
        double rightPower;
        double leftPower;
        double turnspeed = 0.175;
        double stopTime = runtime.seconds() + timeoutS;

        do {
            gHeading = gyro.getHeading();
            //zValue = gyro.getIntegratedZValue();
            //heading360 = zValue % 360;
//            if ( heading360 > 0 )
//                absHeading = heading360;
//            else
//                absHeading = heading360 + 360;
            //deltaHeading = absHeading - heading;
            //caller.telemetry.addData("gyroTurn:", "delta: %d absHeading: %d, currently at %d going to %d", deltaHeading, absHeading, zValue, heading);
            caller.telemetry.addData("gyroTurn:", "gHeading: %d, going to %d", gHeading, heading);
            caller.telemetry.update();
            //caller.sleep(1000);
//            if (Math.abs(deltaHeading) <= 180 && heading360 > 180) {
//                leftPower = -turnspeed;
//                rightPower = turnspeed;
//            } else {
//                leftPower = turnspeed;
//                rightPower = -turnspeed;
//            }
            /*
             * Turn left if the difference from where we're heading to where we want to head
             * is smaller than -180 or is between 1 and 180.  All else (including the 0 and 180
             * situations) turn right.
             */
            deltaHeading = gHeading - heading;
            if ( deltaHeading < -180 || (deltaHeading > 0 && deltaHeading < 180) ) {
                leftPower = -turnspeed;
                rightPower = turnspeed;
            } else {
                leftPower = turnspeed;
                rightPower = -turnspeed;
            }
            if (robotType == FULLTELEOP || robotType == TROLLBOT || robotType == TROLLBOTMANIP) {
                leftMidDrive.setPower(leftPower);
                rightMidDrive.setPower(rightPower);
            }
            leftBackDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);
            gHeading = gyro.getHeading();
        }
//        while (caller.opModeIsActive() && Math.abs(deltaHeading) > 1 && runtime.seconds() < stopTime );
        while (caller.opModeIsActive() && Math.abs(gHeading - heading) > 1 && runtime.seconds() < stopTime );
        if (robotType == FULLTELEOP || robotType == TROLLBOT || robotType == TROLLBOTMANIP) {
            leftMidDrive.setPower(0.0);
            rightMidDrive.setPower(0.0);
        }
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
//        if ( deltaHeading <= 1 )
        if ( Math.abs(gHeading - heading) <= 1 )
            return true;
        else
            return false;
//        if(heading360  < 0){
//              absHeading = heading360 + 360;
//        } else {
//             absHeading = heading360;
//        }

//        int deltaHeading = absHeading - heading;
//
//        if (deltaHeading <=180){
//            do {
//                leftMidDrive.setPower(-turnspeed);
//                rightMidDrive.setPower(turnspeed);
//                leftBackDrive.setPower(-turnspeed);
//                rightBackDrive.setPower(turnspeed);
//                zValue = gyro.getIntegratedZValue();
//                caller.telemetry.addData("gyroTurn:", "Turning to %d, currently at %d", absHeading, zValue);
//                caller.telemetry.update();
//            }
//            while (caller.opModeIsActive() && Math.abs(zValue - absHeading) > 1 && runtime.seconds() < stopTime);
//        } else {
//            do {
//                leftMidDrive.setPower(turnspeed);
//                rightMidDrive.setPower(-turnspeed);
//                leftBackDrive.setPower(turnspeed);
//                rightBackDrive.setPower(-turnspeed);
//                zValue = gyro.getIntegratedZValue();
//                caller.telemetry.addData("gyroTurn:", "Turning to %d, currently at %d", absHeading, zValue);
//                caller.telemetry.update();
//            }
//            while (caller.opModeIsActive() && Math.abs(zValue - absHeading) > 1 && runtime.seconds() < stopTime);
//        }
//        leftMidDrive.setPower(0.0);
//        rightMidDrive.setPower(0.0);
//        leftBackDrive.setPower(0.0);
//        rightBackDrive.setPower(0.0);
//        return Math.abs(gyro.getIntegratedZValue() - absHeading) <= 1;
        }
/*This code we wrote establishes different things 1.  It makes sure that during gyro that if certain
statements are true than the code will stop working, 2. I don't know what else.
 */

    /**
     * Drive by the encoders, running to a position relative to the current position based
     * on encoder ticks for a left and right motor.  It will move to a position for a specified
     * period of time, so it will stop if they get to the desired position, if the time runs out
     * or if the OpMode is cancelled.
     *
     * Originally written in PushbotAutoDriveByEncoder_Linear.java from FtcRobotController area,
     * modified by FTC8424 for defensive encoder moves.
     *
     * @param caller                  Reference to calling class, must be LinearOpMode
     * @param speed                   The speed of the movement
     * @param leftInches              The target position of left motors, in inches from current
     * @param rightInches           The target position of right motors, in inches from current
     *
    * @param timeoutS                The timeout in seconds to allow the move
     * @throws InterruptedException
     */
    public void encoderDrive(LinearOpMode caller,
                             double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {

        int newLeftMidTarget;
        int newRightMidTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        //int getHeading = gyro.getIntegratedZValue();
        long encoderTimeout = 2000;   // Wait no more than two seconds, an eternity, to set

        if ( !caller.opModeIsActive() )
            return;

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if ( robotType == FULLAUTO ) {
            leftMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



        /*
         * Determine new target position and pass to motor controller
         */
        if ( robotType == FULLAUTO ) {
            newLeftMidTarget = leftMidDrive.getCurrentPosition() + (int) Math.round(leftInches * encoderInch);
            newRightMidTarget = rightMidDrive.getCurrentPosition() + (int) Math.round(rightInches * encoderInch);
        } else {
            newLeftMidTarget = 0;
            newRightMidTarget = 0;
        }
        newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)Math.round(leftInches * encoderInch);
        newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)Math.round(rightInches * encoderInch);
//        caller.telemetry.addLine("encoderDrive-MID:")
//                .addData("Left Tgt POS: ", newLeftMidTarget)
//                .addData("Right Tgt POS:" ,  newRightMidTarget);
//        caller.telemetry.addLine("EncoderDrive-BCK:")
//                .addData("Left Tgt POS: ", newLeftBackTarget)
//                .addData("Right Tgt POS: ", newRightBackTarget);
//        caller.telemetry.update();

        boolean lmEncoderSet = false;
        boolean rmEncoderSet = false;
        boolean lbEncoderSet = false;
        boolean rbEncoderSet = false;

        lbEncoderSet = setEncoderPosition(caller, leftBackDrive, newLeftBackTarget, encoderTimeout);
        rbEncoderSet = setEncoderPosition(caller, rightBackDrive, newRightBackTarget, encoderTimeout);
        if ( robotType == FULLAUTO ) {
            lmEncoderSet = setEncoderPosition(caller, leftMidDrive, newLeftMidTarget, encoderTimeout);
            rmEncoderSet = setEncoderPosition(caller, rightMidDrive, newRightMidTarget, encoderTimeout);
        } else {
            lmEncoderSet = true;
            rmEncoderSet = true;
        }
//        caller.telemetry.addLine("EncoderSet:")
//                .addData("LB: ", lbEncoderSet)
//                .addData("RB: ", rbEncoderSet)
//                .addData("LM: ", lmEncoderSet)
//                .addData("RM: ", rmEncoderSet);
//        caller.telemetry.update();
        if ( ! (lmEncoderSet && lbEncoderSet && rmEncoderSet && rbEncoderSet) ) {
            caller.telemetry.addLine("Encoders CANNOT be set, aborting OpMode");
            caller.telemetry.update();
            caller.sleep(10000);    // Can't go any further, allow telemetry to show, then return
            return;
        }

        // reset the timeout time and start motion.

//        caller.telemetry.addLine("Encoder Drive: ")
//                .addData("PowerSet: ", "%.4f", Math.abs(speed));
//        caller.telemetry.update();

        // keep looping while we are still active, and there is time left, and motors haven't made position.
        boolean isBusy;
        int lmCurPos;
        int rmCurPos;
        int lbCurPos;
        int rbCurPos;
        double stopTime = runtime.seconds() + timeoutS;
        double leftPower;
        double rightPower;
        double lastSetTime = runtime.milliseconds();
        int HeadingLoop;

        do {
            leftPower = speed;
            rightPower = speed;
            if (leftPower <= 0.01) {
                lastSetTime = runtime.milliseconds();
                leftPower = speed;
                rightPower = speed;
            }

            leftPower = Range.clip(leftPower, -1.0, 1.0);
            rightPower = Range.clip(rightPower, -1.0, 1.0);
            leftBackDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);

            if(robotType == FULLAUTO){
                leftMidDrive.setPower(leftPower);
                rightMidDrive.setPower(rightPower);
            }
            caller.telemetry.addData("Power:", "Left Power %.2f, Right Power %.2f", leftPower, rightPower);
            caller.telemetry.update();
            lbCurPos = leftBackDrive.getCurrentPosition();
            rbCurPos = rightBackDrive.getCurrentPosition();
            if ( robotType == FULLAUTO ) {
                lmCurPos = leftMidDrive.getCurrentPosition();
                rmCurPos = rightMidDrive.getCurrentPosition();
            } else {
                lmCurPos = Integer.MAX_VALUE;
                rmCurPos = Integer.MAX_VALUE;
            }
            isBusy = (Math.abs(lbCurPos - newLeftBackTarget) >= 5) && (Math.abs(rbCurPos - newRightBackTarget) >= 5);
            if ( robotType == FULLAUTO )
                isBusy = isBusy && (Math.abs(lmCurPos - newLeftMidTarget) >= 5) && (Math.abs(rmCurPos - newRightMidTarget) >= 5);
        }
        while (caller.opModeIsActive() && isBusy && runtime.seconds() < stopTime);

        // Stop all motion;
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        if ( robotType == FULLAUTO ) {
            leftMidDrive.setPower(0);
            rightMidDrive.setPower(0);
        }

        // Turn off RUN_TO_POSITION

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if ( robotType == FULLAUTO ) {
            leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Drive the robot forward/backward based on power settings passed in.
     *
     * @param leftPower      Power setting (-1.0 - 1.0)
     * @param rightPower     Power setting (-1.0 - 1.0)
     */

    public void normalDrive (OpMode caller, double leftPower, double rightPower) {
        leftBackDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        if ( robotType == TROLLBOT ) {
            leftMidDrive.setPower(leftPower);
            rightMidDrive.setPower(rightPower);
        }
        caller.telemetry.addData("normalDrive:", "Power set to L:%.2f, R:%.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
    }

    /**
     * Drive the robot side to side based on power settings passed in.
     *
     * @param leftPower      Power setting (-1.0 - 1.0)
     * @param rightPower     Power setting (-1.0 - 1.0)
     */

    public void sideDrive (OpMode caller, double leftPower, double rightPower) {
        leftBackDrive.setPower(-leftPower);
        rightBackDrive.setPower(rightPower);
        if ( robotType == FULLTELEOP || robotType == TROLLBOT ) {
                leftMidDrive.setPower(leftPower);
                rightMidDrive.setPower(-rightPower);
        }
        caller.telemetry.addData("sideDrive:", "Power set to L:%.2f, R:%.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
    }

    /**
     * Get the voltage of the 12V battery and return it.
     *
     * @return  The voltage of the 12V
     */
    public double getVoltage() {
        return hwMap.voltageSensor.iterator().next().getVoltage();
    }

    /**
     * Waits for the encoders to be reset on the 4 motors, and returns a boolean as to whether
     * the motors have reset or not.  Calls waitForReset() with two motor method signature multiple
     * times to do the actual work and combines the return values to give an overall return of
     * true only if all four motors were properly reset.
     *
     * @param m1    Motor 1 to reset
     * @param m2    Motor 2 to reset
     * @param m3    Motor 3 to reset
     * @param m4    Motor 4 to reset
     * @param msTimeOut The time to wait, in milliseonds, for a valid reset
     * @return      Whether the reset was successful or not
     */
    public boolean waitForReset(DcMotor m1, DcMotor m2, DcMotor m3, DcMotor m4, long msTimeOut) {
        boolean resetOk = false;

        resetOk = waitForReset(m1, m2, msTimeOut/2);
        return resetOk && waitForReset(m3, m4, msTimeOut/2);
    }

    /**
     * Waits for the encoders to be reset on the 2 motors and returns a boolean as to whether
     * the motors have reset or not.  If they haven't reset in the timeOut milliseconds, then
     * it will return false.  This is the true method that does stuff, the other method with the
     * 4 motor signature just calls this method multiple times.
     *
     * @param m1        Motor 1 to reset
     * @param m2        Motor 2 to reset
     * @param msTimeOut   The time to wait, in milliseconds, for a valid reset
     * @return          Whether the reset was successful or not
     */
    public boolean waitForReset(DcMotor m1, DcMotor m2, long msTimeOut) {
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int m1Pos = m1.getCurrentPosition();
        int m2Pos = m2.getCurrentPosition();
        double stopTime = runtime.milliseconds() + msTimeOut;
        while ( (m1Pos != 0 || m2Pos != 0) && runtime.milliseconds() < stopTime ) {
            m1Pos = m1.getCurrentPosition();
            m2Pos = m2.getCurrentPosition();
        }
        return m1Pos == 0 && m2Pos == 0;
    }

    /****************************************************************************************
     * Private methods follow.
     ****************************************************************************************/

    /**
     * Sets the target encoder position for this particular motor and waits to ensure that the
     * position was properly set.  If it didn't set correctly, within the timeOut (in milliseconds)
     * time, then it will returns false.
     *
     * @param caller    The calling OpMode (for opModeIsActive() call)
     * @param m1        The motor on which to set the position
     * @param target    The target position to set
     * @param timeOut   The time, in milliseconds, for it to properly set
     * @return          True if target set within timeOut milliseconds, false otherwise
     */
    private boolean setEncoderPosition (LinearOpMode caller, DcMotor m1, int target, long timeOut) {
        m1.setTargetPosition(target);
        int m1Pos = m1.getTargetPosition();
        double stopTime = runtime.milliseconds() + timeOut;
        while ( caller.opModeIsActive() && m1Pos != target && runtime.milliseconds() < stopTime ) {
            m1.setTargetPosition(target);
            m1Pos = m1.getTargetPosition();
        }
        return m1Pos == target;
    }
}

/************************************************************************************************
 * For encoder math, here is the information from AndyMark's web site, so it will be key in
 * setting up the setMaxSpeed() when in PID mode, as well as when figuring out the counts per
 * inch mode.
 *
 *    NeveRest 40:1 Motors:
 *    ---------------------
 *          7 pulses per revolution of hall effect encoder, and a 40:1 gearbox, so 7*40 ==
 *        280 pulses per revolution of the encoder, there are 4 revolutions of encoder to output
 *       1120 pulses per revolution of the OUTPUT SHAFT (e.g., the motor shaft)
 *        150 revolutions per minute of output shaft (RPM), so (1120 * 150) / 60 ==
 *       2800 pulses per second is the max Speed setting of the encoders on this motor
 *
 *    NeveRest 20:1 Motors:
 *    ----------------------
 *          7 pulses per revolution of hall effect encoder, and a 20:1 gearbox, so 7*20 ==
 *        140 pulses per revolution of the encoder, there are 4 revolutions of encoder to output
 *        560 pulses per revolution of the OUTPUT SHAFT (e.g., the motor shaft)
 *        300 revolutions per minute of output shaft (RPM), so (560 * 300) / 60 ==
 *       2800 pulses per second is the max Speed setting of the encoders on this motor
 *
 * So, for these two motors, the encoder COUNTS_PER_MOTOR_REV above would be 1,120 for the 40:1
 * and 560 for the 20:1, and the COUNTS_PER_SECOND_MAX above would be 2800 for both.
 *
 *************************************************************************************************/
