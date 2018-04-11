package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 2/22/2018.
 */
@Autonomous(name = "Auto Red Front MULTIGLPYH", group = "RedSide")

public class Auto_Red_Front_MULTIGLYPH extends LinearOpMode{    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap);
        int whichColumn = 0;
        int blueValue = 0;
        int redValue = 0;

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double driveSpeed = robot.DRIVE_SPEED;
        double heading = 0;
        robot.color.enableLed(true);
        sleep(1000L);
        robot.color.enableLed(false);
        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        robot.gyroCalibrate();//Start of initialization of Vuforia Code
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // TODO - Production comment previous two lines and uncomment next, don't need view
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AaknBtv/////AAAAmfjrebUDlEXxppBL0lNYYmF+OrNdDEb+iCZTJVBlijmEOVgaagv19vhNOnlYuDK9CDBvcAzED7y/GhmxAzxn9GKPz6PMCbATksji1FgAfR7zLxWRJKGUxLSQoFMHfAem/xSKUwuTNOohEXW74qhy+KD6VuCwmZFamYthtv/ChxI4o2lwNI4aJDmmpK4jf2I5gr1ULsWML3+OauayyNp74Xa7MLV0mEY2m3sWjnFiZyoygU06ht4+PY2Z/S9/bJvxSgqVfzzFAHzepMJoKAvH+iuDhdHctpnIyXSBtVwcYdoc4GX1+0VjZUb3LzyNSje3NzvUZ8s/n7crdkujseblOuRh8HdqJYAwGCRQrW9mVt+F\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        // The ACTIVATE needs to go AFTER you press the Start button, not before


        while (!isStopRequested() && (robot.gyroIsCalibrating())) {
            telemetry.addData("Init:", "Calibrating");
            telemetry.update();
        }

        while (!isStarted()) {
            //robot.gyro.gyroResetZAxisIntegrator();
            heading = robot.getHeading();
            telemetry.addData("Init:", "Calibrated!!");
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        }

        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = null;
        double startTime = runtime.milliseconds();

        do {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        } while ( opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN && runtime.milliseconds() < startTime+2000 );
        telemetry.addData("Vumark", "%s is visible", vuMark);
        telemetry.update();

        robot.deploy(robot.colorGate);
        sleep(500);
        robot.deploy(robot.colorArm);

        sleep(1500);
        if ( !opModeIsActive() ) return;
        robot.color.enableLed(true);
        double turnInch = 2;
        double colorTimer = runtime.milliseconds();

        int times = 0;

        do {
            sleep(10);

            if ( times > 0 && times < 6)
            {
                robot.colorArmAdjust();
                if( times % 3 == 0);

            }
            times++;{
                robot.encoderDrive(this, 0.25, 0.5, -0.5 ,1);

            }

            blueValue = robot.color.blue();
            redValue = robot.color.red();
            telemetry.addData("color blue", blueValue);
            telemetry.addData("color red", redValue);
            telemetry.update();

        } while ( opModeIsActive() && runtime.milliseconds() < colorTimer+10000  && (Math.abs(blueValue-redValue) == 0));
        robot.color.enableLed(false);



        if ( blueValue > redValue ) {
            telemetry.addData("Color", "blue");
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            robot.encoderDrive(this, 0.75, turnInch, -turnInch ,2);
            robot.deploy(robot.colorArm);
            sleep(500);
            robot.deploy(robot.colorGate);
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        }
        else if ( blueValue < redValue ) {
            telemetry.addData("Gyro:", heading);
            telemetry.addData("Color", "red");
            telemetry.update();
            robot.encoderDrive(this, 0.75, -turnInch, turnInch, 2);
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            robot.deploy(robot.colorArm);
            sleep(500);
            robot.deploy(robot.colorGate);
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        }
        else {
            telemetry.addData("Color", "cant detect color");
            telemetry.update();
            robot.deploy(robot.colorArm);
            sleep(500);
            robot.deploy(robot.colorGate);
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        }

        int driveForwardFirst = 27;
// Turn on stone
        int driveForwardSecond = 6;
        int firstTurn = 0;
        if ( !opModeIsActive() ) return;
         //robot.encoderDrive(this, driveSpeed, -1, -1, 10);
        robot.gyroTurn(this, 268, 2); //Turns to 268
        firstTurn++;

        int driveSecond = 4;
        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();

        if ( !opModeIsActive() ) return;
        if(vuMark == RelicRecoveryVuMark.RIGHT) {
            driveForwardFirst = -18;
            //driveForwardSecond = 20;
        }
        else if(vuMark == RelicRecoveryVuMark.LEFT) {
            driveForwardFirst = -32;
            //driveForwardSecond = 20;
            }
        else{
            driveForwardFirst = -25;
            //driveForwardSecond = 20;
        }
        if ( !opModeIsActive() ) return;
        telemetry.update();
        robot.encoderDrive(this, driveSpeed, driveForwardFirst, driveForwardFirst, 10);
        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
// Off Stone
        robot.gyroTurn(this, 0, 5);
        robot.leftManip.setPower(-1);
        robot.rightManip.setPower(-1);
        robot.encoderDrive(this, driveSpeed, 22, 22, 3);
        robot.encoderDrive(this, driveSpeed, -4, -4, 3);
        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() + 691);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.6);
        robot.gyroTurn(this, 180, 6);
        robot.leftManip.setPower(-0.1);
        robot.rightManip.setPower(-0.1);
        robot.encoderDrive(this, driveSpeed, 25, 25, 5);
        robot.lift.setTargetPosition(robot.lift.getCurrentPosition() - 691);
        double manipTimer = runtime.milliseconds();
        do {
            robot.leftManip.setPower(0.75);
            robot.rightManip.setPower(0.75);
        } while ( opModeIsActive() && runtime.milliseconds() < manipTimer+1000 );
        robot.encoderDrive(this, driveSpeed, -4, -4, 5);
        robot.leftManip.setPower(0);
        robot.rightManip.setPower(0);
    }
}
