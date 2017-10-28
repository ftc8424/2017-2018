package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.COLORTEST;

/**
 * Created by FTC8424 on 1/14/2017.
 *
 * Autonomous blue, in the front (nearest audience)
 *
 */
@Autonomous(name = "Auto Blue Front", group = "BlueSide")
public class Auto_Blue_Front extends LinearOpMode {
    HardwareHelper robot = new HardwareHelper(COLORTEST);
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap);

        int blueValue = 0;
        int redValue = 0;
        double colorTimer = 0;
        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double driveSpeed = .5;
        robot.color.enableLed(true);
        sleep(1000L);
        robot.color.enableLed(false);
        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
            while(!isStopRequested() && robot.gyro.isCalibrating()){
            telemetry.addData("Init:", "Calibrating");
            telemetry.update();
        }
        telemetry.addData("Init:", "Calibrated!!");
        telemetry.update();
        waitForStart();

        robot.deploy(robot.colorArm);

        sleep(1500);
        if ( !opModeIsActive() ) return;
        robot.color.enableLed(true);
        double turnInch = 2;
        colorTimer = runtime.milliseconds();
        do {
            sleep(10);
            blueValue = robot.color.blue();
            redValue = robot.color.red();
        telemetry.addData("color blue", blueValue);
            telemetry.addData("color red", redValue);
            telemetry.update();
        }while ( opModeIsActive() && runtime.milliseconds() < colorTimer+10000  && (Math.abs(blueValue-redValue) == 0));
        robot.color.enableLed(false);
        if ( blueValue > redValue ) {
            telemetry.addData("Color", "blue");
            telemetry.update();
            robot.encoderDrive(this, 0.75, turnInch, -turnInch ,2);
            robot.deploy(robot.colorArm);
            sleep(100);
            robot.encoderDrive(this, 0.75, -turnInch, turnInch, 2);
        } else if ( blueValue < redValue ) {
            telemetry.addData("Color", "red");
            telemetry.update();
            robot.encoderDrive(this, 0.75, -turnInch, turnInch, 2);
            robot.deploy(robot.colorArm);
            sleep(100);
            robot.encoderDrive(this, 0.75, turnInch, -turnInch, 2);
        } else {
            telemetry.addData("Color", "cant detect color");
            telemetry.update();
            robot.deploy(robot.colorArm);
            sleep(100);
        }


        if ( !opModeIsActive() ) return;

        robot.gyroTurn(this, robot.TURN_SPEED, 90);
        if ( !opModeIsActive() ) return;
        robot.gyroDrive(this, robot.DRIVE_SPEED, 34, 90);
        if ( !opModeIsActive() ) return;
        robot.gyroTurn(this, robot.TURN_SPEED, 180);


        // TODO Complete Blue Auto with deliver glyph

    }
}
