/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLTELEOP;

*/
/**
 * Created by aagrockstar on 11/9/2017.
 *//*



*/
/*
For this program, we are going to program a crab drive robot. It will be able to move left right, forward, backward, and also
diagonal, all 4 ways. Then, we are going to program a color sensor program that senses the red and blue value. The Color Sensor
is a REV sensor.
 *//*

public class Krab_Drive{
    HardwareHelper KrabDrive = new HardwareHelper(HardwareHelper.RobotType.FULLTELEOP);

    */
/* Declare OpMode members. *//*

    private ElapsedTime runtime = new ElapsedTime();
    private HardwareHelper robot = new HardwareHelper(FULLTELEOP);

    public Gamepad gamepad1 = null;
    public Telemetry telemetry = new TelemetryImpl (this);
    public Gamepad gamepad2 = null;   // will be set in OpModeManager.runActiveOpMode
    public HardwareMap hardwareMap = null;

    public void init() {
        robot.robot_init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

     telemetry.addData("Status", "Running: " + runtime.toString());

    double rightStickVal = -gamepad1.right_stick_y;
    double leftStickVal = -gamepad1.left_stick_y;



        robot.normalDrive(this, leftStickVal, rightStickVal);

    double rightManipVal = gamepad2.right_stick_y;
    double leftManipVal = gamepad2.left_stick_y;

}
*/

