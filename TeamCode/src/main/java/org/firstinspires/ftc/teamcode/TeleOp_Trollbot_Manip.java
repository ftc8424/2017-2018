/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.TROLLBOTMANIP;

/**
 * Created by FTC8424 on 9/15/2016.
 */

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="TrollbotManip", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class TeleOp_Trollbot_Manip extends OpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareHelper robot = new HardwareHelper(TROLLBOTMANIP);

//    private Servo LPush;
//    double LPushStart = 0.1;
//    double LPushEnd = 0.9;
//    double LPushPower = 1.1;

    private double LservoUpTime = 0;
    private double RservoUpTime = 0;
    private double powerSetTime = 0;
    private boolean pastX = false;
    private boolean pastY = false;
    private double lastManipTime = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.robot_init(hardwareMap);
        telemetry.addData("Status", "Initialized");
//        LPush = hardwareMap.servo.get("left_push");
//        LPush.setPosition(LPushStart);
//        wait(2000);
//        LPush.setPosition(LPushEnd);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Status", "Running: " + runtime.toString());

        double rightStickVal = -gamepad1.right_stick_y;
        double leftStickVal = -gamepad1.left_stick_y;
        double rightSquaredVal = rightStickVal * rightStickVal;
        double leftSquaredVal = leftStickVal * leftStickVal;

        double rightStickSide = gamepad1.right_stick_x;
        double leftStickSide = gamepad1.left_stick_x;    // Is this right or left?
        double rightSquaredSide = rightStickSide * rightStickSide;
        double leftSquaredSide = leftStickSide * leftStickSide;

        if (rightStickSide < 0) rightSquaredSide = -rightSquaredSide;
        if (leftStickSide < 0) leftSquaredSide = -leftSquaredSide;

        if (rightStickVal < 0) rightSquaredVal = -rightSquaredVal;
        if (leftStickVal < 0) leftSquaredVal = -leftSquaredVal;
        robot.normalDrive(this, leftSquaredVal, rightSquaredVal);


        if (Math.abs(leftSquaredSide) > 0.1 || Math.abs(rightSquaredSide) > 0.1) {
            robot.sideDrive(this, leftSquaredSide, rightSquaredSide);
        }

        int speed = 0;
        int manipMove;
        if (gamepad2.y && lastManipTime + 1 < runtime.seconds()) {
            if (pastY){
                robot.leftManip.setPosition(0.5);
                robot.rightManip.setPosition(0.5);
                pastY = false;
            } else{
                robot.leftManip.setPosition(0);
                robot.rightManip.setPosition(1);
                pastY = true;
                pastX = false;
            }
            lastManipTime = runtime.seconds();
        }

        if (gamepad2.x && lastManipTime + 1 < runtime.seconds()) {
            if (pastX) {
                robot.leftManip.setPosition(0.5);
                robot.rightManip.setPosition(0.5);
                pastX = false;
                pastY = false;
            } else {
                robot.leftManip.setPosition(0);
                robot.rightManip.setPosition(1);
                pastX = true;
            }
            lastManipTime = runtime.seconds();

        }


        telemetry.addData("Drive:", "Lft Power %.2f, Rgt Power %.2f", leftSquaredVal, rightSquaredVal);

        telemetry.addData("Path0", "Position at %7d :%7d",
                robot.leftBackDrive.getCurrentPosition(),
                robot.rightBackDrive.getCurrentPosition());

        telemetry.addData("Status", "Debug 1 at: " + runtime.toString());

/*    Not needed on current Trollbot, has no servos.

        if (gamepad1.left_bumper && LservoUpTime + 2 < runtime.seconds()) {
            //robot.lpushStart robot.lpushDeploy
            if (robot.leftPush.getPosition() == robot.lpushStart) {
                robot.leftPush.setPosition(robot.lpushDeploy);

            } else {
                robot.leftPush.setPosition(robot.lpushStart);
            }
            LservoUpTime = runtime.seconds();
        }
        telemetry.addData("Status", "Debug 2 at: " + runtime.toString());
        telemetry.addData("Servo", " 1 leftPush Servo Set to " + robot.leftPush.getPosition());

        if (gamepad1.right_bumper && RservoUpTime + 2 < runtime.seconds()) {
            telemetry.addData("Status", "Debug 3 at: " + runtime.toString());
            // robot.rpushStart robot.rpushDeploy
            if (robot.rightPush.getPosition() == robot.rpushStart) {
                robot.rightPush.setPosition(robot.rpushDeploy);
            } else {
                robot.rightPush.setPosition(robot.rpushStart);
            }
            RservoUpTime = runtime.seconds();
        }
        telemetry.addData("Servo", "2 rightPush Servo Set to " + robot.rightPush.getPosition());
*/

        telemetry.addData("Status", "Debug 4 at: " + runtime.toString());

    }  // loop


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.normalDrive(this, 0, 0);
    }
}
