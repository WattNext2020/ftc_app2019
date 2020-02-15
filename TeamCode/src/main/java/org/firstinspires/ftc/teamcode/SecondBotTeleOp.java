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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SecondBotTeleOp", group="Linear Opmode")
public class SecondBotTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRDrive = null;
    private DcMotor rightRDrive = null;
    private DcMotor leftFDrive = null;
    private DcMotor rightFDrive = null;


    double leftRPower = 0;
    double rightRPower = 0;
    double leftFPower = 0;
    double rightFPower = 0;
    double lastBrake = 0;
    double lastSlow = 0;

    boolean zeroBrake = true;
    boolean slowMode = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRDrive  = hardwareMap.get(DcMotor.class, "leftb");
        rightRDrive = hardwareMap.get(DcMotor.class, "rightb");
        leftFDrive = hardwareMap.get(DcMotor.class, "leftf");
        rightFDrive = hardwareMap.get(DcMotor.class, "rightf");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            double strafePower = gamepad1.right_stick_x;
            double tankPower = gamepad1.right_stick_y;
            double turnPower = gamepad1.left_stick_x;
            double diagonal = gamepad1.left_stick_y;

            if (gamepad1.b == true){
                if ((runtime.seconds() - lastBrake) > .5) { //slow mode threshold

                    if (zeroBrake == true) {
                        zeroBrake = false;
                        lastBrake = runtime.seconds();
                    } else {
                        zeroBrake = true;
                        lastBrake = runtime.seconds();
                    }

                }
            }

            if (gamepad1.a == true) {
                if ((runtime.seconds() - lastSlow) > .5) { //slow mode threshold

                    if (slowMode == true) {

                        slowMode = false;
                        lastSlow = runtime.seconds();
                    } else {

                        slowMode = true;
                        lastSlow = runtime.seconds();
                    }

                }
            }
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            if (zeroBrake == true) {
                leftFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Makes motors brake when set to 0
                leftRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            } else {
                leftFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //Makes motors brake when set to 0
                leftRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }


            if (slowMode == true) {
                strafePower = strafePower * .5;
                tankPower = tankPower * .5;
                turnPower = turnPower * .5;
            }

            /*

            if(strafePower > 0.5){
                if(tankPower > 0.5){
                    leftFPower = 1.0;
                    rightRPower = 1.0;
                }
            }
            else if(strafePower > 0.5){
                if(tankPower < -0.5){
                    leftRPower = -1.0;
                    rightFPower = -1.0;
                }
            }
            else if(strafePower > 0.5 & tankPower < -0.5){

            }
            else if(strafePower < -0.5 & tankPower > 0.5){
                leftRPower = 1.0;
                rightFPower = 1.0;
            }
            else if(strafePower < -0.5 & tankPower < -0.5){
                leftFPower = -1.0;
                rightRPower = -1.0;
            }
            else if(turnPower < -0.1){
                leftFPower = Range.clip(-turnPower, -1.0, 1.0);
                leftRPower = Range.clip(-turnPower, -1.0, 1.0);
                rightFPower = Range.clip(turnPower, -1.0, 1.0);
                rightRPower = Range.clip(turnPower, -1.0, 1.0);
            }
            else if(turnPower > 0.1){
                leftFPower = Range.clip(-turnPower, -1.0, 1.0);
                leftRPower = Range.clip(-turnPower, -1.0, 1.0);
                rightFPower = Range.clip(turnPower, -1.0, 1.0);
                rightRPower = Range.clip(turnPower, -1.0, 1.0);
            }
            else if(strafePower < -0.1){
                leftFPower = Range.clip(-strafePower, -1.0, 1.0);
                leftRPower = Range.clip(strafePower, -1.0, 1.0);
                rightFPower = Range.clip(strafePower, -1.0, 1.0);
                rightRPower = Range.clip(-strafePower, -1.0, 1.0);
            }
            else if(strafePower > 0.1){
                leftFPower = Range.clip(-strafePower, -1.0, 1.0);
                leftRPower = Range.clip(strafePower, -1.0, 1.0);
                rightFPower = Range.clip(strafePower, -1.0, 1.0);
                rightRPower = Range.clip(-strafePower, -1.0, 1.0);
            }
            else if(tankPower < -0.1){
                leftFPower = Range.clip(tankPower, -1.0, 1.0);
                leftRPower = Range.clip(tankPower, -1.0, 1.0);
                rightFPower = Range.clip(tankPower, -1.0, 1.0);
                rightRPower = Range.clip(tankPower, -1.0, 1.0);
            }
            else if(tankPower > 0.1){
                leftFPower = Range.clip(tankPower, -1.0, 1.0);
                leftRPower = Range.clip(tankPower, -1.0, 1.0);
                rightFPower = Range.clip(tankPower, -1.0, 1.0);
                rightRPower = Range.clip(tankPower, -1.0, 1.0);
            }
            else{
                tankPower = 0.0;
                turnPower = 0.0;
                leftFPower = Range.clip(tankPower, -1.0, 1.0);
                rightFPower = Range.clip(tankPower, -1.0, 1.0);
                leftRPower = Range.clip(tankPower, -1.0, 1.0);
                rightRPower = Range.clip(tankPower, -1.0, 1.0);

            }

            */

            /*
            if(strafePower > 0.5 & tankPower > 0.5){
                leftFPower = 1.0;
                rightRPower = 1.0;
            }
            else if(strafePower > 0.5 & tankPower < -0.5){
                leftRPower = -1.0;
                rightFPower = -1.0;
            }
            else if(strafePower < -0.5 & tankPower > 0.5){
                leftRPower = 1.0;
                rightFPower = 1.0;
            }
            else if(strafePower < -0.5 & tankPower < -0.5){
                leftFPower = -1.0;
                rightRPower = -1.0;
            }
            else if(turnPower < -0.1){
                leftFPower = Range.clip(-turnPower, -1.0, 1.0);
                leftRPower = Range.clip(-turnPower, -1.0, 1.0);
                rightFPower = Range.clip(turnPower, -1.0, 1.0);
                rightRPower = Range.clip(turnPower, -1.0, 1.0);
            }
            else if(turnPower > 0.1){
                leftFPower = Range.clip(-turnPower, -1.0, 1.0);
                leftRPower = Range.clip(-turnPower, -1.0, 1.0);
                rightFPower = Range.clip(turnPower, -1.0, 1.0);
                rightRPower = Range.clip(turnPower, -1.0, 1.0);
            }
            else if(strafePower < -0.1){
                leftFPower = Range.clip(-strafePower, -1.0, 1.0);
                leftRPower = Range.clip(strafePower, -1.0, 1.0);
                rightFPower = Range.clip(strafePower, -1.0, 1.0);
                rightRPower = Range.clip(-strafePower, -1.0, 1.0);
            }
            else if(strafePower > 0.1){
                leftFPower = Range.clip(-strafePower, -1.0, 1.0);
                leftRPower = Range.clip(strafePower, -1.0, 1.0);
                rightFPower = Range.clip(strafePower, -1.0, 1.0);
                rightRPower = Range.clip(-strafePower, -1.0, 1.0);
            }
            else if(tankPower < -0.1){
                leftFPower = Range.clip(tankPower, -1.0, 1.0);
                leftRPower = Range.clip(tankPower, -1.0, 1.0);
                rightFPower = Range.clip(tankPower, -1.0, 1.0);
                rightRPower = Range.clip(tankPower, -1.0, 1.0);
            }
            else if(tankPower > 0.1){
                leftFPower = Range.clip(tankPower, -1.0, 1.0);
                leftRPower = Range.clip(tankPower, -1.0, 1.0);
                rightFPower = Range.clip(tankPower, -1.0, 1.0);
                rightRPower = Range.clip(tankPower, -1.0, 1.0);
            }
            else{
                tankPower = 0.0;
                turnPower = 0.0;
                leftFPower = Range.clip(tankPower, -1.0, 1.0);
                rightFPower = Range.clip(tankPower, -1.0, 1.0);
                leftRPower = Range.clip(tankPower, -1.0, 1.0);
                rightRPower = Range.clip(tankPower, -1.0, 1.0);

            }

            */

            if(turnPower < -0.1){
                leftFPower = Range.clip(-turnPower, -1.0, 1.0);
                leftRPower = Range.clip(-turnPower, -1.0, 1.0);
                rightFPower = Range.clip(turnPower, -1.0, 1.0);
                rightRPower = Range.clip(turnPower, -1.0, 1.0);
            }
            else if(turnPower > 0.1){
                leftFPower = Range.clip(-turnPower, -1.0, 1.0);
                leftRPower = Range.clip(-turnPower, -1.0, 1.0);
                rightFPower = Range.clip(turnPower, -1.0, 1.0);
                rightRPower = Range.clip(turnPower, -1.0, 1.0);
            }
            else if(strafePower < -0.1){
                leftFPower = Range.clip(-strafePower, -1.0, 1.0);
                leftRPower = Range.clip(strafePower, -1.0, 1.0);
                rightFPower = Range.clip(strafePower, -1.0, 1.0);
                rightRPower = Range.clip(-strafePower, -1.0, 1.0);
            }
            else if(strafePower > 0.1){
                leftFPower = Range.clip(-strafePower, -1.0, 1.0);
                leftRPower = Range.clip(strafePower, -1.0, 1.0);
                rightFPower = Range.clip(strafePower, -1.0, 1.0);
                rightRPower = Range.clip(-strafePower, -1.0, 1.0);
            }
            else if(tankPower < -0.1){
                leftFPower = Range.clip(tankPower, -1.0, 1.0);
                leftRPower = Range.clip(tankPower, -1.0, 1.0);
                rightFPower = Range.clip(tankPower, -1.0, 1.0);
                rightRPower = Range.clip(tankPower, -1.0, 1.0);
            }
            else if(tankPower > 0.1){
                leftFPower = Range.clip(tankPower, -1.0, 1.0);
                leftRPower = Range.clip(tankPower, -1.0, 1.0);
                rightFPower = Range.clip(tankPower, -1.0, 1.0);
                rightRPower = Range.clip(tankPower, -1.0, 1.0);
            }
            else{
                tankPower = 0.0;
                turnPower = 0.0;
                leftFPower = Range.clip(tankPower, -1.0, 1.0);
                rightFPower = Range.clip(tankPower, -1.0, 1.0);
                leftRPower = Range.clip(tankPower, -1.0, 1.0);
                rightRPower = Range.clip(tankPower, -1.0, 1.0);

            }

            // Send calculated power to wheels
            leftFDrive.setPower(leftFPower);
            rightFDrive.setPower(rightFPower);
            leftRDrive.setPower(leftRPower);
            rightRDrive.setPower(rightRPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftr (%.2f), rightr (%.2f), leftf(%.2f), rightf(%.2f)", leftRPower, rightRPower, leftFPower, rightFPower);
            telemetry.update();
        }
    }
}
