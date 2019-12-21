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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name= "Blue Foundation", group= "Linear Opmode")

public class BLUE_Foundation extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfr = null;
    private DcMotor leftback = null;
    private DcMotor rightfr = null;
    private DcMotor rightback = null;

    private CRServo leftWheels = null;
    private CRServo rightWheels = null;
    //private CRServo rackPinionUD = null;
    //private CRServo rackPinionLR = null;

    private Servo rightHook = null;
    private Servo leftHook = null;

    private DistanceSensor sensorRange;

    double leftfrPower;
    double leftbackPower;
    double rightfrPower;
    double rightbackPower;

    double leftPower;
    double rightPower;
    double rackPowerUD;
    double rackPowerLR;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftfr = hardwareMap.get(DcMotor.class, "leftf");
        leftback = hardwareMap.get(DcMotor.class, "leftb");
        rightfr = hardwareMap.get(DcMotor.class, "rightf");
        rightback = hardwareMap.get(DcMotor.class, "rightb");

        leftWheels = hardwareMap.get(CRServo.class, "lw");
        rightWheels = hardwareMap.get(CRServo.class, "rw");
        //rackPinionUD = hardwareMap.get (CRServo.class, "rpUpDown");
        //rackPinionLR = hardwareMap.get (CRServo.class, "rpLeftRight");

        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        leftWheels.setDirection(CRServo.Direction.REVERSE);
        rightWheels.setDirection(CRServo.Direction.FORWARD);
        //rackPinionUD.setDirection(CRServo.Direction.FORWARD);
        //rackPinionLR.setDirection(CRServo.Direction.FORWARD);

        rightHook.setDirection(Servo.Direction.REVERSE);
        //rightfr.setmode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftfr.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        //int MOTORTICKS = 1680;

        leftHook.setPosition(0);
        rightHook.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            //BLUE MODE
            while (runtime.seconds()< .9){
                leftfr.setPower(0.5);
                leftback.setPower(-0.5);
                rightfr.setPower(-0.5);
                rightback.setPower(0.5);
            }
            while (runtime.seconds() < 2) {
                leftfr.setPower(-0.5);
                leftback.setPower(-0.5);
                rightfr.setPower(-0.5);
                rightback.setPower(-0.5);
            }
            while (runtime.seconds() < 3.5) {
                leftfr.setPower(0.0);
                leftback.setPower(0.0);
                rightfr.setPower(0.0);
                rightback.setPower(0.0);
                leftHook.setPosition(1);
                rightHook.setPosition(1);
            }
            while (runtime.seconds() < 5) {
                leftfr.setPower(0.5);
                leftback.setPower(0.5);
                rightfr.setPower(0.5);
                rightback.setPower(0.5);
            }
            while (runtime.seconds() < 5.05){
                leftfr.setPower(-0.5);
                leftback.setPower(-0.5);
                rightfr.setPower(0.5);
                rightback.setPower(0.5);
            }
            while (runtime.seconds() < 7.5){
                leftfr.setPower(0.0);
                leftback.setPower(0.0);
                rightfr.setPower(0.0);
                rightback.setPower(0.0);
                leftHook.setPosition(0);
                rightHook.setPosition(0);
            }
            while (runtime.seconds() < 7.55){
                leftfr.setPower(0.5);
                leftback.setPower(0.5);
                rightfr.setPower(-0.5);
                rightback.setPower(-0.5);
            }
            while (runtime.seconds() < 11) {
                leftfr.setPower(-0.5);
                leftback.setPower(0.5);
                rightfr.setPower(0.5);
                rightback.setPower(-0.5);
            }
            leftfr.setPower(0.0);
            leftback.setPower(0.0);
            rightfr.setPower(0.0);
            rightback.setPower(0.0);



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

