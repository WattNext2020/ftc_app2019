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
import com.qualcomm.robotcore.util.Range;

@Autonomous(name= "Red_Block_Pickup", group= "Linear Opmode")

public class Red_Block_Pickup extends LinearOpMode {
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



    double leftPower;
    double rightPower;
    double rackPowerUD;
    double rackPowerLR;

    double initTime;


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


        rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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






            leftfr.setPower(0.0);
            leftback.setPower(0.0);
            rightfr.setPower(0.0);
            rightback.setPower(0.0);

            leftHook.setPosition(0);
            rightHook.setPosition(0);



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            leftHook.setPosition(1);

            tmove(2.1,runtime.seconds(), 0,0,.3);

            initTime = runtime.seconds();
            telemetry.addData("Pre-Turn", "Active");
            telemetry.update();
            while((runtime.seconds()-initTime) < 2.8  )
            {
                telemetry.addData("Turn", "Active");
                telemetry.update();
                leftfr.setPower(-0.35);
                leftback.setPower(-0.35);
                rightfr.setPower(0);
                rightback.setPower(0);
                telemetry.addData("Turn", "Active");
                telemetry.update();
            }
            tmove(.9,runtime.seconds(),-.5, 0,0);

            tmove(1,runtime.seconds(),0,0,0);
            tmove(.6,runtime.seconds(),0,0,.4);
            rightHook.setPosition(1);
            tmove(.8,runtime.seconds(),0,0,.4);
            tmove(.7,runtime.seconds(),.4, 0,0);


            tmove(1.2,runtime.seconds(),0,0,.4);









            leftHook.setPosition(0);
            rightHook.setPosition(0);



            tmove(2, runtime.seconds(), 0,.6,0);

            tmove(.6,runtime.seconds(), 0,0,-.4);




            stop();

        }
    }

    public void tmove(double runSeconds, double presentRuntime, double turnPower, double straifPower, double tankPower)

    {

        while(runtime.seconds() < (runSeconds+presentRuntime))
        {
            bMove(turnPower, straifPower, tankPower);

        }
    }

    public double bMove(double turnPower, double straifPower, double tankPower )
    {
        double leftfrPower;
        double leftbackPower;
        double rightfrPower;
        double rightbackPower;


        turnPower = turnPower*-1;

        tankPower = tankPower*-1;

        if (turnPower < -0.1) {
            leftfrPower = Range.clip(-turnPower, -1.0, 1.0);
            leftbackPower = Range.clip(-turnPower, -1.0, 1.0);
            rightfrPower = Range.clip(turnPower, -1.0, 1.0);
            rightbackPower = Range.clip(turnPower, -1.0, 1.0);
        } else if (turnPower > 0.1) {
            leftfrPower = Range.clip(-turnPower, -1.0, 1.0);
            leftbackPower = Range.clip(-turnPower, -1.0, 1.0);
            rightfrPower = Range.clip(turnPower, -1.0, 1.0);
            rightbackPower = Range.clip(turnPower, -1.0, 1.0);
        } else if (straifPower < -0.1) {
            leftfrPower = Range.clip(-straifPower, -1.0, 1.0);
            leftbackPower = Range.clip(straifPower, -1.0, 1.0);
            rightfrPower = Range.clip(straifPower, -1.0, 1.0);
            rightbackPower = Range.clip(-straifPower, -1.0, 1.0);
        } else if (straifPower > 0.1) {
            leftfrPower = Range.clip(-straifPower, -1.0, 1.0);
            leftbackPower = Range.clip(straifPower, -1.0, 1.0);
            rightfrPower = Range.clip(straifPower, -1.0, 1.0);
            rightbackPower = Range.clip(-straifPower, -1.0, 1.0);
        } else if (tankPower < -0.1) {
            leftbackPower = Range.clip(tankPower, -1.0, 1.0);
            rightbackPower = Range.clip(tankPower, -1.0, 1.0);
            leftfrPower = Range.clip(tankPower, -1.0, 1.0);
            rightfrPower = Range.clip(tankPower, -1.0, 1.0);
        } else if (tankPower > 0.1) {
            leftfrPower = Range.clip(tankPower, -1.0, 1.0);
            rightfrPower = Range.clip(tankPower, -1.0, 1.0);
            leftbackPower = Range.clip(tankPower, -1.0, 1.0);
            rightbackPower = Range.clip(tankPower, -1.0, 1.0);
        } else {
            tankPower = 0.0;
            turnPower = 0.0;
            leftfrPower = Range.clip(tankPower, -1.0, 1.0);
            rightfrPower = Range.clip(tankPower, -1.0, 1.0);
            leftbackPower = Range.clip(tankPower, -1.0, 1.0);
            rightbackPower = Range.clip(tankPower, -1.0, 1.0);
        }
        leftfr.setPower(leftfrPower);
        leftback.setPower(leftbackPower);
        rightfr.setPower(rightfrPower);
        rightback.setPower(rightbackPower);

        return 0;

    }
}

