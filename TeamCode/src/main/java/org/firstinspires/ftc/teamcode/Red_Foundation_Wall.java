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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.*;
import java.text.DecimalFormat;

import java.math.BigDecimal;
import java.math.RoundingMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import java.lang.Math;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


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

@Autonomous(name="Red Foundation Wall", group="Linear Opmode")
public class Red_Foundation_Wall extends LinearOpMode {

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

    Servo newCap;


    double leftfrPower;
    double leftbackPower;
    double rightfrPower;
    double rightbackPower;

    double leftPower;
    double rightPower;
    double rackPowerUD;
    double rackPowerLR;


    double initHeading;

        // The IMU sensor object
        BNO055IMU imu;

        // State used for updating telemetry
        Orientation angles;
        Acceleration gravity;


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

            leftWheels = hardwareMap.get(CRServo.class, "lw");   //leftwheels
            rightWheels = hardwareMap.get(CRServo.class, "rw");  //rightwheels
            // capStone = hardwareMap.get (CRServo.class, "Cap");  // Rack and Pinion Vertical
            // rackPinionLR = hardwareMap.get (CRServo.class, "rpLeftRight");   //Rack and Pinion Horizontal
            rightHook = hardwareMap.get(Servo.class, "rightHook");
            leftHook = hardwareMap.get(Servo.class, "leftHook");
            newCap = hardwareMap.get(Servo.class, "NewCap");

            newCap.setPosition(0);


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

            leftWheels.setPower(0);
            rightWheels.setPower(0);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);



            leftfr = hardwareMap.get(DcMotor.class, "leftf");
            leftback = hardwareMap.get(DcMotor.class, "leftb");
            rightfr = hardwareMap.get(DcMotor.class, "rightf");
            rightback = hardwareMap.get(DcMotor.class, "rightb");


            // Set up our telemetry dashboard
            composeTelemetry();

            // Wait until we're told to go
            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {

                while (runtime.seconds() < .66) {
                    leftfr.setPower(-0.5);
                    leftback.setPower(0.535);
                    rightfr.setPower(0.5);
                    rightback.setPower(-0.5);
                }
                while (runtime.seconds() < 1.7) {
                    leftfr.setPower(-0.5);
                    leftback.setPower(-0.535);
                    rightfr.setPower(-0.5);
                    rightback.setPower(-0.5);
                }
                while (runtime.seconds() < 3.6) {
                    leftfr.setPower(0.0);
                    leftback.setPower(0.0);
                    rightfr.setPower(0.0);
                    rightback.setPower(0.0);
                    rightHook.setPosition(1);
                }
                while (runtime.seconds() < 5) { //bring foundation backwards
                    leftfr.setPower(0.5);
                    leftback.setPower(0.535);
                    rightfr.setPower(0.5);
                    rightback.setPower(0.5);
                }
                //AcuTurn(90,true);
                //stop();
                while (runtime.seconds() < 6){
                    rightHook.setPosition(0);
                    leftfr.setPower(0.5);
                    leftback.setPower(-0.535);
                    rightfr.setPower(-0.5);
                    rightback.setPower(0.5);
                }
                while (runtime.seconds() < 7){
                    leftfr.setPower(-0.5);
                    leftback.setPower(-0.535);
                    rightfr.setPower(-0.5);
                    rightback.setPower(-0.5);
                }
                while (runtime.seconds() < 7.3){
                    leftfr.setPower(-0.5);
                    leftback.setPower(0.535);
                    rightfr.setPower(0.5);
                    rightback.setPower(-0.5);
                }
                while (runtime.seconds() < 7.8){
                    leftfr.setPower(-0.5);
                    leftback.setPower(-0.535);
                    rightfr.setPower(-0.5);
                    rightback.setPower(-0.5);
                }
                while (runtime.seconds() < 8.5){
                    leftfr.setPower(-0.5);
                    leftback.setPower(0.535);
                    rightfr.setPower(0.5);
                    rightback.setPower(-0.5);
                }
                while (runtime.seconds() < 8.8){
                    leftfr.setPower(-0.5);
                    leftback.setPower(-0.535);
                    rightfr.setPower(-0.5);
                    rightback.setPower(-0.5);
                }
                while (runtime.seconds() < 9.5){
                    leftfr.setPower(-0.5);
                    leftback.setPower(0.535);
                    rightfr.setPower(0.5);
                    rightback.setPower(-0.5);
                }
                while (runtime.seconds() < 10.4){
                    leftfr.setPower(-0.5);
                    leftback.setPower(-0.535);
                    rightfr.setPower(-0.5);
                    rightback.setPower(-0.5);
                }
                while (runtime.seconds() < 11.3){
                    leftfr.setPower(-0.5);
                    leftback.setPower(0.535);
                    rightfr.setPower(0.5);
                    rightback.setPower(-0.5);
                }
                while (runtime.seconds() < 12){
                    leftfr.setPower(0.5);
                    leftback.setPower(0.535);
                    rightfr.setPower(0.5);
                    rightback.setPower(0.5);
                }
                //ADD HERE
                leftfr.setPower(0.0);
                leftback.setPower(0.0);
                rightfr.setPower(0.0);
                rightback.setPower(0.0);


                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }
        }

        void AcuMove(double StraifMeters, double TankMeters) {


            telemetry.update();
            TankMeters = TankMeters * 10000;


            double initXaccel = gravity.yAccel;
            double Xaccel;
            double Yaccel = 0;
            double Speed = 0;


            double lastTime = 0;
            double lastSpeed = 0;


            double movedTank = 0;

            double runTimes = 0;
            double initialTime = runtime.seconds();

            while ((runtime.seconds() - initialTime) < 5) {
                idle();
            }


            while (movedTank < TankMeters) {
                telemetry.update();
                runTimes = runTimes + 1;

                Xaccel = (gravity.yAccel) - initXaccel;


                if (Xaccel < .05 && Xaccel > -.05) {
                    Xaccel = 0;
                    telemetry.addData("Zeroed:", "True");
                } else {
                    telemetry.addData("Zeroed:", "False");
                }


                Speed = lastSpeed + (Xaccel / (runtime.seconds() - lastTime));
                telemetry.addData("Speed:", Speed + "mm/s");
                telemetry.addData("X acceleration:", Xaccel);
                telemetry.addData("Y acceleration:", Yaccel);


                telemetry.addData("Equation(Speed)=", lastSpeed + "+(" + Xaccel + "/(", runtime.seconds() + "-" + lastTime + "))");


                telemetry.addData("Moved", movedTank / 10000);

                telemetry.addData("Times Run:", runTimes);

                movedTank = movedTank + (Speed * (runtime.seconds() - lastTime));
                telemetry.addData("Moved", movedTank);



                lastTime = runtime.seconds();


            }

            telemetry.addData("Moved Post Finish", movedTank * 1000);
            telemetry.update();
            leftfr.setPower(0);
            leftback.setPower(0);
            rightback.setPower(0);
            rightfr.setPower(0);


            initialTime = runtime.seconds();
            while ((runtime.seconds() - initialTime) < 5) {
                idle();
            }
            return;


        }

        public static void roundAndPrint(double n, int round2DecimalPlace) {
            String temp;
            BigDecimal instance = new BigDecimal(Double.toString(n));
            instance = instance.setScale(round2DecimalPlace, RoundingMode.HALF_UP);

        }


        void AcuTurn(double Degrees, boolean Clockwise) {
            rightfr.setDirection(DcMotor.Direction.REVERSE);
            rightback.setDirection(DcMotor.Direction.REVERSE);
            leftback.setDirection(DcMotor.Direction.REVERSE);
            leftfr.setDirection(DcMotor.Direction.REVERSE);


            Degrees = (Math.abs(Degrees));
            double moved = 0;

            double lastHead;


            telemetry.update();
            initHeading = angles.firstAngle;
            lastHead = angles.firstAngle;


            while (moved < Degrees ) {
                telemetry.addData("Test Data", Degrees - Math.abs(angles.firstAngle - initHeading));
                telemetry.addData("Test Data 2", Math.abs(angles.firstAngle - initHeading));
                telemetry.addData("Moved: ", moved);
                telemetry.addData("Degrees: ", Degrees);
                telemetry.addData("Velocity", imu.getVelocity());


                if (Clockwise == true) {
                    if ((Degrees - moved) > 70) {
                        leftfr.setPower(.45);
                        leftback.setPower(.45);
                        rightfr.setPower(.4);
                        rightback.setPower(.4);
                    } else {
                        if ((Degrees - moved) > 40) {
                            leftfr.setPower(.35);
                            leftback.setPower(.35);
                            rightfr.setPower(.3);
                            rightback.setPower(.3);
                        } else {
                            if ((Degrees - moved) > 20) {
                                leftfr.setPower(.25);
                                leftback.setPower(.25);
                                rightfr.setPower(.2);
                                rightback.setPower(.2);
                            }
                            if ((Degrees - moved) > 10) {
                                leftfr.setPower(.155);
                                leftback.setPower(.155);
                                rightfr.setPower(.155);
                                rightback.setPower(.155);
                            } else {
                                if ((Degrees - moved) > 5) {
                                    leftfr.setPower(.15);
                                    leftback.setPower(.15);
                                    rightfr.setPower(.15);
                                    rightback.setPower(.15);
                                }
                            }
                        }
                    }

                } else {
                    leftfr.setPower(-.2);
                    leftback.setPower(-.2);
                    rightfr.setPower(-.2);
                    rightback.setPower(-.2);
                }


                moved = moved + (Math.abs(angles.firstAngle - lastHead));
                lastHead = angles.firstAngle;
                telemetry.update();

            }

            leftfr.setPower(0);
            leftback.setPower(0);
            rightfr.setPower(0);
            rightback.setPower(0);

            return;

        }

        //----------------------------------------------------------------------------------------------
        // Telemetry Configuration
        //----------------------------------------------------------------------------------------------

        void composeTelemetry() {

            // At the beginning of each telemetry update, grab a bunch of data
            // from the IMU that we will then display in separate lines.
            telemetry.addAction(new Runnable() {
                @Override
                public void run() {
                    // Acquiring the angles is relatively expensive; we don't want
                    // to do that in each of the three items that need that info, as that's
                    // three times the necessary expense.
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                }
            });

            telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override
                        public String value() {
                            return imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override
                        public String value() {
                            return imu.getCalibrationStatus().toString();
                        }
                    });

            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.thirdAngle);
                        }
                    });

            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override
                        public String value() {
                            return gravity.toString();
                        }
                    })
                    .addData("mag", new Func<String>() {
                        @Override
                        public String value() {
                            return String.format(Locale.getDefault(), "%.3f",
                                    Math.sqrt(gravity.xAccel * gravity.xAccel
                                            + gravity.yAccel * gravity.yAccel
                                            + gravity.zAccel * gravity.zAccel));
                        }
                    });
        }

        //----------------------------------------------------------------------------------------------
        // Formatting
        //----------------------------------------------------------------------------------------------

        String formatAngle(AngleUnit angleUnit, double angle) {
            return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        String formatDegrees(double degrees) {
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }


        public void tmove(double runSeconds, double presentRuntime, double turnPower, double straifPower, double tankPower) {

            while (runtime.seconds() < (runSeconds + presentRuntime)) {
                bMove(turnPower, straifPower, tankPower);

            }
        }

        public void bMove(double turnPower, double straifPower, double tankPower) {
            double leftfrPower;
            double leftbackPower;
            double rightfrPower;
            double rightbackPower;


            turnPower = turnPower * -1;

            tankPower = tankPower * -1;

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


        }
    }
