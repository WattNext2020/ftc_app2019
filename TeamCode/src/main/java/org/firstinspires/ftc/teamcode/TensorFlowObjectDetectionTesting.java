/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "TensorFlow Object Detection Test", group = "Concept")
//@Disabled
public class TensorFlowObjectDetectionTesting extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfr = null;
    private DcMotor leftback = null;
    private DcMotor rightfr = null;
    private DcMotor rightback = null;

    private Servo rightHook = null;
    private Servo leftHook = null;


    double leftfrPower;
    double leftbackPower;
    double rightfrPower;
    double rightbackPower;
    int location = 0;
    int MOTOR_TICKS = 850;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            " AbDFg7f/////AAABmUAnCfvXrUNGvMJDTDBWqy4Wti73gaRA6vw8oXW1gGnwhH90MD/IQ4ka4at/PmY/NrsGZKPwELOsHt2zwUhMXFR75NW4+VdHI/xfyFDqVkULBsWaE5Yh/KHLCYG/fpXAU1kR0O2/lNcEAMnH27oIgRMbtJ53fFJznmi7tGI5GU7AUnCZs2DpZ7yCI79gmaLCEzL1bWjoDGiv6pLg4Tl27pOQzGEe+105tV8vmsVkpv9qRc0oTNn32dPKayB+PYoKZbBB1rciTsiZXMdEEerpyE/bGHNc3vcd1DQ05ifZ35J0KSFW+3V4PVfEgc+qJr61m0bZwr6D89UONVq3U/fSnwA/A86wKfp8tIa4+4hOOTh/ ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        leftfr = hardwareMap.get(DcMotor.class, "leftf");
        leftback = hardwareMap.get(DcMotor.class, "leftb");
        rightfr = hardwareMap.get(DcMotor.class, "rightf");
        rightback = hardwareMap.get(DcMotor.class, "rightb");

        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");

        //set direction of motors
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        //set positions of servos
        leftHook.setPosition(0);
        rightHook.setPosition(1);

        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        ElapsedTime LoopTimer=new ElapsedTime();


        while (opModeIsActive()) {

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.update();
                    for (Recognition recognition : updatedRecognitions) {

                        EncoderMove(rightback, rightfr, leftfr, leftback);
                        LoopTimer.startTime();
//                        sleep(2000);
                        if(recognition.getLabel()  != "Skystone") {
                            while (LoopTimer.seconds() < 2){
                                leftfr.setPower(-0.5);
                                leftback.setPower(0.5);
                                rightfr.setPower(0.5);
                                rightback.setPower(-0.5);
                                telemetry.addData("Robot is: ", "Strafing");
                            }
                            leftfr.setPower(0);
                            leftback.setPower(0);
                            rightfr.setPower(0);
                            rightback.setPower(0);
                            telemetry.addData("Skystone: ", recognition.getLabel());
                            telemetry.addData("Location: ", location);
                            telemetry.update();
                        }
                        else{
                            while (LoopTimer.seconds() < 1){
                                leftfr.setPower(0.5);
                                leftback.setPower(0.5);
                                rightfr.setPower(0.5);
                                rightback.setPower(0.5);
                            }
                            leftfr.setPower(0);
                            leftback.setPower(0);
                            rightfr.setPower(0);
                            rightback.setPower(0);
                            location = 1;
                            telemetry.addData("Skystone: ", recognition.getLabel());
                            telemetry.addData("Location: ", location);
                            telemetry.update();
                            sleep(1000);
                          }

                          sleep(1000);

                          LoopTimer.reset();
                        if(recognition.getLabel()  != "Skystone") {
                            while (LoopTimer.seconds() < 2){
                                leftfr.setPower(0.5);
                                leftback.setPower(-0.5);
                                rightfr.setPower(-0.5);
                                rightback.setPower(0.5);
                            }
                            while(LoopTimer.seconds() < 3){
                                leftfr.setPower(0.5);
                                leftback.setPower(0.5);
                                rightfr.setPower(0.5);
                                rightback.setPower(0.5);
                            }
                            leftfr.setPower(0);
                            leftback.setPower(0);
                            rightfr.setPower(0);
                            rightback.setPower(0);
                            location = 3;
                            telemetry.addData("Skystone: ", recognition.getLabel());
                            telemetry.addData("Location: ", location);
                            telemetry.update();
                            sleep(1000);
                        }
                        else{
                            while(LoopTimer.seconds() < 1){
                                leftfr.setPower(0.5);
                                leftback.setPower(0.5);
                                rightfr.setPower(0.5);
                                rightback.setPower(0.5);
                            }
                            leftfr.setPower(0);
                            leftback.setPower(0);
                            rightfr.setPower(0);
                            rightback.setPower(0);
                            location = 2;
                            telemetry.addData("Skystone: ", recognition.getLabel());
                            telemetry.addData("Location: ", location);
                            telemetry.update();
                            sleep(1000);
                        }
                        telemetry.addData("time1", "= " + LoopTimer);
                        telemetry.update();
                    }

                }
            }
        }



        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void EncoderMove(DcMotor rightback, DcMotor rightfr,DcMotor leftfr,DcMotor leftback)
    {
        rightback.setTargetPosition(rightback.getCurrentPosition() + MOTOR_TICKS);
        rightback.setPower(0.25);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfr.setPower(0.25);
        leftfr.setPower(0.25);
        leftback.setPower(0.25);


        while (rightback.isBusy()) {

            if(rightback.getCurrentPosition()>= (rightback.getTargetPosition()-400)) {
                rightback.setPower(0.2);
                rightfr.setPower(0.2);
                leftfr.setPower(0.2);
                leftback.setPower(0.2);


                if (rightback.getCurrentPosition() >= (rightback.getTargetPosition() - 5)) {
                    leftback.setPower(0);
                    rightback.setPower(0);
                    rightfr.setPower(0);
                    leftfr.setPower(0);
                    leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    sleep(3000);
                    return;
                }
            }
        }
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
