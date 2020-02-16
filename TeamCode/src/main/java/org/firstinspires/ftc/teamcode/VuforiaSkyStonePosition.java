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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import java.util.ArrayList;
import java.util.List;
import java.lang.*;
import java.text.DecimalFormat;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.lang.Math;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


@Autonomous(name="SKYSTONE Vuforia Position Update", group ="Concept")
public class VuforiaSkyStonePosition extends LinearOpMode {

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfr = null;
    private DcMotor leftback = null;
    private DcMotor rightfr = null;
    private DcMotor rightback = null;

    private CRServo leftWheels = null;
    private CRServo rightWheels = null;

    private Servo rightHook = null;
    private Servo leftHook = null;
    private Servo autoHook = null;

    //private DistanceSensor sensorRange;


    double leftfrPower;
    double leftbackPower;
    double rightfrPower;
    double rightbackPower;

    double leftPower;
    double rightPower;
    double rackPowerUD;
    double rackPowerLR;
    int location = 0;
    int MOTOR_TICKS = 850;
    int STRAFE_TICKS = 400;
    int TOSKYSTONE=400;

    boolean first = true;



    /*
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
    private static final String VUFORIA_KEY = "Aclq12D/////AAABmReYPG5/qkp0qP3/6pGHt+xWyWL7hC7mxarpON4/Pamia/rCbIo+26hHFXQr6LDW6BwmopcEA3sowSlSOsPd9AxduJY6MjZfIILe2rAXliUHeFspqohAwhRRxkBILOPsy/kJEw/u1/zrh+VTxOFfOEVGFjEwiMhQ41C/AEVPX3N0dxhT5IgmOL69Vol3zok/idBYDJsX9XyY3cdXnNNegSjRb9XL28T1U27grr96I1Gm5gcOEk0FZuEkMPFb8c4txRADsYIBxkSXrQ2OmjgCj1/aQMbi+jUlIYbqpaZKOciBzd5zV0LiDeh/QFi1Xdq4x2pZ9COnGZqrC8JmbBAjmDfiA70H7BZheLOhbAo/K3gU";
    //   " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    boolean skystoneVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() {
        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = .0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled      = true;
        parameters1.loggingTag          = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftfr = hardwareMap.get(DcMotor.class, "leftf");
        leftback = hardwareMap.get(DcMotor.class, "leftb");
        rightfr = hardwareMap.get(DcMotor.class, "rightf");
        rightback = hardwareMap.get(DcMotor.class, "rightb");

        leftWheels = hardwareMap.get(CRServo.class, "lw");
        rightWheels = hardwareMap.get(CRServo.class, "rw");


        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");
        autoHook = hardwareMap.get(Servo.class,"autoHook");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        autoHook.setPosition(0);
        double initTime=0;


        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:
        targetsSkyStone.activate();
        waitForStart();
        runtime.reset();
        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        while (opModeIsActive() && !isStopRequested()) {

            EncoderMove(rightback, rightfr, leftfr, leftback);
            ElapsedTime LoopTimer = new ElapsedTime();
            LoopTimer.reset();
            LoopTimer.startTime();
            while (LoopTimer.seconds()<2) {
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                        OpenGLMatrix robotlocation
//                        VectorF translation = lastLocation.getTranslation();
                        telemetry.addData("Skystone:", "Visible Position 1");
//                        telemetry.addData("Location ", translation.get(0));
                        skystoneVisible = true;
                        location = 1;
                        MovingtoSkystone(rightback, rightfr, leftfr, leftback);
                        telemetry.update();
                        sleep(2000);
                        initTime = 0;
                        PickupMove(rightback, rightfr, leftfr, leftback,location, initTime);

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        break;
                    }
                }
            }
            // Provide feedback as to where the robot is located (if we know).

            if (skystoneVisible == false) {
                LoopTimer.reset();
                telemetry.addData("Not in Position 1", " ");
                telemetry.update();
                EncoderStrafe(rightback, rightfr, leftfr, leftback, initTime);
                ElapsedTime LoopTimer2 = new ElapsedTime();
                LoopTimer2.reset();
                LoopTimer2.startTime();
                while (LoopTimer2.seconds() < 2) {
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                            telemetry.addData("Skystone:", "Visible Position 2");
                            skystoneVisible = true;
                            location = 2;
                            MovingtoSkystone(rightback, rightfr, leftfr, leftback);
                            telemetry.update();
                            sleep(2000);
                            stop();
                            initTime = 0;
                            PickupMove(rightback, rightfr, leftfr, leftback,location, initTime);
                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            break;
                        }
                    }
                }
                if (skystoneVisible == false)
                {
                    initTime=0;
                    EncoderStrafe(rightback, rightfr, leftfr, leftback, initTime);
                    telemetry.addData("Skystone:", "Assumed Position 3");
                    skystoneVisible = true;
                    location = 3;
                    MovingtoSkystone(rightback, rightfr, leftfr, leftback);
                    telemetry.update();
                    stop();
                    initTime = 0;
                    PickupMove(rightback, rightfr, leftfr, leftback,location, initTime);

                }

            }

        }

     }


    public void EncoderMove(DcMotor rightback, DcMotor rightfr, DcMotor leftfr, DcMotor leftback) {

        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setTargetPosition(rightback.getCurrentPosition() + MOTOR_TICKS);
        rightback.setPower(0.3);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfr.setPower(0.3);
        leftfr.setPower(0.3);
        leftback.setPower(0.3);


        while (rightback.isBusy()) {

            if (rightback.getCurrentPosition() >= (rightback.getTargetPosition() - 400)) {
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
                    rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return;
                }
            }
        }


    }
    public void MovingtoSkystone(DcMotor rightback, DcMotor rightfr, DcMotor leftfr, DcMotor leftback)
    {

        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setTargetPosition(rightback.getCurrentPosition() + TOSKYSTONE);
        rightback.setPower(0.2);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfr.setPower(0.2);
        leftfr.setPower(0.2);
        leftback.setPower(0.2);


        while (rightback.isBusy()) {

            if (rightback.getCurrentPosition() >= (rightback.getTargetPosition() - 5)) {
                leftback.setPower(0);
                rightback.setPower(0);
                rightfr.setPower(0);
                leftfr.setPower(0);
                leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return;
            }
        }

    }

    public void PickupMove (DcMotor rightback, DcMotor rightfr, DcMotor leftfr, DcMotor leftback, int location, double initTime){
        initTime = runtime.seconds();
        //move to pickup
        while ((runtime.seconds()-initTime)<0.5){
            rightback.setPower(0.3);
            rightfr.setPower(0.3);
            leftfr.setPower(0.3);
            leftback.setPower(0.3);
        }
        while ((runtime.seconds()-initTime)<1.5){
            rightback.setPower(0);
            rightfr.setPower(0);
            leftfr.setPower(0);
            leftback.setPower(0);
        }
        while ((runtime.seconds()-initTime)<3){
            rightback.setPower(-0.3);
            rightfr.setPower(-0.3);
            leftfr.setPower(-0.3);
            leftback.setPower(-0.3);
        }
        rightback.setPower(0);
        rightfr.setPower(0);
        leftfr.setPower(0);
        leftback.setPower(0);
        autoHook.setPosition(1);
        sleep(2000);
        //AcuTurn(90,true);
        //moving forward certain amount based on "location"
        if (location ==1){
            telemetry.addData("Picking up", " 1");
            telemetry.update();
            sleep(500);
            while ((runtime.seconds()-initTime)<10){
                rightback.setPower(0.3);
                rightfr.setPower(0.3);
                leftfr.setPower(0.3);
                leftback.setPower(0.3);
            }
            while ((runtime.seconds()-initTime)<13){
                rightback.setPower(0);
                rightfr.setPower(0);
                leftfr.setPower(0);
                leftback.setPower(0);
            }
            while ((runtime.seconds()-initTime)<14){
                rightback.setPower(-0.3);
                rightfr.setPower(-0.3);
                leftfr.setPower(-0.3);
                leftback.setPower(-0.3);
            }
            rightback.setPower(0);
            rightfr.setPower(0);
            leftfr.setPower(0);
            leftback.setPower(0);
        }
        else if (location ==2){
            telemetry.addData("Picking up", " 2");
            telemetry.update();
            sleep(1000);
            while ((runtime.seconds()-initTime)<11){
                rightback.setPower(0.3);
                rightfr.setPower(0.3);
                leftfr.setPower(0.3);
                leftback.setPower(0.3);
            }
            while ((runtime.seconds()-initTime)<14){
                rightback.setPower(0);
                rightfr.setPower(0);
                leftfr.setPower(0);
                leftback.setPower(0);
            }
            while ((runtime.seconds()-initTime)<15){
                rightback.setPower(-0.3);
                rightfr.setPower(-0.3);
                leftfr.setPower(-0.3);
                leftback.setPower(-0.3);
            }
            rightback.setPower(0);
            rightfr.setPower(0);
            leftfr.setPower(0);
            leftback.setPower(0);
        }
        else if (location == 3){
            telemetry.addData("Picking up", " 3");
            telemetry.update();
            sleep(1000);
            while ((runtime.seconds()-initTime)<12){
                rightback.setPower(0.3);
                rightfr.setPower(0.3);
                leftfr.setPower(0.3);
                leftback.setPower(0.3);
            }
            while ((runtime.seconds()-initTime)<15){
                rightback.setPower(0);
                rightfr.setPower(0);
                leftfr.setPower(0);
                leftback.setPower(0);
            }
            while ((runtime.seconds()-initTime)<16){
                rightback.setPower(-0.3);
                rightfr.setPower(-0.3);
                leftfr.setPower(-0.3);
                leftback.setPower(-0.45);
            }
            rightback.setPower(0);
            rightfr.setPower(0);
            leftfr.setPower(0);
            leftback.setPower(0);
        }
    }

    public void EncoderStrafe(DcMotor rightback, DcMotor rightfr, DcMotor leftfr, DcMotor leftback, double initTime) {

        initTime=runtime.seconds();
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while((runtime.seconds()-initTime)<1.7)
        {
            rightback.setPower(-0.3);
            rightfr.setPower(0.3);
            leftfr.setPower(-0.3);
            leftback.setPower(0.3);
            telemetry.addData("Strafing"," ");
            telemetry.update();
        }
        rightback.setPower(0);
        rightfr.setPower(0);
        leftfr.setPower(0);
        leftback.setPower(0);
        return;

 /*      rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rightback.setTargetPosition(-rightback.getCurrentPosition() - STRAFE_TICKS);
       rightback.setPower(-0.3);
       rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rightfr.setPower(0.3);
       leftfr.setPower(-0.3);
       leftback.setPower(0.3);

       while (rightback.isBusy()) {
           telemetry.addData("Right Motor Curresnt: ", rightback.getCurrentPosition());
           telemetry.addData("Right Motor Target: ", rightback.getTargetPosition());
           telemetry.addData("Right Motor Ticks To Add: ", STRAFE_TICKS);
           telemetry.update();
           if (rightback.getCurrentPosition() <= (rightback.getTargetPosition() + 10)) {
               rightback.setPower(0);
               rightfr.setPower(0);
               leftfr.setPower(0);
               leftback.setPower(0);
               rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
               rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
               leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
               leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

           }
       }
       */

    }


    public void targetTelemetry(boolean targetVisible) {
        if (targetVisible) {
            telemetry.addData("Skystone Identified:", "True");
            telemetry.update();

        } else {
            telemetry.addData("Skystone Identified:", "False");
            telemetry.update();

        }

        return;

    }




    // check all the trackable targets to see which one (if any) is visible.
    // Provide feedback as to where the robot is located (if we know).







/*   public void SkystoneLocation(int location) {


       switch (location) {
           case 0:
               telemetry.addData("Skystone", "Not Detected");
               telemetry.update();
               break;
           case 1:
               telemetry.addData("Skystone: ", "Right");
               telemetry.update();

               break;
           case 2:
               telemetry.addData("Skystone: ", "Middle");
               telemetry.update();
               break;

           case 3:
               telemetry.addData("Skystone: ", "Left");
               telemetry.update();
               break;

       }
//*/
//    double initXaccel = gravity.yAccel;
//    double Xaccel;
//    double Yaccel = 0;
//    double Speed =  0;
//
//
//    double lastTime = 0;
//    double lastSpeed = 0;
//
//
//    double movedTank = 0;
//
//    double runTimes = 0;
//    double initialTime =runtime.seconds();

    void AcuTurn(double Degrees, boolean Clockwise)

    {
        Degrees = (Math.abs(Degrees));
        double moved =0;

        double lastHead;


        telemetry.update();
        double initHeading = angles.firstAngle;
        lastHead = angles.firstAngle;


        while(moved <= Degrees-3 && moved >= Degrees+3)
        {
            telemetry.addData("Test Data",Degrees- Math.abs(angles.firstAngle - initHeading)   );
            telemetry.addData("Test Data 2", Math.abs(angles.firstAngle - initHeading)   );
            telemetry.addData("Moved: ", moved);
            telemetry.addData("Degrees: ", Degrees);
            telemetry.addData("Velocity", imu.getVelocity());
            if (moved <= Degrees) {
                telemetry.addData("WHILE: ", true);

            }else{
                telemetry.addData("WHILE: ", false);
            }




            if(Clockwise == true)
            {
                if((Degrees-moved) >70)
                {
                    leftfr.setPower(.4);
                    leftback.setPower(.4);
                    rightfr.setPower(.4);
                    rightback.setPower(.4);
                }else {
                    if((Degrees-moved) >40)
                    {
                        leftfr.setPower(.3);
                        leftback.setPower(.3);
                        rightfr.setPower(.3);
                        rightback.setPower(.3);
                    }else{
                        if((Degrees-moved) >20)
                        {
                            leftfr.setPower(.2);
                            leftback.setPower(.2);
                            rightfr.setPower(.2);
                            rightback.setPower(.2);
                        }
                        if((Degrees-moved) >10)
                        {
                            leftfr.setPower(.155);
                            leftback.setPower(.155);
                            rightfr.setPower(.155);
                            rightback.setPower(.155);
                        }else{
                            if((Degrees-moved) >5)
                            {
                                leftfr.setPower(.15);
                                leftback.setPower(.15);
                                rightfr.setPower(.15);
                                rightback.setPower(.15);
                            }
                        }
                    }
                }

            }else{
                leftfr.setPower(-.2);
                leftback.setPower(-.2);
                rightfr.setPower(-.2);
                rightback.setPower(-.2);
            }


            moved = moved + (Math.abs( angles.firstAngle - lastHead));
            lastHead = angles.firstAngle;
            telemetry.update();

        }

        leftfr.setPower(0);
        leftback.setPower(0);
        rightfr.setPower(0);
        rightback.setPower(0);

        return;

    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}












