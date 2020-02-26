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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;
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


@Autonomous(name="SKYSTONE Vuforia Position RED 1", group ="Concept")
public class VuforiaSkystonePositionRED1 extends LinearOpMode {

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry

    double initHeading;
    double runTime;


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
    //private CRServo rackPinionUD = null;
    //private CRServo rackPinionLR = null;

    private Servo rightHook = null;
    private Servo leftHook = null;

    public ModernRoboticsI2cRangeSensor rangeMRSensor;
    public ModernRoboticsI2cColorSensor colorMRSensor;

    private Servo autoHook;

    //public static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0,0,0);
    double leftfrPower;
    double leftbackPower;
    double rightfrPower;
    double rightbackPower;

    double leftPower;
    double rightPower;
    double rackPowerUD;
    double rackPowerLR;
    int location = 0;
    int location2;
    int MOTOR_TICKS = 700;
    int DELIVERY = 2000;
    int SECONDSKYSTONE1;
    int SECONDSKYSTONE2;
    int SECONDSKSTONE3;
    int GOFORWARD = 300;

    int STRAFE_TICKS = 400;
    int TOSKYSTONE = 550;

    boolean first = true;

    double newInitTime;
    double firstHeading;

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


    @Override
    public void runOpMode() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters();

        parametersV.vuforiaLicenseKey = VUFORIA_KEY;
        parametersV.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parametersV);

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
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parametersV.cameraDirection);
        }


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parametersV
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

        autoHook = hardwareMap.get(Servo.class, "autoHook");



//        rangeMRSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeMRSensor");
//        colorMRSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorMRSensor");

        CameraDevice.getInstance().setFlashTorchMode(true);


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
        rightWheels.setPower(0);
        leftWheels.setPower(0);

        autoHook.setPosition(0);
        double initTime = 0;


        ElapsedTime InitTimer = new ElapsedTime();

        InitTimer.reset();
        InitTimer.startTime();
        while (InitTimer.seconds() < 45 && !isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Time In Seconds:", InitTimer.seconds());
            telemetry.update();
        }
        telemetry.addData("Ready to Start!", " ");
        telemetry.update();
        InitTimer.reset();


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

            telemetry.update();
            firstHeading = angles.firstAngle;
            EncoderMove(rightback, rightfr, leftfr, leftback);
            ElapsedTime LoopTimer = new ElapsedTime();
            LoopTimer.reset();
            LoopTimer.startTime();
            while (LoopTimer.seconds() < 1) {
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Skystone:", "Visible Position 6");
                        telemetry.update();
                        skystoneVisible = true;
                        location = 6;
                        location2 = 3;
                        boolean counterClockwise = true;
                        targetsSkyStone.deactivate();
                        MovingtoSkystoneAndBack(rightback, rightfr, leftfr, leftback,0);
                        sleep(300);
                        AcuTurn(-90, true, true);
                        //Distance Sensor code
//while((rangeMRSensor.getDistance(DistanceUnit.INCH)<=24))
//{
//    leftfr.setPower(.2);
//    leftback.setPower(-.2);
//    rightfr.setPower(-.2);
//    rightback.setPower(.2);
//}                        //
                        DeliverySkystone(rightback, rightfr, leftfr, leftback, location);
//                        GoingToWall(rightback, rightfr, leftfr, leftback, location2);
//                        AcuTurnOpposite(80,true);
//
//                        EncoderStrafe2nd(rightback, rightfr, leftfr, leftback, location,.4, location2);
//                        MovingtoSkystoneAndBack(rightback, rightfr, leftfr, leftback, location2);
//                        AcuTurn(-90, true);
//                        DeliverySkystone2(rightback, rightfr, leftfr, leftback, location2);
                        sleep(2000);
                        Parking(rightback, rightfr, leftfr, leftback, location);

                        // AcuTurn(90,false);
                        // DeliverySkystone2 (rightback, rightfr, leftfr, leftback, location2);
                        // Parking(rightback, rightfr, leftfr, leftback);
                        stop();
                        break;
                    }
                }

                // Provide feedback as to where the robot is located (if we know).

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible
            }
            // Provide feedback as to where the robot is located (if we know).
//        while(skystoneVisible) {
//                // express position (translation) of robot in inches.
//                VectorF translation = lastLocation.getTranslation();
//                telemetry.addData("Position",translation.get(0)/mmPerInch);
//                // express the rotation of the robot in degrees.
//
//            telemetry.update();
//        }


            if (skystoneVisible == false) {
                LoopTimer.reset();
                telemetry.addData("Not in Position 1", " ");
                telemetry.update();
                runTime=0.8;
                EncoderStrafe(rightback, rightfr, leftfr, leftback, initTime, runTime);
                ElapsedTime LoopTimer2 = new ElapsedTime();
                LoopTimer2.reset();
                LoopTimer2.startTime();
                while (LoopTimer2.seconds() < 1) {
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
//                            if (robotLocationTransform != null) {
//                                lastLocation = robotLocationTransform;
//                            }
                            telemetry.addData("Skystone:", "Visible Position 5");
                            telemetry.update();
                            skystoneVisible = true;
                            location = 5;
                            location2 = 2;
                            boolean counterClockwise = true;
                            MovingtoSkystoneAndBack(rightback, rightfr, leftfr, leftback, 0);
                            AcuTurn(-90, true, true);
                            //Distance Sensor code

                            //
                            DeliverySkystone(rightback, rightfr, leftfr, leftback, location);
                            autoHook.setPosition(0.1);
                            //tmove(.2,0,0,.35,0);
//                            GoingToWall(rightback, rightfr, leftfr, leftback, location2);
//
//                            AcuTurnOpposite(80, true);
//                            runTime=0.46;
//                            EncoderStrafe2nd(rightback, rightfr, leftfr, leftback, location,runTime, location2);
//
//                            sleep(200);
//                            MovingtoSkystoneAndBack(rightback, rightfr, leftfr, leftback, location2);
//
//                            tmove(.07, runtime.seconds(), 0,0,.8);
//
//                            AcuTurn(-90, true);
//                            DeliverySkystone2(rightback, rightfr, leftfr, leftback, location2);
                            sleep(2000);
                            Parking(rightback, rightfr, leftfr, leftback, location);

                            //  DeliverySkystone2 (rightback, rightfr, leftfr, leftback, location2);
//                            Parking(rightback, rightfr, leftfr, leftback);
                            stop(); //place pickup 1 here
                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            break;
                        }
                    }
                }
            }

            if (skystoneVisible == false) {
                initTime = 0;
                runTime=0.8;
                EncoderStrafe(rightback, rightfr, leftfr, leftback, initTime, runTime);
                telemetry.addData("Skystone:", "Assumed Position 4");
                telemetry.update();
                skystoneVisible = true;
                location = 4;
                location2 = 1;
                boolean counterClockwise = true;
                targetsSkyStone.deactivate();
                MovingtoSkystoneAndBack(rightback, rightfr, leftfr, leftback, 0);
                AcuTurn(-92, true, true);
                DeliverySkystone(rightback, rightfr, leftfr, leftback, location);
//                GoingToWall(rightback, rightfr, leftfr, leftback, location2);
//                AcuTurnOpposite(80,true);
//                EncoderStrafe2nd(rightback, rightfr, leftfr, leftback, initTime, 1, location2);
//                tmove(.3,runtime.seconds(),0,.8,0);
//
//                MovingtoSkystoneAndBack(rightback, rightfr, leftfr, leftback, location2);
//                tmove(.3,runtime.seconds(),0,.8,0);
//
//                AcuTurn(88,true);
//                DeliverySkystone2(rightback, rightfr, leftfr, leftback, location2);



             //    DeliverySkystone2(rightback, rightfr, leftfr, leftback, location2);
                sleep(2000);
                Parking(rightback, rightfr, leftfr, leftback, location);
                stop();
            }

        }


    }

//up till here




        /*LoopTimer.startTime();
                telemetry.addLine("Scanning Position 1");

                /*

                }
            }

                /*
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        telemetry.update();
                   /*     if (trackable.getName().equals("Stone Target")) {
                            telemetry.addLine("Skystone is Position 1");
                            skystoneVisible = true;
                            location = 1;
                            //         SkystoneLocation(location);
                            sleep(2000);
                            stop();  //place pickup for 1 here
                        }

                    */

                   /* else {
                        skystoneVisible = false;
                        telemetry.addLine("Skystone Not 1");
                        telemetry.update();
                    }
                }
                /*
            stop();
            LoopTimer.reset();
            sleep(2000);
            EncoderStrafe(rightback, rightfr, leftfr, leftback);
            LoopTimer.startTime();
            while (LoopTimer.seconds() < 2) {
                telemetry.addLine("Scannng Position 2");
                for (VuforiaTrackable trackable : allTrackables) {
                    if (trackable.getName().equals("Stone Target")) {
                        telemetry.addLine("Skystone is Position 2");
                        skystoneVisible = true;
                        location = 2;
                        //     SkystoneLocation(location);
                        sleep(2000);
                        stop(); //place pickup for 2 here
                    } else {
                        skystoneVisible = false;
                        telemetry.addLine("Skystone: " + "Not 2");
                    }
                }
            }

            LoopTimer.reset();
            EncoderStrafe(rightback, rightfr, leftfr, leftback);
            LoopTimer.startTime();
            while (LoopTimer.seconds() < 2) {
                telemetry.addLine("Scannng Position 3");
                for (VuforiaTrackable trackable : allTrackables) {
                    if (trackable.getName().equals("Stone Target")) {
                        telemetry.addLine("Skystone is Position 3");
                        skystoneVisible = true;
                        location = 3;
                        //  SkystoneLocation(location);
                        sleep(2000);
                        stop(); //place pickup for 3 here
                    } else {
                        telemetry.addLine("Skystone Not Detected: Backup Route");
                    }
                }

                 */


    // }
    // }

    public void tmove(double runSeconds, double presentRuntime, double turnPower, double straifPower, double tankPower)

    {

        while(runtime.seconds() < (runSeconds+presentRuntime))
        {
            bMove(turnPower, straifPower, tankPower);

        }
    }

    public void bMove(double turnPower, double straifPower, double tankPower )
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


    }


    public void GoingToWall(DcMotor rightback, DcMotor rightfr, DcMotor leftfr, DcMotor leftback, double location2) {




        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int move = 0;

        if(location2 == 3)
        {
            move = 3200;
        }
        if(location2 == 2)
        {
            move = 2800;
        }
        if (location2 == 1)
        {
            move = 3150;
        }






        rightback.setTargetPosition(rightback.getCurrentPosition() - move);
        rightback.setPower(-0.50);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfr.setPower(-0.50);
        leftfr.setPower(-0.50);
        leftback.setPower(-0.65);
        ElapsedTime tempTimer = new ElapsedTime();
        tempTimer.reset();
        while (rightback.isBusy()) {
            if (rightback.getCurrentPosition() <= (rightback.getTargetPosition() + 15)) {
                leftback.setPower(0);
                rightback.setPower(0);
                rightfr.setPower(0);
                leftfr.setPower(0);
                leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                sleep(200);
                rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightback.setTargetPosition(rightback.getCurrentPosition() + GOFORWARD);
                rightback.setPower(0.50);
                rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightfr.setPower(0.50);
                leftfr.setPower(0.50);
                leftback.setPower(0.65);


                while (rightback.isBusy() && tempTimer.seconds() > 2) {
                    if (rightback.getCurrentPosition() <= (rightback.getTargetPosition() + 15)) {
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
    }


    public void Parking (DcMotor rightback, DcMotor rightfr, DcMotor leftfr, DcMotor leftback, int location)
    {

        int back = 0;
        if(location == 1)
        {
            back = 100;
        }
        telemetry.addData("Parking", " ");
        telemetry.update();
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setTargetPosition(rightback.getCurrentPosition() - 500 - back) ;
        rightback.setPower(0.50);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfr.setPower(-0.50);
        leftfr.setPower(-0.50);
        leftback.setPower(-0.65);
        while (rightback.isBusy()) {
            if (rightback.getCurrentPosition() <= (rightback.getTargetPosition() + 15)) {
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

    public void DeliverySkystone2 (DcMotor rightback, DcMotor rightfr, DcMotor leftfr, DcMotor
            leftback,int location2)
    {
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setTargetPosition(rightback.getCurrentPosition() + DELIVERY + 500);
        rightback.setPower(0.80);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfr.setPower(0.80);
        leftfr.setPower(0.80);
        leftback.setPower(0.9);
        if(location2 == 1)
        {
            rightback.setTargetPosition(rightback.getCurrentPosition() - DELIVERY - 500);

        }
        else{
            rightback.setTargetPosition(rightback.getCurrentPosition() + DELIVERY + 500);

        }
        while (rightback.isBusy()) {

            if (rightback.getCurrentPosition() >= (rightback.getTargetPosition() + 15)) {
                leftback.setPower(0);
                rightback.setPower(0);
                rightfr.setPower(0);
                leftfr.setPower(0);
                leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addData("Delivered", " ");
                telemetry.update();
                return;

            }
        }
        autoHook.setPosition(0.1);
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);




    }
    public void DeliverySkystone (DcMotor rightback, DcMotor rightfr, DcMotor leftfr, DcMotor
            leftback,int location)
    {

        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        switch (location)
        {
            case 6:
                rightback.setTargetPosition(rightback.getCurrentPosition() + (DELIVERY +30));
                break;
            case 5:
                rightback.setTargetPosition(rightback.getCurrentPosition() + DELIVERY);
                break;
            case 4:
                rightback.setTargetPosition(rightback.getCurrentPosition() + (DELIVERY +150)) ;
                break;
        }

        rightback.setPower(0.80);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfr.setPower(0.80);
        leftfr.setPower(0.80);
        leftback.setPower(0.8);
        while (rightback.isBusy()) {

            if (rightback.getCurrentPosition() >= (rightback.getTargetPosition() - 15)) {
                leftback.setPower(0);
                rightback.setPower(0);
                rightfr.setPower(0);
                leftfr.setPower(0);
                leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                autoHook.setPosition(0.1);
                telemetry.addData("Delivered", " ");
                telemetry.update();
                return;

            }
        }

    }//
    public void EncoderMove (DcMotor rightback, DcMotor rightfr, DcMotor leftfr, DcMotor
            leftback){

        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setTargetPosition(rightback.getCurrentPosition() + MOTOR_TICKS);
        rightback.setPower(0.45);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfr.setPower(0.45);
        leftfr.setPower(0.45);
        leftback.setPower(0.45);


        while (rightback.isBusy()) {

            if (rightback.getCurrentPosition() >= (rightback.getTargetPosition() - 400)) {
                rightback.setPower(0.25);
                rightfr.setPower(0.25);
                leftfr.setPower(0.25);
                leftback.setPower(0.35);


                if (rightback.getCurrentPosition() >= (rightback.getTargetPosition() - 15)) {
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

    public void MovingtoSkystoneAndBack (DcMotor rightback, DcMotor rightfr, DcMotor
            leftfr, DcMotor leftback, double location2){
        newInitTime = runtime.seconds();
        int back = 0;
        int forward = 0;
//        while ((runtime.seconds() - newInitTime) < 1.2) {
//            rightback.setPower(-0.3);
//            rightfr.setPower(0.3);
//            leftfr.setPower(-0.3);
//            leftback.setPower(0.45);
//            telemetry.addData("Strafing just for Position 1", " ");
//            telemetry.update();
//        }

        if(location2 == 2)
        {
            back = -50;
        }

        if (location2 == 1)
        {
            forward = -150;
            back = -250;
        }
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setTargetPosition(rightback.getCurrentPosition() + TOSKYSTONE + forward);
        rightback.setPower(0.3);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfr.setPower(0.3);
        leftfr.setPower(0.3);
        leftback.setPower(0.4);
        telemetry.addData("Going forward to Position 1", " ");
        telemetry.update();


        while (rightback.isBusy()) {

            if (rightback.getCurrentPosition() >= (rightback.getTargetPosition() - 15)) {
                leftback.setPower(0);
                rightback.setPower(0);
                rightfr.setPower(0);
                leftfr.setPower(0);
                leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addData("Stopping at Position 1", " ");
                telemetry.update();
//        while (sensorDistance.getDistance(DistanceUnit.CM) > 7 || (Double.toString(sensorDistance.getDistance(DistanceUnit.CM))=="NaN")) {
//            rightback.setPower(0.2);
//            rightfr.setPower(0.2);
//            leftfr.setPower(0.2);
//            leftback.setPower(0.25);
//            telemetry.addData("Getting Distance", sensorDistance.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }

                autoHook.setPosition(1);
                sleep(500);

                rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(1000);
                rightback.setTargetPosition(-TOSKYSTONE + back);
                rightback.setPower(0.2);
                rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightfr.setPower(-0.2);
                leftfr.setPower(-0.2);
                leftback.setPower(-0.2);
                while (rightback.isBusy()) {
                    if (rightback.getCurrentPosition() <= (rightback.getTargetPosition() + 15)) {
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


//        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightback.setTargetPosition(rightback.getCurrentPosition() + TOSKYSTONE);
//        rightback.setPower(0.2);
//        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightfr.setPower(0.2);
//        leftfr.setPower(0.2);
//        leftback.setPower(0.2);
//        telemetry.addData("Going forward to Position 1", " ");
//        telemetry.update();
//
//
//        while (rightback.isBusy()) {
//
//            if (rightback.getCurrentPosition() >= (rightback.getTargetPosition() - 15)) {
//                leftback.setPower(0);
//                rightback.setPower(0);
//                rightfr.setPower(0);
//                leftfr.setPower(0);
//                leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                telemetry.addData("Stopping at Position 1", " ");
//                telemetry.update();
//                rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                autoHook.setPosition(1);
//                sleep(1000);
//                rightback.setTargetPosition(-TOSKYSTONE);
//                rightback.setPower(0.2);
//                rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightfr.setPower(-0.2);
//                leftfr.setPower(-0.2);
//                leftback.setPower(-0.2);
//                while (rightback.isBusy()) {
//                    if (rightback.getCurrentPosition() <= (rightback.getTargetPosition() + 15)) {
//                        leftback.setPower(0);
//                        rightback.setPower(0);
//                        rightfr.setPower(0);
//                        leftfr.setPower(0);
//                        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                        rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                        leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        return;
//                    }


    }
    public void EncoderStrafe2nd(DcMotor rightback, DcMotor rightfr, DcMotor leftfr, DcMotor leftback,double initTime, double runTime, int location2)
    {
        initTime = runtime.seconds();
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(location2==3)
        {
            while ((runtime.seconds() - initTime) < runTime) {
                rightback.setPower(0.4);
                rightfr.setPower(-0.4);
                leftfr.setPower(0.4);
                leftback.setPower(-0.6);
                telemetry.addData("Strafing", " ");
                telemetry.update();
            }
            rightback.setPower(0);
            rightfr.setPower(0);
            leftfr.setPower(0);
            leftback.setPower(0);
        }
        if ( location2==2)
        {
            while ((runtime.seconds() - initTime) < runTime) {
                rightback.setPower(-0.4);
                rightfr.setPower(0.4);
                leftfr.setPower(-0.4);
                leftback.setPower(0.6);
                telemetry.addData("Strafing", " ");
                telemetry.update();
            }
            rightback.setPower(0);
            rightfr.setPower(0);
            leftfr.setPower(0);
            leftback.setPower(0);
        }


        if (location2==1)
        {

            while ((runtime.seconds() - initTime) < runTime) {
                rightback.setPower(-0.4);
                rightfr.setPower(0.4);
                leftfr.setPower(-0.4);
                leftback.setPower(0.6);
                telemetry.addData("Strafing", " ");
                telemetry.update();
            }
            rightback.setPower(0);
            rightfr.setPower(0);
            leftfr.setPower(0);
            leftback.setPower(0);
        }

    }



    public void EncoderStrafe (DcMotor rightback, DcMotor rightfr, DcMotor leftfr, DcMotor leftback,double initTime, double runTime){

        initTime = runtime.seconds();
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((runtime.seconds() - initTime) < runTime) {
            rightback.setPower(-0.4);
            rightfr.setPower(0.4);
            leftfr.setPower(-0.4);
            leftback.setPower(0.6);
            telemetry.addData("Strafing", " ");
            telemetry.update();
        }
        rightback.setPower(0);
        rightfr.setPower(0);
        leftfr.setPower(0);
        leftback.setPower(0);


        return;


    }






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


    public void targetTelemetry ( boolean targetVisible){
        if (targetVisible) {
            telemetry.addData("Skystone Identified:", "True");
            telemetry.update();

        } else {
            telemetry.addData("Skystone Identified:", "False");
            telemetry.update();

        }

        return;

    }


    public static void roundAndPrint ( double n, int round2DecimalPlace){
        String temp;
        BigDecimal instance = new BigDecimal(Double.toString(n));
        instance = instance.setScale(round2DecimalPlace, RoundingMode.HALF_UP);

    }

    public void AcuTurnOpposMOD (double Degrees, boolean counterClockwise) {
        Degrees = (Math.abs(Degrees));
        double moved = 0;

        double lastHead;


        telemetry.addData("Test Uday", true);
        telemetry.update();
        initHeading = angles.firstAngle;
        lastHead = angles.firstAngle;
        telemetry.addData("TestSTATEMENT:", moved != Degrees);
        telemetry.addData("Heading: ", angles.firstAngle);

        while (moved < (Degrees - 3)) {
            rightback.setDirection(DcMotor.Direction.FORWARD);
            rightfr.setDirection(DcMotor.Direction.FORWARD);
            leftfr.setDirection(DcMotor.Direction.FORWARD);
            leftback.setDirection(DcMotor.Direction.FORWARD);

            telemetry.addData("Test Data", Degrees - Math.abs(angles.firstAngle - initHeading));
            telemetry.addData("Test Data 2", Math.abs(angles.firstAngle - initHeading));
            telemetry.addData("Moved: ", moved);
            telemetry.addData("Degrees: ", Degrees);
            telemetry.addData("Velocity", imu.getVelocity());


            if (counterClockwise == true) {
                if ((Degrees - moved) > 70) {
                    leftfr.setPower(.3);
                    leftback.setPower(.3);
                    rightfr.setPower(.67);
                    rightback.setPower(.67);
                } else {
                    if ((Degrees - moved) > 40) {
                        leftfr.setPower(.2625);
                        leftback.setPower(.2625);
                        rightfr.setPower(.4975);
                        rightback.setPower(.4975);
                    } else {
                        if ((Degrees - moved) > 30) {
                            leftfr.setPower(.1775);
                            leftback.setPower(.1775);
                            rightfr.setPower(.3425);
                            rightback.setPower(.3425);
                        }

                        if ((Degrees - moved) > 10) {
                            leftfr.setPower(.2);
                            leftback.setPower(.2);
                            rightfr.setPower(.3);
                            rightback.setPower(.3);
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
        sleep(500);
        telemetry.addData("Setting Power to 0", true);
        telemetry.update();
        leftfr.setPower(0);
        leftback.setPower(0);
        rightfr.setPower(0);
        rightback.setPower(0);
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);

    }/////////////////







    public void AcuTurnOpposite (double Degrees, boolean counterClockwise) {
        Degrees = (Math.abs(Degrees));
        double moved = 0;

        double lastHead;


        telemetry.addData("Test Uday", true);
        telemetry.update();
        initHeading = angles.firstAngle;
        lastHead = angles.firstAngle;
        telemetry.addData("TestSTATEMENT:", moved != Degrees);
        telemetry.addData("Heading: ", angles.firstAngle);

        while (moved < (Degrees - 3)) {
            rightback.setDirection(DcMotor.Direction.FORWARD);
            rightfr.setDirection(DcMotor.Direction.FORWARD);
            leftfr.setDirection(DcMotor.Direction.FORWARD);
            leftback.setDirection(DcMotor.Direction.FORWARD);

            telemetry.addData("Test Data", Degrees - Math.abs(angles.firstAngle - initHeading));
            telemetry.addData("Test Data 2", Math.abs(angles.firstAngle - initHeading));
            telemetry.addData("Moved: ", moved);
            telemetry.addData("Degrees: ", Degrees);
            telemetry.addData("Velocity", imu.getVelocity());


            if (counterClockwise == true) {
                if ((Degrees - moved) > 70) {
                    leftfr.setPower(.45);
                    leftback.setPower(.45);
                    rightfr.setPower(.45);
                    rightback.setPower(.45);
                } else {
                    if ((Degrees - moved) > 40) {
                        leftfr.setPower(.35);
                        leftback.setPower(.35);
                        rightfr.setPower(.35);
                        rightback.setPower(.35);
                    } else {
                        if ((Degrees - moved) > 30) {
                            leftfr.setPower(.25);
                            leftback.setPower(.25);
                            rightfr.setPower(.25);
                            rightback.setPower(.25);
                        }

                        if ((Degrees - moved) > 10) {
                            leftfr.setPower(.2);
                            leftback.setPower(.2);
                            rightfr.setPower(.2);
                            rightback.setPower(.2);
                        } else {
                            if ((Degrees - moved) > 5) {
                                leftfr.setPower(.2);
                                leftback.setPower(.2);
                                rightfr.setPower(.2);
                                rightback.setPower(.2);
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
        sleep(500);
        telemetry.addData("Setting Power to 0", true);
        telemetry.update();
        leftfr.setPower(0);
        leftback.setPower(0);
        rightfr.setPower(0);
        rightback.setPower(0);
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);

    }

    public void AcuTurn ( double Degrees, boolean Clockwise, boolean First) {
        Degrees = (Math.abs(Degrees));
        double moved = 0;

        double lastHead;


        telemetry.addData("Test Uday", true);
        telemetry.update();
        initHeading = angles.firstAngle;

        if(First == true)
        {
           moved = -firstHeading + (angles.firstAngle) ;
        }
        lastHead = angles.firstAngle;
        telemetry.addData("TestSTATEMENT:", moved != Degrees);
        telemetry.addData("Heading: ", angles.firstAngle);

        while (moved < (Degrees)) {
            rightback.setDirection(DcMotor.Direction.REVERSE);
            rightfr.setDirection(DcMotor.Direction.REVERSE);
            leftfr.setDirection(DcMotor.Direction.REVERSE);
            leftback.setDirection(DcMotor.Direction.REVERSE);

            telemetry.addData("Test Data", Degrees - Math.abs(angles.firstAngle - initHeading));
            telemetry.addData("Test Data 2", Math.abs(angles.firstAngle - initHeading));
            telemetry.addData("Moved: ", moved);
            telemetry.addData("Degrees: ", Degrees);
            telemetry.addData("Velocity", imu.getVelocity());


            if (Clockwise == true) {
                if ((Degrees - moved) > 70) {
                    leftfr.setPower(.45);
                    leftback.setPower(.45);
                    rightfr.setPower(.45);
                    rightback.setPower(.45);
                } else {
                    if ((Degrees - moved) > 40) {
                        leftfr.setPower(.35);
                        leftback.setPower(.35);
                        rightfr.setPower(.35);
                        rightback.setPower(.35);
                    } else {
                        if ((Degrees - moved) > 30) {
                            leftfr.setPower(.25);
                            leftback.setPower(.25);
                            rightfr.setPower(.25);
                            rightback.setPower(.25);
                        }

                        if ((Degrees - moved) > 10) {
                            leftfr.setPower(.2);
                            leftback.setPower(.2);
                            rightfr.setPower(.2);
                            rightback.setPower(.2);
                        } else {
                            if ((Degrees - moved) > 5) {
                                leftfr.setPower(.2);
                                leftback.setPower(.2);
                                rightfr.setPower(.2);
                                rightback.setPower(.2);
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
        sleep(500);
        telemetry.addData("Setting Power to 0", true);
        telemetry.update();
        leftfr.setPower(0);
        leftback.setPower(0);
        rightfr.setPower(0);
        rightback.setPower(0);
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);


        return;

    }








    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    public  void composeTelemetry () {

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

    String formatAngle (AngleUnit angleUnit,double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees ( double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

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
*/












