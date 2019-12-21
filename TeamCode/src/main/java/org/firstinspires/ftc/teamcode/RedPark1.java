
    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
    import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
    import com.qualcomm.hardware.rev.RevColorSensorV3;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.ColorSensor;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.I2cAddr;
    import com.qualcomm.robotcore.hardware.Servo;


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

    @Autonomous(name="RED Park1", group="Linear Opmode")
    public class RedPark1 extends LinearOpMode {
        DcMotor rightfDrive, rightbDrive, leftfDrive, leftbDrive;
        Servo leftHook, rightHook;
        int MOTOR_TICKS = 1680;
        int TURN_TICKS = 420;


        @Override
        public void runOpMode() throws InterruptedException {


            rightfDrive = hardwareMap.get(DcMotor.class, "rightf");
            rightbDrive = hardwareMap.get(DcMotor.class, "rightb");
            leftfDrive = hardwareMap.get(DcMotor.class, "leftf");
            leftbDrive = hardwareMap.get(DcMotor.class, "leftb");

            leftHook = hardwareMap.get(Servo.class, "leftHook");
            rightHook = hardwareMap.get(Servo.class, "rightHook");
            //                   MRcs1.resetDeviceConfigurationForOpMode();
            leftfDrive.setDirection(DcMotor.Direction.REVERSE);
            leftbDrive.setDirection(DcMotor.Direction.REVERSE);
            rightfDrive.setDirection(DcMotor.Direction.FORWARD);
            rightbDrive.setDirection(DcMotor.Direction.FORWARD);

            rightbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightHook.setDirection(Servo.Direction.REVERSE);
            leftHook.setDirection(Servo.Direction.FORWARD);

            rightHook.setPosition(0);
            leftHook.setPosition(0);


            waitForStart();

            while (opModeIsActive()) {


                //       telemetry.addData("MRcs1 name:",MRcs1.getI2cAddress());
                //        telemetry.addData("Connection: ", MRcs1.getConnectionInfo());

    /*            telemetry.addLine(" Value:");
                telemetry.addData("Red Value",REVcs1.red());
                telemetry.addData("Green Value",REVcs1.green());
                telemetry.addData("Blue Value",REVcs1.blue());
                telemetry.update();

    if (REVcs1.red()>450 && REVcs1.green()>300)
    {
    telemetry.addData("Yellow Stone Detected:","True");


    }
    else
    {

        telemetry.addData("ellow Stone Detected:","False");

    }
    */


      /*          rightbDrive.setTargetPosition(MOTOR_TICKS);
                rightbDrive.setPower(0.5);
                rightbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightfDrive.setPower(0.5);
                leftfDrive.setPower(0.5);
                leftbDrive.setPower(0.5);
                telemetry.addData("Right Motor Current: ", rightbDrive.getCurrentPosition());
                telemetry.addData("Right Motor Target: ", rightbDrive.getTargetPosition());
                telemetry.addData("Right Motor Ticks To Add: ", MOTOR_TICKS);
                telemetry.update();

                if (rightbDrive.getCurrentPosition() >= (MOTOR_TICKS - 20))
                {
                    rightbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightbDrive.setPower(0);
                    rightfDrive.setPower(0);
                    leftfDrive.setPower(0);
                    leftbDrive.setPower(0);
                    sleep(1000);

       */

                telemetry.addData("Right Motor Current: ", rightbDrive.getCurrentPosition());
                telemetry.addData("Right Motor Target: ", rightbDrive.getTargetPosition());
                telemetry.addData("Right Motor Ticks To Add: ", TURN_TICKS);
                telemetry.update();
                rightbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(2000);
                rightbDrive.setTargetPosition(-TURN_TICKS);
                rightbDrive.setPower(-0.5);
                    rightbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightfDrive.setPower(-0.5);
                    leftfDrive.setPower(0.5);
                    leftbDrive.setPower(0.5);

                    if (rightbDrive.getCurrentPosition()<=(-TURN_TICKS+20))
                    {
                        telemetry.addData("Setting to","0");
                        telemetry.update();
                        rightbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightbDrive.setPower(0);
                        rightfDrive.setPower(0);
                        leftfDrive.setPower(0);
                        leftbDrive.setPower(0);
                        telemetry.addData("Reached:","End");
                        telemetry.update();
                        sleep(1000);
                        stop();
                    }

                }


         /*           sleep(2000);
                    rightbDrive.setTargetPosition(MOTOR_TICKS2);
                    rightbDrive.setPower(0.5);
                    rightfDrive.setPower(-0.5);
                    leftfDrive.setPower(0.5);
                    leftbDrive.setPower(-0.5);
                    rightbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    if (!rightbDrive.isBusy()) {
                        leftfDrive.setPower(0);
                        leftbDrive.setPower(0);
                        rightfDrive.setPower(0);
                        rightbDrive.setPower(0);
                        telemetry.addData("Motors:", "Finished");
                        telemetry.update();
                    }

                    sleep(2000);
                    telemetry.addData("Motors: ", "Target Position Reached");
                    telemetry.update();
                    stop();
                }

*/
                }

            }












