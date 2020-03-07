package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name= "TeleOp UPDATE", group= "Linear Opmode")

public class TeleOpMode extends LinearOpMode {
//@Disabled


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfr = null;
    private DcMotor leftback = null;
    private DcMotor rightfr = null;
    private DcMotor rightback = null;

    private CRServo leftWheels = null;
    private CRServo rightWheels = null;
    private DcMotor intakeArm = null;
    double lastEncoderHold;
    int intakeArmCurrentPosition;
    boolean needsToRun =false;





    private DistanceSensor sensorRange;

    double leftfrPower;
    double leftbackPower;
    double rightfrPower;
    double rightbackPower;

    double leftPower;
    double rightPower;
    double intakeArmPower;
    double intakeArmSlowPower;
    double intakeArmCounter = 0;


    boolean capDown;
    boolean intakeArmMode = true;
    boolean intakeArmSlowMode = false;
    double time;


    double lastSlow = 0;
    double lastCap = 0;

    boolean zeroBrake = true;
    double lastBrake = 0;

    double lastAutoHook =0;

    private Servo leftHook;
    private Servo rightHook;

    private Servo autoHook;
    double side;

    boolean first = true;
    Servo newCap;

    //CRServo capStone;

    boolean rhook;
    boolean lhook;
    int initialEncoder;

    double lastrhook = 0;
    double lastlhook = 0;


    double lastTank = 0;
    boolean Tank = false;

    boolean slowMode = false;
    int counter=0;
    int currentPosition;
    int c=0;


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

        leftWheels = hardwareMap.get(CRServo.class, "lw");   //leftwheels
        rightWheels = hardwareMap.get(CRServo.class, "rw");  //rightwheels
        //   capStone = hardwareMap.get(CRServo.class, "Cap");  // Rack and Pinion Vertical
        rightHook = hardwareMap.get(Servo.class, "rightHook");
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        newCap = hardwareMap.get(Servo.class, "NewCap");
        intakeArm = hardwareMap.get(DcMotor.class, "intakeArm");   //Rack and Pinion Horizontal


        autoHook = hardwareMap.get(Servo.class,"autoHook");
        //comment

        rightHook.setDirection(Servo.Direction.REVERSE);
        leftHook.setDirection(Servo.Direction.FORWARD);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        leftWheels.setDirection(CRServo.Direction.REVERSE);
        rightWheels.setDirection(CRServo.Direction.FORWARD);

        intakeArm.setDirection(DcMotor.Direction.FORWARD);

        autoHook.setPosition(0);
        newCap.setPosition(0);

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime loopTimer2 = new ElapsedTime();
        rightWheels.setPower(0);
        leftWheels.setPower(0);





       /*
       public void pickUp{
           leftPower= 0.5;
           rightPower= -0.5;

       }
       public void putDown{

       } */

// Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double straifPower = gamepad1.right_stick_x;
            double tankPower = gamepad1.right_stick_y;
            double turnPower = gamepad1.left_stick_x;

            if (first == true) {
                rightHook.setPosition(0);
                leftHook.setPosition(0);
                slowMode = false;
            }


            first = false;

            // Zero Brake Code
            if (gamepad1.b == true) {
                if ((runtime.seconds() - lastBrake) > .5)
                { //slow mode threshold

                    if (zeroBrake == true) {
                        zeroBrake = false;
                        lastBrake = runtime.seconds();
                    } else {
                        zeroBrake = true;
                        lastBrake = runtime.seconds();
                    }

                }

            }

            if (gamepad2.right_bumper == true || gamepad1.right_bumper == true) {
                if ((runtime.seconds() - lastrhook) > .5) { //slow mode threshold

                    if (rhook == true) {
                        rhook = false;
                        lastrhook = runtime.seconds();
                    } else {
                        rhook = true;
                        lastrhook = runtime.seconds();
                    }
                }

            }

            if (gamepad2.left_bumper == true || gamepad1.left_bumper == true) {
                if ((runtime.seconds() - lastlhook) > .5) { //slow mode threshold

                    if (lhook == true) {
                        lhook = false;
                        lastlhook = runtime.seconds();
                    } else {
                        lhook = true;
                        lastlhook = runtime.seconds();
                    }

                }


            }


            //Slowmode Code

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



            //zero brake
            if (gamepad2.x) {
                newCap.setPosition(1);
            }
            if (gamepad2.y) {
                newCap.setPosition(0);
            }

            if(counter==0 && gamepad2.left_trigger>0.5 && runtime.seconds()-lastAutoHook>0.75)
            {
                counter=1;
                autoHook.setPosition(1);
                lastAutoHook=runtime.seconds();

            }
            if(counter==1 && gamepad2.left_trigger>0.5 && runtime.seconds()-lastAutoHook>0.75)
            {
                counter=0;
                autoHook.setPosition(0);
                lastAutoHook=runtime.seconds();
            }


            if(gamepad2.b)
            {
                leftPower=0.75;
                rightPower=0.75;
            }
            if(gamepad2.a) {
                leftPower = -0.75;
                rightPower = -0.75;
            }
            if(!gamepad2.a&&!gamepad2.b)
            {
                leftPower=0;
                rightPower=0;
            }

            side = gamepad2.left_stick_y;
            intakeArmSlowPower = gamepad2.right_stick_y;
//

//          if(gamepad2.dpad_down)
//          {
//              if(intakeArmCounter==1) {
//                  if(intakeArmSlowMode)
//                  {
//                      intakeArmMode = true;
//                      intakeArmSlowMode=false;
//                      intakeArmCounter=0;
//                  }
//              }
//              else if(intakeArmCounter==0)
//              {
//                  if(intakeArmMode)
//                  {
//                      intakeArmSlowMode=true;
//                      intakeArmMode=false;
//                      intakeArmCounter=1;
//                  }
//              }
//          }
//
//if(intakeArmMode)
//{
    if (side < -0.1)
    {
        intakeArmPower=-1;
    }
    else if (side > 0.1)
    {
        intakeArmPower=1;
    }
//
//}
//if (intakeArmSlowMode)
//{
    if(intakeArmSlowPower<-0.1)
    {
        intakeArmSlowPower=-0.5;

    }
    if (intakeArmSlowPower>0.1)
    {
        intakeArmSlowPower=0.5;
    }




if((!(intakeArmSlowPower>0 || intakeArmSlowPower<0)) && (!(side>0 || side<0)))
            {
                intakeArmSlowPower=0;
                intakeArmPower=0;
            }


//            if (gamepad2.right_trigger > 0.3 && runtime.seconds() - lastEncoderHold > 1 )
//            {
//                if(c==1)
//                {
//                    c=0;
//                }
//                else if(c==0)
//                {
//                    c=1;
//                    initialEncoder=intakeArm.getCurrentPosition();
//                }
//                lastEncoderHold=runtime.seconds();
//            }
//
//            if(c==1)
//            {
//                intakeArmSlowPower=0;
//                intakeArmPower=0;
//                intakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                intakeArm.setTargetPosition(initialEncoder);
//                if(intakeArm.getCurrentPosition()-5<=initialEncoder || intakeArm.getCurrentPosition()+5>=initialEncoder)
//                {
















        /*    if (gamepad2.x== true) {
                    if ((runtime.seconds() - lastCap) > .5) { //slow mode threshold

                        if (down == true) {

                            down = false;
                            lastCap = runtime.seconds();
                        } else {

                            down = true;
                            lastCap = runtime.seconds();
                        }

             */





        /*   if(gamepad2.x == true)
            {
                if ((runtime.seconds()-lastTank) > .5)
                {
                    if(Tank == false)
                    {
                        cap = true;
                    }
                    else
                    {
                        cap = false;
                    }
                }
            }
*/
            if (zeroBrake == true) {
                leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Makes motors brake when set to 0
                leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            } else {
                leftfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //Makes motors brake when set to 0
                leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            if (slowMode == true) {
                straifPower = straifPower * .5;
                tankPower = tankPower * .5;
                turnPower = turnPower * .5;
            }
            if (lhook == true) {
                leftHook.setPosition(1);

            } else {
                leftHook.setPosition(0);
            }

            if (rhook == true) {
                rightHook.setPosition(1);
            } else {
                rightHook.setPosition(0);
            }



     /*      if (gamepad2.x == true)
            {
                capStone.setPower(1);
            }

      */


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

            //pickup mechanism


            //    capStone.setPower(0);

       /*     if(gamepad2.a == true)
            {
                capStone.setPower(1);
            }


            if(gamepad2.b == true)
            {
                capStone.setPower(-1);
            }
*/







      /*      if (gamepad2.a)   //braki
      /*      if (gamepad2.a)   //braking of intakeArm
            {
                if(counter==1)  //braking on
                {
                    intakeArm.set

                }


                if(counter==2) //braking off
                {

                 counter=1; //

                }

            }
            */

       /*    if (leftPower>0&&rightPower>0) {
                loopTimer2.startTime();
                while (loopTimer2.seconds() <= 1) {
                    telemetry.addData("Only Running Outake"," ");
                    telemetry.update();
                    intakeArmPower=0;
                }
                loopTimer2.reset();
                loopTimer2.startTime();
                while(loopTimer2.seconds()<=1)
                {
                    intakeArmPower = 0.3;
                    telemetry.addData("Setting Power for Intake Arm"," ");
                    telemetry.update();
                }
                loopTimer2.reset();

            }
*/



            // Send calculated power to wheels
            leftfr.setPower(leftfrPower);
            leftback.setPower(leftbackPower);
            rightfr.setPower(rightfrPower);
            rightback.setPower(rightbackPower);

            //intake power
            leftWheels.setPower(leftPower);
            rightWheels.setPower(rightPower);

            //arm movement power
            intakeArm.setPower(intakeArmPower);
            intakeArm.setPower(intakeArmSlowPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left fr (%.2f), right fr (%.2f), left back (%.2f), right back (%.2f)", leftfrPower, rightfrPower, leftbackPower, rightbackPower);
            telemetry.addData("Servos", "left wheels (%.2f), right wheel (%.2f)", leftPower, rightPower);
            telemetry.addData("lhook", lhook);
            telemetry.addData("rhook", rhook);
            telemetry.addData("A: ", gamepad2.a);
            telemetry.addData("B:", gamepad2.b);
            telemetry.addData("Left Power from Servo", leftWheels.getPower());
            telemetry.addData("Right Power from Servo", rightWheels.getPower());
            telemetry.addData("Port Number", rightWheels.getPortNumber());
            telemetry.addData("Port Number", leftWheels.getPortNumber());
            telemetry.addData("IntakeArmSlowPower Variable", intakeArmSlowPower);
            telemetry.addData("Intake Arm Variable Power: ", intakeArmPower);
            telemetry.addData("Intake Arm Port : ", intakeArm.getPortNumber());
            telemetry.addData("Intake Arm Counter",intakeArmCounter);
            telemetry.addData("Intake Arm DPad Down",gamepad2.dpad_down);
            telemetry.addData("Intake Arm Mode",intakeArmMode);
            telemetry.addData("Intake Arm Slow Mode",intakeArmSlowMode);

            telemetry.update();



            // Rev2mDistanceSensor specific methods.

            telemetry.update();
        }
    }
}





