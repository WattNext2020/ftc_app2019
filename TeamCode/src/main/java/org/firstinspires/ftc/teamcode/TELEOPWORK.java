package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;





@TeleOp(name= "TeleOpWork", group= "Linear Opmode")

public class TELEOPWORK extends LinearOpMode {



    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfr = null;
    private DcMotor leftback = null;
    private DcMotor rightfr = null;
    private DcMotor rightback = null;

    private CRServo leftWheels = null;
    private CRServo rightWheels = null;
    private CRServo rackPinionUD = null;
    private CRServo rackPinionLR = null;

    private DistanceSensor sensorRange;

    double leftfrPower;
    double leftbackPower;
    double rightfrPower;
    double rightbackPower;

    double leftPower;
    double rightPower;
    double rackPowerUD;
    double rackPowerLR;


    double cap;

    double wheelPower =0;


    double lastSlow = 0;

    boolean zeroBrake = true;
    double lastBrake = 0;

    private Servo leftHook;
    private Servo rightHook;

    boolean first = true;
    boolean gatherA;
    boolean gatherB;

    boolean rhook;
    boolean lhook;

    double lastrhook =0;
    double lastlhook = 0;

    double lastTank = 0;
    boolean Tank = false;

    boolean slowMode = false;

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
        rackPinionUD = hardwareMap.get (CRServo.class, "rpUpDown");  // Rack and Pinion Vertical
        rackPinionLR = hardwareMap.get (CRServo.class, "rpLeftRight");   //Rack and Pinion Horizontal
        rightHook = hardwareMap.get(Servo.class,"rightHook");
        leftHook = hardwareMap.get(Servo.class,"leftHook");

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
        rackPinionUD.setDirection(CRServo.Direction.FORWARD);
        rackPinionLR.setDirection(CRServo.Direction.FORWARD);






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

            if(first ==true)
            {
                rightHook.setPosition(0);
                leftHook.setPosition(0);
                slowMode = false;
            }

            first = false;

            // Zero Brake Code
            if (gamepad1.b == true) {
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

            //intake mechanism

            wheelPower=gamepad2.right_stick_y;
            if(wheelPower>0.1)
            {
                wheelPower=1;
            }
            if(wheelPower<-0.1)
            {
                wheelPower=-1;
            }






            if (gamepad2.right_bumper == true ) {
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

            if (gamepad2.left_bumper == true) {
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

                //zero brake




            }
/*           if(gamepad2.x == true)
           {
               if ((runtime.seconds()-lastTank) > .5)
               {
                   if(Tank == false)
                   {
                       Tank = true;
                   }
                   else
                   {
                       Tank = false;
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
            if(lhook ==true)
            {
                leftHook.setPosition(1);

            }
            else
            {
                leftHook.setPosition(0);
            }

            if(rhook == true){
                rightHook.setPosition(1);
            }
            else
            {
                rightHook.setPosition(0);
            }





//







            double Theta = Math.atan2(gamepad1.right_stick_y,gamepad1.right_stick_y)*180/Math.PI;
            double Turn =Math.atan2(gamepad1.left_stick_y,gamepad1.left_stick_y)*180/Math.PI;

            double Vx = gamepad1.right_stick_y;
            double Vy = gamepad1.right_stick_x;

            if (-.1< Vx && Vx < .1)
            {                       //Dead Zone
                Vx = 0;
            }


            if (-.1< Vy && Vy < .1)
            {
                Vy = 0;
            }


            if(Vx == 0 && Vy == 0)    //Accounting for error given
            {
                rightfrPower = 0;
                leftfrPower = 0;
                leftbackPower = 0;
                rightbackPower = 0;
            }
            else {
                rightfrPower = Vx - Vy - Constants.WHEEL_DIST_V * Turn - Constants.WHEEL_DIST_H * Turn;
                leftfrPower = Vx + Vy + Constants.WHEEL_DIST_V * Turn + Constants.WHEEL_DIST_H * Turn;
                leftbackPower = Vx + Vy - Constants.WHEEL_DIST_V * Turn - Constants.WHEEL_DIST_H * Turn;
                rightbackPower = Vx - Vy + Constants.WHEEL_DIST_V * Turn + Constants.WHEEL_DIST_H * Turn;
                telemetry.addData("Initial Left Front ;", leftfrPower);   //Check Power`
                telemetry.addData("Initial Right Front;", rightfrPower);
                telemetry.addData("Initial Left Back;", leftbackPower);
                telemetry.addData("Initial Right Back;", rightbackPower);
                double max1;
                double max2;
                double max;
                double average;

                int temp = 0;

                if (leftfrPower != 0)
                {
                    temp = temp +1;
                }
                if (leftbackPower != 0)
                {
                    temp = temp +1;
                }
                if (rightfrPower != 0)
                {
                    temp = temp +1;
                }
                if (rightbackPower != 0)
                {
                    temp = temp +1;
                }

                average = (leftfrPower+leftbackPower+rightfrPower+rightbackPower)/temp;

                //get maximum power value
                max1 = Math.max(Math.abs(rightfrPower), Math.abs(leftfrPower));
                max2 = Math.max(Math.abs(leftbackPower), Math.abs(rightbackPower));

                max = Math.abs(Math.max(max1, max2));

                rightfrPower = rightfrPower / (max/average); //refactorization to a ccount for values greater than 1
                leftfrPower = leftfrPower / (max/average);
                leftbackPower = leftbackPower / (max/average);
                rightbackPower = rightbackPower / (max/average);


                rightfr.setPower(Range.clip(rightfrPower, Constants.MIN_POWER, Constants.MAX_POWER)); //Send Power
                leftfr.setPower(Range.clip(leftfrPower, Constants.MIN_POWER, Constants.MAX_POWER));
                leftback.setPower(Range.clip(leftbackPower, Constants.MIN_POWER, Constants.MAX_POWER));
                rightback.setPower(Range.clip(rightbackPower, -Constants.MIN_POWER, Constants.MAX_POWER));


                telemetry.addData("Left Front;", leftfr.getPower());   //Check Power`
                telemetry.addData("Right Front;", rightfr.getPower());
                telemetry.addData("Left Back;", leftback.getPower());
                telemetry.addData("Right Back;", rightback.getPower());
                //pickup mechanism

            }

            //              leftPower = Range.clip(gather, -1.0, 1.0);
            //   rightPower = Range.clip(gather, -1.0, 1.0);



  /*         if(gamepad2.b == true)
           {
               cap = -1;
           }

           if(gamepad2.a == true)
           {
               cap = 1;
           }

*/


            double intake =0;


            if (gamepad2.a == true)
            {
                intake = 1;
            }

            if (gamepad2.b == true)
            {
                intake = -1;
            }



            if (intake > 0.0){
                leftPower = Range.clip(intake, -1.0, 1.0);
                rightPower = Range.clip(intake, -1.0, 1.0);
            }
            else if (intake < 0.0) {
                leftPower = Range.clip(intake, -1.0, 1.0);
                rightPower = Range.clip(intake, -1.0, 1.0);
            }
            else {
                intake = 0.0;
                leftPower = 0.0;
                rightPower = 0.0;
            }




            cap=-gamepad2.left_stick_y *0.75;

















            double upDown = cap;

            if (upDown > 0.0){
                rackPowerUD = Range.clip(upDown, -1.0, 1.0);
            }
            else if (upDown < 0.0) {
                rackPowerUD = Range.clip(upDown, -1.0, 1.0);
            }
            else {
                upDown = 0.0;
                rackPowerUD = 0.0;
            }

            double side = cap;

            if (side > 0.0){
                rackPowerLR = Range.clip(side, -1.0, 1.0);
            }
            else if (side < 0.0) {
                rackPowerLR = Range.clip(side, -1.0, 1.0);
            }
            else {
                side = 0.0;
                rackPowerLR = 0.0;
            }


            // Send calculated power to wheels


            //intake power
            leftWheels.setPower(1);
            rightWheels.setPower(1);

            //arm movement power
            rackPinionLR.setPower(rackPowerLR);
            rackPinionUD.setPower(rackPowerUD);







            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left fr (%.2f), right fr (%.2f), left back (%.2f), right back (%.2f)", leftfrPower, rightfrPower, leftbackPower, rightbackPower);
            telemetry.addData("Servos", "left wheels (%.2f), right wheel (%.2f)", leftPower, rightPower);
            telemetry.addData("lhook", lhook);
            telemetry.addData("rhook", rhook);
            telemetry.addData("Value:",wheelPower);
            telemetry.addData("A:",gamepad2.a);
            telemetry.addData("B:",gamepad2.b);




            // Rev2mDistanceSensor specific methods.

            telemetry.update();

        }


    }
}



