 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@Autonomous(name = "1. Autonomous", group = "")
public class autonomoustest extends LinearOpMode {

 //left motors
    private DcMotor front_left = null;
    private DcMotor back_left = null;
    //right motors
    private DcMotor front_right = null;
    private DcMotor back_right = null;
    //duck turntable motor
    private DcMotor duck_spinner = null;
    private DcMotor extenderMotor = null;
    private DcMotor liftMotor = null;
    private Servo armServo = null;
    
    
  
private void moveMotor(double fl, double fr,double bl,double br)
{
    front_left.setPower(fl);
    front_right.setPower(fr);
    back_left.setPower(bl);
    back_right.setPower(br);
}

private void moveMotor(double fl, double fr,double bl,double br, int ms)
{
    front_left.setPower(fl);
    front_right.setPower(fr);
    back_left.setPower(bl);
    back_right.setPower(br);
    sleep(ms);
    stopDrive();
}

private void stopDrive()
{
    front_left.setPower(0);
    front_right.setPower(0);
    back_left.setPower(0);
    back_right.setPower(0);
}

private void strafe(double speed,int time)
{
  //positive = right, negative = left
  moveMotor(-speed, -speed, speed, speed, time);
}


private void turn(double speed, int time)
{
  //postive = right, negative = left
  moveMotor(-speed, speed, -speed, speed, time);
}

private void goStraight(double speed, int  time)
{
  //positive = forward, negative = backward
  moveMotor(speed, speed, speed, speed, time);
}

  @Override
  public void runOpMode() {
    // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");
        duck_spinner = hardwareMap.get(DcMotor.class, "duck_spinner");
        extenderMotor = hardwareMap.get(DcMotor.class, "extenderMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        duck_spinner.setDirection(DcMotor.Direction.FORWARD);
        extenderMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
  

    waitForStart();
    goStraight(.5,1750);
    stopDrive();
    
    
  }
}
