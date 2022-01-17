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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="1.OMNIStrafingv2", group="Iterative Opmode")
//@Disabled
public class OMNIStrafingv2 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
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

    ////////Claw Gripper
    private Servo leftClaw = null;
    private Servo rightClaw = null;


    String duckdirection = null;
    double duckspeed = 0.23;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

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

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        duck_spinner.setDirection(DcMotor.Direction.FORWARD);
        extenderMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double minSpeed = -0.6;
        double maxSpeed = 0.6;

        //triggers
        double lt1 = gamepad1.left_trigger;
        double rt1 = gamepad1.right_trigger;

        double lt2 = gamepad2.left_trigger;
        double rt2 = gamepad2.right_trigger;

        //gamepad1 joysticks
        double lsX1 = gamepad1.left_stick_x;
        double lsY1 = gamepad1.left_stick_y;

        double rsX1 = gamepad1.right_stick_x;
        double rsY1 = gamepad1.right_stick_y;

        //gamepad 2 joysticks
        double lsX2 = gamepad2.left_stick_x;
        double lsY2 = gamepad2.left_stick_y;

        double rsX2 = gamepad2.right_stick_x;
        double rsY2 = gamepad2.right_stick_y;

        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;

        //buttons
        boolean b1 = gamepad1.b;
        boolean x1 = gamepad1.x;

        //dpad


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;

        if (lt1 > 0) {
            minSpeed = -0.25;
            maxSpeed = 0.25;
        } else {
            minSpeed = -0.60;
            maxSpeed = 0.60;
        }

        leftPower    = Range.clip(drive + turn, minSpeed, maxSpeed) ;
        rightPower   = Range.clip(drive - turn, minSpeed, maxSpeed) ;

        boolean threshold = (Math.abs(lsX1) > 0.1 || Math.abs(lsY1) > 0.1  || Math.abs(rsX1) > 0.1);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;



        // button commands and controls
        if (gamepad2.dpad_up){
            armServo.setPosition(1);
        }
        if (gamepad2.dpad_down){
            armServo.setPosition(0);
        }

        ////////CLAW SERVO STUFF
        if (gamepad1.dpad_up){
            leftClaw.setPosition(leftClaw.getPosition()+0.1);
        }

        //set duck spinner direction
        if (b1){
            duckdirection = "red";
        }

        if (x1){
            duckdirection = "blue";
        }



        //OMNI Directional Strafing
        if (threshold) {
            //drive forward
            if (Math.abs(lsX1) < 0.2 && lsY1 < -0.2){
                front_left.setPower(leftPower);
                back_left.setPower(leftPower);
                front_right.setPower(rightPower);
                back_right.setPower(rightPower);
            }//drive backward
            else if (Math.abs(lsX1) < 0.2 && lsY1 > 0.2){
                front_left.setPower(leftPower);
                back_left.setPower(leftPower);
                front_right.setPower(rightPower);
                back_right.setPower(rightPower);
            }
            //strafe left
            else if (lsX1 < -0.2 && lsY1 < 0.1 && lsY1 > -0.1){
                front_left.setPower(-.5);
                back_left.setPower(.5);
                front_right.setPower(.5);
                back_right.setPower(-.5);
            }
            // strafe right
            else if (lsX1 > 0.2 && lsY1 < 0.1 && lsY1 > -0.1) {
                front_left.setPower(.5);
                back_left.setPower(-.5);
                front_right.setPower(-.5);
                back_right.setPower(.5);
            }
            //diagonals

            /*//forward right
            else if (lsX1 > 0.2 && lsY1 < -0.2 && lsY1 > -0.8) {
                front_left.setPower(leftPower);
                back_left.setPower(0);
                front_right.setPower(0);
                back_right.setPower(rightPower);
            }
            //forward left
            else if (lsX1 < -0.2 && lsY1 < -0.2 && lsY1 > -0.8) {
                front_right.setPower(rightPower);
                back_left.setPower(leftPower);
                front_left.setPower(0);
                back_right.setPower(0);
            }
            //backward right
            else if (lsX1 > 0.2 && lsY1 > 0.2 && lsY1 < 0.8) {
                front_right.setPower(rightPower);
                back_left.setPower(leftPower);
                front_left.setPower(0);
                back_right.setPower(0);
            }
            //backward left
            else if (lsX1 < -0.2 && lsY1 > 0.2 && lsY1 < 0.8) {
                front_left.setPower(leftPower);
                back_right.setPower(rightPower);
                front_right.setPower(0);
                back_left.setPower(0);


            //turn left
            }*/
            else if ((rsX1) < -0.2 ) {

                front_left.setPower(-0.5);
                back_left.setPower(-0.5);
                front_right.setPower(0.5);
                back_right.setPower(0.5);
            }
            //turn right
            else if ((rsX1) > 0.2 ) {
                front_left.setPower(0.5);
                back_left.setPower(0.5);
                front_right.setPower(-0.5);
                back_right.setPower(-0.5);
            }
        }else {
            stopDrive();
        }

        if (rt1 > 0.25){
            if (duckdirection == "red") {
                duck_spinner.setPower(Range.clip((rt1), -duckspeed, duckspeed));
            }else if(duckdirection == "blue"){
                duck_spinner.setPower(Range.clip((-rt1), -duckspeed, duckspeed));
            }else{
                duck_spinner.setPower(0);
            }

        }else{
            duck_spinner.setPower(0);
        }

        // Send calculated power to wheels
        /*
        front_left.setPower(leftPower);
        back_left.setPower(leftPower);
        front_right.setPower(rightPower);
        back_right.setPower(rightPower);
         */

        //duck turntable motor mapped to gamepad1.left_stick_x
        extenderMotor.setPower(Range.clip((lsY2),-0.9 ,0.9));
        //liftMotor.setPower(Range.clip((rsY2),-0.38,0.38));
        liftMotor.setPower(Range.clip((rsY2),-1,1));

        /*if (lt2 >0){
            duck_spinner.setPower(Range.clip((lt2),-0.4,0.4));
        }else{
            duck_spinner.setPower(Range.clip((-rt2),-0.4,0.4));
        }*/
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("lsX1","Left stick X1:" +lsX1);
        telemetry.addData("lsY1","Left stick Y1:" +lsY1);
        telemetry.addData("RightTrigger 1", ""+rt1);
        telemetry.addData("duckdirection", ""+duckdirection);


    }

    public void stopDrive(){

        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);

    }
    //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
    }

}