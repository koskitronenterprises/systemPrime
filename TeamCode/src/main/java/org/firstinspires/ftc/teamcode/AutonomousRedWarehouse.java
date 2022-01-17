//Default include and Opmode includes
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Common utilities and peripherals
import com.qualcomm.robotcore.util.Range;

//Motors and Servos
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


//Roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


//Vuforia
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;




 @Autonomous(name = "Red Warehouse", group = "")
 public class AutonomousRedWarehouse extends LinearOpMode {
     //////////MOTOR AND SERVO DEFINITIONS

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
     //private Servo armServo = null;
     private Servo leftClaw = null;
     private Servo rightClaw = null;

     //////////Vuforia Variables

     final double MM_PER_INCH = 25.40;   //  Metric conversion

     private static final String VUFORIA_KEY =
             "Ad1u0cX/////AAABmSuPiafxZUXLisBkrqZvKakdkJlQlNy1pSwPKr80TeG+3kJBTuFSsmFkJdfFOngQTSKEDt9/WwcHgbEli4A0EbEOdeB73sxwWDjhNNYWqauwqFLLL2TujgAT1X42LY+fchRxvE7AcQWWS5bF+FYRY+NTLaWHbtTBY0Ta+N+Ozw72QOjq1RNv0o4+Cl9SuTRfr2YVn8ySYl48HZ6v3vujobcCrJqNv24+7Nul/XUulPeKyGFT3ye+CZha8uVKiDCyPiMXH4OrMzhCKjipfbVwjRFjAIAd91U27ZlRHhKOy4TCBBqYzkDFarpcJTkvXSIaU7C08U0G0kZipTEIEYBjxgzEQ/P9LxkKrAQ1fxFQmXKD ";

     VuforiaLocalizer vuforia = null;
     OpenGLMatrix targetPose = null;
     String targetName = "";
     String elementPosition = "";


     private void moveMotor(double fl, double fr, double bl, double br) {
         front_left.setPower(fl);
         front_right.setPower(fr);
         back_left.setPower(bl);
         back_right.setPower(br);
     }

     private void moveMotor(double fl, double fr, double bl, double br, int ms) {
         front_left.setPower(fl);
         front_right.setPower(fr);
         back_left.setPower(bl);
         back_right.setPower(br);
         sleep(ms);
         stopDrive();
     }

     private void stopDrive() {
         front_left.setPower(0);
         front_right.setPower(0);
         back_left.setPower(0);
         back_right.setPower(0);
     }

     private void strafe(double speed, int time) {
         //positive = right, negative = left
         moveMotor(speed, -speed, -speed, speed, time);
     }


     private void turn(double speed, int time) {
         //postive = right, negative = left
         moveMotor(-speed, speed, -speed, speed, time);
     }

     private void goStraight(double speed, int time) {
         //positive = forward, negative = backward
         moveMotor(speed, speed, speed, speed, time);
     }


     @Override
     public void runOpMode() {
         telemetry.addData(">", "Initializing camera stream. Please wait to press play!");
         telemetry.update();


         //////////Initialize the hardware variables.
         // Motors and servos
         front_left = hardwareMap.get(DcMotor.class, "front_left");
         back_left = hardwareMap.get(DcMotor.class, "back_left");
         front_right = hardwareMap.get(DcMotor.class, "front_right");
         back_right = hardwareMap.get(DcMotor.class, "back_right");
         duck_spinner = hardwareMap.get(DcMotor.class, "duck_spinner");
         extenderMotor = hardwareMap.get(DcMotor.class, "extenderMotor");
         liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

         //armServo = hardwareMap.get(Servo.class, "armServo");

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


         /*
          * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
          * To get an on-phone camera preview, use the code below.
          * If no camera preview is desired, use the parameter-less constructor instead (commented out below).
          */
         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
         // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

         parameters.vuforiaLicenseKey = VUFORIA_KEY;

         // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
         parameters.useExtendedTracking = false;

         // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
         parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");
         this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

         // Load the trackable objects from the Assets file, and give them meaningful names
         VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("UseMe");
         targetsFreightFrenzy.get(0).setName("Scoring Element");

         // Start tracking targets in the background
         targetsFreightFrenzy.activate();

         telemetry.addData(">", "Press Play to start, or tap the ellipsis to view the camera stream!");
         telemetry.update();

         waitForStart();


//////////Step 0. Lift the arm a bit and squeeze the claw shut.
         liftMotor.setPower(0.4);
         sleep(1000);
         leftClaw.setPosition(0);
         rightClaw.setPosition(1);
         sleep(2500);
         liftMotor.setPower(0);
         moveMotor(0.3,0.3,0.3,0.3,800);
         moveMotor(0,0.3,0,0.3,1000);
         extenderMotor.setPower(-0.4);
         sleep(1900);
         extenderMotor.setPower(0);
         leftClaw.setPosition(1);
         rightClaw.setPosition(0);
         turn(0.3, 1000);
         strafe(-0.5, 3500);
         moveMotor(-0.3,-0.3,-0.3,-0.3,3000);





//////////End step 0.


//////////Step 1. Detect the scoring element position.
         boolean targetFound = false;    // Set to true when a target is detected by Vuforia
         double targetRange = 0;        // Distance from camera to target in Inches
         double targetBearing = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.

         while (opModeIsActive()) {
             // Look for first visible target, and save its pose.
             targetFound = false;
             for (VuforiaTrackable trackable : targetsFreightFrenzy) {
                 if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                     targetPose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getVuforiaCameraFromTarget();

                     // if we have a target, process the "pose" to determine the position of the target relative to the robot.
                     if (targetPose != null) {
                         targetFound = true;
                         targetName = trackable.getName();
                         VectorF trans = targetPose.getTranslation();

                         // Extract the X & Y components of the offset of the target relative to the robot
                         double targetX = trans.get(0) / MM_PER_INCH; // Image X axis
                         double targetY = trans.get(2) / MM_PER_INCH; // Image Z axis

                         // target range is based on distance from robot position to origin (right triangle).
                         targetRange = Math.hypot(targetX, targetY);

                         // target bearing is based on angle formed between the X axis and the target range line
                         targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));

                         break;  // jump out of target tracking loop if we find a target.
                     }
                 }
             }

             if (targetFound) {
                 if (targetBearing < -8) {
                     //The element is on the left
                     elementPosition = "left";
                     telemetry.addData("Target", " %s", targetName);
                     telemetry.addData("Range", "%5.1f inches", targetRange);
                     telemetry.addData("Bearing", "%3.0f degrees", targetBearing);
                 } else if (targetBearing > 8) {
                     //The element is in the center
                     elementPosition = "center";
                     telemetry.addData("Target", " %s", targetName);
                     telemetry.addData("Range", "%5.1f inches", targetRange);
                     telemetry.addData("Bearing", "%3.0f degrees", targetBearing);
                 }
             } else {
                 //The target is on the right
                 elementPosition = "right";
                 telemetry.addData(">", "No target found. Time to drop the ball in the top!\n");
             }
//////////End step one

             /*
//////////Step 2. Set current "Pose and location"
             //TODO: Edit the current desired starting location and pose.
             SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

             // If we wanted to start the bot at x: 10, y: -8, heading: 90 degrees
             Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

             drive.setPoseEstimate(startPose);

//////////End step two

//////////Step 3. Drive to the cake tier thingy and deposit the cube
             //TODO: edit the trajectory to the cake thingy
             Trajectory traj1 = drive.trajectoryBuilder(startPose)
                     .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                     .build();

             drive.followTrajectory(traj1);

             //TODO: add the code to position the arm and claw and drop the good ol' freight into the cake thingy


//////////End step three

//////////Step 4. Drive to the duck motor and spin it
             //TODO: edit the trajectory to the duck spinner and edit the pose
             Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                     .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                     .build();
             drive.followTrajectory(traj2);

             //TODO: add the code to spin the cute little ducky

//////////End step four

//////////Step 5. Park
             //TODO: edit the trajectory to park
             Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                     .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                     .build();
             drive.followTrajectory(traj3);
//////////End step five
*/

             stopDrive();


         }
     }
 }
