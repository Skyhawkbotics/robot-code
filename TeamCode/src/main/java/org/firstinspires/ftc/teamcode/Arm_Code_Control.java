package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Arm_Code_Control")
public class Arm_Code_Control extends LinearOpMode {

    /*
    Current Ports
    Front is on expansion hub port 3
     viper slide is on expansion hub port 2
     outtake rotator on control hub 0
     outtake claw on control hub 1
     back left is on Control hub 2 ????
     front left is on Control hub 1 ??????
     We need to also config touch sensor
     */
    // These variables can be inside the field but they don't have to - its just because less writing in general I think
    private DcMotorEx revMotor;
    private CRServo servo_Intake;
    private CRServo servo_intake_wrist;

    private DcMotorEx viper_slide_Motor;
    // From my understanding - Up true Target_pos stores the position of viper slide as it changes from set.velocity
    // int viper_true_target_pos;
    //private TouchSensor up_zero;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // sets up tel em

        //setup Rev Motor arm to use velocity
        revMotor = hardwareMap.get(DcMotorEx.class, "revMotor");
        revMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        revMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        viper_slide_Motor = hardwareMap.get(DcMotorEx.class, "Viper_Slide_Motor");
        viper_slide_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper_slide_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        viper_slide_Motor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Setup both servos
        servo_Intake = hardwareMap.get(CRServo.class, "outtake"); //control hub 1
        servo_intake_wrist = hardwareMap.get(CRServo.class, "outtakeWrist");

        // Upzero Sensor setup
        // up_zero = hardwareMap.get(TouchSensor.class, "up_zero");



        // Wait for the game to start
        waitForStart();

        // Controls
        /*
        All controls on Game Pad two
         a - move misumi slide out
         y - move misumi slide in
         right trigger  - servo intake
         right bumper - servo outtake?
        left trigger - wrist , with sensitivity
        bumper - wrist, other way, no sensitivity
         */
        while (opModeIsActive()) {
            if (gamepad2.a) {
                //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                revMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                revMotor.setVelocity(1000);
                 telemetry.addData("A down", true);
            } else if (gamepad2.y) {
                revMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                revMotor.setVelocity(-1000);
                telemetry.addData("Y down", true);
            } else {
                revMotor.setVelocity(0);
                telemetry.addData("Intake Slide Stationary", true);
            }
           /* if (gamepad2.right_stick_y > 0.5) {
                //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                viper_slide_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                viper_slide_Motor.setVelocity(1000);
                telemetry.addData("A down", true);
            } else if (gamepad2.y) {
                viper_slide_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                viper_slide_Motor.setVelocity(-1000);
                telemetry.addData("Y down", true);
            } else {
                viper_slide_Motor.setVelocity(0);
                telemetry.addData("Viper slide stationary ", true);
            }
            */
            /* if (gamepad2.left_stick_y > 0.5) {
                //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
                viper_slide_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                viper_slide_Motor.setVelocity(1000);
                viper_true_target_pos = 0;
                telemetry.addData("Y stick Up", true);

                // why is this declared again?
            } else if (!up_zero.isPressed() && gamepad2.left_stick_y < -0.5) {

                viper_slide_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                viper_slide_Motor.setVelocity(-1000);
                viper_true_target_pos = 0;
                telemetry.addData("Y stick Down", true);



            } else {
                viper_slide_Motor.setPower(500);
                //use position mode to stay up, as otherwise it would fall down. do some fancy stuff with up_true_target_pos to avoid the issue of it very slightly falling every tick
                if (viper_true_target_pos == 0) { // always true after any input and from the start
                    viper_slide_Motor.setTargetPosition(viper_slide_Motor.getCurrentPosition());
                    // makes the encoder set the target position based on the current position of the motor, which I think is different depending on position

                    viper_true_target_pos = viper_slide_Motor.getCurrentPosition();
                    // sets true target to current position -
                    telemetry.addData("Viper slide stationary ", true);

                }
                viper_slide_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Viper slide stationary ", true);

                // returns to current position set from set target position
                // OKAY SO basically after each action, the viper slide sets its target position to the current position, then calls it motor to run to position, aka making it stay there
            }
            */

            // Servo Controlling Outtake
            if (gamepad2.right_trigger > 0.8/* && servo_CLAW_position < 1000000000*/) {
                // Uses set power for movement (parameters of servo is [-1,1])
                servo_Intake.setPower(1);
                telemetry.addData("Right_trigger down, servo power 1", true);
            } else if (gamepad2.right_bumper/* && servo_CLAW_position > -100000000*/) {
                servo_Intake.setPower(-1);
                telemetry.addData("Right_bumper down, servo power -1", true);

            } else {
                // setPower of 0 brakes the servo
                servo_Intake.setPower(0);
                telemetry.addData("Intake Servo Stationary", true);

            }

            // Servo Controlling the wrist of the servo
            if (gamepad2.left_trigger > 0.8) {
                servo_intake_wrist.setPower(1);
                telemetry.addData("Left_trigger down, servo power 1", true);

            } else if (gamepad2.left_trigger <= 0.8) {
                // Setting a lower power for more precision
                servo_intake_wrist.setPower(0.5);
                telemetry.addData("left+trigger down half-way, servo power 0.5", true);

            } else if (gamepad2.left_bumper) { //TODO: these limits too.
                // game pad2.left_bumper is a boolean variable, so I can't set lower powers for them as it only returns trues/false
                servo_intake_wrist.setPower(-1);
                telemetry.addData("left_bumper down, servo power -1", true);

            } else {
                servo_intake_wrist.setPower(0);
                telemetry.addData("Intake Wrist Servo Stationary", true);

            }

            // Telemetry to display motor status on the driver station
            //telemetry stuff (prints stuff on the telemetry (driver hub))
            telemetry.addData("armCurrentPosition", revMotor.getCurrentPosition());
            telemetry.update();

            //idk what this does, something for ftc dashboard i think
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


        }
    }
}

