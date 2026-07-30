//SUPER ULTRA IMPORTANT CODE!!!!! DO NOT DELETE!!!!!!!!!!!!!!!!
package org.firstinspires.ftc.teamcode;

/*
TODO:



Updates:

1. Fixed launcher power code
2. Updated telemetry
3. Made it so that you can press the B button once to turn on aimbot and another time to turn off aimbot. Pressing X force disables aimbot.
5. Changed angle of limelight variable (angle1) to 20.0
6. Implemented auto trigger; now you don't have to manually shoot. The bot shoots 3 balls for you
7. Flipped the gears so there is more torque instead of more speed. It's proven to have more consistent launches, and a more stable RPM for the motor.
8. You can disable autoshoot when it's running to make some changes
9. Cleaned up the code
10. Implemented dynamic rpm switch

*/

//Libraries
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "AAIMTRIG (Auto Aim & Trigger)", group = "Sensor")
public class limelight_track_bot2 extends LinearOpMode {

    //Defines motors and the limelight
    private Limelight3A limelight;
    private DcMotor lf, rf, lb, rb;
    private boolean isTrackingEnabled = false;
    private DcMotorEx launch, launch2, transfer;

    static final double TPE = 28.0; //Ticks per revolution
    double TARGET_RPM = 0.0; //Desired RPM for launcher motor

    // --- ADJUST THESE CONSTANTS FOR YOUR ROBOT ---
    static final double h1 = 14;    // Height of Limelight lens from ground (inches)
    static final double h2 = 29.5;    // Height of AprilTag center from ground (inches - center of small tags)
    static final double angle1 = 20.0; // Mounting angle of Limelight (degrees tilted up from horizontal)
    boolean hasStartedFiring = false; // "Latch" to track if we started the 4 spins
    double lockedRPM = 0;             // Variable to lock the RPM so it doesn't fluctuate

    //Used when near goal
    public static double distRPM_Near(
            double inDist
    ){
        return (25.5 * inDist) + 2500;
    }

    //Used when far from goal
    public static double distRPM_Far(
            double inDist
    ){
        return (27 * inDist) + 2550;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Limelight def
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //Motors
        lf = hardwareMap.get(DcMotor.class, "front_left_drive");
        rf = hardwareMap.get(DcMotor.class, "front_right_drive");
        lb = hardwareMap.get(DcMotor.class, "back_left_drive");
        rb = hardwareMap.get(DcMotor.class, "back_right_drive");
        //Launcher motor
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
        launch2.setDirection(DcMotorSimple.Direction.REVERSE);

        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Allows motor to run at desired RPM instead of launch power
        launch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Intake motor
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Zeros motor
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setTargetPosition(0);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(0);

        //AAIMTRIG variables
        boolean lastB = false; // "Memory" of the button state
        boolean lastA = false; // Memory for manual trigger
        boolean autoshoot = false; //
        int shotsFired = 0;
        int autoShootStage = 0; // 0=Idle, 1=SpinUp, 2=Feed, 3=ResetFeeder
        long stateTimer = 0;    // To track time for delays
        int rpmStableCount = 0; // Count to stabilize RPM for intake motor
        final int TRANSFER_TICKS_PER_REV = 538;

        //Motor directions
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        //Brake motors when no power applied
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Switches limelight pipeline to 0 (detects apriltag 24 only)
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            //AAIMTRIG on/off using gampad button B, X to force disable
            if (gamepad1.b && !lastB) {
                isTrackingEnabled = !isTrackingEnabled;
            }
            lastB = gamepad1.b;
            if (gamepad1.x) {
                isTrackingEnabled = false;
            }

            //Gampad input variables
            double vy = gamepad1.left_stick_y;
            double vx = gamepad1.left_stick_x;
            double turnPower = -gamepad1.right_stick_x;
            //Distance from limelight to center of apriltag
            double distance = 0;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Calculate Distance
                double ty = result.getTy();
                distance = (h2 - h1) / Math.tan(Math.toRadians(angle1 + ty));

                // Auto-Aim Logic
                if (isTrackingEnabled) {
                    double tx = result.getTx();
                    double Kp = 0.03;
                    turnPower = -(tx * Kp); // Fixed Inversion
                    if (turnPower > 0.5) turnPower = 0.5;
                    if (turnPower < -0.5) turnPower = -0.5;
                }
            }

            // Mecanum Mixer
            double lfPow = vy - vx + turnPower;
            double rfPow = vy - vx - turnPower;
            double lbPow = vy + vx + turnPower;
            double rbPow = vy + vx - turnPower;

            //No idea what this does. Possibly a safety clamp for drive power?
            double max = Math.max(1.0, Math.max(Math.abs(lfPow), Math.abs(rfPow)));
            max = Math.max(max, Math.max(Math.abs(lbPow), Math.abs(rbPow)));

            lf.setPower(lfPow / max);
            rf.setPower(rfPow / max);
            lb.setPower(lbPow / max);
            rb.setPower(rbPow / max);
            //Can only shoot when aimbot is enabled
            if(gamepad1.right_bumper && isTrackingEnabled){
                autoshoot = true;
                hasStartedFiring = false; // Reset the latch
                if(distance >= 0.0 && distance <= 40.0){
                    lockedRPM = distRPM_Near(distance);
                }else{
                    lockedRPM = distRPM_Far(distance);
                }
                // Safety Clamps for the locked value
                if(lockedRPM > 6000.0) lockedRPM = 6000.0;
                if(lockedRPM < 1000.0) lockedRPM = 1000.0;
            }
            //Force cancel
            if(gamepad1.left_bumper){
                autoshoot = false;
                hasStartedFiring = false;
                launch.setVelocity(0);
                launch2.setVelocity(0);
                transfer.setVelocity(0);
            }

            final double RPM_Tolerance = 20;
            //Auto adjusts RPM and shoots when desired rpm = current rpm from motor
            if(autoshoot){
                // 1. Always keep flywheels spinning at the LOCKED target
                double targetTPS = (lockedRPM * TPE) / 60.0;
                launch.setVelocity(targetTPS);
                launch2.setVelocity(targetTPS);

                // 2. Logic to Start the Transfer (Only runs once per activation)
                if (!hasStartedFiring) {
                    double currentTPS = launch.getVelocity();
                    double currentRPM = (currentTPS * 60.0) / TPE;

                    // Check if RPM is ready (Tolerance: 25)
                    if (Math.abs(currentRPM - lockedRPM) <= RPM_Tolerance) {

                        // Calculate 4 full rotations relative to CURRENT position
                        // (4 * 538) = ~2152 ticks
                        int fourRotations = 4 * TRANSFER_TICKS_PER_REV;
                        int targetPos = transfer.getCurrentPosition() + fourRotations;

                        transfer.setTargetPosition(targetPos);
                        transfer.setVelocity((100.0 * TRANSFER_TICKS_PER_REV) / 60.0); // 100 RPM speed

                        hasStartedFiring = true; // Latch: We have started, don't enter this block again
                    }
                }

                // 3. Logic to Stop (Wait for transfer to finish)
                if (hasStartedFiring && !transfer.isBusy()) {
                    // The transfer motor has stopped moving, meaning the 4 shots are done.
                    // Turn everything off
                    launch.setVelocity(0);
                    launch2.setVelocity(0);
                    transfer.setVelocity(0);
                    transfer.setTargetPosition(0);
                    transfer.setVelocity(2100);
                    // Reset flags
                    autoshoot = false;
                    hasStartedFiring = false;
                }

            }

            // --- MANUAL CONTROL ---
            // Allows manual shooting if autoshoot is NOT active
            if (!autoshoot) {
                // Manual Flywheel Spin (Hold DPAD_UP)
                if (gamepad1.right_bumper && !isTrackingEnabled) {
                    double manualRPM = 4500.0; // Fixed Manual Speed
                    double manualTPS = (manualRPM * TPE) / 60.0;
                    launch.setVelocity(manualTPS);
                    launch2.setVelocity(manualTPS);
                } else {
                    launch.setVelocity(0);
                    launch2.setVelocity(0);
                }

                // Manual Trigger (Press A)
                if (gamepad1.a && !lastA) {
                    // Manually advance the transfer motor one full rotation
                    transfer.setTargetPosition(transfer.getTargetPosition() + TRANSFER_TICKS_PER_REV);
                    transfer.setVelocity((250.0 * TRANSFER_TICKS_PER_REV) / 60.0);
                }
                lastA = gamepad1.a;
            }

            // --- TELEMETRY ---
            telemetry.addData("Tracking", isTrackingEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Desired RPM", "%.0f", ((distance >= 0.0 && distance <= 40.0) ? distRPM_Near(distance) : distRPM_Far(distance)));
            telemetry.addData("Intake motor current degree: ", "%.1f", ((transfer.getCurrentPosition() / (double)TRANSFER_TICKS_PER_REV) * 360.0) % 360.0);
            if (result != null && result.isValid()) {
                telemetry.addData("Distance to Tag", "%.2f inches", Math.abs(distance));
                telemetry.addData("Horizontal Offset (TX)", "%.2f", result.getTx());
            } else {
                telemetry.addData("Distance to Tag", "No Target Seen");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
