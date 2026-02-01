package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

@TeleOp(name = "Limelight AprilTag Dual Flywheel Shooter (6 Buckets)", group = "Template")
public class LimelightAprilTagDualFlywheelShooter6Buckets extends LinearOpMode {

    
    // ======= EDIT  =========
    

    
    private static final String LIMELIGHT_TABLE_NAME = "limelight"; 

    limelight.getEntry("pipeline").setNumber(0);  // AprilTag pipeline index

    // How you get distance from Limelight NetworkTables.
    // Limelight does NOT always publish "distance" by default.
    // Put the correct entry name your robot uses (or one you publish yourself).
    private static final String DISTANCE_ENTRY_NAME = "distance"; 

    // Encoder / gearing constants
    // TICKS_PER_REV = encoder ticks per MOTOR revolution (or encoder rev if external encoder).
    // GEAR_RATIO = motor_rev / flywheel_rev  (1.0 if direct drive)
    private static final double TICKS_PER_REV = 28; //  CHANGE 
    private static final double GEAR_RATIO    = input...; //  CHANGE 

    // 6 distance buckets => 5 edges (meters)
    private static final double EDGE_1_M = input...; //  CHANGE 
    private static final double EDGE_2_M = input...; //  CHANGE 
    private static final double EDGE_3_M = input...; //  CHANGE 
    private static final double EDGE_4_M = input...; //  CHANGE 
    private static final double EDGE_5_M = input...; //  CHANGE 

    // RPM targets for each bucket
    private static final double RPM_BUCKET_0 = input...; //  CHANGE 
    private static final double RPM_BUCKET_1 = input...; //  CHANGE 
    private static final double RPM_BUCKET_2 = input...; //  CHANGE 
    private static final double RPM_BUCKET_3 = input...; //  CHANGE 
    private static final double RPM_BUCKET_4 = input...; //  CHANGE 
    private static final double RPM_BUCKET_5 = input...; //  CHANGE 

    
    private static final int BLUE_TARGET_TAG_ID = 20; //  blue backdrop tag
    private static final int RED_TARGET_TAG_ID  = 24; //  red backdrop tag

    private static final int[] SCORING_TAG_IDS = {
    BLUE_TARGET_TAG_ID,
    RED_TARGET_TAG_ID
    };

    // If true: keep spinning at last RPM when target disappears.
    // If false: stop flywheel when no valid scoring tag.
    private static final boolean HOLD_LAST_RPM_WHEN_LOST = true; // 

    // If one motor is mirrored mechanically, set this true to reverse flywheelB direction.
    private static final boolean REVERSE_FLYWHEEL_B = false; //(true/false)

    
    // ===== END EDIT =======
    

    private DcMotorEx flywheelA;
    private DcMotorEx flywheelB;

    private NetworkTable limelightTable;

    private double lastTargetRpm = 0.0;

    @Override
    public void runOpMode() {
        // --- Motors ---
        flywheelA = hardwareMap.get(DcMotorEx.class, "flywheelA");
        flywheelB = hardwareMap.get(DcMotorEx.class, "flywheelB");

        flywheelA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (REVERSE_FLYWHEEL_B) {
            flywheelB.setDirection(DcMotor.Direction.REVERSE);
        }

        // --- Limelight NetworkTables ---
        limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_NAME);

        waitForStart();

        while (opModeIsActive()) {
            LimelightTagData data = readLimelightTagData(limelightTable);

            boolean shouldSpin = false;
            double targetRpm = lastTargetRpm;

            if (data.hasValidTarget && isScoringTag(data.tagId)) {
                // Valid tag from the side we score on
                targetRpm = chooseRpm6Bucket(data.distanceMeters);
                lastTargetRpm = targetRpm;
                shouldSpin = true;
            } else {
                // No valid scoring tag
                shouldSpin = HOLD_LAST_RPM_WHEN_LOST && (lastTargetRpm > 0.0);
                if (!shouldSpin) {
                    targetRpm = 0.0;
                    lastTargetRpm = 0.0;
                }
            }

            // Command both motors
            if (shouldSpin && targetRpm > 0.0) {
                double tps = rpmToTicksPerSecond(targetRpm);
                flywheelA.setVelocity(tps);
                flywheelB.setVelocity(tps);
            } else {
                flywheelA.setVelocity(0);
                flywheelB.setVelocity(0);
            }

            telemetry.addData("Has Target (tv)", data.hasValidTarget);
            telemetry.addData("Tag ID (tid)", data.tagId);
            telemetry.addData("tx", "%.2f", data.tx);
            telemetry.addData("ty", "%.2f", data.ty);
            telemetry.addData("Distance (m)", "%.3f", data.distanceMeters);
            telemetry.addData("Scoring Tag?", isScoringTag(data.tagId));
            telemetry.addData("Target RPM", "%.1f", targetRpm);
            telemetry.addData("A vel (tps)", "%.1f", flywheelA.getVelocity());
            telemetry.addData("B vel (tps)", "%.1f", flywheelB.getVelocity());
            telemetry.update();
        }
    }

    // --------- Logic -----------
   

    private boolean isScoringTag(int tagId) {
        for (int id : SCORING_TAG_IDS) {
            if (id == tagId) return true;
        }
        return false;
    }

    // 6 buckets using 5 edges
    private double chooseRpm6Bucket(double dMeters) {
        if (dMeters < EDGE_1_M) return RPM_BUCKET_0;
        if (dMeters < EDGE_2_M) return RPM_BUCKET_1;
        if (dMeters < EDGE_3_M) return RPM_BUCKET_2;
        if (dMeters < EDGE_4_M) return RPM_BUCKET_3;
        if (dMeters < EDGE_5_M) return RPM_BUCKET_4;
        return RPM_BUCKET_5;
    }

    // Convert flywheel RPM to encoder ticks/sec
    private double rpmToTicksPerSecond(double rpmFlywheel) {
        double flywheelRps = rpmFlywheel / 60.0;
        double motorRps = flywheelRps * GEAR_RATIO; // GEAR_RATIO = motor_rev / flywheel_rev
        return motorRps * TICKS_PER_REV;
    }

    // ---- Limelight Reading ----

    private LimelightTagData readLimelightTagData(NetworkTable table) {
        LimelightTagData out = new LimelightTagData();

        // These keys are standard on Limelight:
        // tv = 1 when target is visible
        // tid = AprilTag ID
        // tx, ty = aiming offsets
        NetworkTableEntry tv  = table.getEntry("tv");
        NetworkTableEntry tid = table.getEntry("tid");
        NetworkTableEntry tx  = table.getEntry("tx");
        NetworkTableEntry ty  = table.getEntry("ty");

        out.hasValidTarget = tv.getDouble(0) > 0.5;
        out.tagId = (int) tid.getDouble(-1);
        out.tx = tx.getDouble(0);
        out.ty = ty.getDouble(0);

        // Distance entry name is robot-specific (you must set DISTANCE_ENTRY_NAME above)
        NetworkTableEntry dist = table.getEntry(DISTANCE_ENTRY_NAME);
        out.distanceMeters = dist.getDouble(0);

        return out;
    }

    private static class LimelightTagData {
        boolean hasValidTarget = false;
        int tagId = -1;
        double tx = 0.0;
        double ty = 0.0;
        double distanceMeters = 0.0;
    }
}
