package org.firstinspires.ftc.teamcode.Module;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

@Configurable
public class OuttakeModule {
    private final HardwareMap hardwareMap;

    public OuttakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    private DcMotorEx motorDR, motorST;
    public Servo servo_rampa, servo_blocaj;
    private VoltageSensor voltageSensor;

    public static double kp = 0.0023, ki = 0.0, kd = 0.0, ks = 0.09, kv = 0.0004969568, ka = 0.005;
    public static double target_velocity, velocity;
    public static boolean act_outtake;
    public static double voltage = 12.0;
    public static double nominal_voltage = 10.67;

    public static double POS_RAMPA_INITIAL = 0.68;
    public static double POS_RAMPA_TARGET = 1.0;
    public static double POS_RAMPA_JOS = POS_RAMPA_INITIAL;
    public static double POS_RAMPA_SUS = POS_RAMPA_TARGET;

    // --- BOOST VELOCITY LOGIC (pentru tragerea celor 4 bile) ---
    public static boolean enable_boost = true;
    public static double drop_threshold = 50.0;
    public static double boost_velocity = 30.0;
    public static boolean cumulative_boost = true;
    public static double max_boost = 150.0;
    public static double ready_margin = 35.0;
    public static double boost_cooldown_ms = 200.0;

    public static double current_boost = 0.0;
    public static boolean has_reached_target = false;
    private static boolean is_in_drop = false;
    private ElapsedTime boostCooldownTimer = new ElapsedTime();

    private PIDFController controller;
    private SimpleMotorFeedforward feedforward;

    public void init() {
        motorDR = hardwareMap.get(DcMotorEx.class, "dreapta");
        motorST = hardwareMap.get(DcMotorEx.class, "stanga");

        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");
        servo_rampa = hardwareMap.get(Servo.class, "servo_shooter");

        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        motorDR.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(kp, ki, kd, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        blocat();
        rampaInitial();

        resetBoost();
        target_velocity = 0;
    }

    public void blocat() {
        servo_blocaj.setPosition(0.02);
    }

    public void deblocat() {
        servo_blocaj.setPosition(0.3);
    }

    public void rampaInitial() {
        servo_rampa.setPosition(POS_RAMPA_INITIAL);
    }

    public void rampaTarget() {
        servo_rampa.setPosition(POS_RAMPA_TARGET);
    }

    public void rampaJos() {
        rampaInitial();
    }

    public void rampaSus() {
        rampaTarget();
    }

    public void rampaInit() {
        rampaInitial();
    }

    public void setRampa(double position) {
        servo_rampa.setPosition(position);
    }

    /**
     * Reseteaza starea de boost (cand shooter-ul este oprit sau la inceputul unei noi trageri).
     */
    public void resetBoost() {
        current_boost = 0.0;
        has_reached_target = false;
        is_in_drop = false;
        boostCooldownTimer.reset();
    }

    /**
     * Functie care verifica daca s-au pierdut drop_threshold unitati de velocity
     * si creste viteza tinta cu boost_velocity.
     */
    public double checkVelocityBoost(double baseVel) {
        if (!enable_boost || !act_outtake) {
            resetBoost();
            return 0.0;
        }

        double currentTarget = baseVel + current_boost;

        // 1. Verificam daca shooter-ul a ajuns la viteza inainte de a detecta pierderile cauzate de bila
        if (!has_reached_target) {
            if (velocity >= (currentTarget - ready_margin)) {
                has_reached_target = true;
                is_in_drop = false;
            }
        } else {
            // 2. Cand shooter-ul e la viteza si intra bila, viteza scade
            double drop = currentTarget - velocity;
            if (drop >= drop_threshold && !is_in_drop && boostCooldownTimer.milliseconds() >= boost_cooldown_ms) {
                is_in_drop = true;
                boostCooldownTimer.reset();

                if (cumulative_boost) {
                    current_boost = Math.min(current_boost + boost_velocity, max_boost);
                } else {
                    current_boost = boost_velocity;
                }

                // Dupa adaugarea boost-ului, asteptam sa atinga noul target inainte de urmatoarea bila
                has_reached_target = false;
            }
        }

        return current_boost;
    }

    public double getBoostedVelocity(double baseVel) {
        return baseVel + current_boost;
    }

    public void update(double vel) {
        velocity = motorDR.getVelocity();

        if (voltageSensor != null) {
            try {
                voltage = voltageSensor.getVoltage();
            } catch (Exception ignored) {
            }
        }

        if (act_outtake) {
            checkVelocityBoost(vel);
            target_velocity = vel + current_boost;
        } else {
            resetBoost();
            target_velocity = vel;
        }

        if (target_velocity <= 0) {
            motorDR.setPower(0);
            motorST.setPower(0);
            return;
        }

        double output = controller.calculate(velocity, target_velocity) + feedforward.calculate(target_velocity);
        double compFactor = (voltage > 6.0) ? (nominal_voltage / voltage) : 1.0;
        double power = output * compFactor;

        motorDR.setPower(power);
        motorST.setPower(power);
    }
}
