package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.teamcode.Module.PinpointModule.distanta;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

@Configurable
public class OuttakeModule {
    HardwareMap hardwareMap;

    public OuttakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    DcMotorEx motorDR, motorST;
    public Servo servo_rampa, servo_blocaj;
    public static double vel50_60 = 960, vel60_70 = 970, vel70_75 = 1045, vel75_80 = 1140, vel80_86 = 1200, vel86_95 = 1180, vel95_105 = 1250, vel105_116 = 1320, vel120 = 1450, progresie = 4.8; //1060 1090
    public static double kp = 0.0023, ki = 0.0, kd = 0.0, ks = 0.09, kv = 0.0004969568, ka = 0.005;
    public static double target_velocity, velocity;
    public static double vel_modificabil;
    public static boolean act_outtake;
    public static double voltage;
    public static double nominal_voltage = 10.67; // Voltajul nominal de referinta pentru compensare shooter

    public static double POS_RAMPA_INITIAL = 1.0;
    public static double POS_RAMPA_TARGET = 0.8;
    public static double POS_RAMPA_JOS = POS_RAMPA_INITIAL;
    public static double POS_RAMPA_SUS = POS_RAMPA_TARGET;

    // --- BOOST VELOCITY LOGIC (pentru tragerea celor 4 bile) ---
    public static boolean enable_boost = true;              // Activeaza/dezactiveaza mecanismul de boost
    public static double drop_threshold = 50.0;             // Daca pierde >= 50 unitati de velocity sub target
    public static double boost_velocity = 30.0;             // Cu cate unitati creste velocity (configurabil in Panels)
    public static boolean cumulative_boost = true;          // Daca adauga boost cumulativ pentru fiecare bila (1..4)
    public static double max_boost = 150.0;                 // Limita maxima de boost cumulativ
    public static double ready_margin = 35.0;               // Toleranta pana la care se considera ca flywheel-ul a atins viteza
    public static double boost_cooldown_ms = 200.0;         // Cooldown minim intre 2 detectii de scadere (ms)

    public static double current_boost = 0.0;               // Boost-ul curent activ aplicat peste target
    public static boolean has_reached_target = false;       // True daca flywheel-ul a atins viteza inainte de drop
    private static boolean is_in_drop = false;
    private ElapsedTime boostCooldownTimer = new ElapsedTime();

    PIDFController controller = new PIDFController(kp, 0, 0, 0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);

    public void init() {
        motorDR = hardwareMap.get(DcMotorEx.class, "dreapta");
        motorST = hardwareMap.get(DcMotorEx.class, "stanga");

        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");
        servo_rampa = hardwareMap.get(Servo.class, "servo_shooter");

        motorDR.setDirection(DcMotorSimple.Direction.REVERSE);

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
     * Functie care verifica daca s-au pierdut drop_threshold unitati de velocity (default 50)
     * si creste viteza tinta cu boost_velocity (variabila configurabila in Panels/Dashboard).
     *
     * @param baseVel Viteza de baza ceruta din TeleOp
     * @return Valoarea curenta a boost-ului aplicat
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

    /**
     * Returneaza viteza tinta finala (viteza de baza + boost).
     */
    public double getBoostedVelocity(double baseVel) {
        return baseVel + current_boost;
    }

    public void update(double vel) {
        velocity = motorDR.getVelocity();

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        controller.setPIDF(kp, ki, kd, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        if (act_outtake) {
            checkVelocityBoost(vel);
            target_velocity = vel + current_boost;
        }

        else {
            resetBoost();
            target_velocity = vel_modificabil;
        }

        double output = controller.calculate(velocity, target_velocity) + feedforward.calculate(target_velocity);

        motorDR.setPower(output * (nominal_voltage / voltage));
        motorST.setPower(output * (nominal_voltage / voltage));
    }
}