package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.util.ElapsedTime;


    public class PIDController {
        private double kP, kI, kD;
        private double integral = 0;
        private double previousError = 0;
        private ElapsedTime timer = new ElapsedTime();

        public PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            timer.reset();
        }

        public double calculate(double current, double target) {
            double dt = timer.seconds();
            timer.reset();

            if (dt == 0) dt = 0.001;

            double error = target - current;
            double p = kP * error;

            integral += error * dt;
            integral = Math.max(-1000, Math.min(1000, integral));
            double i = kI * integral;

            double derivative = (error - previousError) / dt;
            double d = kD * derivative;

            previousError = error;

            double output = p + i + d;
            return Math.max(-1, Math.min(1, output));
        }
        public boolean atTarget(double current, double target, double tolerance) {
            return Math.abs(target - current) < tolerance;
        }

        public void reset() {
            integral = 0;
            previousError = 0;
            timer.reset();
        }
    }






