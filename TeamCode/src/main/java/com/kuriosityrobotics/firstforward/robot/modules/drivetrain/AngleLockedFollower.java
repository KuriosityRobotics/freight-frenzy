package com.kuriosityrobotics.firstforward.robot.modules.drivetrain;

import static java.text.MessageFormat.format;

import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.Variable;

import java.text.MessageFormat;
import java.util.HashMap;
import java.util.Objects;

public class AngleLockedFollower {
    public static final double maxXVelocity, maxYVelocity, maxAngularVelocity;
    private static final double r = 3.77952756 / 2, WHEEL_MAX_VEL = 29.1041666, Lx = 4.16700, Ly = 4.2125;

    private final ExpressionsBasedModel model;
    private final Variable angularMovement, xMovement, yMovement, fr, fl, bl, br;

    public static final AngleLockedFollower theAngleLockedFollower;

    static {
        theAngleLockedFollower = new AngleLockedFollower();

        maxXVelocity = theAngleLockedFollower.maximiser(theAngleLockedFollower.getxMovement())
                .solve().xMov();
        maxYVelocity = theAngleLockedFollower.maximiser(theAngleLockedFollower.getyMovement())
                .solve().yMov();
        maxAngularVelocity = theAngleLockedFollower.maximiser(theAngleLockedFollower.getangularMovement())
                .solve().angularMov();
    }

    private AngleLockedFollower() {
        model = new ExpressionsBasedModel();
        angularMovement = model.addVariable("angularMovement");
        xMovement = model.addVariable("xMovement");
        yMovement = model.addVariable("yMovement");

        fl = model.addVariable("w1").upper(WHEEL_MAX_VEL).lower(-WHEEL_MAX_VEL);
        fr = model.addVariable("w2").upper(WHEEL_MAX_VEL).lower(-WHEEL_MAX_VEL);
        bl = model.addVariable("w3").upper(WHEEL_MAX_VEL).lower(-WHEEL_MAX_VEL);
        br = model.addVariable("w4").upper(WHEEL_MAX_VEL).lower(-WHEEL_MAX_VEL);

        model.addExpression("Left-right velocity")
                .lower(0)
                .upper(0)
                .set(fl, -r / 4)
                .set(fr, r / 4)
                .set(bl, r / 4)
                .set(br, -r / 4)
                .set(xMovement, -1);

        model.addExpression("Angular Velocity")
                .upper(0)
                .lower(0)
                .set(fl, -r / (4 * (Lx + Ly)))
                .set(fr, r / (4 * (Lx + Ly)))
                .set(bl, -r / (4 * (Lx + Ly)))
                .set(br, r / (4 * (Lx + Ly)))
                .set(angularMovement, -1);

        model.addExpression("Forward Velocity")
                .lower(0)
                .upper(0)
                .set(fr, r / 4)
                .set(fl, r / 4)
                .set(bl, r / 4)
                .set(br, r / 4)
                .set(yMovement, -1);
    }

    private static WheelMovements getState(AngleLockedFollower angleLockedFollower) {
        return new WheelMovements
                (
                        angleLockedFollower.xMovement.getValue().doubleValue(),
                        angleLockedFollower.yMovement.getValue().doubleValue(),
                        angleLockedFollower.angularMovement.getValue().doubleValue(),
                        angleLockedFollower.fl.getValue().doubleValue(),
                        angleLockedFollower.fr.getValue().doubleValue(),
                        angleLockedFollower.bl.getValue().doubleValue(),
                        angleLockedFollower.br.getValue().doubleValue()
                );
    }

    public Variable getangularMovement() {
        return angularMovement;
    }

    public Variable getxMovement() {
        return xMovement;
    }

    public Variable getyMovement() {
        return yMovement;
    }

    private void clearConstraints() {
        angularMovement.upper(null).lower(null).weight(0);
        xMovement.upper(null).lower(null).weight(0);
        yMovement.upper(null).lower(null).weight(0);
    }

    public WheelSolver solver() {
        return new WheelSolver();
    }

    public WheelSolver maximiser(Variable optimisationVariable) {
        return new WheelSolver(optimisationVariable, true);
    }

    public WheelSolver minimiser(Variable optimisationVariable) {
        return new WheelSolver(optimisationVariable, false);
    }

    public static final class WheelMovements {
        private final double xMov;
        private final double yMov;
        private final double angularMov;
        private final double fl;
        private final double fr;
        private final double bl;
        private final double br;
        
        public static WheelMovements fromMovements(double xMov, double yMov, double turnMov) {
            return new WheelMovements(
                    xMov,
                    yMov,
                    turnMov,
                    yMov + turnMov + xMov,
                    yMov - turnMov - xMov,
                    yMov + turnMov - xMov,
                    yMov - turnMov + xMov
            );
        }

        public WheelMovements(double xMov, double yMov, double angularMov, double fl, double fr, double bl,
                              double br) {
            this.xMov = xMov;
            this.yMov = yMov;
            this.angularMov = angularMov;
            this.fl = fl;
            this.fr = fr;
            this.bl = bl;
            this.br = br;
        }

        public double xMov() {
            return xMov;
        }

        public double yMov() {
            return yMov;
        }

        public double angularMov() {
            return angularMov;
        }

        public double fl() {
            return fl / WHEEL_MAX_VEL;
        }

        public double fr() {
            return fr / WHEEL_MAX_VEL;
        }

        public double bl() {
            return bl / WHEEL_MAX_VEL;
        }

        public double br() {
            return br / WHEEL_MAX_VEL;
        }

        @Override
        public int hashCode() {
            return Objects.hash(xMov, yMov, angularMov, fl, fr, bl, br);
        }

        @Override
        public String toString() {
            return format("[xMov={0,number,#.#}, yMov={1,number,#.#}, angularMov={2,number,#.#}, fl={3,number,.##}, fr={4,number,.##}, bl={5,number,.##}, br={6,number,.##}]", xMov, yMov, angularMov, fl, fr, bl, br);
        }

    }

    public class WheelSolver {
        private final HashMap<Variable, Double> constraintVariable = new HashMap<>();
        private final Variable optimisationVariable;
        private final boolean maximise;

        WheelSolver(Variable optimisationVariable, boolean maximise) {
            this.optimisationVariable = optimisationVariable;
            this.maximise = maximise;
        }

        public WheelSolver() {
            this.maximise = false;
            optimisationVariable = null;
        }

        public WheelSolver constrain(Variable variable, double value) {
            constraintVariable.put(variable, value);
            return this;
        }

        public WheelMovements solve() {
            synchronized (AngleLockedFollower.this) {
                validateAndConstrain();

                Optimisation.Result result;
                if (optimisationVariable == null) {
                    result = model.maximise();
                } else if (maximise) {
                    optimisationVariable.weight(1);
                    result = model.maximise();
                } else {
                    optimisationVariable.weight(1);
                    result = model.minimise();
                }

                if (!result.getState().isFeasible())
                    return null;

                return getState(AngleLockedFollower.this);
            }
        }

        private void validateAndConstrain() {
            clearConstraints();

            constraintVariable.forEach(
                    (variable, value) -> variable.lower(value).upper(value)
            );
        }
    }
}
