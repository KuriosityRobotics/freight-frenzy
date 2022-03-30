package com.kuriosityrobotics.firstforward.robot.modules.drivetrain;

import org.ojalgo.optimisation.*;

import java.util.HashMap;
import java.util.Objects;

import static java.lang.Math.PI;
import static java.text.MessageFormat.format;

public class ConstrainedMovementCalculator {
    public static final double maxxMovement, maxyMovement, maxangularMovement;
    public static final ConstrainedMovementCalculator CONSTRAINED_MOVEMENT_CALCULATOR;
    private static final double r = 3.77952756 / 2, WHEEL_MAX_VEL = 29.1041666, Lx = 4.16700, Ly = 4.2125;

    static {
        CONSTRAINED_MOVEMENT_CALCULATOR = new ConstrainedMovementCalculator();

        maxxMovement = CONSTRAINED_MOVEMENT_CALCULATOR.maximiser(CONSTRAINED_MOVEMENT_CALCULATOR.getxMovement())
                .solve().xMov();
        maxyMovement = CONSTRAINED_MOVEMENT_CALCULATOR.maximiser(CONSTRAINED_MOVEMENT_CALCULATOR.getyMovement())
                .solve().yMov();
        maxangularMovement =
                CONSTRAINED_MOVEMENT_CALCULATOR.maximiser(CONSTRAINED_MOVEMENT_CALCULATOR.getangularMovement())
                        .solve().angularMov();
    }

    private final ExpressionsBasedModel model;
    private final Variable angularMovement, xMovement, yMovement, fr, fl, bl, br;
    private final Expression manhattanMovement;

    private ConstrainedMovementCalculator() {
        model = new ExpressionsBasedModel();
        angularMovement = model.addVariable("angularMovement");
        xMovement = model.addVariable("xMovement");
        yMovement = model.addVariable("yMovement");

        fl = model.addVariable("w1").upper(WHEEL_MAX_VEL).lower(-WHEEL_MAX_VEL);
        fr = model.addVariable("w2").upper(WHEEL_MAX_VEL).lower(-WHEEL_MAX_VEL);
        bl = model.addVariable("w3").upper(WHEEL_MAX_VEL).lower(-WHEEL_MAX_VEL);
        br = model.addVariable("w4").upper(WHEEL_MAX_VEL).lower(-WHEEL_MAX_VEL);

        manhattanMovement = model.addExpression("Manhattan Movement")
                .set(xMovement, 1)
                .set(yMovement, 1);

        model.addExpression("Front left")
                .lower(0)
                .upper(0)
                .set(yMovement, 1)
                .set(xMovement, 1)
                .set(angularMovement, 1)
                .set(fl, -1);

        model.addExpression("Front right")
                .lower(0)
                .upper(0)
                .set(yMovement, 1)
                .set(xMovement, -1)
                .set(angularMovement, -1)
                .set(fr, -1);

        model.addExpression("Back left")
                .lower(0)
                .upper(0)
                .set(yMovement, 1)
                .set(xMovement, -1)
                .set(angularMovement, 1)
                .set(bl, -1);

        model.addExpression("Back right")
                .lower(0)
                .upper(0)
                .set(yMovement, 1)
                .set(xMovement, 1)
                .set(angularMovement, -1)
                .set(br, -1);
    }

    public static void main(String[] args) {
        var follower = new ConstrainedMovementCalculator();
        var max =
                follower.maximiser(follower.angularMovement)
                        .constrainEq(follower.yMovement, 0)
                        .constrainEq(follower.xMovement, 0)
                        .solve();
        var min = follower.minimiser(follower.angularMovement)
                .constrainEq(follower.yMovement, 0)
                .constrainEq(follower.xMovement, 0)
                .solve();

        var maxY =
                follower.maximiser(follower.manhattanMovement)
                        .constrainEq(follower.angularMovement, 0)
                        .constrainEq(follower.xMovement, 0)
                        .solve();
        System.out.println(follower.maximiser(follower.manhattanMovement)
                .constrainEq(follower.angularMovement, PI/6)
                .constrainLeq(follower.xMovement, 3)
                .constrainLeq(follower.yMovement, 5).solve());


        var start = System.currentTimeMillis();
        for (double Mov = min.angularMov; Mov <= max.angularMov; Mov += (max.angularMov - min.angularMov) / 10000)
            follower.maximiser(follower.yMovement)
                    .constrainEq(follower.xMovement, 0)
                    .constrainEq(follower.angularMovement, Mov).solve();

        System.out.println(System.currentTimeMillis() - start);
    }

    private static WheelMovements getState(ConstrainedMovementCalculator constrainedMovementCalculator) {
        return new WheelMovements
                (
                        constrainedMovementCalculator.xMovement.getValue().doubleValue(),
                        constrainedMovementCalculator.yMovement.getValue().doubleValue(),
                        constrainedMovementCalculator.angularMovement.getValue().doubleValue(),
                        constrainedMovementCalculator.fl.getValue().doubleValue(),
                        constrainedMovementCalculator.fr.getValue().doubleValue(),
                        constrainedMovementCalculator.bl.getValue().doubleValue(),
                        constrainedMovementCalculator.br.getValue().doubleValue()
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

    public Expression getManhattanMovement() {
        return manhattanMovement;
    }

    private void clearConstraints() {
        angularMovement.upper(null).lower(null).weight(0);
        xMovement.upper(null).lower(null).weight(0);
        yMovement.upper(null).lower(null).weight(0);
    }

    public WheelSolver solver() {
        return new WheelSolver();
    }

    public WheelSolver maximiser(ModelEntity<?> optimisationVariable) {
        return new WheelSolver(optimisationVariable, true);
    }

    public WheelSolver minimiser(ModelEntity<?> optimisationVariable) {
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
            return format("[xMov={0,number,#.#}, yMov={1,number,#.#}, angularMov={2,number,#.#}, fl={3,number,.##}, " +
                            "fr={4,number,.##}, bl={5,number,.##}, br={6,number,.##}]", xMov, yMov, angularMov, fl, fr,
                    bl, br);
        }

    }

    public class WheelSolver {
        private final HashMap<Variable, Double> equalityConstraints = new HashMap<>();
        private final HashMap<Variable, Double> lessThanEqConstraints = new HashMap<>();
        private final ModelEntity<?> optimisationVariable;
        private final boolean maximise;

        WheelSolver(ModelEntity<?> optimisationVariable, boolean maximise) {
            this.optimisationVariable = optimisationVariable;
            this.maximise = maximise;
        }

        public WheelSolver() {
            this.maximise = false;
            optimisationVariable = null;
        }

        public WheelSolver constrainEq(Variable variable, double value) {
            equalityConstraints.put(variable, value);
            return this;
        }

        public WheelSolver constrainLeq(Variable variable, double value) {
            lessThanEqConstraints.put(variable, value);
            return this;
        }

        public WheelMovements solve() {
            synchronized (ConstrainedMovementCalculator.this) {
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

                return getState(ConstrainedMovementCalculator.this);
            }
        }

        private void validateAndConstrain() {
            clearConstraints();

            equalityConstraints.forEach(
                    (variable, value) -> variable.lower(value).upper(value)
            );

            lessThanEqConstraints.forEach(Variable::upper);
        }
    }
}
