package com.kuriosityrobotics.firstforward.robot.sensors.kf;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.kuriosityrobotics.firstforward.robot.sensors.RollingVelocityCalculator;

import org.ojalgo.matrix.Primitive64Matrix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

public class ExtendedKalmanFilter extends RollingVelocityCalculator implements Telemeter {
    private static final int MOVING_WINDOW_SIZE = 500;
    private final Object lock = new Object();
    private final LinkedList<PostPredictionState> history = new LinkedList<>();

    private Primitive64Matrix mean, covariance;
    private ExtendedKalmanFilter chain;
    private long lastChainPrediction = -1;

    /**
     * @param initialState starting state
     */
    public ExtendedKalmanFilter(double... initialState) {
        synchronized (lock) {
            reset(initialState);
        }
    }

    public static Primitive64Matrix propagateError(
            Primitive64Matrix function,
            Primitive64Matrix startingCovariance
    ) {
        return function.multiply(startingCovariance).multiply(function.transpose());
    }

    public static Primitive64Matrix diagonal(double... diagonal) {
        var data = new double[diagonal.length][diagonal.length];

        for (int i = 0; i < diagonal.length; i++) {
            data[i][i] = diagonal[i];
        }
        return Primitive64Matrix.FACTORY.rows(data);
    }

    public static void assertThat(boolean condition) {
        if (!condition)
            throw new AssertionError();
    }

    public void reset(double... initialState) {
        var variables = initialState.length;

        history.clear();
        mean = Primitive64Matrix.FACTORY.column(initialState);
        covariance = Primitive64Matrix.FACTORY.make(variables, variables);

        history.add(new PostPredictionState(mean, covariance, null, false));
    }

    public void replayHistory(int after) {
        synchronized (lock) {
            Log.d("ExtendedKalmanFilter", "Replayed " + (history.size() - after) + "measurements.");
            var iter = history.listIterator(after);
            var state = iter.next();
            this.mean = state.getMean();
            this.covariance = state.getCovariance();

            while (iter.hasNext()) {
                var idx = iter.nextIndex();
                var measurement = iter.next();

                if (measurement.isCorrection())
                    correction(measurement.getDatum(), idx);
                else
                    predict(measurement.getDatum(), idx);
            }
        }
    }

    /**
     * Run the output of this filter as a prediction into next
     */
    @SuppressWarnings("unused")
    public void chainTo(ExtendedKalmanFilter next) {
        synchronized (lock) {
            this.chain = next;
        }
    }

    public void predict(KalmanDatum datum) {
        synchronized (lock) {
            if (history.size() == 1 || history.getLast().getDatum() == null || datum.time > history.getLast().getDatum().time)
                predict(datum, history.size() - 1);
            else {
                var iter = history.listIterator(history.size());
                while (iter.previousIndex() > 1 && iter.previous().getDatum().time > datum.time) ;

                iter.add(new PostPredictionState(null, null, datum, false));
                replayHistory(iter.previousIndex() - 1);
            }
        }

    }

    private void predict(KalmanDatum datum, int index) {
        if (!datum.isFullState())
            throw new RuntimeException("Prediction data must be full-state.  Perhaps you could pass in 0 for the parameters you don't want to muck with.");

        synchronized (lock) {
            mean = mean.add(datum.outputToState().multiply(datum.getMean()));
            covariance = covariance.add(propagateError(datum.outputToState(), datum.getCovariance()));

            var state = new PostPredictionState(mean, covariance, datum, false);
            if (index >= history.size())
                pushState(state);
            else
                history.set(index, state);

            chain();
        }
    }

    private void pushState(PostPredictionState state) {
        synchronized (lock) {
            history.add(state);
            if (history.size() > MOVING_WINDOW_SIZE)
                for (int i = 0; i < (history.size() - MOVING_WINDOW_SIZE); i++)
                    history.removeFirst();
        }
    }

    public double[] outputVector() {
        synchronized (lock) {
            return mean.columns().next().toRawCopy1D();
        }
    }

    private Primitive64Matrix innovation(Primitive64Matrix stateToOutput, Primitive64Matrix output) {
        synchronized (lock) {
            return output.subtract(stateToOutput.multiply(mean));
        }
    }

    private Primitive64Matrix stateCorrectionCovariance(Primitive64Matrix stateToOutput,
                                                        Primitive64Matrix outputCovariance) {
        synchronized (lock) {
            assertThat(outputCovariance.isSquare() && outputCovariance.getDeterminant() != 0);
            return covariance.multiply(stateToOutput.transpose()).multiply(propagateError(stateToOutput,
                    covariance).add(outputCovariance).invert());
        }
    }

    public void correction(KalmanDatum datum) {
        synchronized (lock) {
            correction(datum, history.size() - 1);
        }
    }

    private void correction(KalmanDatum datum, int index) {
        synchronized (lock) {
            var W = stateCorrectionCovariance(datum.getStateToOutput(), datum.getCovariance());

            var innovation = innovation(datum.getStateToOutput(), datum.getMean());

            mean = mean.add(W.multiply(innovation));
            covariance = covariance.subtract(propagateError(W, propagateError(datum.getStateToOutput(), covariance)));

            var state = new PostPredictionState(mean, covariance, datum, true);
            if (index >= history.size())
                pushState(state);
            else
                history.set(index, state);

            chain();
        }
    }

    private void chain() {
        synchronized (lock) {
            if (chain != null) {
                var now = SystemClock.elapsedRealtime();
                if (lastChainPrediction != -1)
                    chain.predict(new KalmanDatum(mean.multiply((now - lastChainPrediction) / 1000.), covariance));

                lastChainPrediction = now;
            }
        }
    }

    @Override
    public List<String> getTelemetryData() {
        return new ArrayList<>() {{
            add(Arrays.toString(outputVector()));
            add(covariance.toString());
        }};
    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public boolean isOn() {
        return false;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        return Telemeter.super.getDashboardData();
    }

    static class PostPredictionState {
        private final Primitive64Matrix mean;
        private final Primitive64Matrix covariance;
        private final KalmanDatum datum;
        private final boolean isCorrection;

        PostPredictionState(Primitive64Matrix mean, Primitive64Matrix covariance, KalmanDatum datum, boolean isCorrection) {
            this.mean = mean;
            this.covariance = covariance;
            this.datum = datum;
            this.isCorrection = isCorrection;
        }

        public Primitive64Matrix getMean() {
            return mean;
        }

        public Primitive64Matrix getCovariance() {
            return covariance;
        }

        public KalmanDatum getDatum() {
            return datum;
        }

        public boolean isCorrection() {
            return isCorrection;
        }
    }
}
