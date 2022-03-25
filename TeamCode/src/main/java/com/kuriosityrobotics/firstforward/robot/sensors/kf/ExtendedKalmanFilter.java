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

    /**
     * @param initialState starting state
     */
    public ExtendedKalmanFilter(double... initialState) {
        reset(initialState);
    }

    public ExtendedKalmanFilter(double[] initialState, double... initialVariance) {
        reset(initialState, initialVariance);
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

    public double[] getVariance() {
        return covariance.diagonal().toRawCopy1D();
    }

    public void reset(double[] initialState, double... initialVariance) {
        var variables = initialState.length;

        history.clear();
        mean = Primitive64Matrix.FACTORY.column(initialState);
        covariance = diagonal(initialVariance);

        history.add(new PostPredictionState(mean, covariance, null, false));
    }

    public void reset(double... initialState) {
        reset(initialState, new double[initialState.length]);
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
        synchronized (lock) {
            mean = mean.add(datum.outputToState().multiply(datum.getMean()));
            covariance = covariance.add(propagateError(datum.outputToState(), datum.getCovariance()));

            var state = new PostPredictionState(mean, covariance, datum, false);
            if (index >= history.size())
                pushState(state);
            else
                history.set(index, state);
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

        }
    }

    private void smoothe(KalmanDatum datum) {
        synchronized (this) {
            // TODO:  make this start when the kalmandatum was collected
            for (int t = history.size() - 2; t >= 0; t--) {
                PostPredictionState currentState = history.get(t);
                var P_t = currentState.getCovariance();
                PostPredictionState nextState = history.get(t + 1);
                var P_t1 = nextState.getCovariance();

                var L = P_t.multiply(datum.getStateToOutput().transpose()).multiply(P_t1.invert());

                var X_t = currentState.getMean();
                var X_t1 = nextState.getMean();
                var X_tT = X_t.add(
                        L.multiply(
                                nextState.smoothedMean.subtract(nextState.mean)
                        )
                );

                var P_tT = P_t.add(propagateError(L,
                        nextState.smoothedCovariance.subtract(nextState.covariance)
                ));

                currentState.smoothedMean = X_tT;
                currentState.smoothedCovariance = P_tT;
            }

            history.forEach(state -> {
                state.mean = state.smoothedMean;
                state.covariance = state.smoothedCovariance;
            });
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

    public KalmanDatumBuilder datumBuilder() {
        return this.new KalmanDatumBuilder();
    }

    static class PostPredictionState {
        private final KalmanDatum datum;
        private final boolean isCorrection;
        public Primitive64Matrix smoothedMean, smoothedCovariance;
        private Primitive64Matrix mean;
        private Primitive64Matrix covariance;


        PostPredictionState(Primitive64Matrix mean, Primitive64Matrix covariance, KalmanDatum datum, boolean isCorrection) {
            this.mean = mean;
            this.smoothedMean = mean;
            this.covariance = covariance;
            this.smoothedCovariance = covariance;
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

    @SuppressWarnings("unused")
    public class KalmanDatumBuilder {
        private Long time = SystemClock.elapsedRealtime();
        private Primitive64Matrix mean, covariance, stateToOutput;

        public KalmanDatumBuilder mean(double... mean) {
            this.mean = Primitive64Matrix.FACTORY.column(mean);
            return this;
        }

        public KalmanDatumBuilder mean(Primitive64Matrix mean) {
            this.mean = mean;
            return this;
        }

        public KalmanDatumBuilder variance(double... variance) {
            this.covariance = diagonal(variance);
            return this;
        }

        public KalmanDatumBuilder covariance(Primitive64Matrix covariance) {
            this.covariance = covariance;
            return this;
        }

        public KalmanDatumBuilder stateToOutput(Primitive64Matrix stateToOutput) {
            this.stateToOutput = stateToOutput;
            return this;
        }

        public KalmanDatumBuilder outputToState(Primitive64Matrix outputToState) {
            this.stateToOutput = outputToState.invert();
            return this;
        }

        public KalmanDatumBuilder time(long time) {
            this.time = time;
            return this;
        }

        private KalmanDatum build() {
            if (mean == null)
                throw new IllegalArgumentException("Mean must not be null.");
            if (covariance == null)
                throw new IllegalArgumentException("Covariance must not be null.");
            if (stateToOutput == null)
                stateToOutput = Primitive64Matrix.FACTORY.makeIdentity(mean.getRowDim());

            if (!covariance.isSquare())
                throw new IllegalArgumentException("Covariance must be square.");

//            if (covariance.getRank() != covariance.getMinDim())
//                throw new IllegalArgumentException("Covariance must be invertible.");

            if (covariance.getRowDim() != mean.getRowDim())
                throw new IllegalArgumentException("Covariance does not fit mean.");

            if (!covariance.isSymmetric())
                throw new IllegalArgumentException("Covariance matrix must be symmetrical.");

            if (stateToOutput.getColDim() != ExtendedKalmanFilter.this.mean.getRowDim())
                throw new IllegalArgumentException("State to output matrix does not fit filter.");

            return new KalmanDatum(time, mean, covariance, stateToOutput);
        }

        public void predict() {
            var datum = build();
            if (!(datum.isFullState() && mean.getRowDim() == ExtendedKalmanFilter.this.mean.getRowDim()))
                throw new RuntimeException("Prediction data must be full-state.  Perhaps you could pass in 0 for the parameters you don't want to muck with.");

            ExtendedKalmanFilter.this.predict(datum);
        }

        public void correct() {
            ExtendedKalmanFilter.this.correction(build());
        }
    }
}
