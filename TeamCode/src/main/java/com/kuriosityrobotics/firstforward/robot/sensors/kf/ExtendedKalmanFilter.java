package com.kuriosityrobotics.firstforward.robot.sensors.kf;

import static com.kuriosityrobotics.firstforward.robot.Robot.assertThat;

import android.os.SystemClock;

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

    public void reset(double[] initialState, double... initialVariance) {
        history.clear();
        mean = Primitive64Matrix.FACTORY.column(initialState);
        covariance = diagonal(initialVariance);

        history.add(new PostPredictionState(mean, covariance, null, false));
    }

    public void reset(double... initialState) {
        reset(initialState, new double[initialState.length]);
    }

    public void forwardPass(int startingFrom) {
        synchronized (lock) {
//            Log.d("EKF", "Replayed " + (history.size() - startingFrom - 1) + "measurements.");
            var iter = history.listIterator(startingFrom);
            var state = iter.next();
            this.mean = state.getMean();
            this.covariance = state.getCovariance();

            while (iter.hasNext()) {
                var idx = iter.nextIndex();
                var measurement = iter.next();

                if (measurement.isCorrection())
                    correct(measurement.getDatum(), idx);
                else
                    predict(measurement.getDatum(), idx);
            }
        }
    }

    @SuppressWarnings("ConstantConditions")
    private void backwardPass(int before) {
        synchronized (lock) {
            var smoothedMeans = new HashMap<PostPredictionState, Primitive64Matrix>();
            var smoothedVariances = new HashMap<PostPredictionState, Primitive64Matrix>();

            history.forEach(n -> {
                smoothedMeans.put(n, n.getMean());
                smoothedVariances.put(n, n.getCovariance());
            });

            if (!history.get(before).isCorrection())
                throw new IllegalArgumentException("Can only smoothe starting from a correction.");

            assertThat(history.get(before).isCorrection() && history.get(before).getDatum().isFullState());
            var latestCorrectionH = history.get(before).getDatum().getStateToOutput();

            for (int t = before - 1; t >= 0; t--) {
                PostPredictionState currentState = history.get(t);
                var P_t = currentState.getCovariance();
                PostPredictionState nextState = history.get(t + 1);
                var P_t1 = nextState.getCovariance();

//                Log.d("EKF", format("curr:  {0}, next:  {1}", currentState.isCorrection, nextState.isCorrection));

                assertThat(P_t.isSquare());
//                Log.d("EKF", format("latestCorrectionH:  {0}", Arrays.deepToString(latestCorrectionH.toRawCopy2D())));
                assertThat(P_t.multiply(
                        latestCorrectionH.transpose()
                ).isSquare());
                assertThat(P_t1.isSquare());

                var L = P_t.multiply(
                        latestCorrectionH.transpose()
                ).multiply(P_t1.invert());

                var X_t = currentState.getMean();
                var X_tT = X_t.add(
                        L.multiply(
                                smoothedMeans.get(nextState).subtract(nextState.getMean())
                        )
                );

                var P_tT = P_t.add(propagateError(L,
                        smoothedVariances.get(nextState).subtract(nextState.getCovariance())
                ));

                smoothedMeans.put(currentState, X_tT);
                smoothedVariances.put(currentState, P_tT);
            }

            history.forEach(state -> {
                state.setMean(smoothedMeans.get(state));
                state.setCovariance(smoothedVariances.get(state));
            });
        }
    }

    public void predict(KalmanDatum datum) {
        synchronized (lock) {
            int nextIndex = getInsertionIndex(datum);

            pushState(nextIndex, new PostPredictionState(null, null, datum, false));
            forwardPass(nextIndex - 1); // TODO:  might have introduced a bug changing this from - 2 to -1
        }
    }

    private void predict(KalmanDatum datum, int index) {
        synchronized (lock) {
            mean = mean.add(datum.outputToState().multiply(datum.getMean()));
            covariance = covariance.add(propagateError(datum.outputToState(), datum.getCovariance()));

            var state = new PostPredictionState(mean, covariance, datum, false);
            history.set(index, state);
        }
    }

    public void correct(KalmanDatum datum) {
        synchronized (lock) {
            int nextIndex = getInsertionIndex(datum);

            pushState(nextIndex, new PostPredictionState(null, null, datum, true));
            // nextIndex - 1 because we need to calculate state for the newly-inserted datum
            forwardPass(nextIndex - 1); // TODO:  might have introduced a bug changing this from - 2 to -1
            var latest = findLatestCorrection();
            if (latest != -1)
                backwardPass(latest);
        }
    }

    private int findLatestCorrection() {
        synchronized (lock) {
            var iter = history.listIterator(history.size() - 1);
            PostPredictionState thing = null;
            while (iter.hasPrevious() && !(thing = iter.previous()).isCorrection() && !thing.getDatum().isFullState())
                ;
            if (thing == null || !thing.isCorrection() || !thing.getDatum().isFullState())
                return -1;
            else
                return iter.nextIndex();
        }
    }

    private void correct(KalmanDatum datum, int index) {
        synchronized (lock) {
            var W = stateCorrectionCovariance(datum.getStateToOutput(), datum.getCovariance());

            var innovation = innovation(datum.getStateToOutput(), datum.getMean());

            mean = mean.add(W.multiply(innovation));
            covariance = covariance.subtract(propagateError(W, propagateError(datum.getStateToOutput(), covariance)));

            var state = new PostPredictionState(mean, covariance, datum, true);
            history.set(index, state);
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

    private Primitive64Matrix innovation(Primitive64Matrix stateToOutput, Primitive64Matrix output) {
        synchronized (lock) {
            return output.subtract(stateToOutput.multiply(mean));
        }
    }

    private int getInsertionIndex(KalmanDatum datum) {
        var iter = history.listIterator(history.size());
        while (iter.previousIndex() >= 1 && iter.previous().getDatum().time > datum.time) ;
        return iter.nextIndex();
    }

    private void pushState(int index, PostPredictionState state) {
        synchronized (lock) {
            if (history.size() > MOVING_WINDOW_SIZE)
                for (int i = 0; i < (history.size() - MOVING_WINDOW_SIZE); i++) {
                    assertThat(history.get(i).getMean() != null);
                    history.removeFirst();
                }

            history.add(index, state);
        }
    }

    public double[] outputVector() {
        synchronized (lock) {
            return mean.columns().next().toRawCopy1D();
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
        private final boolean correction;
        private Primitive64Matrix mean;
        private Primitive64Matrix covariance;


        PostPredictionState(Primitive64Matrix mean, Primitive64Matrix covariance, KalmanDatum datum, boolean correction) {
            this.setMean(mean);
            this.setCovariance(covariance);
            this.datum = datum;
            this.correction = correction;
        }

        public Primitive64Matrix getMean() {
            return mean;
        }

        public void setMean(Primitive64Matrix mean) {
            this.mean = mean;
        }

        public Primitive64Matrix getCovariance() {
            return covariance;
        }

        public void setCovariance(Primitive64Matrix covariance) {
            this.covariance = covariance;
        }

        public KalmanDatum getDatum() {
            return datum;
        }

        public boolean isCorrection() {
            return correction;
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

            System.out.println("prediction:  " + datum);
            ExtendedKalmanFilter.this.predict(datum);
        }

        public void correct() {
            var datum = build();
            System.out.println("correction:  " + datum);
            ExtendedKalmanFilter.this.correct(datum);
        }
    }
}
