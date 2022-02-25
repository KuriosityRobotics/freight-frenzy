package com.kuriosityrobotics.firstforward.robot.sensors.KalmanFilter;

import android.os.SystemClock;

import org.apache.commons.math3.linear.RealMatrix;

import java.util.Comparator;
import java.util.LinkedList;

public class KalmanGoodieBag {

    private LinkedList<KalmanGoodie> goodieBag;

    public KalmanGoodieBag(LinkedList<KalmanGoodie> goodieBag) {
        this.goodieBag = goodieBag;
    }

    public KalmanGoodieBag(KalmanState startState){
        this.goodieBag = new LinkedList<>();
        goodieBag.add(new KalmanGoodie(null, SystemClock.elapsedRealtime(), startState));
    }

    public KalmanGoodieBag(RealMatrix startMean, RealMatrix startCov){
        this.goodieBag = new LinkedList<>();
        goodieBag.add(new KalmanGoodie(null, SystemClock.elapsedRealtime(), new KalmanState(startMean, startCov)));
    }

    public void addGoodie(KalmanGoodie goodie){
        goodieBag.add(goodie);
    }

    public void addGoodie(KalmanData data, long timeStamp){
        goodieBag.add(new KalmanGoodie(data,timeStamp));
    }

    public KalmanGoodie getGoodie(int i){
        return goodieBag.get(i);
    }

    public KalmanGoodie getLastGoodie(){
        return goodieBag.get(goodieBag.size()-1);
    }

    public void setGoodie(int i, KalmanGoodie goodie){
        goodieBag.set(i, goodie);
    }

    public void setGoodieState(int i, KalmanState state){
        KalmanGoodie prevGoodie = goodieBag.get(i);
        goodieBag.set(i, new KalmanGoodie(prevGoodie.getData(), prevGoodie.getTimeStamp(), state));
    }

    public int getBagSize(){
        return goodieBag.size();
    }

    public void sortGoodieBag(){
        goodieBag.sort(Comparator.comparing(KalmanGoodie::getTimeStamp));
    }

    public void refreshGoodieBag(long currentTime, long timeWindow){
        goodieBag.removeIf(n -> (n.getTimeStamp() > currentTime || n.getTimeStamp() < currentTime - timeWindow));
    }

    public void updateGoodieBag(long currentTime, long timeWindow){
        refreshGoodieBag(currentTime, timeWindow);
        sortGoodieBag();
    }
}
