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

    public KalmanGoodieBag(){
        this.goodieBag = new LinkedList<>();
    }

    public void clearGoodieBag(){
        synchronized (goodieBag){
            goodieBag.clear();
        }
    }

    public void addGoodieBag(KalmanGoodieBag newGoodieBag){
        synchronized (goodieBag){
            goodieBag.addAll(newGoodieBag.getLinkedList());
        }
    }

    public void addGoodie(KalmanGoodie goodie){
        synchronized (goodieBag){
            goodieBag.add(goodie);
        }
    }

    public void addGoodie(KalmanData data, long timeStamp){
        synchronized (goodieBag){
            goodieBag.add(new KalmanGoodie(data,timeStamp));
        }
    }

    public void addState(KalmanState state, long timeStamp){
        synchronized (goodieBag){
            goodieBag.add(new KalmanGoodie(null, timeStamp, state));
        }
    }

    public KalmanGoodie getGoodie(int i){
        synchronized (goodieBag){
            return goodieBag.get(i);
        }
    }

    public KalmanGoodie getLastGoodie(){
        synchronized (goodieBag){
            return goodieBag.get(goodieBag.size()-1);
        }
    }

    public void setGoodie(int i, KalmanGoodie goodie){
        synchronized (goodieBag){
            goodieBag.set(i, goodie);
        }
    }

    public void setGoodieState(int i, KalmanState state){
        synchronized (goodieBag){
            KalmanGoodie prevGoodie = goodieBag.get(i);
            goodieBag.set(i, new KalmanGoodie(prevGoodie.getData(), prevGoodie.getTimeStamp(), state));
        }
    }

    public int getBagSize(){
        synchronized (goodieBag){
            return goodieBag.size();
        }
    }

    public void sortGoodieBag(){
        synchronized (goodieBag){
            goodieBag.sort(Comparator.comparing(KalmanGoodie::getTimeStamp));
        }
    }

    public void refreshGoodieBag(long currentTime, long timeWindow){
        synchronized (goodieBag){
            goodieBag.removeIf(n -> (n.getTimeStamp() > currentTime || n.getTimeStamp() < currentTime - timeWindow));
        }
    }

    public void updateGoodieBag(long currentTime, long timeWindow){
        synchronized (goodieBag){
            if (goodieBag == null) return;
            refreshGoodieBag(currentTime, timeWindow);
            sortGoodieBag();
        }
    }

    public LinkedList<KalmanGoodie> getLinkedList(){
        return this.goodieBag;
    }
}
