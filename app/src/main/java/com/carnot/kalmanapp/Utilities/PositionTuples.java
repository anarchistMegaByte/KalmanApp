package com.carnot.kalmanapp.Utilities;

/**
 * Created by carnot on 06/12/2017 AD.
 */

public class PositionTuples {
    double latitude = 0.0;
    double longitude = 0.0;
    long timeStamp = 0; //in millisecs
    double deltaTime = 0; // in millisecs (Current - prev)
    double speed =0; // in m/s
    double Vlat = 0;
    double Vlon = 0;
    double acceleration =0;

    //Flags
    boolean isFiltered = false;
    boolean isAccelFiltered = false;
    boolean isRetrieved = false;
    boolean isInterpolated = false;

    public boolean isInterpolated() {
        return isInterpolated;
    }

    public void setInterpolated(boolean interpolated) {
        isInterpolated = interpolated;
    }

    public boolean isRetrieved() {
        return isRetrieved;
    }

    public void setRetrieved(boolean retrieved) {
        isRetrieved = retrieved;
    }

    public boolean isAccelFiltered() {
        return isAccelFiltered;
    }

    public void setAccelFiltered(boolean accelFiltered) {
        isAccelFiltered = accelFiltered;
    }

    public boolean isFiltered() {
        return isFiltered;
    }

    public void setFiltered(boolean filtered) {
        isFiltered = filtered;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    public double getVlon() {
        return Vlon;
    }

    public void setVlon(double vlon) {
        Vlon = vlon;
    }

    public double getVlat() {
        return Vlat;
    }

    public void setVlat(double vlat) {
        Vlat = vlat;
    }

    public double getDeltaTime() {
        return deltaTime;
    }

    public void setDeltaTime(double deltaTime) {
        this.deltaTime = deltaTime;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

    public long getTimeStamp() {
        return timeStamp;
    }

    public void setTimeStamp(long timeStamp) {
        this.timeStamp = timeStamp;
    }
}
