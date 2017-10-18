package com.carnot.kalmanapp.Activities;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Environment;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ScrollView;

import com.androidplot.xy.BoundaryMode;
import com.androidplot.xy.LineAndPointFormatter;
import com.androidplot.xy.RectRegion;
import com.androidplot.xy.SimpleXYSeries;
import com.androidplot.xy.XYPlot;
import com.androidplot.xy.XYSeries;
import com.carnot.kalmanapp.Interfaces.KalmanFilter;
import com.carnot.kalmanapp.R;
import com.carnot.kalmanapp.Utilities.GlobalVars;
import com.carnot.kalmanapp.Utilities.KalmanFilterEquation;
import org.ejml.data.DMatrixRMaj;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.Calendar;

public class MainActivity extends AppCompatActivity {

    Button startCapturing;
    Button endCapturing;
    Button viewComparison;
    Button backFromGraphButton;

    FrameLayout viewGraph;
    ScrollView scrollView;
    XYPlot plot;
    XYPlot plot1;
    SimpleXYSeries series1;
    SimpleXYSeries series2;

    LineAndPointFormatter series1Format;
    LineAndPointFormatter series2Format;


    BufferedReader reader;
    KalmanFilter f;

    DMatrixRMaj RMat;

    boolean start = false;
    boolean isFirst = true;
    boolean isGps = true;

    public static File usbFile = null;
    public static OutputStreamWriter usbOutputStreamWriter = null;
    public static FileOutputStream usbFos = null;

    public static final String ROOT_PATH_FOR_SAVING_TRIP_FOLDERS = Environment.getExternalStorageDirectory() + File.separator + GlobalVars.ROOT_FOLDER_NAME;
    public static final String DEFAULT_FILE_NAME_FOR_RAW_USB_DATA = "rawData";
    public static String TRIP_FOLDER_NAME = "";

    public void initLocationManager() {

        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    ActivityCompat#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for ActivityCompat#requestPermissions for more details.
            return;
        }

        // Acquire a reference to the system Location Manager
        LocationManager locationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);

// Define a listener that responds to location updates
        LocationListener locationListener = new LocationListener() {
            public void onLocationChanged(Location location) {
                // Called when a new location is found by the network location provider.
                //makeUseOfNewLocation(location);
                Log.e("Location",location.getLatitude() + "-" + location.getLongitude() +"," + location.getSpeed());

                if (GlobalVars.lastGpsTime != 0) {
                    GlobalVars.deltaTime = (location.getTime() - GlobalVars.lastGpsTime) / 1000.0;

                }
                GlobalVars.lastGpsTime = location.getTime();
                try {
                    if (usbOutputStreamWriter != null) {
                        usbOutputStreamWriter.write(location.getLatitude() + "," + location.getLongitude() +
                                "," + location.getBearing() + "," + location.getSpeed() + "," +
                                GlobalVars.deltaTime + "\n");
                    }

                }catch (Exception e) {
                    e.printStackTrace();
                }

                double useBearing = location.getBearing() * (3.14 / 180.0);
                if (start) {
                    DMatrixRMaj measureX = new DMatrixRMaj(4,1, true, location.getLatitude(),
                            location.getLongitude(),
                            useBearing, location.getSpeed());
                    if (isFirst) {
                        isFirst = false;

                        initKalmanProcss(measureX,GlobalVars.deltaTime,useBearing);
                    } else {
                        kalmanProcess(location.getLatitude(),
                                location.getLongitude(), location.getBearing(),
                                location.getSpeed(), GlobalVars.deltaTime,RMat);
                    }
                }
            }

            public void onStatusChanged(String provider, int status, Bundle extras) {}

            public void onProviderEnabled(String provider) {}

            public void onProviderDisabled(String provider) {}
        };

        // Register the listener with the Location Manager to receive location updates
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, locationListener);

    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        scrollView = (ScrollView) findViewById(R.id.scrol_view);
        viewGraph = (FrameLayout) findViewById(R.id.graph_view_layout);
        backFromGraphButton = (Button) findViewById(R.id.to_hide_view);
        backFromGraphButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                viewGraph.setVisibility(View.GONE);
                scrollView.setVisibility(View.VISIBLE);
            }
        });
        plot = (XYPlot) findViewById(R.id.plot);
        plot1 = (XYPlot) findViewById(R.id.plot1);


        RMat = new DMatrixRMaj(4,4,true,1.51,0,0,0,
                0,5.58,0,0,
                0,0,1.95,0,
                0,0,0,1.68);

        initLocationManager();

        createRootDirectory();

        startCapturing = (Button) findViewById(R.id.start_capturing);
        startCapturing.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (isGps) {
                    start = true;
                    Calendar cal = Calendar.getInstance();
                    TRIP_FOLDER_NAME = cal.get(Calendar.DATE) + "z" + (cal.get(Calendar.MONTH) + 1) + "z" + cal.get(Calendar.HOUR_OF_DAY)
                            + "z" + cal.get(Calendar.MINUTE) + "z" + cal.get(Calendar.SECOND);
                    File createTripFolder = new File(ROOT_PATH_FOR_SAVING_TRIP_FOLDERS, TRIP_FOLDER_NAME);

                    if (!createTripFolder.exists()) {
                        createTripFolder.mkdirs();
                    }

                    String PATH_FOR_SAVING_FILES = ROOT_PATH_FOR_SAVING_TRIP_FOLDERS + File.separator
                            + TRIP_FOLDER_NAME;
                    try {
                        usbFile = new File(PATH_FOR_SAVING_FILES, DEFAULT_FILE_NAME_FOR_RAW_USB_DATA + ".txt");
                        usbFos = new FileOutputStream(usbFile);
                        usbOutputStreamWriter = new OutputStreamWriter(usbFos);

                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                } else {
                    try {
                        InputStream file = getAssets().open(GlobalVars.DEFAULT_FILE_NAME);
                        reader = new BufferedReader(new InputStreamReader(file));

                        String line = reader.readLine();
                        String[] val = line.split(",");

                        double x_gps = Double.parseDouble(val[0]);
                        double y_gps = Double.parseDouble(val[1]);
                        //in radians
                        double heading_gps = Double.parseDouble(val[2]) * (3.14 / 180.0);
                        double speed_gps = Double.parseDouble(val[3]);
                        double deltaT = Double.parseDouble(val[4]);
                        DMatrixRMaj priorX = new DMatrixRMaj(4,1, true, x_gps, y_gps, heading_gps, speed_gps);
                        initKalmanProcss(priorX,deltaT,heading_gps);


                        while ((line = reader.readLine())!=null) {
                            val = line.split(",");
                            kalmanProcess(Double.parseDouble(val[0]),Double.parseDouble(val[1])
                                    ,Double.parseDouble(val[2]) * (3.14 / 180.0),Double.parseDouble(val[3]),
                                    Double.parseDouble(val[4]),RMat);
                        }
                    }catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        });

        viewComparison = (Button) findViewById(R.id.view_comparison);
        viewComparison.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                viewGraph.setVisibility(View.VISIBLE);
                scrollView.setVisibility(View.GONE);
                //series1 = generateScatter("series1", 80, new RectRegion(-100, 100, -100, 100));
                //series2 = generateScatter("series2", 80, new RectRegion(-100, 100, -100, 100));

                series1 = new SimpleXYSeries("series1");
                series2 = new SimpleXYSeries("series2");

                series1Format =
                        new LineAndPointFormatter(getApplicationContext(), R.xml.point_formatter);

                series2Format =
                        new LineAndPointFormatter(getApplicationContext(), R.xml.point_formatter_2);

                plot.setDomainBoundaries(-10, 10, BoundaryMode.FIXED);
                plot.setRangeBoundaries(-50, 50, BoundaryMode.FIXED);

                plot1.setDomainBoundaries(-10, 10, BoundaryMode.FIXED);
                plot1.setRangeBoundaries(-50, 50, BoundaryMode.FIXED);

                plot1.setLinesPerRangeLabel(2);
                // reduce the number of range labels
                plot.setLinesPerRangeLabel(2);

            }
        });

        endCapturing = (Button) findViewById(R.id.end_capturing);
        endCapturing.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                start = false;
            }
        });
    }

    public void createRootDirectory() {
        File createRootFolder = new File(Environment.getExternalStorageDirectory(), GlobalVars.ROOT_FOLDER_NAME);

        if (!createRootFolder.exists()) {
            createRootFolder.mkdirs();
        }

    }

    public void initKalmanProcss(DMatrixRMaj priorX,double deltaT, double posteriorTheta) {

        //Convert posteriorTheta into Radians and user.

        DMatrixRMaj F_k = makeF_KMatrix(deltaT,posteriorTheta);

        DMatrixRMaj H_k = new DMatrixRMaj(4,4,true,1,0,0,0,
                                                   0,1,0,0,
                                                   0,0,1,0,
                                                   0,0,0,1);

        DMatrixRMaj Q = new DMatrixRMaj(4,4,true,0.23,0,0,0,
                                                 0,0.26,0,0,
                                                 0,0,0.01,0,
                                                 0,0,0,1.05);

        DMatrixRMaj P_0_posterior = new DMatrixRMaj(4,4,true,3.7,0,0,0,
                                                             0,6.4,0,0,
                                                             0,0,3.7,0,
                                                             0,0,0,6.7);

        f = new KalmanFilterEquation();
        f.configure(F_k,Q,H_k);
        f.setState(priorX,P_0_posterior,F_k);

    }

    public DMatrixRMaj makeF_KMatrix(double deltaT, double posteriorTheta) {
        return new DMatrixRMaj(4,4,true,1,0,0,(deltaT*Math.sin(posteriorTheta)),
                0,1,0,(deltaT*Math.cos(posteriorTheta)),
                0,0,1,0,
                0,0,0,1);
    }

    double shiftOriginX = -999;
    double getShiftOriginY = -999;
    public void kalmanProcess(double xGps, double yGps, double headingGps, double speedGps,
                              double deltaT, DMatrixRMaj RMat) {

        DMatrixRMaj measuredX = new DMatrixRMaj(4,1, true, xGps, yGps, headingGps, speedGps);

        f.predict(makeF_KMatrix(deltaT,headingGps));
        f.update(measuredX,RMat);

        DMatrixRMaj newX = f.getState();
        newX.print("%e");
        Log.e("Predicted Lat: ",newX.get(0,0) + "-" + xGps);
        Log.e("Predicted Lon: ",newX.get(1,0) + "-" + yGps);
        Log.e("Predicted Speed: ",newX.get(2,0) + "-" + speedGps);

        System.out.println();
        if (viewGraph.getVisibility() == View.VISIBLE) {
            if (shiftOriginX == -999) {
                shiftOriginX = newX.get(0,0);
                getShiftOriginY = newX.get(1,0);
            }
            if (series1.size() > 100) {
                plot.clear();
                plot1.clear();
                plot.invalidate();
                plot1.invalidate();
            }
            series1.addLast( (newX.get(0,0) - shiftOriginX)*10000000, (newX.get(1,0)- getShiftOriginY)*10000000);
            series2.addLast( (xGps - shiftOriginX)*10000000 , (yGps - getShiftOriginY)*10000000);

            plot.addSeries(series1Format,series1);
            plot1.addSeries(series2Format,series2);

            plot.invalidate();
            plot1.invalidate();
            Log.e("Diff: ","(" + (newX.get(0,0) - shiftOriginX)*10000000 + "," + (newX.get(1,0)- getShiftOriginY)*10000000 +")" + " - " +
                     "(" + (xGps - shiftOriginX)*10000000 +"," + (yGps - getShiftOriginY)*10000000 +")" );
        } else {
            shiftOriginX = -999;
            getShiftOriginY = -999;

            plot.clear();
            plot1.clear();

            plot.invalidate();
            plot1.invalidate();
        }
    }

    private XYSeries generateScatter(String title, int numPoints, RectRegion region) {
        SimpleXYSeries series = new SimpleXYSeries(title);
        for(int i = 0; i < numPoints; i++) {
            series.addLast(
                    region.getMinX().doubleValue() + (Math.random() * region.getWidth().doubleValue()),
                    region.getMinY().doubleValue() + (Math.random() * region.getHeight().doubleValue())
            );
        }
        return series;
    }
}
