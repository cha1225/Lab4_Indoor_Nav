package lab4_203_29.uwaterloo.ca.lab4_retry;

import android.graphics.PointF;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.os.Bundle;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.ContextMenu;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import java.lang.Math;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class MainActivity extends AppCompatActivity implements SensorEventListener {
    float azimut;
    private SensorManager mSensorManager;
    Sensor mLinearAcceleration, mRotationVectorSensor;
    TextView bearing, stepField,instructions;
    Mapper mv;
    PedometerMap map;
    MapLoader ml = new MapLoader();
    DecimalFormat df = new DecimalFormat("##.##");
    PointF userStart, userEnd;
    List<PointF> route = new ArrayList<>();
    List<InterceptPoint> collisionPoints = new ArrayList<InterceptPoint>();
    double xDoublePosition, yDoublePosition;
    float direction, distance;
    int count = 1;
    boolean hit= false;
    FloatHelper vector = new FloatHelper();
    int steps;
    int directionAngle;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        ///Sensor Implementation
        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        mLinearAcceleration = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        ////
        mv = new Mapper(getApplicationContext(), 1400, 1000, 40, 40);
        registerForContextMenu(mv);
        map = ml.loadMap(getExternalFilesDir(null), "E2-3344.svg");
        mv.setMap(map);
        ////
        RelativeLayout RelativeLayout = (RelativeLayout) findViewById(R.id.relativelayout);
        RelativeLayout.addView(mv);
        mv.setVisibility(View.VISIBLE);
        /////
       /* PointF test1 = new PointF (5, 19);
        PointF test2 = new PointF (5, 18);
        route.add(test1);
        route.add(test2);
        mv.setUserPath(route);*/
        stepField = (TextView) findViewById(R.id.steps);
        bearing = (TextView) findViewById(R.id.bearing);
        instructions = (TextView) findViewById(R.id.instructions);
        mv.addListener(new IMapperListener() {
            @Override
            public void locationChanged(Mapper source, PointF loc) {
                source.setUserPoint(loc);
            }

            @Override
            public void DestinationChanged(Mapper source, PointF dest) {
                source.setDestination(dest);
                routeDrawer();
                hit = true;
            }

        });
    }
   public Void routeDrawer () { //This is the routedrawer
       route.clear();
        userStart = mv.getUserPoint(); //Get the start point or where the user is currently is at
        userEnd = mv.getEndPoint(); //Get the end point
        collisionPoints = mv.calculateIntersections(userStart, userEnd); //This is to calculate the collision points
        if (collisionPoints.isEmpty()) { //If there is no collision we know that there is a direct path
            route.add(userStart);
            route.add(userEnd);
            mv.setUserPath(route);
        }
        else {
            float increment; //Variable for incrementing
            if(userStart.y > userEnd.y) { //Determine which one is higher
                increment = userStart.y;
            }
            else {
                increment = userEnd.y;
            }
            PointF startTemp = new PointF(userStart.x, increment);
            PointF endTemp = new PointF(userEnd.x, increment); //Keep going down until there isn't a wall between them
            Boolean moved = false;
            while(!mv.calculateIntersections(startTemp, endTemp).isEmpty()) { //Go to the very bottom until they meet
                increment += .5; //Add because the top is y=0
                startTemp = new PointF(userStart.x, increment);
                endTemp = new PointF(userEnd.x, increment);
                moved = true;
            }

            route.add(mv.getUserPoint()); //Add the pointFs in order to get the route
            if(!moved){
                if(userStart.y > userEnd.y) { //Determine which one is higher
                    route.add(endTemp);
                }
                else {
                    route.add(startTemp);
                }
            } else {
                route.add(startTemp);
                route.add(endTemp);
            }
            route.add(userEnd);
            mv.setUserPath(route);
        }
        return null;
    }

    @Override
    public void onCreateContextMenu(ContextMenu menu, View v, ContextMenu.ContextMenuInfo menuInfo) {
        super.onCreateContextMenu(menu, v, menuInfo);
        mv.onCreateContextMenu(menu, v, menuInfo);
    }

    @Override
    public boolean onContextItemSelected(MenuItem item) {
        return super.onContextItemSelected(item) || mv.onContextItemSelected(item);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    protected void onResume() {
        super.onResume();
        mSensorManager.registerListener(this, mLinearAcceleration, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mRotationVectorSensor, SensorManager.SENSOR_DELAY_NORMAL);
    }

    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }
    public void onAccuracyChanged(Sensor sensor, int accuracy) {  }
    int stepCounter, stepState;
    long start, interval;
    float orientation [] = new float [3];
    float[] maxAccelerationVal = new float[3];
    double totalPositionX, totalPositionY;
    public void onSensorChanged(SensorEvent event) {
        //This is the state machine for the step counting
        if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
            for (int x = 0; x < maxAccelerationVal.length; x++) {
                if (Math.abs(maxAccelerationVal[x]) < Math.abs(event.values[x]))
                    maxAccelerationVal[x] = event.values[x];
            }
            if (stepState == 0 && event.values[2] > 1.3 && event.values[2] < 5) {
                stepState = 1;  //This means that we're climbing the graph currently
                start = System.currentTimeMillis(); //Getting the system time to see the time it takes for the data to go down
            }
            if (stepState == 1 && event.values[2] < 1.2) { //So if the Z data rised up and down a certain acc
                interval = System.currentTimeMillis() - start;
                if (interval > 80) {
                    stepCounter++;
                    calcDisplacement();
                }
                stepState = 0;
                stepField.setText("The number of steps:" + stepCounter);
            }
            //The cases where it's not steps
            if (event.values[2] > 10) { //If the phone is being shaken hard, then it won't
                stepState = -1;    //Consider steps
            } else if (stepState == -1 && event.values[2] < 1) {
                stepState = 0; //Reset the state if you think that it's not being shaken
            }
        }
        //Implementation taken from stack overflow
        if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            float [] mRotationMatrix = new float[9];
            SensorManager.getRotationMatrixFromVector(mRotationMatrix, event.values);
            SensorManager.getOrientation(mRotationMatrix, orientation);
            azimut = orientation[0]; //Gives a displacement from -pi to 2pi
            bearing.setText("The bearing is:" + azimut);
        }
        if(hit) {
            calcDisplacement(false);
            HelperFunction();
            instructions.setText("Steps:" + steps + " Turn:" + directionAngle + " from direction");
        }
    }

    public void calcDisplacement() {
        calcDisplacement(true);
    }

    public void calcDisplacement(Boolean step) {

        xDoublePosition = Math.sin(azimut)*1.1; //Make a right angle triangle, multiply it by 0.5 because half a meter is fair for distance
        yDoublePosition = -Math.cos(azimut)*1.1;//

        if(step){
            totalPositionX += xDoublePosition;
            totalPositionY += yDoublePosition;

            userLocation();
        }
    }

    public void userLocation () {
        PointF currentLocation = new PointF(0,0);
        PointF temp = new PointF(0,0);
        currentLocation = mv.getUserPoint();
        temp = new PointF(currentLocation.x, currentLocation.y);
        List<InterceptPoint> collision = new ArrayList<InterceptPoint>();
        currentLocation.x += xDoublePosition;
        currentLocation.y += yDoublePosition;
        collision = mv.calculateIntersections(temp, currentLocation);
        if (collision.isEmpty()) { //If there is no wall between the temp and the current point update it
            mv.setUserPoint(currentLocation);
        } else {
            mv.setUserPoint(temp); //If there is a wall go back to the original point
        }
        ///Update instructions to direct the user to the destination
        if (vector.distance(mv.getUserPoint(), mv.getEndPoint()) <= 1) {
            //instructions.setText("You have reached your destination.");
            Toast.makeText(this, "You have reached your destination.", Toast.LENGTH_SHORT).show();
        //} else if (collision.isEmpty()){ //This is the case where it's just a straight line
        //    HelperFunction(0);
        }/* else { //If there are walls between the things, we need to go through basically 3 destiantions
            HelperFunction();
        }*/
        //Finally, update the new route
        routeDrawer();
    }
    public void HelperFunction () {
        PointF tempBearing = new PointF (mv.getUserPoint().x + (float)xDoublePosition, mv.getUserPoint().y + (float)yDoublePosition);
        PointF whichOne = new PointF (0,0);
        whichOne = route.get(count);

        direction = vector.angleBetween(mv.getUserPoint(), tempBearing, whichOne);
        distance = vector.distance(mv.getUserPoint(), whichOne);

        steps = (int)Math.ceil(distance/0.65);
        directionAngle = (int)Math.round(direction * 180 / Math.PI);
    }
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }
}
