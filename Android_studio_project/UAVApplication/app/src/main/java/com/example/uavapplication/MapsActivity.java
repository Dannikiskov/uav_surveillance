package com.example.uavapplication;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.FragmentActivity;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.location.Address;
import android.location.Geocoder;
import android.location.Location;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Toast;

import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.GoogleMap.OnMyLocationButtonClickListener;
import com.google.android.gms.maps.GoogleMap.OnMyLocationClickListener;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptor;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.example.uavapplication.databinding.ActivityMapsBinding;
import com.google.android.gms.tasks.OnCompleteListener;
import com.google.android.gms.tasks.Task;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.Socket;
import java.sql.SQLOutput;
import java.util.List;
import java.util.Locale;

public class MapsActivity extends FragmentActivity implements
        OnMapReadyCallback,
        OnMyLocationButtonClickListener,
        OnMyLocationClickListener,
        ActivityCompat.OnRequestPermissionsResultCallback {

    private GoogleMap mMap;
    private ActivityMapsBinding binding;
    private BitmapDescriptor markerIcon;

    // Markers for drone zone
    private Marker dragMarker1;
    private Marker dragMarker2;

    // To get accurate device location
    private FusedLocationProviderClient fusedLocationProviderClient;
    private Location lastKnownLocation;

    // Socket for server connection
    private Socket client;
    private BufferedReader reader;
    private PrintWriter writer;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Construct a FusedLocationProviderClient.
        fusedLocationProviderClient = LocationServices.getFusedLocationProviderClient(this);

        binding = ActivityMapsBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        // Obtain the SupportMapFragment and get notified when the map is ready to be used.
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);

        // Button only shown when dragging
        findViewById(R.id.dragging_Done).setVisibility(View.INVISIBLE);
    }

    /**
     * Manipulates the map once available.
     * This callback is triggered when the map is ready to be used.
     * This is where we can add markers or lines, add listeners or move the camera. In this case,
     * we just add a marker near Sydney, Australia.
     * If Google Play services is not installed on the device, the user will be prompted to install
     * it inside the SupportMapFragment. This method will only be triggered once the user has
     * installed Google Play services and returned to the app.
     */
    @Override
    public void onMapReady(GoogleMap googleMap) {

        // Setting up Google maps
        mMap = googleMap;
        System.out.println("HOWDYPARTNER");
        mMap.setOnMyLocationButtonClickListener(this);
        mMap.setOnMyLocationClickListener(this);
        markerIcon = BitmapDescriptorFactory.fromBitmap(resizeMapIcons("red_circle", 50, 50));
        enableMyLocation();
        getUserLocation();

        // Add a marker in Sydney and move the camera
        LatLng odense = new LatLng(55.415643, 10.373886);
        LatLng copenhagen = new LatLng(55.676098, 12.568337);
        mMap.addMarker(new MarkerOptions()
                .position(odense)
                .title("x targets") // Title of marker
                .alpha(0.5f) // Opacity of the marker
                .anchor(0.5f, 0.5f) // Sets position to be middle of the marker
                .flat(true)
                .icon(markerIcon)); // Icon of the marker

        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(odense, 15f));
        //enableMyLocation();
        //getUserLocation(); // Update last known location
        //Log.i("lastknownlocation", "lat" + lastKnownLocation.getLatitude());
        dragMarker1 = mMap.addMarker(new MarkerOptions()
                .position(copenhagen)
                .title("draggable marker1")
                .draggable(true)
                .visible(false));
        dragMarker2 = mMap.addMarker(new MarkerOptions()
                .position(copenhagen)
                .title("draggable marker2")
                .draggable(true)
                .visible(false));

        // Listeners for the draggable markers. Allows the markers to update position after drag.
        if (mMap != null) {
            mMap.setOnMarkerDragListener(new GoogleMap.OnMarkerDragListener() {
                @Override
                public void onMarkerDrag(@NonNull Marker marker) {
                }

                @Override
                public void onMarkerDragEnd(@NonNull Marker marker) {
                }

                @Override
                public void onMarkerDragStart(@NonNull Marker marker) {
                }
            });
        }

        // Seperate thread for server connection
        new Thread(new ClientThread()).start();



    }

    public void field(View view) {
        System.out.println("AYO");
        findViewById(R.id.dragging_Done).setVisibility(View.VISIBLE);
        findViewById(R.id.field_button).setVisibility(View.INVISIBLE);
        //getUserLocation();
        Log.i("lastknownlocation", "lat" + lastKnownLocation.getLatitude());
        dragMarker1.setPosition(new LatLng(lastKnownLocation.getLatitude(),
                lastKnownLocation.getLongitude() + 0.0001));
        dragMarker2.setPosition(new LatLng(lastKnownLocation.getLatitude(),
                lastKnownLocation.getLongitude() - 0.0001));
        dragMarker1.setVisible(true);
        dragMarker2.setVisible(true);

    }

    public void done(View view) throws IOException {
        findViewById(R.id.dragging_Done).setVisibility(View.INVISIBLE);
        findViewById(R.id.field_button).setVisibility(View.VISIBLE);
        dragMarker1.setVisible(false);
        dragMarker2.setVisible(false);
        String dragMarkerCoords1 = "DragMarker1: (" + Double.toString(dragMarker1.getPosition().latitude) + "," + Double.toString(dragMarker1.getPosition().longitude) + ")";
        String dragMarkerCoords2 = "DragMarker2: (" + Double.toString(dragMarker2.getPosition().latitude) + "," + Double.toString(dragMarker2.getPosition().longitude) + ")";
        System.out.println("Coordinates for marker 1: " + dragMarkerCoords1);
        System.out.println("Coordinates for marker 2: " + dragMarkerCoords2);

        //new Thread() {
        //    public void run() {
        //        String serverIP = "192.168.87.39";
        //        int serverPort = 8888;
        //        try {
        //            Socket sendClient = new Socket(serverIP, serverPort);
        //            OutputStream output = sendClient.getOutputStream();
        //            PrintWriter writer = new PrintWriter(output, true);
        //            writer.write(dragMarkerCoords1 + dragMarkerCoords2);
        //            writer.flush();
        //            writer.close();
        //        } catch (IOException e) {
        //            e.printStackTrace();
        //        }
        //    }
        //}.start();
        //OutputStream output = client.getOutputStream();
        //PrintWriter writer = new PrintWriter(output, true);
        //writer.write("Can you hear me?");
        //writer.flush();
        //writer.close();


    }

    public Bitmap resizeMapIcons(String iconName, int width, int height){
        Bitmap imageBitmap = BitmapFactory.decodeResource(getResources(),getResources().getIdentifier(iconName, "drawable", getPackageName()));
        Bitmap resizedBitmap = Bitmap.createScaledBitmap(imageBitmap, width, height, false);
        return resizedBitmap;
    }

    /**
     * Enables the My Location layer if the fine location permission has been granted.
     */
    @SuppressLint("MissingPermission")
    private void enableMyLocation() {
        // 1. Check if permissions are granted, if so, enable the my location layer
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED
                || ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION)
                == PackageManager.PERMISSION_GRANTED) {
            mMap.setMyLocationEnabled(true);
            return;
        }

        // 2. Otherwise, request location permissions from the user.
        shareLocation();
    }
    public void shareLocation() {
        ActivityCompat.requestPermissions(this, new String[] {Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, 1);
    }

    // Is called after a permission request has been made
    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        if (requestCode != 1) {
            super.onRequestPermissionsResult(requestCode, permissions, grantResults);
            return;
        }
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            enableMyLocation();
        }

    }

    @Override
    public void onMyLocationClick(@NonNull Location location) {
        Toast.makeText(this, "Current location:\n" + location, Toast.LENGTH_LONG)
                .show();
    }

    @Override
    public boolean onMyLocationButtonClick() {
        getUserLocation();
        //Toast.makeText(this, "MyLocation button clicked", Toast.LENGTH_SHORT)
        //        .show();
        // Return false so that we don't consume the event and the default behavior still occurs
        // (the camera animates to the user's current position).
        return false;
    }

    public void getUserLocation() {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            Task<Location> locationResult = fusedLocationProviderClient.getLastLocation();
            locationResult.addOnCompleteListener(this, new OnCompleteListener<Location>() {
                @Override
                public void onComplete(@NonNull Task<Location> task) {
                    if(task.isSuccessful()) {
                        // Zoom in on current device
                        lastKnownLocation = task.getResult();
                        if (lastKnownLocation != null) {
                            mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(
                                    new LatLng(lastKnownLocation.getLatitude(),
                                            lastKnownLocation.getLongitude()), 17));
                        }
                    }
                }
            });
        };
    }

    public boolean isPermissionGranted() {
        return (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED);
    }

    // Creates new socket connection to server
    public void connectToServer() {
        // Establishing server connection
        String serverIP = "192.168.87.39";
        int serverPort = 8888;
        try {
            client = new Socket(serverIP, serverPort);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    // the ClientThread class performs
    // the networking operations
    class ClientThread implements Runnable {

        @Override
        public void run() {
            try {
                // the IP and port should be correct to have a connection established
                // Creates a stream socket and connects it to the specified port number on the named host.

                //String serverIP = "192.168.87.39";
                //int serverPort = 8888;

                //Socket client = new Socket(serverIP, serverPort);
                connectToServer();
                InputStream input = client.getInputStream();
                OutputStream output = client.getOutputStream();

                reader = new BufferedReader(new InputStreamReader(input));
                writer = new PrintWriter(output, true);
                //BufferedReader reader = new BufferedReader(new InputStreamReader(input));
                //PrintWriter writer = new PrintWriter(output, true);
                // send message
                //output.write
                writer.write("lul");
                writer.flush();
                writer.write("yeet");
                writer.flush();
                //writer.close();


                while (true) {
                    String message = reader.readLine();
                    if (message != null) {
                        String[] msgSplit = message.split("_", 0);
                        runOnUiThread(new Runnable() {

                            @Override
                            public void run() {
                                mMap.addMarker(new MarkerOptions()
                                        .position(new LatLng(Double.parseDouble(msgSplit[1]), Double.parseDouble(msgSplit[2])))
                                        .title("Target: " + msgSplit[0]) // Title of marker
                                        .alpha(0.5f) // Opacity of the marker
                                        .anchor(0.5f, 0.5f) // Sets position to be middle of the marker
                                        .flat(false)
                                        .icon(markerIcon)); // Icon of the marker
                            }
                        });
                        System.out.println(message);
                    }
                }



                // closing the connection
                //client.close();

            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

}