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
import com.google.android.gms.tasks.Task;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;

public class MapsActivity extends FragmentActivity implements
        OnMapReadyCallback,
        OnMyLocationButtonClickListener,
        OnMyLocationClickListener,
        ActivityCompat.OnRequestPermissionsResultCallback {

    private GoogleMap mMap;
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

        com.example.uavapplication.databinding.ActivityMapsBinding binding = ActivityMapsBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        // Obtain the SupportMapFragment and get notified when the map is ready to be used.
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        assert mapFragment != null;
        mapFragment.getMapAsync(this);

        // Button only shown when dragging
        findViewById(R.id.dragging_Done).setVisibility(View.INVISIBLE);
        findViewById(R.id.dragging_cancel).setVisibility(View.INVISIBLE);
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
    public void onMapReady(@NonNull GoogleMap googleMap) {

        // Setting up Google maps
        mMap = googleMap;
        mMap.setOnMyLocationButtonClickListener(this);
        mMap.setOnMyLocationClickListener(this);
        markerIcon = BitmapDescriptorFactory.fromBitmap(resizeMapIcons("red_circle", 50, 50));
        enableMyLocation();
        getUserLocation();

        // Add invisible draggable markers
        LatLng odense = new LatLng(55.415643, 10.373886);
        dragMarker1 = mMap.addMarker(new MarkerOptions()
                .position(odense)
                .title("draggable marker1")
                .draggable(true)
                .visible(false));
        dragMarker2 = mMap.addMarker(new MarkerOptions()
                .position(odense)
                .title("draggable marker2")
                .draggable(true)
                .visible(false));

        // Listener for the draggable markers. Allows the markers to update position after drag.
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

        // Separate thread for server connection
        new Thread(new ClientThread()).start();



    }

    // Closes server connection before destroying app process
    @Override
    protected void onDestroy() {
        super.onDestroy();
        new Thread(new closeConnection()).start();
        finish();
    }

    // Deploys two markers at user location, that can be placed as desired
    public void field(View view) {
        // Similar to getLocation, but updates the new draggable markers position also
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            Task<Location> locationResult = fusedLocationProviderClient.getLastLocation();
            locationResult.addOnCompleteListener(this, task -> {
                if(task.isSuccessful()) {
                    // Zoom in on current device
                    lastKnownLocation = task.getResult();
                    if (lastKnownLocation != null) {
                        mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(
                                new LatLng(lastKnownLocation.getLatitude(),
                                        lastKnownLocation.getLongitude()), 17));
                        findViewById(R.id.dragging_Done).setVisibility(View.VISIBLE);
                        findViewById(R.id.dragging_cancel).setVisibility(View.VISIBLE);
                        findViewById(R.id.dragging_field).setVisibility(View.INVISIBLE);

                        Log.i("lastknownlocation", "lat" + lastKnownLocation.getLatitude());
                        dragMarker1.setPosition(new LatLng(lastKnownLocation.getLatitude(),
                                lastKnownLocation.getLongitude() + 0.0001));
                        dragMarker2.setPosition(new LatLng(lastKnownLocation.getLatitude(),
                                lastKnownLocation.getLongitude() - 0.0001));
                        dragMarker1.setVisible(true);
                        dragMarker2.setVisible(true);
                    }
                }
            });
        }
    }

    // Marks the draggable markers invisible, and sends the markers coordinates to backend
    public void done(View view) {
        findViewById(R.id.dragging_Done).setVisibility(View.INVISIBLE);
        findViewById(R.id.dragging_cancel).setVisibility(View.INVISIBLE);
        findViewById(R.id.dragging_field).setVisibility(View.VISIBLE);
        dragMarker1.setVisible(false);
        dragMarker2.setVisible(false);
        String dragMarkerCoords = "[" + dragMarker2.getPosition().latitude + ", " + dragMarker2.getPosition().longitude + "], " + "[" + dragMarker1.getPosition().latitude + ", " + dragMarker1.getPosition().longitude + "]";
        System.out.println("Coordinates for area: " + dragMarkerCoords);
        new Thread(new Sender(dragMarkerCoords)).start();

    }

    // Cancel the draggable markers
    public void cancel(View view) {
        findViewById(R.id.dragging_cancel).setVisibility(View.INVISIBLE);
        findViewById(R.id.dragging_Done).setVisibility(View.INVISIBLE);
        findViewById(R.id.dragging_field).setVisibility(View.VISIBLE);
        dragMarker1.setVisible(false);
        dragMarker2.setVisible(false);
    }

        // Resizes given image with given width and height
    public Bitmap resizeMapIcons(String iconName, int width, int height){
        Bitmap imageBitmap = BitmapFactory.decodeResource(getResources(),getResources().getIdentifier(iconName, "drawable", getPackageName()));
        return Bitmap.createScaledBitmap(imageBitmap, width, height, false);
    }

    // Enables access to user location if user accepts permissions
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
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        if (requestCode != 1) {
            super.onRequestPermissionsResult(requestCode, permissions, grantResults);
            return;
        }
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            enableMyLocation();
        }

    }

    // Shows current user's coordinates on screen
    @Override
    public void onMyLocationClick(@NonNull Location location) {
        Toast.makeText(this, "Current location:\n" + location, Toast.LENGTH_LONG)
                .show();
    }

    // Button which returns screen to current user location
    @Override
    public boolean onMyLocationButtonClick() {
        getUserLocation();
        return false;
    }

    // Retrieves current user precise location, and zooms in on the user
    public void getUserLocation() {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            Task<Location> locationResult = fusedLocationProviderClient.getLastLocation();
            locationResult.addOnCompleteListener(this, task -> {
                if(task.isSuccessful()) {
                    // Zoom in on current device
                    lastKnownLocation = task.getResult();
                    if (lastKnownLocation != null) {
                        mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(
                                new LatLng(lastKnownLocation.getLatitude(),
                                        lastKnownLocation.getLongitude()), 17));
                    }
                }
            });
        }
    }

    // Creates new socket connection to server
    public void connectToServer() throws UnknownHostException {
        // Establishing server connection
        // Connects to local host
        String serverIP = "10.0.2.2";
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

                connectToServer();
                InputStream input = client.getInputStream();
                OutputStream output = client.getOutputStream();

                reader = new BufferedReader(new InputStreamReader(input));
                writer = new PrintWriter(output, true);
                writer.write("frontend");
                writer.flush();


                while (true) {
                    String message = reader.readLine();
                    if (message != null) {
                        String[] msgSplit = message.split("_", 0);
                        runOnUiThread(() -> {
                            mMap.addMarker(new MarkerOptions()
                                    .position(new LatLng(Double.parseDouble(msgSplit[1]), Double.parseDouble(msgSplit[2])))
                                    .title("Target: " + msgSplit[0]) // Title of marker
                                    .alpha(0.5f) // Opacity of the marker
                                    .anchor(0.5f, 0.5f) // Sets position to be middle of the marker
                                    .flat(false)
                                    .icon(markerIcon)); // Icon of the marker
                        });
                        System.out.println(message);
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    // Thread for sending coordinates to server
    class Sender implements Runnable {
        private String message;
        Sender(String message) {
            this.message = message;
        }
        @Override
        public void run() {
            writer.write(message);
            writer.flush();
        }
    }

    // Thread for closing all connections
    class closeConnection implements Runnable {
        @Override
        public void run() {
            try {
                writer.close();
                reader.close();
                client.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}