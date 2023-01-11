package com.example.uavapplication;

import android.util.Log;
import android.widget.Toast;

import java.io.*;
import java.net.*;

public class NetworkManager implements Runnable {
    private Thread thread;
    public NetworkManager() {
        this.thread = new Thread( this );
        this.thread.setPriority( Thread.NORM_PRIORITY );
        this.thread.start();
    }

    @Override
    public void run() {
        Log.i("server run", "start");
        String serverIP = "192.168.87.39";
        int serverPort = 9999;

        Log.i("create Socket", "pending");
        Socket socket = null;
        try {
            socket = new Socket(serverIP, serverPort);
        } catch (IOException e) {
            e.printStackTrace();
        }
        Log.i("create Socket", "done");
        InputStream input = null;
        try {
            input = socket.getInputStream();
        } catch (IOException e) {
            e.printStackTrace();
        }
        BufferedReader reader = new BufferedReader(new InputStreamReader((input)));

        String message;
        try {
            while ((message = reader.readLine()) != null) {
                Log.i("Server message", message);
                System.out.println("Server message" + message);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            socket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
