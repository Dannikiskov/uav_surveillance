package com.example.uavapplication;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.widget.Switch;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        switchActivity(UserChoice.class);
    }

    private void switchActivity(Class nextClass) {
        Intent intent = new Intent(this, nextClass);
        startActivity(intent);
        finish();
    }
}

