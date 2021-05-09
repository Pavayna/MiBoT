package com.example.mibot_design;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.ProgressBar;
import android.widget.SeekBar;
import android.widget.TextView;

import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.ValueEventListener;

public class MainActivity extends AppCompatActivity {
    ImageButton buttonF;
    ImageButton buttonB;
    ImageButton reset;
    SeekBar seekBar;
    TextView speedTxt;
    ProgressBar batteryProgress;
    TextView batteryTxt;
    DatabaseReference reff;


    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);



        reff=FirebaseDatabase.getInstance().getReference();

        buttonF = (ImageButton) findViewById(R.id.buttonForward);
        buttonB = (ImageButton) findViewById(R.id.buttonBackward);
        reset = (ImageButton) findViewById(R.id.imageButton);
        seekBar = (SeekBar) findViewById(R.id.seekBar);
        speedTxt=(TextView) findViewById(R.id.textView);
        batteryProgress=(ProgressBar) findViewById(R.id.progressBar);
        batteryTxt=(TextView)findViewById(R.id.textView4);
        this.seekBar.setMax(100);
        this.seekBar.setProgress(0);
        speedTxt.setText("0.0 m/s");


        seekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {

            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                float speed;
                speed=(float)seekBar.getProgress()/500;
                //reff.child("speed").setValue(speed);
                speedTxt.setText(Float.toString(speed)+" m/s");
            }

            public void onStartTrackingTouch(SeekBar seekBar) {
                // TODO Auto-generated method stub
            }

            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        reset.setOnClickListener(new View.OnClickListener(){
            public void onClick(View view){
                seekBar.setProgress(0);
                reff.child("speed").setValue(0);
                speedTxt.setText("0 m/s");
            }
        });

        buttonF.setOnClickListener(new View.OnClickListener(){
            public void onClick(View view){
                float speed;
                speed=(float)seekBar.getProgress()/500;
                reff.child("speed").setValue(speed);
                speedTxt.setText(Float.toString(speed)+" m/s");
            }

        });

        buttonB.setOnClickListener(new View.OnClickListener(){
            public void onClick(View view){
                float speed;
                speed=(float)-seekBar.getProgress()/500;
                reff.child("speed").setValue(speed);
                speedTxt.setText(Float.toString(speed)+" m/s");
            }
        });

        reff.child("battery").addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(DataSnapshot snapshot) {
                String battery;
                System.out.println(snapshot.getValue());
                battery=snapshot.getValue().toString();
                batteryTxt.setText(snapshot.getValue().toString() + "%");
                batteryProgress.setProgress(Integer.parseInt(battery));

            }
            @Override
            public void onCancelled(DatabaseError databaseError) {
            }
        });

    }

    public void settings(View view) {
        Intent intent = new Intent(this, SettingsActivity.class);
        startActivity(intent);
    }
}