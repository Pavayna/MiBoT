package com.example.mibot_design;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.util.Log;
//import android.view.KeyEvent;
//import android.view.View;
//import android.view.inputmethod.EditorInfo;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Switch;
//import android.widget.ImageButton;
//import android.widget.TextView;

import com.google.android.gms.tasks.OnCompleteListener;
import com.google.android.gms.tasks.Task;
import com.google.firebase.database.DataSnapshot;
//import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
//import com.google.firebase.database.ValueEventListener;

public class SettingsActivity extends AppCompatActivity {
    EditText editText1,editText2;
    Button apply;
    DatabaseReference reff;
    Switch switch1;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        reff= FirebaseDatabase.getInstance().getReference();
        editText1=(EditText) findViewById(R.id.editTextNumber);
        editText2=(EditText) findViewById(R.id.editTextNumber2);
        apply=(Button) findViewById(R.id.apply);
        switch1=(Switch) findViewById(R.id.switch1);

        /*reff.child("loadedPod").addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(DataSnapshot snapshot) {
                //System.out.println(snapshot.getValue());
                editText2.setText(snapshot.getValue().toString());
            }

            @Override
            public void onCancelled(DatabaseError databaseError) {
            }
        });*/

        reff.child("pod").get().addOnCompleteListener(new OnCompleteListener<DataSnapshot>() {
            @Override
            public void onComplete(@NonNull Task<DataSnapshot> task) {
                if (!task.isSuccessful()) {
                    Log.e("firebase", "Error getting data", task.getException());
                }
                else {
                    //Log.d("firebase", String.valueOf(task.getResult().getValue()));
                    editText1.setText(task.getResult().getValue().toString());
                }
            }
        });

        reff.child("loadedPod").get().addOnCompleteListener(new OnCompleteListener<DataSnapshot>() {
            @Override
            public void onComplete(@NonNull Task<DataSnapshot> task) {
                if (!task.isSuccessful()) {
                    Log.e("firebase", "Error getting data", task.getException());
                }
                else {
                    //Log.d("firebase", String.valueOf(task.getResult().getValue()));
                    editText2.setText(task.getResult().getValue().toString());
                }
            }
        });

        apply.setOnClickListener(new View.OnClickListener(){
            public void onClick(View view){
                int val1,val2;
                val1=Integer.parseInt(editText1.getText().toString());
                val2=Integer.parseInt(editText2.getText().toString());
                reff.child("pod").setValue(val1);
                if (switch1.isChecked()) {
                    reff.child("loadedPod").setValue(val2);
                }
                else{
                    reff.child("loadedPod").setValue(0);
                }
            }
        });



        /*editText1.setOnEditorActionListener(new EditText.OnEditorActionListener() {
            @Override
            public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
                if (actionId == EditorInfo.IME_ACTION_DONE) {
                    reff.child("pod").setValue(editText1.getText());
                    Log.d("firebase", "ok");
                    return true;
                }
                Log.d("firebase", "nope");
                return false;
            }
        });*/

        /*editText1.setOnKeyListener(new View.OnKeyListener() {
            public boolean onKey(View v, int keyCode, KeyEvent event) {
                // If the event is a key-down event on the "enter" button
                if ((event.getAction() == KeyEvent.ACTION_DOWN) &&
                        (keyCode == KeyEvent.KEYCODE_ENTER)) {
                    // Perform action on key press
                    //Toast.makeText(HelloFormStuff.this, edittext.getText(), Toast.LENGTH_SHORT).show();
                    Log.d("firebase", "ok");
                    reff.child("pod").setValue(editText1.getText());
                    return true;
                }
                Log.d("firebase", "nope");
                return false;

            }
        });*/




    }
}