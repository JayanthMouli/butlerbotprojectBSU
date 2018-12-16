package com.ece218.jm.butlerbotv3;

import java.io.BufferedReader;
import java.io.File;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URL;
import java.net.URLConnection;

import android.content.Context;
import android.os.AsyncTask;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.AutoCompleteTextView;
import android.widget.Button;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.Scanner;

import android.graphics.Typeface;
import android.widget.ArrayAdapter;

//CODE CREATED BY JAYANTH MOULI, ECE218, BSU INTERN, 2018

public  class MainActivity extends AppCompatActivity {
    TextView textView;
    TextView textView2;
    AutoCompleteTextView editText;
    Button button;
    private static final String DB_URL = "jdbc://10.4.14.246/destinations";
    private static final String USER = "remote_user";
    private static final String PASS = "butlercall";
    ;
    String[] suggestions = new String[]{"mec202a", "mec202b", "mec202c", "mec202d", "mec202e", "mec202f" ,"mec202g", "mec202h", "mec202j", "mec202k", "mec202l", "mec202m", "mec202n", "mec202p", "mec202r" ,"mec202s", "mec202t" ,"mec202u" ,"cubicle1", "cubicle2", "cubicle3", "cubicle4" ,"cubicle5" ,"cubicle6" ,"cubicle7" ,"cubicle8" ,"cubicle9" ,"cubicle10" ,"cubicle11", "cubicle12", "cubicle13" ,"cubicle14" ,"cubicle15", "cubicle16" ,"cubicle17", "cubicle18", "cubicle19", "cubicle20" ,"cubicle21", "cubicle22" ,"cubicle23" ,"cubicle24", "cubicle25" ,"cubicle26", "cubicle27" ,"engr203" ,"engr204" ,"engr205" ,"engr206" ,"engr207" ,"engr208" ,"engr209" ,"engr212", "engr213", "engr214", "engr215" ,"engr221", "engr222" ,"engr223", "engr224" ,"engr225" ,"engr227" ,"engr229", "engr230" ,"engr231" ,"engr232", "engr233" ,"engr234", "engr235" ,"engr236" ,"engr238", "engr239", "engr201a", "engr201b", "engr201c", "engr201-1" ,"engr201-2", "engr240a" ,"engr240b" ,"engr240c" ,"engr240-1" ,"engr240-2", "engr240-3" };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        textView = (TextView) findViewById(R.id.textView);
        textView2 = (TextView) findViewById(R.id.textView2);
        editText = (AutoCompleteTextView) findViewById(R.id.editText);
        ArrayAdapter<String> adapter = new ArrayAdapter<String>(this, R.layout.support_simple_spinner_dropdown_item, suggestions);
        editText.setAdapter(adapter);
        button = (Button) findViewById(R.id.button);
        Typeface myCustomFont = Typeface.createFromAsset(getAssets(), "fonts/Foxy.ttf");
        button.setTypeface(myCustomFont);
        textView2.setTypeface(myCustomFont);
        editText.setTypeface(myCustomFont);

    }
    public void btnConn(View view) {
        Send objSend = new Send();
        objSend.execute("");
    }
    private class Send extends AsyncTask<String, String, String> {
        String msg = "";
        String text = editText.getText().toString();
        @Override
        protected void onPreExecute() {
            textView.setText("Please Wait Processing Request");
        }

        @Override
        protected String doInBackground(String... strings) {
            try {
                URL myURL = new URL("http://butlerbot.000webhostapp.com/insert.php?destination=" + text);
                URLConnection myURLConnection = myURL.openConnection();
                int a = myURLConnection.getContentLength();
                msg = "The ButlerBot will travel to " + text + " shortly.";
            }
            catch (Exception e) {
                msg = e.getMessage();
                e.printStackTrace();
            }
            return msg;
        }

        @Override
        protected void onPostExecute(String msg) {
            textView.setText(msg);
        }
    }
}