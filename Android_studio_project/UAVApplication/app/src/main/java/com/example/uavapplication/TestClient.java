package com.example.uavapplication;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.Objects;

public class TestClient
{
    private Socket socket;
    private DataInputStream dataInputStream;
    private DataOutputStream dataOutputStream;

    public static void main( String[] args )
    {
        TestClient client = new TestClient();
    }

    public TestClient()
    {
        try
        {
            // create new socket and connect to the server
            this.socket = new Socket( "192.168.87.39" , 12345 );
        }
        catch( IOException e )
        {
            System.out.println( "failed to create socket" );
            e.printStackTrace();
        }

        System.out.println( "connected" );

        try
        {
            this.dataInputStream = new DataInputStream( new BufferedInputStream( this.socket.getInputStream() ) );
            this.dataOutputStream = new DataOutputStream( new BufferedOutputStream( this.socket.getOutputStream() ) );
        }
        catch ( IOException e )
        {
            System.out.println( "failed to create streams" );
            e.printStackTrace();
        }

        while ( true )
        {
            try
            {
                String test = this.dataInputStream.readUTF();
                System.out.println( "int received: "+test );

                if (Objects.equals(test, "hello world!")) break;
            }
            catch ( IOException e )
            {
                System.out.println( "failed to read data" );
                e.printStackTrace();
                break;
            }
        }
    }
}