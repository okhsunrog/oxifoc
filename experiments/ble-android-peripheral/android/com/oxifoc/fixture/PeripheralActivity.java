package com.oxifoc.fixture;

import android.app.Activity;
import android.os.Bundle;
import android.bluetooth.*;
import android.bluetooth.le.*;
import android.content.pm.PackageManager;
import android.os.ParcelUuid;
import android.widget.*;
import java.net.Socket;
import java.io.*;
import java.util.*;
import java.util.concurrent.*;

/** Dedicated NUS peripheral fixture. Only bridges to loopback, never generates telemetry. */
public class PeripheralActivity extends Activity {
    // Keep in sync with oxifoc_core::ble.
    static final UUID SERVICE=UUID.fromString("0b960001-d748-4b2e-b03a-6ada6467f15d");
    static final UUID RX=UUID.fromString("0b960002-d748-4b2e-b03a-6ada6467f15d");
    static final UUID TX=UUID.fromString("0b960003-d748-4b2e-b03a-6ada6467f15d");
    static final UUID CCCD=UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
    BluetoothGattServer server;
    BluetoothLeAdvertiser advertiser;
    BluetoothGattCharacteristic tx;
    volatile Socket socket;
    TextView status;
    final ExecutorService writes=Executors.newSingleThreadExecutor();
    final Semaphore sent=new Semaphore(0);
    volatile long generation=0;
    final AdvertiseCallback advertising=new AdvertiseCallback(){
        public void onStartSuccess(AdvertiseSettings s){show("Advertising OxiFOC fixture. Waiting for a central.");}
        public void onStartFailure(int code){show("Advertising failed: "+code);}
    };
    void show(String s){android.util.Log.i("OxiFOCFixture",s);runOnUiThread(()->status.setText(s));}
    public void onCreate(Bundle b){
        super.onCreate(b);
        if(getActionBar()!=null)getActionBar().hide();
        LinearLayout layout=new LinearLayout(this);layout.setOrientation(1);layout.setPadding(28,Math.round(64*getResources().getDisplayMetrics().density),28,28);
        status=new TextView(this);status.setText("NUS peripheral → 127.0.0.1:25267\nUse adb reverse to reach oxifoc-virtual.");layout.addView(status);
        Button start=new Button(this);start.setText("Start advertising");start.setOnClickListener(v->prepare());layout.addView(start);
        Button stop=new Button(this);stop.setText("Stop");stop.setOnClickListener(v->stop());layout.addView(stop);setContentView(layout);
        if(getIntent().getBooleanExtra("start",false))prepare();
    }
    void prepare(){
        if(checkSelfPermission("android.permission.BLUETOOTH_CONNECT")!=PackageManager.PERMISSION_GRANTED || checkSelfPermission("android.permission.BLUETOOTH_ADVERTISE")!=PackageManager.PERMISSION_GRANTED){
            requestPermissions(new String[]{"android.permission.BLUETOOTH_CONNECT","android.permission.BLUETOOTH_ADVERTISE"},1);return;
        }
        start();
    }
    public void onRequestPermissionsResult(int code,String[] p,int[] r){super.onRequestPermissionsResult(code,p,r);if(code==1 && r.length==2 && r[0]==0 && r[1]==0)start();else show("Bluetooth permission denied");}
    void endSession(){generation++;Socket old=socket;socket=null;if(old!=null)try{old.close();}catch(IOException ignored){}sent.release();}
    void stop(){endSession();if(advertiser!=null)advertiser.stopAdvertising(advertising);advertiser=null;if(server!=null)server.close();server=null;show("Stopped");}
    public void onDestroy(){stop();writes.shutdownNow();super.onDestroy();}
    void start(){
        stop();BluetoothManager manager=getSystemService(BluetoothManager.class);BluetoothAdapter adapter=manager.getAdapter();
        if(adapter==null || !adapter.isEnabled()){show("Enable Bluetooth first");return;}
        advertiser=adapter.getBluetoothLeAdvertiser();if(advertiser==null){show("BLE advertising unavailable");return;}
        server=manager.openGattServer(this,new BluetoothGattServerCallback(){
            public void onServiceAdded(int result,BluetoothGattService service){
                if(result!=BluetoothGatt.GATT_SUCCESS){show("GATT service failed: "+result);return;}
                advertiser.startAdvertising(new AdvertiseSettings.Builder().setConnectable(true).setAdvertiseMode(AdvertiseSettings.ADVERTISE_MODE_LOW_LATENCY).setTimeout(0).build(),
                    new AdvertiseData.Builder().addServiceUuid(new ParcelUuid(SERVICE)).build(),advertising);
            }
            public void onConnectionStateChange(BluetoothDevice d,int result,int state){if(state==BluetoothProfile.STATE_DISCONNECTED){endSession();show("Disconnected; advertising remains active");}}
            public void onMtuChanged(BluetoothDevice d,int mtu){show("Central connected; ATT MTU "+mtu);}
            public void onDescriptorReadRequest(BluetoothDevice d,int id,int offset,BluetoothGattDescriptor desc){server.sendResponse(d,id,BluetoothGatt.GATT_SUCCESS,offset,new byte[]{(byte)(socket!=null?1:0),0});}
            public void onDescriptorWriteRequest(BluetoothDevice d,int id,BluetoothGattDescriptor desc,boolean prepared,boolean response,int offset,byte[] data){
                boolean valid=CCCD.equals(desc.getUuid()) && !prepared && offset==0 && data.length==2;
                if(valid){if(data[0]==1)bridge(d);else endSession();}
                if(response)server.sendResponse(d,id,valid?BluetoothGatt.GATT_SUCCESS:BluetoothGatt.GATT_REQUEST_NOT_SUPPORTED,offset,null);
            }
            public void onCharacteristicWriteRequest(BluetoothDevice d,int id,BluetoothGattCharacteristic c,boolean prepared,boolean response,int offset,byte[] data){
                byte[] packet=data.clone();
                if(!prepared && offset==0 && RX.equals(c.getUuid()))writes.execute(()->{try{Socket s=socket;if(s==null)throw new IOException("NUS not subscribed");s.getOutputStream().write(encode(packet));}catch(Exception e){show("Write: "+e);}});
                if(response)server.sendResponse(d,id,prepared?BluetoothGatt.GATT_REQUEST_NOT_SUPPORTED:BluetoothGatt.GATT_SUCCESS,offset,null);
            }
            public void onNotificationSent(BluetoothDevice d,int result){sent.release();}
        });
        BluetoothGattService service=new BluetoothGattService(SERVICE,BluetoothGattService.SERVICE_TYPE_PRIMARY);
        service.addCharacteristic(new BluetoothGattCharacteristic(RX,BluetoothGattCharacteristic.PROPERTY_WRITE|BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE,BluetoothGattCharacteristic.PERMISSION_WRITE));
        tx=new BluetoothGattCharacteristic(TX,BluetoothGattCharacteristic.PROPERTY_NOTIFY,0);
        tx.addDescriptor(new BluetoothGattDescriptor(CCCD,BluetoothGattDescriptor.PERMISSION_READ|BluetoothGattDescriptor.PERMISSION_WRITE));service.addCharacteristic(tx);server.addService(service);
    }
    void bridge(BluetoothDevice device){
        endSession();final long current=generation;
        new Thread(()->{try(Socket s=new Socket("127.0.0.1",25267)){
            if(current!=generation)return;socket=s;show("NUS subscribed → virtual TCP");
            InputStream in=s.getInputStream();ByteArrayOutputStream frame=new ByteArrayOutputStream();int ch,count=0;
            while(current==generation && (ch=in.read())!=-1){
                if(ch!=0){if(frame.size()>520)throw new IOException("Frame too large");frame.write(ch);continue;}
                byte[] packet=decode(frame.toByteArray());frame.reset();sent.drainPermits();
                if(current!=generation)break;
                BluetoothGattServer g=server;if(g==null)break;
                if(android.os.Build.VERSION.SDK_INT>=33){if(g.notifyCharacteristicChanged(device,tx,false,packet)!=BluetoothStatusCodes.SUCCESS)throw new IOException("Notification rejected");}
                else{tx.setValue(packet);if(!g.notifyCharacteristicChanged(device,tx,false))throw new IOException("Notification rejected");}
                if(!sent.tryAcquire(5,TimeUnit.SECONDS))throw new IOException("Notification timeout");
                if(++count%100==0)show("BLE packets sent: "+count);
            }
        }catch(Exception e){if(current==generation)show("Session: "+e);}finally{if(current==generation){socket=null;show("NUS session ended");}}},"nus-tcp").start();
    }
    static byte[] encode(byte[] data){
        byte[] out=new byte[data.length+data.length/254+2];int pos=1,codePos=0,code=1;
        for(byte b:data){if(b==0){out[codePos]=(byte)code;codePos=pos++;code=1;}else{out[pos++]=b;if(++code==255){out[codePos]=(byte)code;codePos=pos++;code=1;}}}
        out[codePos]=(byte)code;out[pos++]=0;return Arrays.copyOf(out,pos);
    }
    static byte[] decode(byte[] data)throws IOException{
        ByteArrayOutputStream out=new ByteArrayOutputStream();int i=0;
        while(i<data.length){int code=data[i++]&255;if(code==0||i+code-1>data.length)throw new IOException("Invalid COBS");for(int n=1;n<code;n++)out.write(data[i++]);if(code<255&&i<data.length)out.write(0);}return out.toByteArray();
    }
}
