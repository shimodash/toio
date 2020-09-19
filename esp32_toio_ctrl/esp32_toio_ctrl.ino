/******************************************************************************

This program is created based on BLE_client.ino (1.0.2).
toio Core CUBE control program
programed by H.Niikura 2019
programed by S.Shimoda 2020
“toio(TM)” is a registered trademark or trademark of Sony Interactive Entertainment Inc.

---History---
Rev.0.6  SSD1306 display function add.
Rev.0.5  The following changes are required in BLERemoteService.ccp.
         line 167: uint16_t count = 10; --> uint16_t count = 1; 
******************************************************************************/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


 #include "BLEDevice.h"
//#include "BLEScan.h"

// The service of toio CUBE.
static BLEUUID serviceUUID("10b20100-5b3b-4571-9508-cf3efcd7bbae");    // toio Core CUBE Service UUID
// The characteristic of toio CUBE.
static BLEUUID charUUID_ID("10b20101-5b3b-4571-9508-cf3efcd7bbae");    // ID Information Characteristics UUID
static BLEUUID charUUID_MO("10B20102-5b3b-4571-9508-cf3efCD7BBAE");    // Motor Control Characteristics UUID
static BLEUUID charUUID_LC("10B20103-5b3b-4571-9508-cf3efCD7BBAE");    // Light Control Characteristics UUID
static BLEUUID charUUID_SC("10B20104-5b3b-4571-9508-cf3efCD7BBAE");    // Sound Control Characteristics UUID
static BLEUUID charUUID_SI("10B20106-5b3b-4571-9508-cf3efCD7BBAE");    // Sensor Information Characteristics UUID
static BLEUUID charUUID_BU("10B20107-5b3b-4571-9508-cf3EFCD7BBAE");    // Button Information Characteristics UUID
static BLEUUID charUUID_BT("10B20108-5b3b-4571-9508-cf3EFCD7BBAE");    // Battery Information Characteristics UUID
static BLEUUID charUUID_CO("10B201FF-5b3b-4571-9508-cf3EFCD7BBAE");    // Configuration Characteristics UUID

// キャラクタリスティックを格納する配列のINDEX ------------- 
 #define ID_INFO     0     //　ID読み取り 
 #define MOTOR_CONT  1     //  モータコントロール 
 #define LIGHT_CONT  2     //　ライトコントロール 
 #define SOUND_CONT  3     //　サウンドコントロール 
 #define SENSOR_INFO 4     //　センサー読み取り 
 #define BUTTON_INFO 5     //　ボタン読み取り 
 #define BATT_INFO   6     //　バッテリー残量読み取り 
 #define CONFIG_INFO 7     //　コンフィグレーション
 #define NUM_OF_MAX_CUBE 2 // CUBEの最大接続数

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic[ NUM_OF_MAX_CUBE ][ 8 ];
static BLEAdvertisedDevice* myDevice[ NUM_OF_MAX_CUBE ];
static uint16_t pRemotteHandler[8];

// Cube Notify data-------------------------
// ID Position value
static int id_pos_x[ NUM_OF_MAX_CUBE ];
static int id_pos_y[ NUM_OF_MAX_CUBE ];
static int id_pos_a[ NUM_OF_MAX_CUBE ];
// Standard ID
static long StdID[ NUM_OF_MAX_CUBE ];
static long actID[ NUM_OF_MAX_CUBE ];
// Button
static int button_status[ NUM_OF_MAX_CUBE ];
// Sensor
static int sensor_flat[ NUM_OF_MAX_CUBE ];
static int collision[ NUM_OF_MAX_CUBE ];
//Remaining battery capacity
static int batt_rem[ NUM_OF_MAX_CUBE ];
// flages
bool f_get_stndard_ID[ NUM_OF_MAX_CUBE ];
bool f_get_position_ID[ NUM_OF_MAX_CUBE ];
bool f_ID_missed[ NUM_OF_MAX_CUBE ];
bool f_get_sensor[ NUM_OF_MAX_CUBE ];
bool f_get_button[ NUM_OF_MAX_CUBE ];
// state
enum { CUBE_FWD, CUBE_STOP, CUBE_REV, CUBE_CCW, CUBE_CW } Cube_state[ NUM_OF_MAX_CUBE ];
int cube_speed[ NUM_OF_MAX_CUBE ];


//-------------------------------------------
// Serial port  
int RxBUFF_MAX = 128;
String Rxbuff[ 2 ];
int  RxCount=0;
int  Rxdata=0;

// CUBE
int cube_index = 0;

// BLE Notify Call Back read / Notifyを指定したキャラクタリスティックスを受信する部分
static void notifyCallback_0(
 BLERemoteCharacteristic* pBLERemoteCharacteristic,
 uint8_t* pData,
 size_t length,
 bool isNotify){
   int i;
   uint16_t hnd = pBLERemoteCharacteristic->getHandle(); 
   notifyRecive( 0, hnd, pData, length );
 }

static void notifyCallback_1(
 BLERemoteCharacteristic* pBLERemoteCharacteristic,
 uint8_t* pData,
 size_t length,
 bool isNotify){
   int i;
   uint16_t hnd = pBLERemoteCharacteristic->getHandle(); 
   notifyRecive( 1, hnd, pData, length );
 }

static void notifyRecive( int cid, uint16_t hnd, uint8_t* pData, size_t length ){
 int i;
   if( hnd == pRemotteHandler[ID_INFO] ){
     // Position ID 
     if( pData[0]==0x01 ){
       id_pos_x[cid] = (int)pData[1] + (int)pData[2]*256;
       id_pos_y[cid] = (int)pData[3] + (int)pData[4]*256;   
       id_pos_a[cid] = (int)pData[5] + (int)pData[6]*256;    
       f_get_position_ID[cid] = true;  
       f_get_stndard_ID[cid] = false;
       actID[cid] = -1;
       Serial.printf("Cno=%2d:X=%5d, Y=%5d, A=%5d\n", cid, id_pos_x[cid], id_pos_y[cid], id_pos_a[cid]);     
     }    
     // Standard ID
     if( pData[0]==0x02 ){
       StdID[cid] =  (long)pData[1] + (long)pData[2]*256 *  (long)pData[3]*65536 + (long)pData[4]*16777216;
       id_pos_a[cid] = (int)pData[5] + (int)pData[6]*256;
       f_get_stndard_ID[cid] = true;  
       Serial.printf("Cno %2d:Standard ID=%ld A=%5d\n", cid, StdID[cid], id_pos_a[cid]);        
//       display.clearDisplay();
//       display.setCursor(0,0);             // Start at top-left corner
//       display.println(String(cid)+String(StdID[cid])+String(id_pos_a[cid]));
//       display.display();

     
     } 
     // ID Missed
     if( pData[0]==0x03 ){
       f_ID_missed[cid] = true;
       Serial.printf("Cno %2d:ID Missed!!\n", cid);          
     }
   //** Button infomation **
   }else if( hnd == pRemotteHandler[BUTTON_INFO]){      
     if( pData[0]==0x01){
       button_status[cid] = (int)pData[1];
       f_get_button[cid] = true;
       Serial.printf("Cno %2d:Button:%d\n ", cid, button_status[cid]);      
     }    
   //** Battery infomation **
   }else if( hnd == pRemotteHandler[BATT_INFO]){      
     batt_rem[cid] = (int)pData[0];
     Serial.printf("Cno %2d:Battery=%3d%%\n", cid, batt_rem[cid]);
   //** Sensor information ** 
   }else if( hnd == pRemotteHandler[SENSOR_INFO]){
     if( pData[0]==0x01){
       sensor_flat[cid] =  (int)pData[1];
       collision[cid] =  (int)pData[2];
       f_get_sensor[cid] = true;
       Serial.printf("Cno %2d:Flat:%d Collision:%d\n ", cid, sensor_flat[cid], collision[cid]);      
     }
   }else{
     Serial.printf("Handle: %02d ", hnd);
     Serial.print("data: ");
     for( i=0 ; i<(int)length ; i++ ){
       Serial.printf("%02x ", pData[i]);
     }
     Serial.println("");
   }
} 

//*************************************************************************************************
class MyClientCallback : public BLEClientCallbacks {
 void onConnect(BLEClient* pclient) {
   Serial.println("**** onConnect ****");    
 }

 void onDisconnect(BLEClient* pclient) {
   connected = false;
   Serial.println("onDisconnect");
 }
};
//*************************************************************************************************
bool connectToServer( int n ) {
   
   BLEClient*  pClient  = BLEDevice::createClient();   

   pClient->setClientCallbacks(new MyClientCallback());

   // Connect to the remove BLE Server.
   pClient->connect(myDevice[n]);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)

   // Obtain a reference to the service we are after in the remote BLE server.
   BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
   if (pRemoteService == nullptr) {
     Serial.print("Failed to find our service UUID: ");
     Serial.println(serviceUUID.toString().c_str());
     pClient->disconnect();
     return false;
   }
   Serial.printf(" -Connect CUBE-%d\n", n );    

   // Obtain a reference to the characteristic in the service of the remote BLE server.    
   pRemoteCharacteristic[n][ID_INFO] = pRemoteService->getCharacteristic(charUUID_ID);
   if (pRemoteCharacteristic[n][ID_INFO] == nullptr) { pClient->disconnect(); return false; } 
 
   pRemoteCharacteristic[n][MOTOR_CONT] = pRemoteService->getCharacteristic(charUUID_MO);
   if (pRemoteCharacteristic[n][MOTOR_CONT] == nullptr) { pClient->disconnect(); return false; }

   pRemoteCharacteristic[n][LIGHT_CONT] = pRemoteService->getCharacteristic(charUUID_LC);
   if (pRemoteCharacteristic[n][LIGHT_CONT] == nullptr) { pClient->disconnect(); return false; }

   pRemoteCharacteristic[n][SOUND_CONT] = pRemoteService->getCharacteristic(charUUID_SC);
   if (pRemoteCharacteristic[n][SOUND_CONT] == nullptr) { pClient->disconnect(); return false; }  

   pRemoteCharacteristic[n][SENSOR_INFO] = pRemoteService->getCharacteristic(charUUID_SI);
   if (pRemoteCharacteristic[n][SENSOR_INFO] == nullptr) { pClient->disconnect(); return false; }   

   pRemoteCharacteristic[n][BUTTON_INFO] = pRemoteService->getCharacteristic(charUUID_BU);
   if (pRemoteCharacteristic[n][BUTTON_INFO] == nullptr) { pClient->disconnect(); return false; }   

   pRemoteCharacteristic[n][BATT_INFO] = pRemoteService->getCharacteristic(charUUID_BT);
   if (pRemoteCharacteristic[n][BATT_INFO] == nullptr) { pClient->disconnect(); return false; }
   
   pRemoteCharacteristic[n][CONFIG_INFO] = pRemoteService->getCharacteristic(charUUID_CO);
   if (pRemoteCharacteristic[n][CONFIG_INFO] == nullptr) { pClient->disconnect(); return false; }

   // Set handler for notitycation
   pRemotteHandler[ID_INFO] = pRemoteCharacteristic[n][ID_INFO]->getHandle();
   pRemotteHandler[SENSOR_INFO] = pRemoteCharacteristic[n][SENSOR_INFO]->getHandle();
   pRemotteHandler[BUTTON_INFO] = pRemoteCharacteristic[n][BUTTON_INFO]->getHandle(); 
   pRemotteHandler[BATT_INFO] = pRemoteCharacteristic[n][BATT_INFO]->getHandle();        
   pRemotteHandler[CONFIG_INFO] = pRemoteCharacteristic[n][CONFIG_INFO]->getHandle(); 

   // Notify Start 
   
   if(pRemoteCharacteristic[n][ID_INFO]->canNotify()){
     switch(n){
       case 0:pRemoteCharacteristic[n][ID_INFO]->registerForNotify(notifyCallback_0);break;
       case 1:pRemoteCharacteristic[n][ID_INFO]->registerForNotify(notifyCallback_1);break;     
     }
   }
   if(pRemoteCharacteristic[n][BUTTON_INFO]->canNotify()){
     switch(n){
       case 0:pRemoteCharacteristic[n][BUTTON_INFO]->registerForNotify(notifyCallback_0);break;
       case 1:pRemoteCharacteristic[n][BUTTON_INFO]->registerForNotify(notifyCallback_1);break;               
     }
   }
   if(pRemoteCharacteristic[n][SENSOR_INFO]->canNotify()){
     switch(n){
       case 0:pRemoteCharacteristic[n][SENSOR_INFO]->registerForNotify(notifyCallback_0);break;
       case 1:pRemoteCharacteristic[n][SENSOR_INFO]->registerForNotify(notifyCallback_1);break;       
     } 
   }
   if(pRemoteCharacteristic[n][BATT_INFO]->canNotify()){
     switch(n){
       case 0:pRemoteCharacteristic[n][BATT_INFO]->registerForNotify(notifyCallback_0);break;
       case 1:pRemoteCharacteristic[n][BATT_INFO]->registerForNotify(notifyCallback_1);break;               
     } 
   }
   connected = true;
}
/**************************************************************************************************
* Scan for BLE servers and find the first one that advertises the service we are looking for.
* アドバタイズデータの表示
*************************************************************************************************/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
/**
  * Called for each advertising BLE server.
  */
 void onResult(BLEAdvertisedDevice advertisedDevice) {
   int rssi = advertisedDevice.getRSSI();  //RSSIの取得
   if( rssi > -60 ){                       //RSSIが大きいもののみ表示する
     Serial.printf(">>>: RSSI=%d : %s\n",rssi, advertisedDevice.toString().c_str() );
   }
   // We have found a device, let us now see if it contains the service we are looking for.
   // 指定のServiceUUIDの場合、コネクトする
   if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
     if( advertisedDevice.getRSSI() > -60  ){ // <===== 
       if( cube_index < NUM_OF_MAX_CUBE ){
         Serial.printf("Find toio Cube!! %d\n", cube_index);
         myDevice[ cube_index ] = new BLEAdvertisedDevice(advertisedDevice);

         cube_index++;
       }
       if( cube_index >= NUM_OF_MAX_CUBE ){
         BLEDevice::getScan()->stop();
         doConnect = true;
         doScan = true;
       }        
     }

   } // Found our server
 } // onResult
}; // MyAdvertisedDeviceCallbacks

// 初期化　****************************************************************************************
void setup() {
 Serial.begin(115200);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text

 
 Serial.println("Starting CUBE control application...");
 display.setCursor(0,0);             // Start at top-left corner
 display.println("Starting CUBE control application...");
 display.display();

 BLEDevice::init("");

 // Retrieve a Scanner and set the callback we want to use to be informed when we
 // have detected a new device.  Specify that we want active scanning and start the
 // scan to run for 5 seconds.
 BLEScan* pBLEScan = BLEDevice::getScan();
 pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
 pBLEScan->setInterval(1349);
 pBLEScan->setWindow(449);
 pBLEScan->setActiveScan(true);
 pBLEScan->start(60, false);

 int n;

 for( n=0 ; n<NUM_OF_MAX_CUBE ; n++){
   id_pos_x[ n ] =0;
   id_pos_y[ n ] =0;
   id_pos_a[ n ] =0;
   StdID[ n ] = 0;
   actID[ n ] = -1;
   button_status[ n ] = 0;
   sensor_flat[ n ] = 0;
   collision[ n ] =0;
   batt_rem[ n ] = 0;
   f_get_stndard_ID[ n ] =false;
   f_get_position_ID[ n ] = false;
   f_ID_missed[ n ] = true;
   f_get_sensor[ n ] = false;
   f_get_button[ n ] = false;  
   Cube_state[ n ] = CUBE_STOP;
   cube_speed[ n ] = 30;
 }
} // End of setup.


// This is the Arduino main loop function.=========================================================
void loop() {
 int m;
 // If the flag "doConnect" is true then we have scanned for and found the desired
 // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
 // connected we set the connected flag to be true.
 if (doConnect == true) {
   for( m=0 ; m<NUM_OF_MAX_CUBE ; m++ ){
     Serial.printf("Try to Connect CUBE %d\n", m);
     if (connectToServer( m )) {
       Serial.printf("CUBE %d Connection OK\n",m);
     } else {
       Serial.printf("CUBE %d can't connect\n",m);
     } 
   }
   doConnect = false;
 }

 // Recive Serial port
 Rxdata = Serial.read();       // Recive Command via serial port(USB)
 if( Rxdata != -1 ){
   ReciveData( Rxdata, 0 ); 
 }
 // If we are connected to a peer BLE Server, update the characteristic each time we are reached
 // with the current time since boot.
 if (connected) {
   //String newValue = "Time since boot: " + String(millis()/1000);
   //Serial.println("Setting new characteristic value to \"" + newValue + "\"");

   // Set the characteristic's value to be the array of bytes that is actually a string.
   //pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
   //byte comm[] = {01,01,01,0x0b,0x02,0x02,0x0b};    
   //pRemoteCharacteristic[MOTOR_CONT]->writeValue(comm, 7);
 }else if(doScan){
   BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
 }

 event_detect();
} // End of loop===================================================================================

//Error code 
#define SUCCESSFUL_COMPLETION        1 
#define UNDEFINED_COMMAND           -1 
#define CUBE_NUMBER_OUT_OF_RANGE    -2 
#define TOO_MUCH_DATA_READ_REQUEST  -3 
#define TOO_MUCH_RANGE_OF_DATA      -4 
#define PARAMETER_OUT_OF_RANGE      -5

// Commandsの定義 
#define MOTOR_SET_VALUE           "MOSV"  // MOSV,(Cube no),(方向1),(速度1),(方向2),(速度2) 
#define LED_SET_VALUE             "LESV"  // LESV,(Cube no),(Red),(Green),(Blue)   
#define SOUND_SET_VALUE           "SOSV"  // SOSV,(Cube no),(再生時間),(notr number),(音量)

// ***************************************************************************************
//  受信コマンドの解析
// d: recived charcter, cp: Rxbuff number / Rxbuff[0]:serial port,Rxbuff[1]:bluetooth SPP

String cmds[6] = {"\0"}; // 分割された文字列を格納する配列 

//シリアルで受信したコマンドの解析
void ReciveData( int d, int cp ){
 int rs = UNDEFINED_COMMAND;  // Variable indicating execution result. 
 if( d == '\n' || d == '\r'){  
   int index = split( Rxbuff[ cp ], ',' , cmds ); 

   if( cmds[0]==MOTOR_SET_VALUE  && connected ) { rs = MotorSetValue( cmds[1].toInt(), cmds[2].toInt(), cmds[3].toInt(), cmds[4].toInt(), cmds[5].toInt() );  }
   if( cmds[0]==LED_SET_VALUE    && connected ) { rs = LEDSetValue( cmds[1].toInt(), cmds[2].toInt(), cmds[3].toInt(), cmds[4].toInt() );  }    
   if( cmds[0]==SOUND_SET_VALUE  && connected ) { rs = SoundSetValue( cmds[1].toInt(), cmds[2].toInt(), cmds[3].toInt(), cmds[4].toInt() );  }    

   Rxbuff[ cp] ="";  
   cmds[0]="";
   cmds[1]="";
   cmds[2]="";
   cmds[3]="";
   cmds[4]="";
   cmds[5]="";    
   if( rs > 0 ){
     Serial.println("OK\r\n");
   }else{
     Serial.println("ERROR "+String(rs,DEC)+"\r\n");
   }
 }else{
   Rxbuff[cp] = Rxbuff[cp]+(char)d;  
 }
}
//split関数の定義（文字列を指定の文字で分割）
int split(String data, char delimiter, String *dst){
   int index = 0;
   int arraySize = (sizeof(data)/sizeof((data)[0]));
   int datalength = data.length();
   for (int i = 0; i <datalength; i++) {
     char tmp = data.charAt(i);
     if ( tmp == delimiter ) {
         index++;
         if ( index > (arraySize - 1)) return -1;
     }
     else dst[index] += tmp;
   }
   return (index + 1);
}

//Motor Set Value
int MotorSetValue ( int n, int dir1, int sp1,  int dir2, int sp2){
 byte comm[] = {01,01,01,0x0b,0x02,0x02,0x0b}; 
 comm[2] = (byte)dir1;
 comm[3] = (byte)sp1;
 comm[5] = (byte)dir2;
 comm[6] = (byte)sp2;
 Serial.println(String(sp1));
 Serial.println(String(sp2));

 pRemoteCharacteristic[n][MOTOR_CONT]->writeValue(comm, 7);
 return (SUCCESSFUL_COMPLETION);
}

//LED Set Value
int LEDSetValue( int n, int LED_R, int LED_G, int LED_B){
 byte comm[] = {03,00,01,01,0,0,0}; 
 comm[4] = (byte)LED_R;
 comm[5] = (byte)LED_G;
 comm[6] = (byte)LED_B;
 pRemoteCharacteristic[n][LIGHT_CONT]->writeValue(comm, 7);
 return (SUCCESSFUL_COMPLETION);
}

//Sound Set Value
int SoundSetValue( int n, int dur, int nnum, int lud){
 byte comm[] = {03,01,01,0,0,0}; 
 comm[3] = (byte)dur;
 comm[4] = (byte)nnum;
 comm[5] = (byte)lud;
 pRemoteCharacteristic[n][SOUND_CONT]->writeValue(comm, 6);
 return (SUCCESSFUL_COMPLETION);
}

//イベントの処理 
#define FWD_ID    10 
#define REV_ID    48 
#define STOP_ID   12 
#define LEFT_ID    8 
#define RIGHT_ID  46

void event_detect(){
 int n;
 int cube_no = -1;
 int curID = -1;
 int curAng = -1;
 int tg_cube = -1;
 int sp_left = 0;
 int sp_right = 0;

 for( n=0 ;n<NUM_OF_MAX_CUBE; n++){
   // Standerd IDを読み込んだ時の処理
   if( f_get_stndard_ID[n] ){
     if(  StdID[n] != actID[n] ){
       cube_no = n;
       curID = StdID[n];
       actID[n] = StdID[n];
       break;
     }
     tg_cube = (n==0) ? 1 : 0 ;
     if( Cube_state[ tg_cube ] == CUBE_FWD ){
       curAng = id_pos_a[n];       
       sp_left  = cube_speed[tg_cube]  + (curAng - 270 )/5;
       sp_right = cube_speed[tg_cube]  - (curAng - 270 )/5;
       MotorSetValue( tg_cube, 1, (byte)sp_left, 1, (byte)sp_right);       
     }              
     f_get_stndard_ID[n] = false;      
   }
 }
 //
 if( curID >= 0){
   tg_cube = (cube_no==0) ? 1 : 0 ;
   sp_left  = cube_speed[tg_cube];
   sp_right = cube_speed[tg_cube];
   switch( curID ){
     case FWD_ID:
       SoundSetValue( cube_no,30,100,127 );
       MotorSetValue( tg_cube,1,(byte)sp_left, 1, (byte)sp_right); 
       Cube_state[tg_cube] = CUBE_FWD;             
       DisplayWrite("CUBE_FWD");
       break;
     case REV_ID:
       SoundSetValue( cube_no,30,100,127 );
       MotorSetValue( tg_cube,2,(byte)sp_left, 2, (byte)sp_right); 
       Cube_state[tg_cube] = CUBE_REV;                       
       DisplayWrite("CUBE_REV");
       break;
     case STOP_ID:
       SoundSetValue( cube_no,30,100,127 );
       MotorSetValue( tg_cube,2,0, 2, 0);
       Cube_state[tg_cube] = CUBE_STOP;        
       DisplayWrite("CUBE_STOP");
       break;
     case LEFT_ID:
       SoundSetValue( cube_no,30,100,127 );
       MotorSetValue( tg_cube,2,11, 1,11); 
       Cube_state[tg_cube] = CUBE_CCW; 
       DisplayWrite("CUBE_CCW");
       break;  
     case RIGHT_ID:
       SoundSetValue( cube_no,30,100,127 );
       MotorSetValue( tg_cube,1,11, 2,11); 
       Cube_state[tg_cube] = CUBE_CW;                
       DisplayWrite("CUBE_CW");
       break;                
   }
 }
}

void DisplayWrite(String a){
  display.clearDisplay();
  display.setCursor(0,0);             // Start at top-left corner
  display.println(a);
  display.display();
}
