#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#else

// Other Client defined here
// To use custom Client, define ENABLE_CUSTOM_CLIENT in  src/ESP_Mail_FS.h.
// See the example Custom_Client.ino for how to use.

#endif

#include <ESP_Mail_Client.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <nvs_flash.h>




enum DOOR_MODE {
  CLOSING,
  OPENING
};


void printLocalTime(void);
bool tryConnectToWifi(void);
void setupAP(void);
void launchConfigWebInterface(void);
void sendMsgAsMail(String msg);

static bool bleDeviceConnected = false;
Preferences prefs;

static BLEUUID serviceUUID("6A800001-B5A3-F393-E0A9-E50E24dCCA9E");
static BLEUUID charUUID("6A806050-B5A3-F393-E0A9-E50E24dCCA9E");
//static BLEUUID charUUID("6E400003-B5A3-F393-E0A9-E50E24dCCA9E");
static BLEAdvertisedDevice *pServerAddress;
static boolean tryConnectBleServer = false;
static boolean bleConnected = false;

//Establishing Local server at port 80
WebServer server(80);
 SMTP_Message message;

TaskHandle_t WifiConnectionTask;
TaskHandle_t BleConnectionTask;
//https://github.com/mobizt/ESP-Mail-Client/blob/master/examples/SMTP/Send_HTML/Send_HTML.ino
// Set the network reconnection option 

/* Declare the global used SMTPSession object for SMTP transport */
SMTPSession smtp;
// Declare the ESP_Mail_Session for user defined session credentials 
ESP_Mail_Session session;

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status);

int statusCode;
String content;
uint8_t* bleData;
//bool newBleValueAvailable = false;
enum class ServerMode { init, normal }; 
ServerMode serverMode = ServerMode::init;

struct AppSettings { String wlanSsid; String wlanPass; String gmtOffsetInSeconds; String smtpServer; 
          String smtpUser; String smtpPass; String smtpPort; String smtpTlsMode; String smtpRecipient;};
          
String defaultNetworkSid = "unifi";
AppSettings applicationSettings = { defaultNetworkSid, "","0", };
const char* default_hostname = "smartpet";
const char* pref_namespace = "smartpet";
const uint8_t notificationOn[] = {0x1, 0x0};
static BLERemoteCharacteristic* gyroCharacteristic;
static double oldZVal=-1.0;

 
static long getLongValue(byte bh, byte bl) 
{
  int res16 = (bl & 0xFF) | (short) (bh << 8);
  res16 += 32768; // 0 - 64 k instead of -32k - + 32k
  return res16;
}

static double getDoubleValue(byte bh, byte bl) 
{
  long longVal = getLongValue(bh, bl);
  
  double ret = (((double)longVal) / 65536.0);
  return ret;
}

void sendMsg(DOOR_MODE mode)
{
  //Der Code hier ist Müll da die Auswertung nicht korrekt ist und nur beispielhaft dargestellt werden soll wie beim Empfang eines 
  //Tür-Events die Nachricht weitergeleitet werden soll
  String msg = "";
  if(mode == CLOSING)
    msg = "Cat left the house";
  if(mode == OPENING)
    msg = "Cat comes in";
  
  Serial.println(msg);

  //if((WiFi.status() == WL_CONNECTED) || (tryConnectToWifi()))
  //{
    Serial.println("trying to send the message as email");
    
    if(applicationSettings.smtpServer.length()>0 && applicationSettings.smtpUser.length()>0)
    {
      Serial.println("Mail-settings found");
      sendMsgAsMail(msg);
    }

    //Serial.println("DO WHATEVER TO PUBLISH THE EVENT OVER WIFI");
    /*
    //send push notifications, for example using ....
    //https://api.callmebot.com/whatsapp.php?phone=[phone_number]&text=[message]&apikey=[your_apikey]
    //https://textmebot.com/

    WiFiClient client; HTTPClient http; 
    //Declare an object of class HTTPClient 
    //Specify request destination 
    String tobesend = "http://api.callmebot.com/whatsapp.php?"; 
    tobesend = tobesend + "phone=+XXXXXXXXXXX"; //the phone number has to getting stored as a settings in the EEPROM before
    tobesend = tobesend + "&text=The+button+on+the+ESP+was+pressed"; 
    tobesend = tobesend + "&apikey=YYYYYY"; 
    http.begin(client,tobesend); 
    int httpCode = http.GET(); 
    //Send the request 
    if (httpCode > 0) 
    { 
    //Check the returning code 
    String payload = http.getString();	//Get the request response payload 
    Serial.println(payload); 	//Print the response payload 
    } 
    http.end(); //Close connectioncomo criar um blog
    */
  //}
}


void sendMsgAsMail(String msg)
{


Serial.println("check smtp connected");
  ESP_MAIL_PRINTF("Free Heap: %d\n", MailClient.getFreeHeap());
 if  (!smtp.connect(&session /* session credentials */)){
    Serial.println("smtpnot connected - return");
       return;
 }

Serial.println("start send mail");
  ESP_MAIL_PRINTF("Free Heap: %d\n", MailClient.getFreeHeap());
  if (!MailClient.sendMail(&smtp, &message))
    Serial.println("Error sending Email, " + smtp.errorReason());
Serial.println("mail sent");
  ESP_MAIL_PRINTF("Free Heap: %d\n", MailClient.getFreeHeap());

return;

  Serial.println("sendMsgAsMail 1");
  SMTP_Message message;
  Serial.println("sendMsgAsMail 2");
  /* Set the message headers */
  message.sender.name = F("HappyPet");
  Serial.println("sendMsgAsMail 3");
  message.sender.email = applicationSettings.smtpUser;
  Serial.println("sendMsgAsMail 4");
  message.subject = F("HappyPet - Cat Status Update");
  Serial.println("sendMsgAsMail 5");
  //Serial.println(applicationSettings.smtpRecipient);
  message.addRecipient(F(""), applicationSettings.smtpRecipient.c_str());
  Serial.println("sendMsgAsMail 6");
  String htmlMsg = "<p>" + msg + "</p>";
  message.html.content = htmlMsg;

  // The html text message character set e.g.
  Serial.println("sendMsgAsMail 7");
  message.html.charSet = F("enc_8bit");

  /** The content transfer encoding e.g.
   * enc_7bit or "7bit" (not encoded)
   * enc_qp or "quoted-printable" (encoded)
   * enc_base64 or "base64" (encoded)
   * enc_binary or "binary" (not encoded)
   * enc_8bit or "8bit" (not encoded)
   * The default value is "7bit"
   */
  message.html.transfer_encoding = Content_Transfer_Encoding::enc_8bit;

  /** The message priority
   * esp_mail_smtp_priority_high or 1
   * esp_mail_smtp_priority_normal or 3
   * esp_mail_smtp_priority_low or 5
   * The default value is esp_mail_smtp_priority_low
   */
  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_normal;
  Serial.println("sendMsgAsMail 8");
  /** The Delivery Status Notifications e.g.
   * esp_mail_smtp_notify_never
   * esp_mail_smtp_notify_success
   * esp_mail_smtp_notify_failure
   * esp_mail_smtp_notify_delay
   * The default value is esp_mail_smtp_notify_never
   */
  // message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;

  /* Set the custom message header */
  //message.addHeader(F("Message-ID: <abcde.fghij@gmail.com>"));

  /* Connect to the server */
  //if((WiFi.status() == WL_CONNECTED) || (tryConnectToWifi()))
  //{
   
    Serial.println("sendMsgAsMail 9");
    Serial.println("Connected to the SMTP server...");
    /* Start sending Email and close the session */
    if(!smtp.connected())
    {
      Serial.println("reconnect to the mail server...");
      smtp.connect(&session);
    }
    if (!MailClient.sendMail(&smtp, &message))
      Serial.println("Error sending Email, " + smtp.errorReason());

    // to clear sending result log
    smtp.sendingResult.clear();
    Serial.println("mail fee heap");
    ESP_MAIL_PRINTF("Free Heap: %d\n", MailClient.getFreeHeap());
  //}
}



//Send message as email
//https://randomnerdtutorials.com/esp32-send-email-smtp-server-arduino-ide/





//function that prints the latest sensor readings to whereever (Serial, WhatsApp, SMS,...)
void pushBteReadings(){
  
  double newZVal = getDoubleValue(bleData[4],bleData[5]);
  if(oldZVal!=-1.0)
  {
    Serial.print ("OLD: ");
    Serial.print (oldZVal);
    Serial.print (" NEW: ");
    Serial.print (newZVal);

    if(oldZVal > 0.25 && newZVal<= 0.25)
      sendMsg(OPENING);
    if(oldZVal < 0.7 && newZVal >= 0.7)
      sendMsg(CLOSING);
  }
  
 
  
  oldZVal = newZVal;
  

  Serial.print(" ROT X: ");
  Serial.print(getDoubleValue(bleData[0],bleData[1]));
  Serial.print(" Y: ");

  
  Serial.print(getDoubleValue(bleData[2],bleData[3]));
  Serial.print(" Z: ");
 
  Serial.println(getDoubleValue(bleData[4],bleData[5]));
  /*
  Serial.print(" - ACC X: ");
  Serial.print(getDoubleValue(bleData[6],bleData[7]));
  Serial.print(" Y: ");
  Serial.print(getDoubleValue(bleData[8],bleData[9]));
  Serial.print(" Z: ");
  Serial.println(getDoubleValue(bleData[10],bleData[11]));
  */
}


void writeSettings(String wlanSsid, String wlanPass, String gmtTimeZoneOffset,String smtpServer,String smtpUser, String smtpPass,String smtpPort, String smtpTlsMode, String smtpRecipient)
{
  
  Serial.println("Writing settings to EEPROM start");
  
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
  {
      Serial.println("Error on NVS partition found, trying to fix them ....");
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
      Serial.println("Restart ESP32");
      ESP.restart();
  }
 
  if(!prefs.begin(pref_namespace,false))
    Serial.printf("error opening namespace");
  
  //prefs.clear();
  Serial.printf("Anzahl freier Einträge: %d\n", prefs.freeEntries());
  Serial.print ("putting value wlan_sid:");
  Serial.println(wlanSsid);
  prefs.putString("wlan_ssid",wlanSsid);
  prefs.putString("wlan_pass",wlanPass);
  prefs.putString("timez_off_sec",gmtTimeZoneOffset);
  prefs.putString("smtp_server",smtpServer);
  prefs.putString("smtp_user",smtpUser);
  prefs.putString("smtp_recip",smtpRecipient);
  prefs.putString("smtp_pass",smtpPass);
  prefs.putString("smtp_port",smtpPort);
  prefs.putString("smtp_tls_mode",smtpTlsMode);
  prefs.end();
  Serial.println("Writing settings to EEPROM done");
  //delay(10000);
}



//print the local time
void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

/// @brief trief to read the settings for this module from the EEPROM
/// @return true if settings were getting found else false
bool readSettings()
{

  Serial.println("read settings from EEPROM started");
  if (!prefs.begin(pref_namespace, true)) 
  {
    Serial.println("ERROR READING EEPROM SETTINGS");
    return false;
  }
  Serial.println("try to find settings");
  applicationSettings.wlanSsid = prefs.getString("wlan_ssid",defaultNetworkSid);
  applicationSettings.wlanPass= prefs.getString("wlan_pass","");
  applicationSettings.gmtOffsetInSeconds = prefs.getString("timez_off_sec","-1");
  applicationSettings.smtpServer = prefs.getString("smtp_server");
  applicationSettings.smtpUser = prefs.getString("smtp_user");
  applicationSettings.smtpPass = prefs.getString("smtp_pass");
  applicationSettings.smtpPort = prefs.getString("smtp_port");
  applicationSettings.smtpTlsMode = prefs.getString("smtp_tls_mode");
  applicationSettings.smtpRecipient = prefs.getString("smtp_recip");

  Serial.println("Settings:");
  Serial.print("sid: ");
  Serial.println(applicationSettings.wlanSsid);
  Serial.print("pwd: ");
  Serial.println(applicationSettings.wlanPass);
  Serial.print("gmtOffsetInSeconds: ");
  Serial.println(applicationSettings.gmtOffsetInSeconds);
 
  
  //check if settings are stored in the eeprom or the eeprom content is empty
  if(applicationSettings.wlanSsid == defaultNetworkSid || applicationSettings.wlanSsid == "" || applicationSettings.gmtOffsetInSeconds=="-1")
  {
    //not settings found => server has being resetted or starts the first time. Start the server in the INIT mode
    Serial.println("Server is starting into INITIALIZATION mode");
    prefs.end();
    //delay(100);
    return false;
  }
  else
  {
    //server settings are available. start the server in the normal processing mode
    Serial.println("Server starting into the normal mode");
    prefs.end();
    //delay(100);
    return true;
  }

  
}




void checkResetButtonPushed()
{
  vTaskDelay(10);
  bool resetPushed=false;
  
  //Check if the reset button is getting fushed
  if(digitalRead(0)==LOW)
  {
    resetPushed = true;
    Serial.println("Reset button pushed");
    delay(2000);
    if(digitalRead(0)==LOW)
    {
        //HARDRESET
        //delete all EEPROM settings on pressing for more the 5 seconds the reset button
        Serial.println("Do a HARD Reset");
        nvs_flash_erase();
        //writeSettings("","","","","","","","","");
    }
  }

  if(resetPushed)
  {
    Serial.println("RESTART");
    delay(3000);
    ESP.restart();
  }
    
}




//When the BLE Server sends a new gyro reading with the notify property
static void gyroNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                        uint8_t* pData, size_t length, bool isNotify) {
  //store gyro value
  //Serial.print("gyroNotifyCallback:");
  //Serial.println((char*) pData);
  vTaskDelay(10);
  bleData = pData;
  //checkResetButtonPushed();
  pushBteReadings();
  //newBleValueAvailable = true;
}



//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToBleServer() 
{
  vTaskDelay(10);
  Serial.println("Try to find the BLE Server");
  BLEClient* pClient = BLEDevice::createClient(); 

  // Connect to the remove BLE Server.
  if(!pClient->connect(pServerAddress))
  {
    Serial.println("Connectiong BLE Server failed");
    return false;
  }
  
  Serial.println("BLE Server is connected");
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  Serial.println("Check BLE Remote Service");
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    return (false);
  }
 
/*
  //Gib eine List aller Characteristics des Chips aus
  std::map<std::string, BLERemoteCharacteristic*>* listChar = pRemoteService->getCharacteristics();
  std::map<std::string, BLERemoteCharacteristic*>::iterator it = listChar->begin();
  while(it != listChar->end())
  {
    Serial.print("IT: ");
    Serial.print(it->first.c_str());
    BLERemoteCharacteristic* cha = it->second;
    
    Serial.print(" CHAR: ");
    Serial.println(cha->getUUID().toString().c_str());
    it++;
  }
*/
/*
IT: 6a803216-b5a3-f393-e0a9-e50e24dcca9e CHAR: 6a803216-b5a3-f393-e0a9-e50e24dcca9e
IT: 6a806050-b5a3-f393-e0a9-e50e24dcca9e CHAR: 6a806050-b5a3-f393-e0a9-e50e24dcca9e
IT: 6a80b280-b5a3-f393-e0a9-e50e24dcca9e CHAR: 6a80b280-b5a3-f393-e0a9-e50e24dcca9e
IT: 6a80ff0c-b5a3-f393-e0a9-e50e24dcca9e CHAR: 6a80ff0c-b5a3-f393-e0a9-e50e24dcca9e
*/
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  gyroCharacteristic = pRemoteService->getCharacteristic(charUUID);
  Serial.println("Check Characteristics");
  if (gyroCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");
 
  //Assign callback functions for the Characteristics
  gyroCharacteristic->registerForNotify(gyroNotifyCallback);
  return true;
}



//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    if (advertisedDevice->isAdvertisingService(serviceUUID))
    { //}.getName() == "SENSOR_PRO") { //Check if the name of the advertiser matches
      advertisedDevice->getScan()->stop(); //Scan can be stopped, we found what we are looking for
      Serial.print("BLE server address: ");
      Serial.println(advertisedDevice->getAddress().toString().c_str());
      pServerAddress = new BLEAdvertisedDevice(*advertisedDevice);//new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      Serial.print("Service Name:");
      Serial.println(advertisedDevice->getName().c_str());
      tryConnectBleServer = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};



void connectToBleServerTask( void * parameter) 
{
  if(serverMode == ServerMode::init)
    return;

  Serial.println("INIT BLE");
  BLEDevice::init("");

  for(;;)
  {
    if(!bleConnected)
    {
      Serial.println("Scanning for BLE devices");
      BLEScan* pBLEScan = BLEDevice::getScan();
      pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
      pBLEScan->setActiveScan(true);
      pBLEScan->start(0);
    }

    //Serial.println("try to find the BLE seonsor");
    if (tryConnectBleServer == true) //this variable is getting set from the BLE callback function ... means: wait until a matching BLE device has found
    {
      Serial.println("Matching BLE device found, ... try to connect ...");
      if (connectToBleServer()) {
        Serial.println("We are now connected to the BLE Server.");
        //Activate the Notify property of each Characteristic
        gyroCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        bleConnected = true;
      } else {
        Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
      }
      tryConnectBleServer = false;
    }

    delay(500);
  } 
}


//Keep an WIFI connection open
void conntecToWifiTask( void * parameter) 
{
  // if(serverMode == ServerMode::init)
  //   return;

  WiFi.disconnect();
  //start network, ether in INIT or in the normal mode  
  Serial.println("Starting Wifi on Core 2");
  WiFi.setHostname(default_hostname);
  Serial.print("Start WIFI connection to sid ");
  Serial.println(applicationSettings.wlanSsid);
  WiFi.begin(applicationSettings.wlanSsid.c_str(), applicationSettings.wlanPass.c_str());
 


  for(;;) 
  {
    
    if(WiFi.status() != WL_CONNECTED )
    { 
      Serial.println("WIFI connection broken - reopen the connection ....");
      tryConnectToWifi();
    }
    delay(500);
  }

};




void setup()
{

  //init ports and settings
  Serial.begin(115200); 
  
// struct AppSettings { String wlanSsid; String wlanPass; String gmtOffsetInSeconds; String smtpServer; 
//           String smtpUser; String smtpPass; String smtpPort; String smtpTlsMode; String smtpRecipient;};

 applicationSettings = { "unifi", 
                "3N3FXAI4RP",
                "0", 
                "smtp.gmail.com",
                "cfoespmail@gmail.com",
                "phcawtrmkavlitzv", 
                "465",
                "0", 
                "christophe.fous@gmail.com"};




 
  // //Check if the module is configured (settings are available or not)
  // if(readSettings())
  // {
  //   Serial.println("Configuration found.... try to start WLAN and BLE services ...");
  //   //settings were getting found, start WIFI and BLE
  //   serverMode = ServerMode::normal;

  // }
  // else 
  // {
  //   //no settings found, start access point to offer a WLAN 192.168.4.1 to configure the module
  //   serverMode = ServerMode::init;
  //   Serial.println("No configuration found.... starting AP on IP 192.168.4.1 ...");
  //   delay(1000);

  // }

    
  // xTaskCreatePinnedToCore(
  //     conntecToWifiTask, /* Function to implement the task */
  //     "WIFI", /* Name of the task */
  //     10000,  /* Stack size in words */
  //     NULL,  /* Task input parameter */
  //     0,  /* Priority of the task */
  //     &WifiConnectionTask,  /* Task handle. */
  //     0); /* Core where the task should run */



  WiFi.mode(WIFI_STA);
  WiFi.begin("unifi", "3N3FXAI4RP");
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());

serverMode = ServerMode::normal;
//   #define SMTP_HOST "smtp.gmail.com"
// #define SMTP_PORT 465
// #define AUTHOR_EMAIL "cfoespmail@gmail.com"
// #define AUTHOR_PASSWORD "phcawtrmkavlitzv"
// #define RECIPIENT_EMAIL "christophe.fous@gmail.com"



  if(serverMode == ServerMode::normal)
  { 

  
ESP_MAIL_PRINTF("Free Heap: %d\n", MailClient.getFreeHeap());
    MailClient.networkReconnect(true);
    //smtp.debug(1);//debug via serial port or not
    smtp.callback(smtpCallback);

 
    /* Set the session config */
    session.server.host_name = applicationSettings.smtpServer;
    session.server.port = (uint16_t) applicationSettings.smtpPort.toInt();
    session.login.email = applicationSettings.smtpUser;
    session.login.password = applicationSettings.smtpPass;

    /* Set the NTP config time */
    // session.time.ntp_server = F("pool.ntp.org,time.nist.gov");
    // session.time.gmt_offset = 3;
    // session.time.day_light_offset = 0;
    //set hostname and turn on the wifi connection



  message.sender.name = F("ESP Mail"); // This witll be used with 'MAIL FROM' command and 'From' header field.
  message.sender.email =  "cfoespmail@gmail.com"; // This witll be used with 'From' header field.
  message.subject = F("from setup:   Test sending plain text Email");
  message.addRecipient(F("Someone"), F("christophe.fous@gmail.com")); // This will be used with RCPT TO command and 'To' header field.
  String textMsg = "This is simple plain text message";
  message.text.content = textMsg;
  message.text.charSet = F("us-ascii");
  message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;
  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_low;
  message.addHeader(F("Message-ID: <abcde.fghij@gmail.com>"));
  if (!smtp.connect(&session /* session credentials */))
    return;
  if (!MailClient.sendMail(&smtp, &message))
    Serial.println("Error sending Email, " + smtp.errorReason());
  ESP_MAIL_PRINTF("Free Heap: %d\n", MailClient.getFreeHeap());

Serial.println("starting BLE");



  BLEDevice::init("");
  // xTaskCreatePinnedToCore(
  //     connectToBleServerTask, /* Function to implement the task */
  //     "BLE", /* Name of the task */
  //     10000,  /* Stack size in words */
  //     NULL,  /* Task input parameter */
  //     0,  /* Priority of the task */
  //     &BleConnectionTask,  /* Task handle. */
  //     1); /* Core where the task should run */


  }

}


unsigned long p_millis = millis();

void loop() 
{
if (millis()-p_millis>500)
{
  p_millis = millis();
   if(!bleConnected)
    {
      Serial.println("Scanning for BLE devices");
      BLEScan* pBLEScan = BLEDevice::getScan();
      pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
      pBLEScan->setActiveScan(true);
      pBLEScan->start(0);
    }

    //Serial.println("try to find the BLE seonsor");
    if (tryConnectBleServer == true) //this variable is getting set from the BLE callback function ... means: wait until a matching BLE device has found
    {
      Serial.println("Matching BLE device found, ... try to connect ...");
      if (connectToBleServer()) {
        Serial.println("We are now connected to the BLE Server.");
        //Activate the Notify property of each Characteristic
        gyroCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        bleConnected = true;
      } else {
        Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
      }
      tryConnectBleServer = false;
    }

}
  // ESP_MAIL_PRINTF("loop Free Heap: %d\n", MailClient.getFreeHeap());
  
  // vTaskDelay(10);
  // //if the reset button has getting pressed or no WIFI settings are configured 
  // if(serverMode == ServerMode::init)
  // {
  //    //...start the access point configuration
  //   Serial.println("Preparing web interface");
  //   //delay(10000);
  //   launchConfigWebInterface();
  //   //delay(10000);
  //   Serial.println("Turning on the Access Point");
  //   setupAP();// Setup HotSpot   
  //   //delay(10000);
  //   //waiting for clients they want to configure the ESP32 controller
  //   Serial.println();
  //   Serial.println("Waiting for incoming connections to configure the controller");
  //   checkResetButtonPushed();
  //   while ((WiFi.status() != WL_CONNECTED))
  //   {
  //     Serial.print("~");
  //     delay(100);
  //     server.handleClient();
  //   }
  //   delay(500);
  
  // }
 
}

bool tryConnectToWifi(void)
{
  int c = 0;
  
  Serial.println("Trying to connect to WIFI");

  while ( c < 20 ) 
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("");
      Serial.println("WIFI connected");
      return true;
    }
    delay(300);
    Serial.print("+");
    vTaskDelay(20);
    checkResetButtonPushed();
    c++;
  }
  Serial.println("");
  Serial.println("Timeout. No WIFI available.");
  return false;
}

String getAvailableNetworkList()
{
  String availableWifiNetworksAsHtmlList="<select name=\"wlan_ssid\">";
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  
  for (int i = 0; i < n; ++i)
  {
    // Print SSID and RSSI for each network found
    availableWifiNetworksAsHtmlList += "<option value='";
    availableWifiNetworksAsHtmlList += WiFi.SSID(i);
    availableWifiNetworksAsHtmlList += "'>";
    availableWifiNetworksAsHtmlList += WiFi.SSID(i);
    availableWifiNetworksAsHtmlList += "</option>";
  }
  availableWifiNetworksAsHtmlList += "</select>";
  Serial.print("network checkbox:");
  Serial.println(availableWifiNetworksAsHtmlList);
  return availableWifiNetworksAsHtmlList;
}

void createWebServer()
{
  {
    server.on("/", []() {
      Serial.println("client connect to rout /");
      IPAddress ip = WiFi.softAPIP();
      String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);


      String content = "<!DOCTYPE html><script>function activate(choosedApp) { for(i=1; i<=2; ++i) { var x = document.getElementById('settings' + i); if(i==choosedApp) { document.getElementById('settings' + i).className = 'activeDiv'; } else { document.getElementById('settings' + i).className = 'hiddenDiv'; } }}</script><html> <head> <title>ESP32 - config</title> <meta name='viewport' content='width=device-width, initial-scale=1'> <meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate' /> <meta http-equiv='Pragma' content='no-cache' /> <meta http-equiv='Expires' content='0' /> <meta http-equiv='content-type' content='application/json; charset=ISO-8859-1'> </head><head><style>body {margin: 0;font-family: Arial, Helvetica, sans-serif;}.topnav {overflow: hidden;background-color: #333;}.topnav a {float: left;color: #f2f2f2;text-align: center;padding: 14px 16px;text-decoration: none;font-size: 17px;}.topnav a:hover {background-color: #ddd;color: black;}.topnav a.active {background-color: #04AA6D;color: white;}.activeDiv { padding-left:16px; display: block; }.hiddenDiv { padding-left:16px; display: none; }.ws-btn {padding: 10px;box-sizing: border-box;border-radius: 10px 10px 10px 10px;background: #04AA6D;border:none;color: #ffffff;font-size: 17px;border-color: #04AA6D;vertical-align: middle;overflow: hidden;text-decoration: none;cursor: pointer;}.tab{ border:none; white-space: nowrap; text-align: left; border-radius: 10px;border-collapse: collapse; caption-side: top;}.cap{ text-align: left; font-weight: bold; font-size: 20px; border-spacing: 20px; color: #bbbbbb;}</style></head><body><form method='post'enctype='multipart/form-data' action='saveSettings' name='f1'> <div class='topnav'> <a class='active' onclick='activate(1)'>WIFI</a> <a onclick='activate(2)'>SMTP</a> </div> <div id='settings1' style='padding-left:16px' class='activeDiv'> <p>&nbsp;</p> <table id='TabModeNamesSet1' class='tab'> <caption class='cap' >WLAN Settings</caption> <tr> <td> SSID: </td> <td> ##NETWORK_LIST## </td> </tr> <tr> <td> Password: </td> <td> <input type='password' name='wlan_pass' length='64'> </td> </tr> <tr> <td> Timezone: </td> <td> <select name='timezone_offset' length=6><option value='0'>GMT</option><option value='3600'>GMT+1</option><option value='7200'>GMT+2</option><option value='10800'>GMT+3</option><option value='14400'>GMT+4</option><option value='18000'>GMT+5</option><option value='21600'>GMT+6</option><option value='25200'>GMT+7</option><option value='28800'>GMT+8</option><option value='32400'>GMT+9</option><option value='36000'>GMT+10</option><option value='39600'>GMT+11</option><option value='43200'>GMT+12</option><option value='46800'>GMT+12</option><option value='50400'>GMT+12</option><option value='-3600'>GMT-1</option><option value='-7200'>GMT-2</option><option value='-10800'>GMT-3</option><option value='-14400'>GMT-4</option><option value='-18000'>GMT-5</option><option value='-21600'>GMT-6</option><option value='-25200'>GMT-7</option><option value='-28800'>GMT-8</option><option value='-32400'>GMT-9</option><option value='-3600'>GMT-10</option><option value='-39600'>GMT-11</option><option value='-43200'>GMT-11</option></select> </td> </tr> </table> </div> <div id='settings2' style='padding-left:16px' class='activeDiv'> <p>&nbsp;</p> <table id='TabModeNamesSet1' class='tab'> <caption class='cap' >MAIL Settings</caption> <tr> <td> SMTP Server: </td> <td> <input name='smtp_server' length='64'> </td> </tr> <tr> <td> SMTP User: </td> <td> <input name='smtp_user' length='64'> </td> </tr><tr> <td> Mail Recipient: </td> <td> <input name='smtp_recip' length='64'> </td> </tr> <tr> <td> SMTP Password: </td> <td> <input type='password' name='smtp_pass' length='64'> </td> </tr> <tr> <td> SMTP Port: </td> <td> <input type='text' name='smtp_port' length='3' value='587'> </td> </tr> <tr> <td> TSL Port: </td> <td> <select name='smtp_tls_mode'><option value='0'>non</option><option value='1'>SSL/TLS</option><option value='2'>SSL/TLS (accept all certificates)</option><option value='3'>STARTTLS</option><option value='4'>STARTTLS (accept all certificates)</option></select> </td> </tr> </table> <button type='submit' class='ws-btn' >Save Configuration</button> </div> <p>&nbsp;</p></form></body></html><script> activate(1);</script>";
      content.replace("##NETWORK_LIST##",getAvailableNetworkList());
      server.send(200, "text/html", content);
  
    });
    server.on("/scan", []() {
      Serial.println("client connect to rout /scann");
      IPAddress ip = WiFi.softAPIP();
      String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
      content = "<!DOCTYPE HTML>\r\n<html>go back</html>";
      server.send(200, "text/html", content);
    });
    server.on("/saveSettings", []() {
      Serial.println("");
      if (server.arg("wlan_ssid").length() > 0 && server.arg("wlan_pass").length() > 0)
      {
        //write the received settings to the EEPROM
        writeSettings(server.arg("wlan_ssid"),server.arg("wlan_pass"),server.arg("timezone_offset"),
                    server.arg("smtp_server"), server.arg("smtp_user"), server.arg("smtp_pass"), 
                    server.arg("smtp_port"), server.arg("smtp_tls_mode"), 
                    server.arg("smtp_recip")  );

        content = "<!DOCTYPE HTML>\r\n<html><h1>Configuration received and stored. Restarting ESP32...</h1></html>";
        server.send(200, "text/html", content);
        delay(3000);
        
        ESP.restart();
      } 
      else {
        content = "{\"Error\":\"404 not found\"}";
        statusCode = 404;
        Serial.println("Sending 404: problem on receiving the get parameters");
      }
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(statusCode, "application/json", content);
    });
  }
}

//Start "Configuration"-Webserver
void launchConfigWebInterface()
{
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("WiFi connected");

  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());

  Serial.println("Creating Webserver");
  createWebServer();
  // Start the server
  server.begin();
  Serial.println("Server started");
}


/*
Read out the available WLAN networks, create a dynamic HTML page with this information an start a Webserver on IP 192.168.4.1 to show it to the user
*/
void setupAP(void)
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  //open a AP with the default IP,SSID and hostname settings
  WiFi.softAP(default_hostname, "");
}


/* Callback function to get the Email sending status */

void smtpCallback(SMTP_Status status)
{
  // Print the current status 
  Serial.println(status.info());

  // Print the sending result 
  if (status.success())
  {
    // ESP_MAIL_PRINTF used in the examples is for format printing via debug Serial port
    // that works for all supported Arduino platform SDKs e.g. AVR, SAMD, ESP32 and ESP8266.
    // In ESP32 and ESP32, you can use Serial.printf directly.

    Serial.println("----------------");
    ESP_MAIL_PRINTF("Message sent success: %d\n", status.completedCount());
    ESP_MAIL_PRINTF("Message sent failed: %d\n", status.failedCount());
    Serial.println("----------------\n");

    for (size_t i = 0; i < smtp.sendingResult.size(); i++)
    {
      // Get the result item 
      SMTP_Result result = smtp.sendingResult.getItem(i);

      // In case, ESP32, ESP8266 and SAMD device, the timestamp get from result.timestamp should be valid if
      // your device time was synched with NTP server.
      // Other devices may show invalid timestamp as the device time was not set i.e. it will show Jan 1, 1970.
      // You can call smtp.setSystemTime(xxx) to set device time manually. Where xxx is timestamp (seconds since Jan 1, 1970)
      time_t ts = (time_t)result.timestamp;

      ESP_MAIL_PRINTF("Message No: %d\n", i + 1);
      ESP_MAIL_PRINTF("Status: %s\n", result.completed ? "success" : "failed");
      ESP_MAIL_PRINTF("Date/Time: %s\n", asctime(localtime(&ts)));
      ESP_MAIL_PRINTF("Recipient: %s\n", result.recipients.c_str());
      ESP_MAIL_PRINTF("Subject: %s\n", result.subject.c_str());
    }
    Serial.println("----------------\n");

    // You need to clear sending result as the memory usage will grow up.
    smtp.sendingResult.clear();
  }
}
