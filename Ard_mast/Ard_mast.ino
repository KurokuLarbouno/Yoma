#pragma once
//#include "./Mypulse.cpp"
#include <Event.h>
#include <Timer.h>
#include "./MyHX711/HX711.cpp"
#include <WiFi.h>
#include <IOXhop_FirebaseESP32.h>
#include <Wire.h>

//體重
  #define calibration_factor -7050.0 //This value is obtained using the SparkFun_HX711_Calibration sketch
//一般
  int device_state = 0, device_able = 0;//0:prepear 1:wait for begining 2:wait for ending
  const int btn_weight = 22; //按鈕腳位
  const int btn_sit =  23;//按鈕腳位
  int post_weight = 0, post_sit = 0, cur_weight = 0, cur_sit = 0; //兩個按紐的判定
  Timer checker;//總計時器
  int stand_checker, sit_checker, online_checker;//感測器取樣時間
  String timeStamp, endStamp;//開始及結束時間郵戳
  String UID;//使用者ID
//firebase setting
  #define FIREBASE_HOST "https://yoma-8d99f.firebaseio.com/"
  #define FIREBASE_AUTH "Ja5ynaDw97xtuMZjQbkENLNvS14mjRs58RFM6pIP"
//WiFi setting
  //"lee""lee@9280"
  //"GDlab""gdlab000"
  //"D-Link_DIR-612""98765432"
  //"TING" "098765432"
  //#define WIFI_SSID "leelee"
  //#define WIFI_PASSWORD "k12345678"
  //#define 
  #define WIFI_SSID "D-Link_DIR-612"
  #define WIFI_PASSWORD "98765432"
//standard time setting
  const char* ntpServer = "pool.ntp.org";
  const long  gmtOffset_sec = 8*60*60;
  const int   daylightOffset_sec = 0;//offset of clk
  struct tm start_time_info, end_time_info;
//體重
  int weight_checker = 0, weight_count = 0, weight_sum = 0, weight_st = 0;//感測器取樣頻率/取樣數/總和/感測器功能
  int weight_dt_count = 0;
  HX711 scale;
//心率
  int pulse_checker = 0, pulse_count = 0, pulse_sum = 0, pulse_st = 0;//感測器取樣頻率/取樣數/總和/感測器功能
  int pulse_dt_count = 0;
  const int PulseWire = 34;      // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
  volatile int rate[10];                    // array to hold last ten IBI values
  volatile unsigned long sampleCounter = 0; // used to determine pulse timing
  volatile unsigned long lastBeatTime = 0;  // used to find IBI
  volatile int P =512;                      // used to find peak in pulse wave, seeded
  volatile int T = 512;                     // used to find trough in pulse wave, seeded
  volatile int thresh = 512;                // used to find instant moment of heart beat, seeded
  volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
  volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
  volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM
  //  VARIABLES
  int fadePin = 12;                 // pin to do fancy classy fading blink at each beat
  int fadeRate = 0;                 // used to fade LED on with PWM on fadePin
  // these variables are volatile because they are used during the interrupt service routine!
  volatile int BPM;                   // used to hold the pulse rate
  volatile int Signal;                // holds the incoming raw data
  volatile int IBI = 600;             // holds the time between beats, must be seeded! 
  volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
  volatile boolean QS = false;        // becomes true when Arduoino finds a beat.
  hw_timer_t * timer = NULL;
  void ISRTr();
//體溫
  int temp_checker = 0, temp_count = 0, temp_sum = 0, temp_st = 0;//感測器取樣頻率/取樣數/總和/感測器功能
  int temp_dt_count = 0;
  int temp_pin= 36;
//如廁時間
  int toilet_dt_count = 0;


void interruptSetup(){     
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);
  
  // Initializes Timer to run the ISR to sample every 2mS as per original Sketch.
  // Attach ISRTr function to our timer.
  timerAttachInterrupt(timer, &ISRTr, true);

  // Set alarm to call isr function every 2 milliseconds (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 2000, true);

  // Start an alarm
  timerAlarmEnable(timer);
   
} 
// THIS IS THE HW-TIMER INTERRUPT SERVICE ROUTINE. 
// Timer makes sure that we take a reading every 2 miliseconds
void ISRTr(){                                 // triggered when timer fires....
  Signal = analogRead(34);                    // read the Pulse Sensor on pin 34 3.3v sensor power......default ADC setup........
  Signal = map(Signal, 0, 4095, 0, 1023);     // Map the value back to original sketch range......
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

    //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
    if (Signal < T){                        // T is the trough
      T = Signal;                         // keep track of lowest point in pulse wave 
    }
  }
  if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
    P = Signal;                             // P is the peak
  }                                        // keep track of highest point in pulse wave
  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250){                                   // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){        
      Pulse = true;                               // set the Pulse flag when we think there is a pulse
      digitalWrite(2,HIGH);                // turn on pin 13 LED
      IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse

      if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
        secondBeat = false;                  // clear secondBeat flag
        for(int i=0; i<=9; i++){             // seed the running total to get a realisitic BPM at startup
          rate[i] = IBI;                      
        }
      }

      if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        sei();                               // enable interrupts again
        return;                              // IBI value is unreliable so discard it
      }   
      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                  // clear the runningTotal variable    

      for(int i=0; i<=8; i++){                // shift data in the rate array
        rate[i] = rate[i+1];                  // and drop the oldest IBI value 
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values 
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag 
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
  }
  if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
    digitalWrite(2,LOW);            // turn off pin 13 LED
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }
  if (N > 2500){                           // if 2.5 seconds go by without a beat
    thresh = 512;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }
}

void weight_check(){
    if(weight_st){
        //scale.power_up();
        //體重
          int value = scale.get_units() * 0.32;
          weight_sum += value;
          //Serial.print(weight_pick:);Serial.println(value);
          weight_count++;
        //scale.power_down();
    }
}

void pulse_check(){
    if(pulse_st){
        //心率
          int value = BPM;
          pulse_sum += value;
          //Serial.print("pulse_pick:");Serial.println(value);
          pulse_count++;
    }
}

void temp_check(){
    if(temp_st){
        //體溫
          int rawvoltage= analogRead(temp_pin);
          float celsius= (rawvoltage / 2048.0) * 330;
          temp_sum += celsius + 3.5f;
          //Serial.print("temp_pick:");Serial.println(value);
          temp_count++;       
    }
}

void stand_check(){
  //停止所有取樣
    checker.stop(weight_checker);
  //如廁
    //路徑
      String countTimePath = String("PJIOT/toilet") + UID + String("/0/");
      String startTimePath = String("PJIOT/toilet") + UID + String("/") + (toilet_dt_count * 3 - 2);
      startTimePath += String("/");
    //Serial.println(startTimePath);
      String endTimePath = String("PJIOT/toilet") + UID + String("/") + (toilet_dt_count * 3 - 1);
      endTimePath += String("/");
      String useTimePath = String("PJIOT/toilet") + UID + String("/") + (toilet_dt_count * 3);
      useTimePath += String("/");
    //Update data
      Firebase.setString(startTimePath, timeStamp);  //填入時間郵戳
      Firebase.setString(endTimePath, "---");
      Firebase.setString(useTimePath, "---");
    // handle error
      if (Firebase.failed()) {
        Serial.print("pushing /logs failed:");
        Serial.println(Firebase.error());  
        return;
      }
  //體重
    //路徑
      String timepath = String("PJIOT/weight") + UID + String("/") + (weight_dt_count * 2 - 1);
      timepath += String("/");
      Serial.println(timepath);
      String datapath = String("PJIOT/weight") + UID + String("/") + (weight_dt_count * 2);
      datapath += String("/");
    //Update data
      int myWieght = weight_sum / weight_count;
      weight_sum = 0; weight_count = 0;
      Firebase.setString(timepath, timeStamp);  //填入時間郵戳
      Firebase.setInt(datapath, myWieght);         //填入指定資料
    // handle error
      if (Firebase.failed()) {
        Serial.print("pushing /logs failed:");
        Serial.println(Firebase.error());  
        return;
      }else weight_dt_count++;
      Firebase.setInt("PJIOT/weight" + UID + "/0/", weight_dt_count);
    //state update
      device_state = 2;
      device_able = 1;
}

void sit_check(){  
  //停止所有取樣
  checker.stop(temp_checker);
  checker.stop(pulse_checker);
  //如廁
    //路徑
      String countTimePath = String("PJIOT/toilet") + UID + String("/0/");
      //String startTimePath = String("PJIOT/toilet/") + (toilet_dt_count * 3 - 2);
      //startTimePath += String("/");
      //Serial.println(startTimePath);
      String endTimePath = String("PJIOT/toilet") + UID + String("/") + (toilet_dt_count * 3 - 1);
      endTimePath += String("/");
      String useTimePath = String("PJIOT/toilet") + UID + String("/") + (toilet_dt_count * 3);
      useTimePath += String("/");
      //toilet_dt_count++;
      Firebase.setInt(countTimePath, toilet_dt_count);
    //計算使用時間
      String useTimeStamp = genUseStamp(start_time_info, end_time_info);
    //Update data
      Firebase.setString(endTimePath, endStamp);//填入結束時間
      Firebase.setString(useTimePath, useTimeStamp);
    // handle error
      if (Firebase.failed()) {
        Serial.print("pushing /logs failed:");
        Serial.println(Firebase.error());  
        return;
      }
      toilet_dt_count++;
      Firebase.setInt("PJIOT/toilet" + UID + "/0/", toilet_dt_count);
  //體溫
    //路徑
      String timepath = String("PJIOT/temp") + UID + String("/") + (temp_dt_count * 2 - 1);
      timepath += String("/");
      Serial.println(timepath);
      String datapath = String("PJIOT/temp") + UID + String("/") + (temp_dt_count * 2);
      datapath += String("/");
    //Update data
      int myTemp = temp_sum / temp_count;
      temp_sum = 0; temp_count = 0;
      Firebase.setString(timepath, timeStamp);   //填入時間郵戳
      Firebase.setInt(datapath, myTemp);         //填入指定資料
    // handle error
      if (Firebase.failed()) {
        Serial.print("pushing /logs failed:");
        Serial.println(Firebase.error());  
        return;
      }
      temp_dt_count++;
      Firebase.setInt("PJIOT/temp" + UID + "/0/", temp_dt_count);
  //心率
    //路徑
      timepath = String("PJIOT/heart_reat") + UID + String("/") + (pulse_dt_count * 2 - 1);
      timepath += String("/");
      Serial.println(timepath);
      datapath = String("PJIOT/heart_reat") + UID + String("/") + (pulse_dt_count * 2);
      datapath += String("/");
    //Update data
      int myPulse = pulse_sum / pulse_count;
      Serial.println("myPulse");
      pulse_sum = 0; pulse_count = 0;
      Firebase.setString(timepath, timeStamp);  //填入時間郵戳
      Firebase.setInt(datapath, myPulse);         //填入指定資料
      Firebase.setInt("PJIOT/heart_reat" + UID + "/0/", pulse_dt_count);
    // handle error
      if (Firebase.failed()) {
        Serial.print("pushing /logs failed:");
        Serial.println(Firebase.error());  
        return;
      }
      pulse_dt_count++; 
      Serial.println("FIN");
      Firebase.setInt("PJIOT/heart_reat" + UID + "/0/", pulse_dt_count);
    //state update
      device_state = 1;
      device_able = 1;
}

void online_check(){
  //更新上線者帳號到UID中
  //Serial.println("HI");
  device_able = 0;//prepear
  String s = Firebase.getString("PJIOT/onlineuser/0/");
  //Serial.println(s);
  if (s.length() > 10){
    String newid = "";
    for (int i=0; i<s.length()+1;i++){
        if(s.charAt(i) > 47 && s.charAt(i) < 173 && s.charAt(i) != 92){
          newid += s.charAt(i);
        }
    }
    //Serial.println(newid);
    if (newid != UID){
      UID = newid;
      Serial.println(UID);
      count_refresh();
      Serial.println("User change to:"+UID);
    }
  }
  Serial.println("user check");
  device_able = 1;
}

void count_refresh(){
    //check data counter(在0的位置)有就取沒就加
  Serial.println("pul/wei/temp/toil");
  String path = String("PJIOT/heart_reat") + UID + String("/0/");
  int count = Firebase.getInt(path);
  if(count == 0){Serial.println("counter not fund");Firebase.setInt(path,1); count = 1;}//設定全部size，0為沒有counter值，1為還沒有任何資料
  pulse_dt_count = count;
  Serial.println(count);
  
  path = String("PJIOT/weight") + UID + String("/0/");
  count = Firebase.getInt(path);
  if(count == 0){Serial.println("counter not fund");Firebase.setInt(path,1); count = 1;}//設定全部size，0為沒有counter值
  weight_dt_count = count;
  Serial.println(count);
  
  path = String("PJIOT/temp") + UID + String("/0/");
  count = Firebase.getInt(path);
  if(count == 0){Serial.println("counter not fund");Firebase.setInt(path,1); count = 1;}//設定全部size，0為沒有counter值
  temp_dt_count = count;
  Serial.println(count);
  
  path = String("PJIOT/toilet") + UID + String("/0/");
  count = Firebase.getInt(path);
  if(count == 0){Serial.println("counter not fund");Firebase.setInt(path,1); count = 1;}//設定全部size，0為沒有counter值
  toilet_dt_count = count;
  Serial.println(count);
}

void stand_event(){
    //1.整理重量資訊 2.整理時間戳記 3.推送"體重"及"如廁時間"
      device_able = 0;
      weight_st = 0; weight_count = 0; weight_sum = 0;
    //開始時間
      UpadateLocalTime(start_time_info);
      timeStamp = genStamp(start_time_info);
      Serial.print("start using at:");
      Serial.println(timeStamp);
      Serial.println("weight");
    //checker startup
      //timer.after(time, callback)
      stand_checker = checker.after(1000, stand_check);
      //timer.every(time, callback)
      weight_st = 1;
      weight_checker = checker.every(13, weight_check);//80HZ
    delay(100);
}

void sit_event(){
    //1.整理體溫資訊 2.整理心率資訊 3.推送"體溫"及"心率"
      device_able = 0;
      temp_st = 0; temp_count = 0; temp_sum = 0;
      pulse_st = 0; pulse_count = 0; pulse_sum = 0;
    //開始時間
      UpadateLocalTime(end_time_info);
      endStamp = genStamp(end_time_info);
      Serial.print("stop using at:");
      Serial.println(endStamp);
      Serial.println("pulse");
    //checker startup
      sit_checker = checker.after(1000, sit_check);
      pulse_st = 1;
      temp_st = 1;  
      //pulse_sum = BPM;   
      pulse_checker = checker.every(100, pulse_check);
      temp_checker = checker.every(20, temp_check);
      delay(100);
}
void setup() {
  pinMode(btn_sit, INPUT);
  pinMode(btn_weight, INPUT);
  Serial.begin(115200);
  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  //體重
    scale.begin(32, 33);        //scale.begin(DOUT, CLK);
    scale.set_scale(6500.0);    // this value is obtained by calibrating the scale with known weights; see the README for details
    scale.tare();                // reset the scale to 0
    scale.power_up();
  //standard time 
  //init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  //上線者檢查
  Serial.println("HI");
  online_check();
  online_checker = checker.every(60000, online_check);
  device_state = 1;//準備接收
  //心率
    pinMode(2,OUTPUT);      // pin that will blink to your heartbeat!
    interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
}

void loop() {
  //更新數據
  //判定按鍵輸入
  //A:體重事件；B馬桶蓋事件
  checker.update();
  cur_weight = digitalRead(btn_weight);
  cur_sit = digitalRead(btn_sit);

  if(!post_weight && cur_weight && device_state == 1 && device_able == 1){
      if(device_able == 0){
        checker.after(500, stand_event);
      }else{stand_event();}
  }
  if(!post_weight && cur_weight && device_state == 2 && device_able == 1){
      if(device_able == 0){
        checker.after(500, sit_event);
      }else{sit_event();}
  }
//Button Update
  post_weight = cur_weight;
  post_sit = cur_sit;
//interrupt update
  yield();
  //Serial.println(BPM);
}

String genStamp(tm& tminfo){
  String timeStamp = String(tminfo.tm_year + 1900);
  timeStamp += String("/");
  timeStamp += String(tminfo.tm_mon + 1);
  timeStamp += String("/");
  timeStamp += String(tminfo.tm_mday);
  timeStamp += String("/");
  timeStamp += String(tminfo.tm_hour);
  timeStamp += String(":");
  timeStamp += String(tminfo.tm_min);
  timeStamp += String(":");
  timeStamp += String(tminfo.tm_sec);
  return(timeStamp);
}

String genUseStamp(tm& st_tm, tm& ed_tm){
  String timeStamp;
  int thour = (ed_tm.tm_hour - st_tm.tm_hour); int tmin = (ed_tm.tm_min - st_tm.tm_min); int tsec = (ed_tm.tm_sec - st_tm.tm_sec);
  if (tsec < 0) {tsec += 60; tmin -= 1;}
  if (tmin < 0) {tmin += 60; thour -= 1;}
  timeStamp = String(thour) + ":" + String(tmin) + ":" + String(tsec) ;
  return(timeStamp);
}
void UpadateLocalTime(tm& timeinfo)
{
  if(!getLocalTime(&timeinfo)){
    Serial.print(!getLocalTime(&timeinfo));
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}
