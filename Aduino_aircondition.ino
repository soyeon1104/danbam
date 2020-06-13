#include <LiquidCrystal.h>

#include <LiquidCrystal_I2C.h>           // LiquidCrystal_I2C의 라이브러리를 불러옵니다.(I2C LCD)
LiquidCrystal_I2C lcd(0x27, 16, 2);      // lcd(LCD의 I2C 슬레이브 주소, lcd 1줄당 출력할 글자수, lcd 줄의 수)

bool BT_ONOFF = true;

int Vo = A0;
int V_LED = 2;

float Vo_value = 0;
float Voltage = 0;
float dustDensity = 0;

int LED_R = 9;
int LED_G = 10;
int LED_B = 11;

int FAN = A1;
/* **************** millis ******************* */
unsigned long millis_100hz, millis_1hz;

/* **************** Filter ***************** */
//////// For Low Pass Filter /////////////
float low_pass_filter_data, ex_low_pass_filter_data;
float alpha = 0.99;
//////// For Moving Average Filter /////////////
float moving_average_filter_data, ex_moving_average_filter_data;
float data_stack[10];
int delta = 10;

/* **************** Bluetooth *************** */
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(6,7); // RX, TX

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(V_LED, OUTPUT);
  pinMode(Vo, INPUT);

  lcd.init();                // LCD_I2C 통신을 시작합니다.
  lcd.backlight();           // LCD backlight를 ON

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);

  pinMode(FAN, OUTPUT);
  digitalWrite(FAN, LOW);

/* ********* Bluetooth ********** */
  BTSerial.begin(9600);  
}

void loop() {
  
  if (BTSerial.available()) {    
    char data = BTSerial.read();
    Serial.write(data);
    
    if(data == '1'){
      Serial.write("1이다");
      BT_ONOFF = true;
    } else if(data == '0'){
      Serial.write("0이다");
      BT_ONOFF = false;
    }
  }
  
  if( (millis() - millis_100hz) >= 10 ){     // 100 Hz //
    millis_100hz = millis();
    // put your main code here, to run repeatedly:
    digitalWrite(V_LED, LOW);
    delayMicroseconds(280);
    Vo_value = analogRead(Vo);
    delayMicroseconds(40);
    digitalWrite(V_LED, HIGH);
    delayMicroseconds(9680);
  
    Voltage = Vo_value / 1024 * 5.0;
    dustDensity = (Voltage - 0.3) / 0.005;

    /* ********************* Low Pass Filter ***************** */
    low_pass_filter_data = alpha * ex_low_pass_filter_data + (1 - alpha) * dustDensity;
    ex_low_pass_filter_data = low_pass_filter_data;

    /* ********************* Moving Average Filter *************** */
    for(int i=0; i<(delta-1); i++){
      data_stack[i] = data_stack[i+1];
    }
    data_stack[(delta-1)] = dustDensity;
    int sum_data_stack = 0;
    for(int i=0; i<delta; i++){
      sum_data_stack += data_stack[i];
    }
    moving_average_filter_data = sum_data_stack / (float)delta;
  }

  if( (millis() - millis_1hz) >= 1000 ){     // 1 Hz //
    millis_1hz = millis();
    ////////////////////////////////////////////////
  
    lcd.clear();        //lcd 화면을 지웁니다.
    lcd.home();        //lcd 커서 위치를 0,1로 위치시킵니다.
    if( dustDensity > 150 ){    // 매우 나쁨 //
      lcd.print("AIR : VERY BAD!!");   
    }else if( dustDensity > 80){   // 나쁨 //
      lcd.print("AIR : BAD!      ");   
    }else if( dustDensity > 30){  // 보통 //
      lcd.print("AIR : NORMAL     ");   
    }else{                    // 좋음
      lcd.print("AIR : GOOD      ");   
    }
    lcd.setCursor(0, 1); // lcd 커서의 위치를 4,0으로 설정합니다.
    lcd.print("ug/m3:"); // 현재 lcd 커서 위치로부터 "Good Day" 내용을 출력합니다.
    lcd.print(moving_average_filter_data);

    Serial.print("RAW : ");
    Serial.print(dustDensity);
    Serial.print("\tMAF : ");
    Serial.print(moving_average_filter_data);
    Serial.print("\tLPF : ");
    Serial.println(low_pass_filter_data);

    if(BT_ONOFF && dustDensity > 30){
      digitalWrite(FAN, HIGH);
    }else{
      digitalWrite(FAN, LOW);    
    }
    
    if(dustDensity > 150){
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, LOW);
    }else if(dustDensity > 80){
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, HIGH);    
    }else if(dustDensity > 30){
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_B, LOW);    
    }else{
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_B, LOW);    
    }
    /* ********* Bluetooth ************ */
    BTSerial.print("DUST:");
    BTSerial.println(low_pass_filter_data);
  }
}
