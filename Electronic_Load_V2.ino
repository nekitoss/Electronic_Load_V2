/////////////////////////////Library for i2c LCD//////////////////////////////////
#include <Wire.h>
#include <LiquidCrystal_I2C.h>      //Download it here: https://www.electronoobs.com/eng_arduino_liq_crystal.php
LiquidCrystal_I2C lcd(0x27,16,2); //slave address sometimes can be 0x3f or 0x27. Try both!
uint8_t arrow[8] = {0x0, 0x4 ,0x6, 0x3f, 0x6, 0x4, 0x0};
uint8_t ohm[8] = {0xE ,0x11, 0x11, 0x11, 0xA, 0xA, 0x1B};
uint8_t up[8] = {0x0 ,0x0, 0x4, 0xE , 0x1F, 0x4, 0x1C, 0x0};


/////////////////////////////Library for ADS1115 ADC//////////////////////////////////
#include <Adafruit_ADS1X15.h>       //Download here: https://www.electronoobs.com/eng_arduino_Adafruit_ADS1015.php
Adafruit_ADS1115 ads;         //Define i2c address 
#define ADS1115_CONVERSIONDELAY  (1)
#define ADS1015_CONVERSIONDELAY  (1)


/////////////////////////////Library for MCP4725 DAC//////////////////////////////////
#include <Adafruit_MCP4725.h>       //Download here: https://www.electronoobs.com/eng_arduino_Adafruit_MCP4725.php      
//You need BusIO library as well    //install it with Arduino library manager if you don't have it
Adafruit_MCP4725 dac;
// Set this value to 9, 8, 7, 6 or 5 to adjust the resolution
#define DAC_RESOLUTION    (9) //DAC resolution 12BIT: 0 to 4056
//////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////INPUTS/OUTPUTS/////////////////////////////////////
#define ENCODER_DT_PIN 2
#define ENCODER_CLK_PIN 3
#define ENCODER_BTN_PIN A3         //push button from encoder
#define STOP_BTN_PIN 7    //(in my case) red push button for stop/resume
#define MENU_BTN_PIN 6   //(in my case) blue push button for menu
#define BUZZER_PIN 9     //BUZZER_PIN connected on pin D3
#define FAN_CONTROL_PIN 5
#define FAN_SPEED_PIN A0
#define TEMPERATURE_1_PIN A1
#define TEMPERATURE_2_PIN A2

//////////////////////////////////////////////////////////////////////////////////////





unsigned int Delay = 300;                    //This is the LCD refresh rate. Each 300ms.
unsigned long previousMillis = 0;   //Variables used for LCD refresh loop
unsigned long currentMillis = 0;    //Variables used for LCD refresh loop
int Rotary_counter = 0;             //Variable used to store the encoder position
int Rotary_counter_prev = 0;        //Variable used to store the previous value of encoder
bool clk_State;                     //State of the CLK pin from encoder (HIGH or LOW)
bool Last_State;                    //Last state of CLK pin from encoder (HIGH or LOW)
bool dt_State;                      //State of the DT pin from encoder (HIGH or LOW)
int Menu_level = 1;                 //Menu is strucured by levels
int Menu_row = 1;                   //Each level could have different rows
int push_count_ON = 0;              //Variable sued as counter to detect when a button is really pushed (debaunce)
int push_count_OFF = 0;             //Variable sued as counter to detect when a button is NOT pushed (debaunce)
String space_string = "______";     //used to print a line on LCD
String space_string_mA = "____";    //used to print a line on LCD
String pause_string = "";           //used to print something on LCD
bool SW_STATUS = false;             //Store the status of the rotary encoder push button (pressed or not)
bool SW_red_status = false;         //Store the status of the pause/resume button (pressed or not)
bool pause = false;                 //store the status of pasue (enabeled or disabled)

//Variables for storing each decimal for current, resistance and power. 
byte Ohms_0 = 0;
byte Ohms_1 = 0;
byte Ohms_2 = 0;
byte Ohms_3 = 0;
byte Ohms_4 = 0;
byte Ohms_5 = 0;
byte Ohms_6 = 0;
byte mA_0 = 0;
byte mA_1 = 0;
byte mA_2 = 0;
byte mA_3 = 0;
byte mW_0 = 0;
byte mW_1 = 0;
byte mW_2 = 0;
byte mW_3 = 0;
byte mW_4 = 0;
byte mV_0 = 0;
byte mV_1 = 0;
byte mV_2 = 0;
byte mV_3 = 0;

//Variables for ADC readings
float ohm_setpoint = 0;
float mA_setpoint = 0;
float mW_setpoint = 0;
float mV_setpoint = 0;
int dac_value = 0;

/////////////////////////////////////////////////////////////IMPORTANT//////////////////////////////////////////////////////////////////
/*This part is important. You see, when you sue the ADS1115, to pass from bit values (0 to 65000), we use a multiplier
  By default that is "0.185mv" or "0.000185V". In the code, to measure current, we make a differential measurement of the voltage
  on the "1ohm" load. Since the load is 1ohm, that will give us DIRECTLY the current value since "I = V/R" and R is 1.
  BUT!!! The resistor is not exactly 1ohm, so in my case I've adapted the multiplier to 0.0001827. You might need to adjust this
  variable to other values till you get good readings, so while measuring the value with an external multimeter at the same time,
  adjust this variable till you get good results. */
const float multiplier = 0.000185817;     //Multiplier used for "current" read between ADC0 and ADC1 of the ADS1115
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*The same goes here. But in this case, the voltage read is from a voltage divider. You see, the ADS1115 can only measure up to 5V. 
  If the input is higer it will get damaged. So, for that btween the ADS1115 and the main input I've used a 10K and 100K divider and 
  that will equal to a divider of 0.0909090. So, now the multiplier is 0.000185 / 0.0909090 = 0.002035
  Now these resistor values are not perfect neighter so we don't have exactly 10K and 100K, that's why my multiplier for voltage read
  is 0.0020645. Just do the same, measure the voltage on the LCD screen and also with an external multimiter and adjust this value till you get 
  good results. I've measure the resistors but that's not enough. We need precise values. */
const float multiplier_A2 = 0.002052026;   //Multiplier for voltage read from the 10K/100K divider
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





void setup() {
  Wire.begin();
  Wire.setClock(400000L);

  lcd.init();                 //Start i2c communication with the LCD
  lcd.backlight();            //Activate backlight
  
  lcd.createChar(0, arrow);   //create the arrow character
  lcd.createChar(1, ohm);     //create the ohm character
  lcd.createChar(2, up);      //create the up arrow character
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("  ELECTRONOOBS  "); 
  tone(BUZZER_PIN, 500, 100);
  delay(100);
  tone(BUZZER_PIN, 700, 100);
  delay(100);
  tone(BUZZER_PIN, 1200, 100);
  delay(300);
  lcd.setCursor(0,1);
  lcd.print("ELECTRONIC  LOAD");  
  delay(1000);
  
  PCICR |= (1 << PCIE2);      //enable PCMSK0 scan
  //PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
  PCMSK2 |= (1 << PCINT18);    //Pin 2 (DT) interrupt. Set pin D9 to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT19);    //Pin 3 (CLK) interrupt. Set pin D10 to trigger an interrupt on state change.
  pinMode(ENCODER_DT_PIN, INPUT);
  pinMode(ENCODER_CLK_PIN, INPUT);
  pinMode(BUZZER_PIN,OUTPUT);     //BUZZER_PIN pin set as OUTPUT
  digitalWrite(BUZZER_PIN, LOW);  //BUZZER_PIN turned OFF
  pinMode(ENCODER_BTN_PIN,INPUT);       //Encoder button set as input with pullup
  pinMode(MENU_BTN_PIN, INPUT);  //Menu button set as input with pullup
  pinMode(STOP_BTN_PIN, INPUT);   //Stop/resume button set as input with pullup
  pinMode(FAN_CONTROL_PIN, OUTPUT);
  digitalWrite(FAN_CONTROL_PIN, HIGH);
  pinMode(FAN_SPEED_PIN, INPUT);
  pinMode(TEMPERATURE_1_PIN, INPUT);
  pinMode(TEMPERATURE_2_PIN, INPUT);
  delay(10);

  
  ads.begin();      //Start i2c communication with the ADC
  ads.startComparator_SingleEnded(2, ADS1X15_REG_CONFIG_MODE_CONTIN);
  ads.startComparator_SingleEnded(3, ADS1X15_REG_CONFIG_MODE_CONTIN);
  delay(10);

  dac.begin(0x60);  //Start i2c communication with the DAC (slave address sometimes can be 0x60, 0x61 or 0x62)
  delay(10);
  dac.setVoltage(0, false); //Set DAC voltage output ot 0V (MOSFET turned off)
  delay(10);
   
  previousMillis = millis();

}

void loop() {
  if(!digitalRead(STOP_BTN_PIN) && !SW_red_status){
    push_count_OFF+=1;
    if(push_count_OFF > 10){  
      tone(BUZZER_PIN, 1000, 300);          
      pause = !pause;
      SW_red_status = true;
      push_count_OFF=0;
    }   
  }
  if(digitalRead(STOP_BTN_PIN) && SW_red_status){
    SW_red_status = false;
  }

  

  
  if(Menu_level == 1)
  {
    if(!digitalRead(ENCODER_BTN_PIN) && !SW_STATUS)    {//button enter press
      
      Rotary_counter = 0;
      tone(BUZZER_PIN, 500, 20);
      if(Menu_row == 1){//load
        Menu_level = 2;
        Menu_row = 1;
      }
      else if(Menu_row == 2){//current
        Menu_level = 3;
        Menu_row = 1;
      }
      else if(Menu_row == 3){//power
        Menu_level = 4;
        Menu_row = 1;
      }
      else if(Menu_row == 4){ //volt
        Menu_level = 8;
        Menu_row = 1;
      }
      
      SW_STATUS = true;
    }

    if(digitalRead(ENCODER_BTN_PIN) && SW_STATUS)
    {      
      SW_STATUS = false;
    }


    
    
    if (Rotary_counter <= 4)
    {
      Menu_row = 1;
    }
    else if (Rotary_counter > 4 && Rotary_counter <= 8)
    {
      Menu_row = 2;
    }
    else if (Rotary_counter > 8 && Rotary_counter <= 12)
    {
      Menu_row = 3;
    }
    else if (Rotary_counter > 12 && Rotary_counter <= 16)//krutim
    {
      Menu_row = 4;
    }

    if(Rotary_counter < 0)
    {
      Rotary_counter = 0;
    }
    if(Rotary_counter > 16)
    {
      Rotary_counter = 16;
    }
    
    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      if(Menu_row == 1)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.write(0); 
        lcd.print(" Load");
        lcd.setCursor(0,1);
        lcd.print("  Current"); 
      }
    
      else if(Menu_row == 2)
      {
        lcd.clear();
        lcd.setCursor(0,0);     
        lcd.print("  Load");
        lcd.setCursor(0,1);
        lcd.write(0);
        lcd.print(" Current"); 
      }
    
      else if(Menu_row == 3)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.write(0); 
        lcd.print(" Power");
        lcd.setCursor(0,1);
        lcd.print("  Voltage");
      }

      else if(Menu_row == 4)
      {
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print("  Power");
        lcd.setCursor(0,1);
        lcd.write(0);
        lcd.print(" Voltage");
      }
    }
  }


  if(Menu_level == 2)//setup Om
  {
    if(Rotary_counter < 0)
    {
      Rotary_counter = 0;
    }
    if(Rotary_counter > 9)
    {
      Rotary_counter = 9;
    }
    if(!digitalRead(ENCODER_BTN_PIN) && !SW_STATUS)
    {
      tone(BUZZER_PIN, 500, 20);
      push_count_ON = push_count_ON + 1;
      push_count_OFF = 0;
      if(push_count_ON > 20)
      {
        Menu_row = Menu_row + 1;
        if(Menu_row > 7)
        {
          Menu_level = 5;
          pause = false;
          ohm_setpoint = Ohms_0*1000000 + Ohms_1*100000 + Ohms_2*10000 + Ohms_3*1000 + Ohms_4*100 + Ohms_5*10 + Ohms_6; 
          
        }
        Rotary_counter = 0;
        SW_STATUS = true;
        space_string = space_string + "_";
        push_count_ON = 0;
      }      
    }

    if(digitalRead(ENCODER_BTN_PIN) && SW_STATUS)
    {      
      push_count_ON = 0; 
      push_count_OFF = push_count_OFF + 1; 
      if(push_count_OFF > 20){
        SW_STATUS = false;
        push_count_OFF = 0;
      }
        
    }
    

    if(Menu_row == 1)
    {
      Ohms_0 = Rotary_counter;      
    }
    if(Menu_row == 2)
    {
      Ohms_1 = Rotary_counter;      
    }
    if(Menu_row == 3)
    {
      Ohms_2 = Rotary_counter;
    }
    if(Menu_row == 4)
    {
      Ohms_3 = Rotary_counter;     
    }
    if(Menu_row == 5)
    {
      Ohms_4 = Rotary_counter;      
    }
    if(Menu_row == 6)
    {
      Ohms_5 = Rotary_counter;      
    }
    if(Menu_row == 7)
    {
      Ohms_6 = Rotary_counter;      
    }
    
    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("Ohms: ");
      lcd.print(Ohms_0);
      lcd.print(Ohms_1);
      lcd.print(Ohms_2);
      lcd.print(Ohms_3);
      lcd.print(Ohms_4);
      lcd.print(Ohms_5);
      lcd.print(Ohms_6);
      lcd.setCursor(0,1);    
      lcd.print(space_string);
      lcd.write(2);
    }

    if(!digitalRead(MENU_BTN_PIN)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      dac.setVoltage(0, false);
      previousMillis = millis();
      SW_STATUS = true;
      space_string = "______";    
      ohm_setpoint = 0;  
      Ohms_1 = 0;
      Ohms_2 = 0;
      Ohms_3 = 0;
      Ohms_4 = 0;
      Ohms_5 = 0;
      Ohms_6 = 0;
    }
    if(!digitalRead(MENU_BTN_PIN)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      dac.setVoltage(0, false);
      previousMillis = millis();
      SW_STATUS = true;
      space_string = "______";    
      ohm_setpoint = 0;  
      Ohms_1 = 0;
      Ohms_2 = 0;
      Ohms_3 = 0;
      Ohms_4 = 0;
      Ohms_5 = 0;
      Ohms_6 = 0;
    }
  }







  if(Menu_level == 3)//setup Amps
  {
    if(Rotary_counter < 0)
    {
      Rotary_counter = 0;
    }
    if(Rotary_counter > 9)
    {
      Rotary_counter = 9;
    }
    
    if(!digitalRead(ENCODER_BTN_PIN) && !SW_STATUS)
    {
      tone(BUZZER_PIN, 500, 20);
      push_count_ON = push_count_ON + 1;
      push_count_OFF = 0;
      if(push_count_ON > 20)
      {
        Menu_row = Menu_row + 1;
        if(Menu_row > 4)
        {
          Menu_level = 6;
          pause = false;
          mA_setpoint = mA_0*1000 + mA_1*100 + mA_2*10 + mA_3; 
          
        }
        Rotary_counter = 0;
        SW_STATUS = true;
        space_string_mA = space_string_mA + "_";
        push_count_ON = 0;
      }      
    }

    if(digitalRead(ENCODER_BTN_PIN) && SW_STATUS)
    {      
      push_count_ON = 0; 
      push_count_OFF = push_count_OFF + 1; 
      if(push_count_OFF > 20){
        SW_STATUS = false;
        push_count_OFF = 0;
      }
        
    }
    

    if(Menu_row == 1)
    {
      mA_0 = Rotary_counter;      
    }
    if(Menu_row == 2)
    {
      mA_1 = Rotary_counter;      
    }
    if(Menu_row == 3)
    {
      mA_2 = Rotary_counter;
    }
    if(Menu_row == 4)
    {
      mA_3 = Rotary_counter;     
    }
    
    
    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("mA: ");
      lcd.print(mA_0);
      lcd.print(mA_1);
      lcd.print(mA_2);
      lcd.print(mA_3);     
      lcd.setCursor(0,1);    
      lcd.print(space_string_mA);
      lcd.write(2);
    }
    if(!digitalRead(MENU_BTN_PIN)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      dac.setVoltage(0, false);
      previousMillis = millis();
      SW_STATUS = true;
      space_string_mA = "____";  
      mA_setpoint = 0;   
      mA_0 = 0;
      mA_1 = 0;
      mA_2 = 0;      
    }     
  }










  

  if(Menu_level == 4)//setup W power
  {
    if(Rotary_counter < 0)
    {
      Rotary_counter = 0;
    }
    if(Rotary_counter > 9)
    {
      Rotary_counter = 9;
    }
    
    if(!digitalRead(ENCODER_BTN_PIN) && !SW_STATUS)
    {
      tone(BUZZER_PIN, 500, 20);
      push_count_ON = push_count_ON + 1;
      push_count_OFF = 0;
      if(push_count_ON > 20)
      {
        Menu_row = Menu_row + 1;
        if(Menu_row > 5)
        {
          Menu_level = 7;
          pause = false;
          mW_setpoint = mW_0*10000 + mW_1*1000 + mW_2*100 + mW_3*10 + mW_4; 
          
        }
        Rotary_counter = 0;
        SW_STATUS = true;
        space_string_mA = space_string_mA + "_";
        push_count_ON = 0;
      }      
    }

    if(digitalRead(ENCODER_BTN_PIN) && SW_STATUS)
    {      
      push_count_ON = 0; 
      push_count_OFF = push_count_OFF + 1; 
      if(push_count_OFF > 20){
        SW_STATUS = false;
        push_count_OFF = 0;
      }
        
    }
    

    if(Menu_row == 1)
    {
      mW_0 = Rotary_counter;      
    }
    if(Menu_row == 2)
    {
      mW_1 = Rotary_counter;      
    }
    if(Menu_row == 3)
    {
      mW_2 = Rotary_counter;
    }
    if(Menu_row == 4)
    {
      mW_3 = Rotary_counter;
    }
    if(Menu_row == 5)
    {
      mW_4 = Rotary_counter;
    }
    
    
    
    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("mW: ");
      lcd.print(mW_0);
      lcd.print(mW_1);
      lcd.print(mW_2);
      lcd.print(mW_3); 
      lcd.print(mW_4);            
      lcd.setCursor(0,1);    
      lcd.print(space_string_mA);
      lcd.write(2);
    }
    if(!digitalRead(MENU_BTN_PIN)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      dac.setVoltage(0, false);
      previousMillis = millis();
      SW_STATUS = true;
      space_string_mA = "____";
      mW_setpoint = 0;
      mW_0 = 0;
      mW_1 = 0;
      mW_2 = 0;  
      mW_3 = 0; 
      mW_4 = 0;     
    }
  }





  //Constant Load Mode
  if(Menu_level == 5)
  {
    if(Rotary_counter > Rotary_counter_prev)
    {
      ohm_setpoint = ohm_setpoint + 1;
      Rotary_counter_prev = Rotary_counter;
      
    }

    if(Rotary_counter < Rotary_counter_prev)
    {
      ohm_setpoint = ohm_setpoint - 1;
      Rotary_counter_prev = Rotary_counter;
      
    }
    
    float voltage_on_load, voltage_read, power_read;//sensosed_voltage,  
    voltage_on_load = ads.readADC_Differential_0_1() * -1;      //Read DIFFERENTIAL voltage between ADC0 and ADC1. (the load is 1ohm, so this is equal to the current)
    voltage_on_load = (voltage_on_load * multiplier)*1000;

    voltage_read = ads.readADC_SingleEnded(2);
    voltage_read = (voltage_read * multiplier_A2);
    
    //sensosed_voltage = ads.readADC_SingleEnded(3);
    //sensosed_voltage = (sensosed_voltage * multiplier);
    
    power_read = voltage_on_load * voltage_read;

    float setpoint_current = (voltage_read / ohm_setpoint) * 1000;

    float error = abs(setpoint_current - voltage_on_load);
    
    if (error > (setpoint_current*0.8))
    {
      if(setpoint_current > voltage_on_load){
        dac_value = dac_value + 300;
      }

      if(setpoint_current < voltage_on_load){
        dac_value = dac_value - 300;
      }
    }

    else if (error > (setpoint_current*0.6))
    {
      if(setpoint_current > voltage_on_load){
        dac_value = dac_value + 170;
      }

      if(setpoint_current < voltage_on_load){
        dac_value = dac_value - 170;
      }
    }

    else if (error > (setpoint_current*0.4))
    {
      if(setpoint_current > voltage_on_load){
        dac_value = dac_value + 120;
      }

      if(setpoint_current < voltage_on_load){
        dac_value = dac_value - 120;
      }
    }
    else if (error > (setpoint_current*0.3))
    {
      if(setpoint_current > voltage_on_load){
        dac_value = dac_value + 60;
      }

      if(setpoint_current < voltage_on_load){
        dac_value = dac_value - 60;
      }
    }
    else if (error > (setpoint_current*0.2))
    {
      if(setpoint_current > voltage_on_load){
        dac_value = dac_value + 40;
      }

      if(setpoint_current < voltage_on_load){
        dac_value = dac_value - 40;
      }
    }
    else if (error > (setpoint_current*0.1))
    {
      if(setpoint_current > voltage_on_load){
        dac_value = dac_value + 30;
      }

      if(setpoint_current < voltage_on_load){
        dac_value = dac_value - 30;
      }
    }
    else
    {
      if(setpoint_current > voltage_on_load){
        dac_value = dac_value + 1;
      }

      if(setpoint_current < voltage_on_load){
        dac_value = dac_value - 1;
      }
    }
    
    
    
    if(dac_value > 4095)
    {
      dac_value = 4095;
    }
    
  
    
    if(!pause){
      dac.setVoltage(dac_value, false);
      pause_string = "";
    }
    else{
      dac.setVoltage(0, false);
      dac_value = 0;
      pause_string = " PAUSE";
    }
    
    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print(ohm_setpoint,0); lcd.write(1); lcd.print(" "); lcd.print(voltage_read,3); lcd.print("V");
      lcd.setCursor(0,1);    
      lcd.print(voltage_on_load,0);  lcd.print("mA"); lcd.print(" "); lcd.print(power_read,0);  lcd.print("mW"); 
      lcd.print(pause_string);
    }
    if(!digitalRead(MENU_BTN_PIN)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      dac.setVoltage(0, false);
      previousMillis = millis();
      SW_STATUS = true;
      space_string = "______";    
      ohm_setpoint = 0;  
      Ohms_1 = 0;
      Ohms_2 = 0;
      Ohms_3 = 0;
      Ohms_4 = 0;
      Ohms_5 = 0;
      Ohms_6 = 0;
    }
  }








  //Constant Current Mode
  if(Menu_level == 6)
  {
    if(Rotary_counter > Rotary_counter_prev)
    {
      mA_setpoint = mA_setpoint + 1;
      Rotary_counter_prev = Rotary_counter;
    }

    if(Rotary_counter < Rotary_counter_prev)
    {
      mA_setpoint = mA_setpoint - 1;
      Rotary_counter_prev = Rotary_counter;
    }
    
    float voltage_on_load, voltage_read, power_read;// sensosed_voltage,
      
    voltage_on_load = ads.readADC_Differential_0_1() * -1;      //Read DIFFERENTIAL voltage between ADC0 and ADC1
    voltage_on_load = (voltage_on_load * multiplier)*1000;

    voltage_read = ads.readADC_SingleEnded(2);
    voltage_read = (voltage_read * multiplier_A2);
    
    //sensosed_voltage = ads.readADC_SingleEnded(3);
    //sensosed_voltage = (sensosed_voltage * multiplier);
    
    power_read = voltage_on_load * voltage_read;

    float error = abs(mA_setpoint - voltage_on_load);
    
    if (error > (mA_setpoint*0.8))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 300;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 300;
      }
    }

    else if (error > (mA_setpoint*0.6))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 170;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 170;
      }
    }

    else if (error > (mA_setpoint*0.4))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 120;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 120;
      }
    }
    else if (error > (mA_setpoint*0.3))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 60;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 60;
      }
    }

    else if (error > (mA_setpoint*0.2))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 40;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 40;
      }
    }
    
    else if (error > (mA_setpoint*0.1))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 30;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 30;
      }
    }
    else
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 1;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 1;
      }
    }
    
    
    
    if(dac_value > 4095)
    {
      dac_value = 4095;
    }
    
  
    if(!pause){
      dac.setVoltage(dac_value, false);
      pause_string = "";
    }
    else{
      dac.setVoltage(0, false);
      dac_value = 0;
      pause_string = " PAUSE";
    }
   
    

    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print(mA_setpoint,0); lcd.print("mA "); lcd.print(voltage_read); lcd.print("V");
      lcd.setCursor(0,1);    
      lcd.print(voltage_on_load,0);  lcd.print("mA"); lcd.print(" "); lcd.print(power_read,0);  lcd.print("mW"); 
      lcd.print(pause_string);
    }
    if(!digitalRead(MENU_BTN_PIN)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      dac.setVoltage(0, false);
      previousMillis = millis();
      SW_STATUS = true;
      space_string_mA = "____";  
      mA_setpoint = 0;   
      mA_0 = 0;
      mA_1 = 0;
      mA_2 = 0;      
    }      
  }










  //Constant Power Mode
  if(Menu_level == 7)
  {
    if(Rotary_counter > Rotary_counter_prev)
    {
      mW_setpoint = mW_setpoint + 1;
      Rotary_counter_prev = Rotary_counter;
    }

    if(Rotary_counter < Rotary_counter_prev)
    {
      mW_setpoint = mW_setpoint - 1;
      Rotary_counter_prev = Rotary_counter;
    }
    
    float voltage_on_load, voltage_read, power_read;//sensosed_voltage, 
      
    voltage_on_load = ads.readADC_Differential_0_1() * -1;      //Read DIFFERENTIAL voltage between ADC0 and ADC1
    voltage_on_load = (voltage_on_load * multiplier)*1000;

    voltage_read = ads.readADC_SingleEnded(2);
    voltage_read = (voltage_read * multiplier_A2);
    
    //sensosed_voltage = ads.readADC_SingleEnded(3);
    //sensosed_voltage = (sensosed_voltage * multiplier);
    
    power_read = voltage_on_load * voltage_read;




    float error = abs(mW_setpoint - power_read);    
    if (error > (mW_setpoint*0.8))
    {
      if(mW_setpoint > power_read){
        dac_value = dac_value + 300;
      }

      if(mW_setpoint < power_read){
        dac_value = dac_value - 300;
      }
    }

    else if (error > (mW_setpoint*0.6))
    {
      if(mW_setpoint > power_read){
        dac_value = dac_value + 170;
      }

      if(mW_setpoint < power_read){
        dac_value = dac_value - 170;
      }
    }

    else if (error > (mW_setpoint*0.4))
    {
      if(mW_setpoint > power_read){
        dac_value = dac_value + 120;
      }

      if(mW_setpoint < power_read){
        dac_value = dac_value - 120;
      }
    }
    else if (error > (mW_setpoint*0.3))
    {
      if(mW_setpoint > power_read){
        dac_value = dac_value + 60;
      }

      if(mW_setpoint < power_read){
        dac_value = dac_value - 60;
      }
    }
    else if (error > (mW_setpoint*0.2))
    {
      if(mW_setpoint > power_read){
        dac_value = dac_value + 40;
      }

      if(mW_setpoint < power_read){
        dac_value = dac_value - 40;
      }
    }
    else if (error > (mW_setpoint*0.1))
    {
      if(mW_setpoint > power_read){
        dac_value = dac_value + 30;
      }

      if(mW_setpoint < power_read){
        dac_value = dac_value - 30;
      }
    }
    else
    {
      if(mW_setpoint > power_read){
        dac_value = dac_value + 1;
      }

      if(mW_setpoint < power_read){
        dac_value = dac_value - 1;
      }
    }
    
    
    
    if(dac_value > 4095)
    {
      dac_value = 4095;
    }
    
  
    
    
    if(!pause){
      dac.setVoltage(dac_value, false);
      pause_string = "";
    }
    else{
      dac.setVoltage(0, false);
      dac_value = 0;
      pause_string = " PAUSE";
    }
    

    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print(mW_setpoint,0); lcd.print("mW "); lcd.print(voltage_read); lcd.print("V");
      lcd.setCursor(0,1);    
      lcd.print(power_read,0);  lcd.print("mW"); lcd.print(" "); lcd.print(voltage_on_load,0);  lcd.print("mA"); 
      lcd.print(pause_string);
    }
    if(!digitalRead(MENU_BTN_PIN)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      dac.setVoltage(0, false);
      previousMillis = millis();
      SW_STATUS = true;
      space_string_mA = "____";
      mW_setpoint = 0;
      mW_0 = 0;
      mW_1 = 0;
      mW_2 = 0;  
      mW_3 = 0; 
      mW_4 = 0;     
    }      
  }


if(Menu_level == 8)//setup Volts
  {
    if(Rotary_counter < 0)
    {
      Rotary_counter = 0;
    }
    if(Rotary_counter > 9)
    {
      Rotary_counter = 9;
    }
    
    if(!digitalRead(ENCODER_BTN_PIN) && !SW_STATUS)
    {
      tone(BUZZER_PIN, 500, 20);
      push_count_ON = push_count_ON + 1;
      push_count_OFF = 0;
      if(push_count_ON > 20)
      {
        Menu_row = Menu_row + 1;
        if(Menu_row > 4)
        {
          Menu_level = 9;
          pause = false;
          mV_setpoint = mV_0*1000 + mV_1*100 + mV_2*10 + mV_3; 
          
        }
        Rotary_counter = 0;
        SW_STATUS = true;
        space_string_mA = space_string_mA + "_";
        push_count_ON = 0;
      }      
    }

    if(digitalRead(ENCODER_BTN_PIN) && SW_STATUS)
    {      
      push_count_ON = 0; 
      push_count_OFF = push_count_OFF + 1; 
      if(push_count_OFF > 20){
        SW_STATUS = false;
        push_count_OFF = 0;
      }
        
    }
    

    if(Menu_row == 1)
    {
      mV_0 = Rotary_counter;      
    }
    if(Menu_row == 2)
    {
      mV_1 = Rotary_counter;      
    }
    if(Menu_row == 3)
    {
      mV_2 = Rotary_counter;
    }
    if(Menu_row == 4)
    {
      mV_3 = Rotary_counter;     
    }
    
    
    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("mV: ");
      lcd.print(mV_0);
      lcd.print(mV_1);
      lcd.print(mV_2);
      lcd.print(mV_3);     
      lcd.setCursor(0,1);    
      lcd.print(space_string_mA);
      lcd.write(2);
    }
    if(!digitalRead(MENU_BTN_PIN)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      dac.setVoltage(0, false);
      previousMillis = millis();
      SW_STATUS = true;
      space_string_mA = "____";  
      mV_setpoint = 0;   
      mV_0 = 0;
      mV_1 = 0;
      mV_2 = 0;      
    }     
  }


//Constant Volt Mode
  if(Menu_level == 9)
  {
    if(Rotary_counter > Rotary_counter_prev)
    {
      mV_setpoint = mV_setpoint + 10;
      Rotary_counter_prev = Rotary_counter;
    }

    if(Rotary_counter < Rotary_counter_prev)
    {
      mV_setpoint = mV_setpoint - 10;
      Rotary_counter_prev = Rotary_counter;
    }
    
    float voltage_on_load, voltage_read;//, power_read;//sensosed_voltage, 
      
    voltage_on_load = ads.readADC_Differential_0_1() * -1;      //Read DIFFERENTIAL voltage between ADC0 and ADC1
    voltage_on_load = (voltage_on_load * multiplier)*1000;

    voltage_read = ads.readADC_SingleEnded(2);//volts real
    voltage_read = (voltage_read * multiplier_A2);
    
    //sensosed_voltage = ads.readADC_SingleEnded(3);
    //sensosed_voltage = (sensosed_voltage * multiplier);
    float current_voltage = voltage_read * 100;

    float error = abs(mV_setpoint - current_voltage);    
    if (error > (mV_setpoint*0.8))
    {
      if(mV_setpoint > current_voltage){
        dac_value = dac_value - 300;
      }

      if(mV_setpoint < current_voltage){
        dac_value = dac_value + 300;
      }
    }
    else if (error > (mV_setpoint*0.6))
    {
      if(mV_setpoint > current_voltage){
        dac_value = dac_value - 170;
      }

      if(mV_setpoint < current_voltage){
        dac_value = dac_value + 170;
      }
    }
    else if (error > (mV_setpoint*0.4))
    {
      if(mV_setpoint > current_voltage){
        dac_value = dac_value - 120;
      }

      if(mV_setpoint < current_voltage){
        dac_value = dac_value + 120;
      }
    }
    else if (error > (mV_setpoint*0.3))
    {
      if(mV_setpoint > current_voltage){
        dac_value = dac_value - 60;
      }

      if(mV_setpoint < current_voltage){
        dac_value = dac_value + 60;
      }
    }
    else if (error > (mV_setpoint*0.2))
    {
      if(mV_setpoint > current_voltage){
        dac_value = dac_value - 40;
      }

      if(mV_setpoint < current_voltage){
        dac_value = dac_value + 40;
      }
    }
    else if (error > (mV_setpoint*0.1))
    {
      if(mV_setpoint > current_voltage){
        dac_value = dac_value - 30;
      }

      if(mV_setpoint < current_voltage){
        dac_value = dac_value + 30;
      }
    }
    else
    {
      if(mV_setpoint > current_voltage){
        dac_value = dac_value - 1;
      }

      if(mV_setpoint < current_voltage){
        dac_value = dac_value + 1;
      }
    }
    
    
    
    if(dac_value > 4095)
    {
      dac_value = 4095;
    }
    
  
    
    
    if(!pause){
      dac.setVoltage(dac_value, false);
      pause_string = "";
    }
    else{
      dac.setVoltage(0, false);
      dac_value = 0;
      pause_string = " PAUSE";
    }
    

    currentMillis = millis();
    if(currentMillis - previousMillis >= Delay){
      previousMillis += Delay;
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print(mV_setpoint,0); lcd.print("mV "); lcd.print(voltage_read); lcd.print("V");
      lcd.setCursor(0,1);    
      lcd.print(voltage_read*voltage_on_load,0);  lcd.print("mW"); lcd.print(" "); lcd.print(voltage_on_load,0);  lcd.print("mA"); 
      lcd.print(pause_string);
    }
    if(!digitalRead(MENU_BTN_PIN)){
      Menu_level = 1;
      Menu_row = 1;
      Rotary_counter = 0;
      Rotary_counter_prev = 0;
      dac.setVoltage(0, false);
      previousMillis = millis();
      SW_STATUS = true;
      space_string_mA = "____";
      mV_setpoint = 0;
      mV_0 = 0;
      mV_1 = 0;
      mV_2 = 0;  
      mV_3 = 0; 
    }      
  }










}//end void loop





ISR(PCINT2_vect){  
cli(); //stop interrupts happening before we read pin values
dt_State  =   (PIND & B00000100); //PD2=DT state
clk_State =   (PIND & B00001000); //PD3=CLK state 
if (clk_State != Last_State){
  // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
  if (dt_State != clk_State){ 
    Rotary_counter ++;    
    // tone(BUZZER_PIN, 700, 5);
    Last_State = clk_State; // Updates the previous state of the outputA with the current state
    sei(); //restart interrupts
  }
  else {
    Rotary_counter --;  
    // tone(BUZZER_PIN, 700, 5); 
    Last_State = clk_State; // Updates the previous state of the outputA with the current state    
    sei(); //restart interrupts
  } 
 }  
}
