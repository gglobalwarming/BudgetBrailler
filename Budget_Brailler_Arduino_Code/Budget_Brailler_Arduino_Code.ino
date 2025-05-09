//BudgetBrailler V1.0 Arduino Code
//By Jost Strnat
#include <Stepper.h>

//set up stepper motors
const int stepsPerRevolution = 200;//number of steps per turn of stepper motor

const int RMotorAin1 = 35;//pin leanding to RMotor driver Ain1 pin
const int RMotorAin2 = 37;//pin leanding to RMotor driver Ain2 pin
const int RMotorBin1 = 18;//pin leanding to RMotor driver Bin1 pin
const int RMotorBin2 = 16;//pin leanding to RMotor driver Bin2 pin
const int RMotorPWMA = 39;//pin leanding to RMotor driver PWMA pin, set analog write to 4095 if not using
const int RMotorPWMB = 15;//pin leanding to RMotor driver PWMB pin, set analog write to 4095 if not using
const int RMotorSTBY = 33;//pin leanding to RMotor driver standby pin. Pull low to turn off motors

const int XMotorAin1 = 34;//pin leanding to XMotor driver Ain1 pin
const int XMotorAin2 = 21;//pin leanding to XMotor driver Ain2 pin
const int XMotorBin1 = 38;//pin leanding to XMotor driver Bin1 pin
const int XMotorBin2 = 40;//pin leanding to XMotor driver Bin2 pin
const int XMotorPWMA = 17;//pin leanding to XMotor driver PWMA pin, set to 4095 if not using
const int XMotorPWMB = 1;//pin leanding to XMotor driver PWMB pin, set to 4095 if not using
const int XMotorSTBY = 36;//pin leanding to XMotor driver standby pin. Pull low to turn off motors

const int XMotorSpeed = 120;//speed in RPM for XMotor
const int RMotorSpeed = 120;//speed in RPM for RMotor

Stepper RMotor(stepsPerRevolution, RMotorAin1, RMotorAin2, RMotorBin1, RMotorBin2);//stepper motor to move toolhead on rail
Stepper XMotor(stepsPerRevolution, XMotorAin1, XMotorAin2, XMotorBin1, XMotorBin2);//stepper motor for rollers to move paper
// Stepper RMotor(stepsPerRevolution, 9, 11, 13, 10);//stepper motor to move toolhead on rail
// Stepper XMotor(stepsPerRevolution, 36, 38, 2, 4);//stepper motor for rollers to move paper

const int laPin = 13;//output pin to transistor controlling linear actuator (LA)
const int pbPin = 14;//output pin to piezo buzzer (PB)
const int esPin = 12;//input pin from x endstop (ES)

  //will need to switch these pinouts in future versions if using wifi as that uses ADC2
const int keyPin1 = 5;//input key from buttons 1-3
const int keyPin2 = 3;//input key from buttons 4-6
const int keyPin3 = 7;//input key from buttons 7
const int keyPin4 = 2;//input key from buttons 8-10
const int keyPin5 = 9;//input key from buttons 11-13

//variables
int keyVal1;//stores analog read of buttons 1-3
int keyVal2;//stores analog read of buttons 4-6
int keyVal3;//stores analog read of buttons 7 (space)
int keyVal4;//stores analog read of buttons 8-10
int keyVal5;//stores analog read of buttons 11-13
int keyVal1High;//stores highest analog read of buttons 1-3
int keyVal2High;//stores highest analog read of buttons 4-6
int keyVal3High;//stores highest analog read of buttons 7 (space)
int keyVal4High;//stores highest analog read of buttons 8-10
int keyVal5High;//stores highest analog read of buttons 11-13
//for converting digital KeyVal reads to which button is pressed, 0 if button not pressed, 1 if button pressed

bool ButtonBuffer[5];
int LineNumber = 0;//current line number
int CharNumber = 0;//current character number on current line
int charStartNum = 0;//used to track last printed character if less than a whole line is printed
const int CharPerLine = 27;//max characters per line
const int NumberofLines = 50;//place holder for now
bool LineBuffer[(CharPerLine+1)*6];//buffer of button presses corresponding to each character in a line
int XSteps = 0;//current position of toolhead in steps
int RSteps = 0;//current position of paper in steps
const float StepsPerMMX = 5;//steps needed to move toolhead 1 mm in the x direction
const float StepsPerMMR = -4.17;//steps needed to move paper 1mm in the r direction  
const float DotSpace = 2.34;//mm between dots in a cell 2.34 standard
const float CellXSpace = 6.2;//mm between corresponding dots in adjacent cells in the same line 6.2 standard
const float CellRSpace = 10;//mm between corresponding dots in corresponding cells on different lines 10 std 
const int DotStepsR = round(-10);//number of steps the stepper motor should move in the r direction between first and second dots in a cell
const int DotStepsR2 = (DotStepsR-2);//number of steps the stepper motor should move in the r direction between second and third dots in a cell, different from DotStepsR due to backlacksh I think
const int DotStepsX = round(-12);//number of steps the stepper motor should move in the x direction between dots in a cell
const int CellStepsR = round(CellRSpace*StepsPerMMR);//number of steps the stepper motor should move in the roller direction between cells
const int CellStepsX = round((CellXSpace-DotSpace)*StepsPerMMR);//number of steps the stepper motor should move in the x direction between cells
const int pbHz = 440;//Hz of piezo buzzer sound for end of line
const int pbTime = 1000;//time in ms the buzzer will sound
const int motorDelayTime = 0;//a delay after each time the motor moves. Left in but at 0ms for troubleshooting purposes, could cause problems interrupting other processes down the line due to delay function
const int laDelayTime = 60;//delay between sending current to LA 
bool currentKeyState = 0;//Whether buttons any buttons were pressed (1) or none pressed (0) in the current voidLoop. Holds state until end of void loop until transfered to previousKeyState
bool previousKeyState = 0;//Whether buttons any buttons were pressed (1) or none pressed (0) in previous voidLoop. Used to track if keys have been held down, in which case it should not register as a second instance of typing a character
const int buttonPressDelay = 0;//used if you want to pause after reading a button. used for troubleshooting
unsigned long ButtonPressTimer = 0;//used to track how long a button has been pressed
const int ButtonPressTime = 1500;//time a button is held to start printing
const int HomeTimeoutTime = 1500;//timeout for home to prevent infinitely ramming the endstop if it fails
unsigned long HomeTimer = 0;//for starting millis timer to timeout home function
int BackspaceStop = 0;//last character that backspace can delete

//Resistor Ladder digital read Values
  const int rlB1l = 1000;//B1 only lower limit
  const int rlB1u = 2500;//B1 only upper limit
  const int rlB2l = 2501;//B2 only lower limit
  const int rlB2u = 3600;//B2 only upper limit
  const int rlB12l = 3601;//B12 only lower limit
  const int rlB12u = 4600;//B12 only upper limit
  const int rlB3l = 4601;//B3 only lower limit
  const int rlB3u = 5450;//B3 only upper limit
  const int rlB13l = 5451;//B13 only lower limit
  const int rlB13u = 6000;//B13 only upper limit
  const int rlB23l = 6001;//B23 only lower limit
  const int rlB23u = 6400;//B23 only upper limit
  const int rlB123l = 6401;//B123 only lower limit

void setup() {
//start serial communcation and print key value reads for troubleshooting
  Serial.begin(115200);
  delay(2000);
  Serial.println("BeginVoidSetup");

//pinModes
  pinMode(XMotorSTBY, OUTPUT);//X-motor standby pin
  pinMode(RMotorSTBY, OUTPUT);//R-motor standby pin
  pinMode(laPin, OUTPUT);//linear actuator activation pin
  pinMode(esPin, INPUT);//end stop pin
  pinMode(pbPin, OUTPUT);//end piezo buzzer
  // pinMode(keyPin1, INPUT);//
  // pinMode(keyPin2, INPUT);//
  // pinMode(keyPin3, INPUT);//
  // pinMode(keyPin4, INPUT);//
  // pinMode(keyPin5, INPUT);//


  tone(pbPin,pbHz,pbTime);//play a piezzo buzzer tone to indicate the brailler has turned on
  noTone(pbPin);//turn of piezo buzzer

//Check key values/////////////////make function?
  keyVal1 = analogRead(keyPin1);
  keyVal2 = analogRead(keyPin2);
  keyVal3 = analogRead(keyPin3);
  keyVal4 = analogRead(keyPin4);
  keyVal5 = analogRead(keyPin5);

//turn motors on, PWMs to high, set speed
  digitalWrite(XMotorSTBY,HIGH);
  digitalWrite(RMotorSTBY,HIGH);
  analogWrite(XMotorPWMA,4095);
  analogWrite(XMotorPWMB,4095);
  analogWrite(RMotorPWMA,4095);
  analogWrite(RMotorPWMB,4095);
  XMotor.setSpeed(XMotorSpeed);//set x motor speed in rpm
  RMotor.setSpeed(RMotorSpeed);//set r motor speed in rpm

//Feed paper in if button 8 is pressed
  while(keyVal3 <= rlB1l || keyVal3 >= rlB2l ){
    keyVal3=analogRead(keyPin3);
    keyVal4=analogRead(keyPin4);
    if (keyVal4 >=rlB2l &&keyVal4 <= rlB2u){//if lineup button move paper 10 steps in
      RMotor.step(-10);
    }
    if (keyVal4 >=rlB3l &&keyVal4 <= rlB3u){//if linedown button is pressed, move paper 10 steps in
      RMotor.step(10);
    }
    else{}
    Serial.println("Setup Read:");//serial print key values for troubleshooting
      Serial.println(analogRead(keyPin1));
      Serial.println(analogRead(keyPin2));
      Serial.println(analogRead(keyPin3));
      Serial.println(analogRead(keyPin4));
      Serial.println(analogRead(keyPin5));
      delay(500);
  }
  FunctionBacklashAdjust();
  delay(100);
  Serial.println("EndVoidSetup");
}

void loop() {
  Serial.println("BeginVoidLoop");  
  keyVal1 = analogRead(keyPin1);
  keyVal2 = analogRead(keyPin2);
  keyVal3 = analogRead(keyPin3);
  keyVal4 = analogRead(keyPin4);
  keyVal5 = analogRead(keyPin5);
  delay(500);
  //Serial print key reads for trouble shooting
    // Serial.println("VoidLoop");
    // Serial.println("Loop Read:");
    // Serial.println(analogRead(keyPin1));
    // Serial.println(analogRead(keyPin2));
    // Serial.println(analogRead(keyPin3));
    // Serial.println(analogRead(keyPin4));
    // Serial.println(analogRead(keyPin5));

  if (keyVal1 >= rlB1l || keyVal2 >= rlB1l || keyVal3 >=rlB1l || keyVal4 >=rlB1l || keyVal5 >=rlB1l){
    // Serial.println("Button Pressed");
    currentKeyState=1;
    if(keyVal1High <= keyVal1){
    keyVal1High = keyVal1;
    }
    if(keyVal2High <= keyVal2){
      keyVal2High = keyVal2;
    }
    if(keyVal3High <= keyVal3){
      keyVal3High = keyVal3;
    }
    if(keyVal4High <= keyVal4){
      keyVal4High = keyVal4;
    }
    if(keyVal5High <= keyVal5){
      keyVal5High = keyVal5;
    }
    if(previousKeyState == 0){
      Serial.println("New Button Pressed");
      ButtonPressTimer = millis();//unused timer of how long buttons pressed in case needed later
      delay(buttonPressDelay);
    }
    else{
        // Serial.println("Old Button Pressed");
      if(millis() >= ButtonPressTimer + ButtonPressTime){//left in in case of used later
      delay(0);
      }
    }
  }
  if (keyVal1 <= rlB1l && keyVal2 <= rlB1l && keyVal3 <= rlB1l && keyVal4 <= rlB1l && keyVal5 <= rlB1l){
    // Serial.println("No Button Pressed");
    currentKeyState = 0;
    if (previousKeyState == 1){
      if (keyVal4High >= rlB1l && keyVal4High <= rlB1u){//Home x axis
        FunctionHome();
      }
      if (keyVal4High >= rlB2l && keyVal4High <= rlB2u){//Move paper out by one line
        FunctionPreviousLine();
      }
      if (keyVal4High >= rlB3l && keyVal4High <= rlB3u){//move paper in by one line
        FunctionNextLine();
      }
      if (keyVal5High >= rlB1l && keyVal5High <= rlB1u){//print buffer
        FunctionPrintBuffer();
      }  
      if (keyVal5High >= rlB2l && keyVal5High <= rlB2u){//unused
        delay(0);
      }    
      if (keyVal5High >= rlB3l && keyVal5High <= rlB3u){//backspace
        FunctionBackspace();
      }  

      if (keyVal1High >= rlB1l || keyVal2High >= rlB1l || keyVal3High >=rlB1l){
      FunctionCheckKeys();
      FunctionAddToBuffer();
        if(CharNumber > CharPerLine){
          FunctionPrintBuffer();
          FunctionNextLine();
          FunctionHome();
        }
      }
      FunctionResetButtons();
    }
  }
  else{}
  // Serial.println("previousKeyState ");
  // Serial.println(previousKeyState);
  // Serial.println("currentKeyState ");
  // Serial.println(currentKeyState);
  // Serial.println();
  previousKeyState = currentKeyState;
  delay(0);
}

void FunctionPrint(){
    Serial.println("FunctionPrint");
  if(ButtonBuffer[2] == 1) {FunctionPunch();}
  delay(motorDelayTime);
  RMotor.step(DotStepsR2);
  delay(motorDelayTime);
  if(ButtonBuffer[1] == 1) {FunctionPunch();}
  delay(motorDelayTime);
  RMotor.step(DotStepsR);
  delay(motorDelayTime);
  if(ButtonBuffer[0] == 1) {FunctionPunch();}
  delay(motorDelayTime);
  RMotor.step(-DotStepsR-DotStepsR2);
  delay(motorDelayTime);
  XMotor.step(DotStepsX);
  delay(motorDelayTime);
  if(ButtonBuffer[3] == 1) {FunctionPunch();}
  delay(motorDelayTime);
  RMotor.step(DotStepsR2);
  delay(motorDelayTime);
  if(ButtonBuffer[4] == 1) {FunctionPunch();}
  delay(motorDelayTime);
  RMotor.step(DotStepsR);
  delay(motorDelayTime);
  if(ButtonBuffer[5] == 1) {FunctionPunch();}
  delay(motorDelayTime);
  RMotor.step(-DotStepsR - DotStepsR2);
  delay(motorDelayTime);
  XMotor.step(CellStepsX);
  FunctionResetButtons();
}

void FunctionPunch(){
  Serial.println("FunctionPunch");
  digitalWrite(laPin,HIGH);
  delay(laDelayTime);
  digitalWrite(laPin,LOW);
  delay(laDelayTime);
}

void FunctionNextLine(){
  Serial.println("FunctionNextLine");
  RMotor.step(CellStepsR);
  LineNumber++;
  FunctionBacklashAdjust();
}

void FunctionPreviousLine(){
  Serial.println("FunctionPreviousLine");
  RMotor.step(-CellStepsR);
  LineNumber--;
  FunctionBacklashAdjust();
}

void FunctionHome(){//add timeout
  Serial.println("FunctionHome");
  HomeTimer = millis();
  while(digitalRead(esPin) == 1){
  XMotor.step(1);
  if(millis() >= (HomeTimer + 1500)){
    break;
  }  
  }
  CharNumber = 0;
  charStartNum = CharNumber;  
  BackspaceStop = CharNumber;
  delay(100);
}

void FunctionBacklashAdjust(){//////////////////////////leave in for now commented out in case its needed
  // Serial.println("FunctionBacklashAdjust");
  // RMotor.step(DotStepsR);
  // delay(motorDelayTime);
  // RMotor.step(DotStepsR);
  // delay(motorDelayTime);
  // RMotor.step(-2*(DotStepsR));
  // delay(motorDelayTime);
}

void FunctionResetButtons(){
  // Serial.println("FunctionResetButtons");  
  ButtonBuffer[0] = 0;//reset buttons
  ButtonBuffer[1] = 0;
  ButtonBuffer[2] = 0;
  ButtonBuffer[3] = 0;
  ButtonBuffer[4] = 0;
  ButtonBuffer[5] = 0;
  keyVal1High = 0;
  keyVal2High = 0;
  keyVal3High = 0;
  keyVal4High = 0;
  keyVal5High = 0;
}

void FunctionCheckKeys(){
  Serial.println("FunctionCheckKeys");
  Serial.println("High Reads:");
  Serial.println(keyVal1High);
  Serial.println(keyVal2High);
  Serial.println(keyVal3High);
  Serial.println(keyVal4High);  
  Serial.println(keyVal5High);    
  if (keyVal1High >= rlB123l){//All 3 buttons pressed
    ButtonBuffer[0] = 1;
    ButtonBuffer[1] = 1;
    ButtonBuffer[2] = 1;}
  if (keyVal1High >= rlB12l && keyVal1High <= rlB12u){//Buttons 4+5 pressed
    ButtonBuffer[0] = 1;
    ButtonBuffer[1] = 1;
    ButtonBuffer[2] = 0;}
  if (keyVal1High >= rlB13l && keyVal1High <= rlB13u){//Buttons 4+6 pressed
    ButtonBuffer[0] = 1;
    ButtonBuffer[1] = 0;
    ButtonBuffer[2] = 1;}
  if (keyVal1High >= rlB1l && keyVal1High <= rlB1u){//button 4 pressed
    ButtonBuffer[0] = 1;
    ButtonBuffer[1] = 0;
    ButtonBuffer[2] = 0;}
  if (keyVal1High >= rlB23l && keyVal1High <= rlB23u){//buttons 5+6 pressed
    ButtonBuffer[0] = 0;
    ButtonBuffer[1] = 1;
    ButtonBuffer[2] = 1;}
  if (keyVal1High >= rlB2l && keyVal1High <= rlB2u){//Button 5 pressed
    ButtonBuffer[0] = 0;
    ButtonBuffer[1] = 1;
    ButtonBuffer[2] = 0;}
  if (keyVal1High >= rlB3l && keyVal1High <= rlB3u){//button 6 pressed
    ButtonBuffer[0] = 0;
    ButtonBuffer[1] = 0;
    ButtonBuffer[2] = 1;}
  else{
    delay(10);}
  if (keyVal2High >= rlB123l){//All 3 buttons pressed
    ButtonBuffer[3] = 1;
    ButtonBuffer[4] = 1;
    ButtonBuffer[5] = 1;}
  if (keyVal2High >= rlB12l && keyVal2High <= rlB12u){//Buttons 1+2 pressed
    ButtonBuffer[3] = 1;
    ButtonBuffer[4] = 1;
    ButtonBuffer[5] = 0;}
  if (keyVal2High >= rlB13l && keyVal2High <= rlB13u){//Buttons 1+3 pressed
    ButtonBuffer[3] = 1;
    ButtonBuffer[4] = 0;
    ButtonBuffer[5] = 1;}
  if (keyVal2High >= rlB1l && keyVal2High <= rlB1u){//button 1 pressed
    ButtonBuffer[3] = 1;
    ButtonBuffer[4] = 0;
    ButtonBuffer[5] = 0;}
  if (keyVal2High >= rlB23l && keyVal2High <= rlB23u){//buttons 2+3 pressed
    ButtonBuffer[3] = 0;
    ButtonBuffer[4] = 1;
    ButtonBuffer[5] = 1;}
  if (keyVal2High >= rlB2l && keyVal2High <= rlB2u){//Button 2 pressed
    ButtonBuffer[3] = 0;
    ButtonBuffer[4] = 1;
    ButtonBuffer[5] = 0;}
  if (keyVal2High >= rlB3l && keyVal2High <= rlB3u){//button 3 pressed
    ButtonBuffer[3] = 0;
    ButtonBuffer[4] = 0;
    ButtonBuffer[5] = 1;}
  else{}
}

void FunctionAddToBuffer(){
  Serial.println("FunctionAddToBuffer");
  for (int i=0; i<=5; i++){
    LineBuffer[CharNumber*6+i] = ButtonBuffer[i];
    // Serial.print("BPi=");
    // Serial.println(ButtonBuffer[i]);
  }
  CharNumber++;
  if(CharNumber == (CharPerLine-7)){
   tone(pbPin,pbHz,pbTime);
  }
}

void FunctionPrintBuffer(){
  Serial.println("FunctionPrintBuffer");
  BackspaceStop = CharNumber-1;
    for (int i = charStartNum; i <= (CharNumber-1)*6+5; i++){ 
      Serial.print("lb");
      Serial.print(i);
      Serial.print("=");
      Serial.println(LineBuffer[i]);
     }
  for (int i = charStartNum; i <= CharNumber-1; i++){
  //   Serial.print("i=");
  //   Serial.println(i);
    for (int x = 0; x <= 5; x++){
      ButtonBuffer[x] = LineBuffer[i*6+x];
  //     Serial.print("X=");
  //     Serial.println(x); 
    }
    FunctionPrint();
  }
  charStartNum=CharNumber;
  tone(pbPin,pbHz,pbTime);
  delay(pbTime);
  tone(pbPin,pbHz,pbTime);
  delay(pbTime);
}

void FunctionBackspace(){//logic to not go too far
  CharNumber--;
}