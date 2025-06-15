//BudgetBrailler V1.0 Arduino Code
//By Jost Strnat
//
//libraries to include
  #include <Stepper.h>
//set up stepper motors
  const int stepsPerRevolution = 200;//number of steps per turn of stepper motor

  const int RMotorAin1 = 35;//pin leanding to RMotor driver Ain1 pin
  const int RMotorAin2 = 37;//pin leanding to RMotor driver Ain2 pin
  const int RMotorBin1 = 16;//pin leanding to RMotor driver Bin1 pin
  const int RMotorBin2 = 18;//pin leanding to RMotor driver Bin2 pin

  const int XMotorAin1 = 34;//pin leanding to XMotor driver Ain1 pin
  const int XMotorAin2 = 21;//pin leanding to XMotor driver Ain2 pin
  const int XMotorBin1 = 38;//pin leanding to XMotor driver Bin1 pin
  const int XMotorBin2 = 40;//pin leanding to XMotor driver Bin2 pin

  const int XMotorSpeed = 120;//speed in RPM for XMotor
  const int RMotorSpeed = 120;//speed in RPM for RMotor
  Stepper RMotor(stepsPerRevolution, RMotorAin1, RMotorAin2, RMotorBin1, RMotorBin2);//stepper motor to move toolhead on rail
  Stepper XMotor(stepsPerRevolution, XMotorAin1, XMotorAin2, XMotorBin1, XMotorBin2);//stepper motor for rollers to move paper
  const int motorDelayTime = 0;//a delay after each time the motor moves. Left in but at 0ms for troubleshooting purposes, could cause problems interrupting other processes down the line due to delay function
//other pinouts
  const int laPin = 10;//output pin to transistor controlling linear actuator (LA)
  const int pbPin = 13;//output pin to piezo buzzer (PB)
  const int esPin = 14;//input pin from x endstop (ES)

//keyboard setup variables
  const int numKeys=13;
  const int numColumns=6;
  const int numRows=3;
  const int outputKeyPins[numColumns] = {3,5,7,9,11,12};
    //Output key to column 1 (buttons 1, 7, and space)
    //Output key to column 2 (buttons 2, 8)
    //Output key to column 3 (buttons 3, 9)
    //Output key to column 4 (buttons 4, 10)
    //Output key to column 5 (buttons 5, 11)
    //Output key to column 6 (buttons 6, 12)
  const int inputKeyPins[numColumns] = {2,4,6};
    //Input key from row 1 (buttons 1-6)
    //Input key from row 2 (buttons 7-12)
    //Input key from row 3 (only space key)

  const int keyPinR1[6] = {0,1,2,3,4,5};//button numbers in row 1
  const int keyPinR2[6] = {6,7,8,9,10,11};//button numbers in row 2
  const int keyPinR3[1] = {12};//button numbers in row 3

  bool ButtonsPressed[13];//for converting digital KeyVal reads to which button is pressed, 0 if button not pressed, 1 if button pressed
    const int dot1ButtonPressed = 2;
    const int dot2ButtonPressed = 1;
    const int dot3ButtonPressed = 0;
    const int dot4ButtonPressed = 3;
    const int dot5ButtonPressed = 4;
    const int dot6ButtonPressed = 5;
    const int lineUpButtonPressed = 6;
    const int lineDownButtonPressed = 7;
    const int homeButtonPressed = 8;
    const int printBufferButtonPressed = 9;
    //const int unused2ButtonPressed = 10;
    const int backspaceButtonPressed = 11;
    const int spaceButtonPressed = 12;
  bool ButtonBuffer[13];//Tracks which buttons have been pressed while at least one button is pressed
  bool previousKeyState = 0;//Whether buttons any buttons were pressed (1) or none pressed (0) in previous voidLoop. Used to track if keys have been held down, in which case it should not register as a second instance of typing a character
  bool currentKeyState = 0;//Whether buttons any buttons were pressed (1) or none pressed (0) in the current voidLoop. Holds state until end of void loop until transfered to previousKeyState
  const int buttonPressDelay = 0;//used if you want to pause after reading a button. used for troubleshooting
  unsigned long ButtonPressTimer = 0;//used to track how long a button has been pressed
  bool anyButtonPressed = 0;//used to store if any button is pressed
//cell dot dimension variables
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
//variables about tracking where on the page is being printed
  int LineNumber = 0;//current line number
  int CharNumber = 0;//current character number on current line
  int charStartNum = 0;//used to track last printed character if less than a whole line is printed, also prevents backspacing past this character
  const int CharPerLine = 27;//max characters per line
  const int NumberofLines = 999;//place holder for now
  bool Dot1Buffer[CharPerLine+1];//buffer of button presses corresponding to each character in a line
  bool Dot2Buffer[CharPerLine+1];//buffer of button presses corresponding to each character in a line
  bool Dot3Buffer[CharPerLine+1];//buffer of button presses corresponding to each character in a line
  bool Dot4Buffer[CharPerLine+1];//buffer of button presses corresponding to each character in a line
  bool Dot5Buffer[CharPerLine+1];//buffer of button presses corresponding to each character in a line
  bool Dot6Buffer[CharPerLine+1];//buffer of button presses corresponding to each character in a line
//piezo buffer variables
  const int pbHz = 440;//Hz of piezo buzzer sound for end of line
  const int pbTime = 250;//time in ms the buzzer will sound
//linear actuator variables
  const int laDelayTime = 100;//delay between sending current to LA 
//home variables
  const int HomeTimeoutTime = 3000;//timeout for home to prevent infinitely ramming the endstop if it fails
  unsigned long HomeTimer = 0;//for starting millis timer to timeout home function
//
//
//
//setup sets pinmodes and motor speed
void setup() {
  //start serial communcation for troubleshooting
    Serial.begin(115200);
    delay(2000);
    Serial.println("BeginVoidSetup");
  //set keyboard column pins to Output
  for (int i=0; i < numColumns; i++){
    pinMode(outputKeyPins[i], OUTPUT);
    }
  //set keyboard row pins to Input and use internal pulldown resistors
  for (int i=0; i < numRows; i++){
    pinMode(inputKeyPins[i], INPUT_PULLDOWN);
    }
  //set pinModes for linear actuator, endstop, and piezo buzzer
    pinMode(laPin, OUTPUT);//linear actuator activation pin
    pinMode(esPin, INPUT_PULLDOWN);//end stop pin
    pinMode(pbPin, OUTPUT);//end piezo buzzer

  //set motor speeds
    XMotor.setSpeed(XMotorSpeed);//set x motor speed in rpm
    RMotor.setSpeed(RMotorSpeed);//set r motor speed in rpm
  //beep buzzer to let user know the brailler is ready for typing
    tone(pbPin,pbHz,pbTime);//play a piezzo buzzer tone to indicate the brailler has turned on
    noTone(pbPin);//turn of piezo buzzer
  Serial.println("EndVoidSetup");
}
//core of the script, checks what buttons are pressed, if buttons were pressed tracks until user releases all the keys, then does the actions of the keys that were pressed
void loop() {
  //print commands for troubleshooting
  Serial.println("BeginVoidLoop");
  Serial.println();
    Serial.println("ButtonsPressed");  
    for (int i = 0; i < numKeys; i++){
    Serial.print(ButtonsPressed[i]);
    }
    Serial.println();
    Serial.println("ButtonBuffer");  
    for (int i = 0; i < numKeys; i++){
    Serial.print(ButtonBuffer[i]);
    }
    Serial.println();
  //read keys thengo through each key in the ButtonsPressed[] short term storage array. If any button is pressed, set the variable anyButtonPressed to 1
  FunctionKeyboardRead();  
  anyButtonPressed = 0;
  for(int i = 0; i < numKeys; i++){
    if(ButtonsPressed[i] == 1){
      anyButtonPressed = 1;
    }
    else{
    }
  }
  //if a button is pressed and previously no button was pressed then print for troubleshooting purposes otherwise do nothing and go to non-if end of void loop actions
  if (anyButtonPressed == 1){
    // Serial.println("Button Pressed");
    currentKeyState = 1;
    if(previousKeyState == 0){
      Serial.println("New Button Pressed");
      ButtonPressTimer = millis();//unused timer of how long buttons pressed in case needed later
      delay(buttonPressDelay);//delay used for troubleshooting
    }
    else{}
  }
    //logic for what to do if multiple buttons pressed needs to be added here
    else{//if no buttons were pressed (anyButtonPressed == 0) go through each of the buttons and do the corresponding action if the button was pressed
      currentKeyState = 0;
      if (previousKeyState == 1){
              Serial.println("DOING SOMETHINGGGGGGGGGG");
        if (ButtonBuffer[homeButtonPressed] == 1){//Home x axis
          Serial.println("Home Pressed");
          FunctionHome();
        }
        if (ButtonBuffer[lineDownButtonPressed] == 1){//Move paper out by one line
          Serial.println("LD Pressed");
          FunctionNextLine();
        }
        if (ButtonBuffer[lineUpButtonPressed] == 1){//move paper in by one line
          Serial.println("LU Pressed");
          FunctionPreviousLine();
        }
        if (ButtonBuffer[printBufferButtonPressed] == 1){//print buffer
          Serial.println("PB Pressed");
          FunctionPrintBuffer();
        }  
        // if (){//unused button
        //   delay(0);
        // }    
        if (ButtonBuffer[backspaceButtonPressed] == 1){//backspace
          Serial.println("BS Pressed");
          FunctionBackspace();
        }  
        //if any of the 6 dot buttons or space was pressed, then add a character to the buffer using FunctionAddToBuffer 
          //if the max characters in the line has been reached, print buffer and carraige return 
        if (ButtonBuffer[dot1ButtonPressed] == 1 || ButtonBuffer[dot2ButtonPressed] == 1 || ButtonBuffer[dot3ButtonPressed] == 1 || 
            ButtonBuffer[dot4ButtonPressed] == 1 || ButtonBuffer[dot5ButtonPressed] == 1 || ButtonBuffer[dot6ButtonPressed] == 1 || 
            ButtonBuffer[spaceButtonPressed] == 1){
          Serial.print("TypeButtonsPressed");
          FunctionAddToBuffer();
              if(CharNumber > CharPerLine){
                FunctionPrintBuffer();
                FunctionNextLine();
                FunctionHome();
            }
          }
        }
        //after completing actions, wipe short term tracking of buttons that were pressed
        FunctionResetButtons();
      }
  //non-if end of void loop actions, track whether a key was pressed in the current cycle so it can be reference in the next cycle
  previousKeyState = currentKeyState;
}
//FunctionNextLine pushes the paper out the front of the the brailler (moves the print line down on the page from the reader's perspective)
  //LineNumber is tracked but not currently used
void FunctionNextLine(){
  Serial.println("FunctionNextLine");
  RMotor.step(CellStepsR);
  LineNumber++;
  FunctionBacklashAdjust();
}
//FunctionPreviousLine pushes the paper out the back of the the brailler (moves the print line up on the page from the reader's perspective)
  //LineNumber is tracked but not currently used
void FunctionPreviousLine(){
  Serial.println("FunctionPreviousLine");
  RMotor.step(-CellStepsR);
  LineNumber--;
  FunctionBacklashAdjust();
}
//FunctionHome moves the x-axis toolhead to the far left (beginning of the line), resets the CharNumber and charStartNum to 0
  //Has timout so if endstop fails it doesn't just keep smashing into stuff
  //add error tone to timeout?
void FunctionHome(){
  Serial.println("FunctionHome");
  HomeTimer = millis();
  while(digitalRead(esPin) == 1){
  XMotor.step(1);
  if(millis() >= (HomeTimer + HomeTimeoutTime)){
    break;
  }  
  }
  CharNumber = 0;
  charStartNum = CharNumber;  
  delay(100);
}
//FunctionBacklashAdjust will move the paper backward and forward to cancel out backlash. Not currently needed but function left in if needed later.
void FunctionBacklashAdjust(){
  // Serial.println("FunctionBacklashAdjust");
  // RMotor.step(DotStepsR);
  // delay(motorDelayTime);
  // RMotor.step(DotStepsR);
  // delay(motorDelayTime);
  // RMotor.step(-2*(DotStepsR));
  // delay(motorDelayTime);
}
//FunctionResetButtons sets the bool arrays ButtonBuffer[] and ButtonsPressed[] to 0 to reset tracking of what buttons were pressed after completing the actions for the buttons
void FunctionResetButtons(){
  // Serial.println("FunctionResetButtons");  
  for(int i = 0; i < numKeys; i++){
  ButtonBuffer[i] = 0;//reset buttons
  }
  for(int i = 0; i < numKeys; i++){
  ButtonsPressed[i] = 0;//reset buttons
  }
}
//FunctionKeyboardRead provides power to each column of the keyboard individually in turn and for each will read the digital outputs of each row to show which keys are pressed
  //Buttons which are pressed are stored as bools in two arrays:
  //ButtonsPressed[] which tracks which buttons are currently pressed
  //ButtonBuffer[] which tracks all buttons that have been pressed as long as at least one button has been continuously held down, even if those buttons have subsequently been released
  //delay(1) added after each column is depowered as I had issues with one pin being slow on a microcontroller and causing keyboard reads. Probably could be a shorter delay.
void FunctionKeyboardRead(){
  digitalWrite(outputKeyPins[0], HIGH);
    if(digitalRead(inputKeyPins[0]) == HIGH){
        ButtonsPressed[0] = 1;
        ButtonBuffer[0] = 1;
      }
      else{
        ButtonsPressed[0] = 0;  
      }
    if(digitalRead(inputKeyPins[1]) == HIGH){
        ButtonsPressed[6] = 1;
        ButtonBuffer[6] = 1;
      }
      else{
        ButtonsPressed[6] = 0;  
      }
    if(digitalRead(inputKeyPins[2]) == HIGH){
        ButtonsPressed[12] = 1;
        ButtonBuffer[12] = 1;
      }
      else{
        ButtonsPressed[12] = 0;  
      }
    digitalWrite(outputKeyPins[0], LOW);
    delay(1);
  digitalWrite(outputKeyPins[1], HIGH);
    if(digitalRead(inputKeyPins[0]) == HIGH){
        ButtonsPressed[1] = 1;
        ButtonBuffer[1] = 1;
      }
      else{
        ButtonsPressed[1] = 0;  
      }
    if(digitalRead(inputKeyPins[1]) == HIGH){
        ButtonsPressed[7] = 1;
        ButtonBuffer[7] = 1;
      }
      else{
        ButtonsPressed[7] = 0;  
      }
    digitalWrite(outputKeyPins[1], LOW);
    delay(1);
  digitalWrite(outputKeyPins[2], HIGH);
    if(digitalRead(inputKeyPins[0]) == HIGH){
        ButtonsPressed[2] = 1;
        ButtonBuffer[2] = 1;
      }
      else{
        ButtonsPressed[2] = 0;  
      }
    if(digitalRead(inputKeyPins[1]) == HIGH){
        ButtonsPressed[8] = 1;
        ButtonBuffer[8] = 1;
      }
      else{
        ButtonsPressed[8] = 0;  
      }
    digitalWrite(outputKeyPins[2], LOW);
    delay(1);
  digitalWrite(outputKeyPins[3], HIGH);
    if(digitalRead(inputKeyPins[0]) == HIGH){
        ButtonsPressed[3] = 1;
        ButtonBuffer[3] = 1;
      }
      else{
        ButtonsPressed[3] = 0;  
      }
    if(digitalRead(inputKeyPins[1]) == HIGH){
        ButtonsPressed[9] = 1;
        ButtonBuffer[9] = 1;
      }
      else{
        ButtonsPressed[9] = 0;  
      }
    digitalWrite(outputKeyPins[3], LOW);
    delay(1);
  digitalWrite(outputKeyPins[4], HIGH);
    if(digitalRead(inputKeyPins[0]) == HIGH){
        ButtonsPressed[4] = 1;
        ButtonBuffer[4] = 1;
      }
      else{
        ButtonsPressed[4] = 0;  
      }
    if(digitalRead(inputKeyPins[1]) == HIGH){
        ButtonsPressed[10] = 1;
        ButtonBuffer[10] = 1;
      }
      else{
        ButtonsPressed[10] = 0;  
      }
    digitalWrite(outputKeyPins[4], LOW);
    delay(1);
  digitalWrite(outputKeyPins[5], HIGH);
      if(digitalRead(inputKeyPins[0]) == HIGH){
          ButtonsPressed[5] = 1;
          ButtonBuffer[5] = 1;
        }
        else{
          ButtonsPressed[5] = 0;  
        }
      if(digitalRead(inputKeyPins[1]) == HIGH){
          ButtonsPressed[11] = 1;
          ButtonBuffer[11] = 1;
        }
        else{
          ButtonsPressed[11] = 0;  
        }
      digitalWrite(outputKeyPins[5], LOW);
    delay(1);
}
//FunctionAddToBuffer takes the ButtonBuffer[] bools for the buttons corresponding to the braille dots and transfers them to DotnBuffer[] arrays for long term storage
  //Piezo buzzer will beep if 7 characters are left to type in the current line
void FunctionAddToBuffer(){
  Serial.println("FunctionAddToBuffer");
  Dot1Buffer[CharNumber] = ButtonBuffer[dot1ButtonPressed];
  Dot2Buffer[CharNumber] = ButtonBuffer[dot2ButtonPressed];
  Dot3Buffer[CharNumber] = ButtonBuffer[dot3ButtonPressed];
  Dot4Buffer[CharNumber] = ButtonBuffer[dot4ButtonPressed];
  Dot5Buffer[CharNumber] = ButtonBuffer[dot5ButtonPressed];
  Dot6Buffer[CharNumber] = ButtonBuffer[dot6ButtonPressed];
    Serial.print("DotBuffer:");
    Serial.print(Dot1Buffer[CharNumber]);
    Serial.print(Dot2Buffer[CharNumber]);
    Serial.print(Dot3Buffer[CharNumber]);
    Serial.print(Dot4Buffer[CharNumber]);
    Serial.print(Dot5Buffer[CharNumber]);
    Serial.print(Dot6Buffer[CharNumber]);
  CharNumber++;
  if(CharNumber == (CharPerLine-7)){
   tone(pbPin,pbHz,pbTime);
  }
}
//FunctionPrintbuffer goes through each character that has been typed since the last print or last home.
  //Loops through each character number, prints the contents of the DotnBuffer[] arrays for trouble shooting, then calls FunctionPrint to print that character.
  //Ends by setting charStartNumber to the current character number to mark the most recent printed character within the line.
  //Has Piezo buzzer beep at start and end to alert user that they can't type 
void FunctionPrintBuffer(){
  tone(pbPin,pbHz,pbTime);
  Serial.println("FunctionPrintBuffer");
  for (int i = charStartNum; i <= CharNumber-1; i++){
    Serial.println("DotBuffer:");
    Serial.print(Dot1Buffer[i]);
    Serial.print(Dot2Buffer[i]);
    Serial.print(Dot3Buffer[i]);
    Serial.print(Dot4Buffer[i]);
    Serial.print(Dot5Buffer[i]);
    Serial.print(Dot6Buffer[i]);
    FunctionPrint(i);
  }
  charStartNum=CharNumber;
  tone(pbPin,pbHz,pbTime);
}
//FunctionPrint moves the paper/toolhead through each of the 6 dots of a braille cell and 
  //uses the linear actuator to punch dots based on the contents of DotnBuffer[] arrays
void FunctionPrint(int PrintChar){
    Serial.println("FunctionPrint");
    Serial.println(PrintChar);
  if(Dot1Buffer[PrintChar] == 1) {FunctionPunch();
    Serial.println("Punch1");}
  delay(motorDelayTime);
  RMotor.step(DotStepsR2);
  delay(motorDelayTime);
  if(Dot2Buffer[PrintChar] == 1) {FunctionPunch();
    Serial.println("Punch2");}
  delay(motorDelayTime);
  RMotor.step(DotStepsR);
  delay(motorDelayTime);
  if(Dot3Buffer[PrintChar] == 1) {FunctionPunch();
    Serial.println("Punch3");}
  delay(motorDelayTime);
  RMotor.step(-DotStepsR-DotStepsR2);
  delay(motorDelayTime);
  XMotor.step(DotStepsX);
  delay(motorDelayTime);
  if(Dot4Buffer[PrintChar] == 1) {FunctionPunch();
    Serial.println("Punch4");}
  delay(motorDelayTime);
  RMotor.step(DotStepsR2);
  delay(motorDelayTime);
  if(Dot5Buffer[PrintChar] == 1) {FunctionPunch();
    Serial.println("Punch5");}
  delay(motorDelayTime);
  RMotor.step(DotStepsR);
  delay(motorDelayTime);
  if(Dot6Buffer[PrintChar] == 1) {FunctionPunch();
    Serial.println("Punch6");}
  delay(motorDelayTime);
  RMotor.step(-DotStepsR - DotStepsR2);
  delay(motorDelayTime);
  XMotor.step(CellStepsX);
  FunctionResetButtons();
}
//FunctionPunch activates the linear actuator transistor with a delay of 40ms to allow the linear actuator to complete its movement
void FunctionPunch(){
  Serial.println("FunctionPunch");
  digitalWrite(laPin,HIGH);
  delay(laDelayTime);
  digitalWrite(laPin,LOW);
  delay(laDelayTime);
}
//FunctionBackspae deletes the last character by reducing CharNumber by 1. Will not make CharNumber go negative or go below the last printed character. 
void FunctionBackspace(){//
  if(CharNumber > charStartNum){
    CharNumber--;
  }
  else {
    Serial.println("NoBackspaceForYou");
  }
}