/*  -Lock Voltage Input                          PC3 
  -Switch Input                                PD4      
  -Boost Shut Down output                      PC2  
  -Serial Buffer Ctrl Output                   PD2   
  -Lock_Ctrl  Output                           PD5
  -Potentiometer Inpt                          PC4  
  
 //*** RS 485 on serial interface*** 
  -RX                                   PD0
  -TX                                   PD1
  -RTS(Dir) Out                         PB2
  
  
  On the circuit board
 
  Serial TX  J11
  Serial RX  J12
  Rs485 Data+ J14
  Rs485 Data- J15  
  
*/
 
 
 
 
 
 
 
 
 
 
 
 
 
 
#include <String.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
 
# define LOCK_ID  23456
 
/////////////////////
#define EverON 4       //EEprom Address 4. This is to determine whether the lock ha      
#define PosAdress 100   //EEprom address 100 to store all 24 bolt step
//int PosAdress =0;
 
#define BoltCycleAdress 96
#define CalibrationPeriod 3   // This is define how often the bolt might to AutoCalibrate
 
/////////////////////////
const int LockID = 23456;    // 12345 for the first XCF install
int LockVoltageInput = A3;              //    PC3  Analog
int SwitchInput = 4;                    //    PD4   It is the same as Open request      J8
int BoostShutDownOut = A2;              //    PC2  Must be changed from Digital I/O
int BufferCtrlOut = 2;                  //    PD2    //Serial bufffer low Enabled 
int LockCtrlOut = 5;                    //    PD5
int PotInput = A4;                      //    PC4  Analog   Must be changed in production   J10
int GeoZoneInput = 3 ;                    //PD3   J7
 
 //*** RS 485 on serial interface*** 
int RE = 10;                     //  PB2    //  high disabled
int DE = 6;                      //  PD6    //  Low   disabled
 // *** Timing
 
long debounceDelay = 40;      // time to confirm a request out of noisy switch input           
long debounceTimeOutDelay = 80;   // time to get out of an excessive loop due to noisy switch   
 
long EventTime=0;
long EventTimeDelay = 60000; // Time to send a periodic message
int HoldOpen = 500; // Time to hold lock Opening     
     
String inputString = "";         // a string to hold incoming data
String OutputString = "";
boolean stringComplete = false;  // whether the string is complete
 
boolean GeoZoneInputValue = 0;   // Digital to determine if the Geozone 
int OpenRequest = 4;             // Switch Input = OpenRequest
boolean OpenRequestValue=1;      // To hold the state of the switch
boolean PastOpenRequestValue=1;  // To hold the previuos state of the switch 
boolean Event =0;                // when O no event to print, 1 print the special status
boolean BoltEvent =0;            // This is to determine if the bolt has been pushed
 
String stringOne = "GeoOK\n";
String stringTwo = "GeoNO\n"; // It is important to include the new line character at the end of the string
//BIN Status =0000000 ; // Switch,BoltPostion,BoltStuckUp,BoltSockDown,PasswordAccess
String PowerUp = "0,2,";
int f;
 
int CurrentStepCount =0;       //
int StepCount =0;
int OutOfDebounce =0;   // taking too long to get out of the debouncing process 
int IncidentCount =0;   // should be recorded to the EEPROM
int BoltCycle = 1;        // to keep track of the number of  bolt opening 
int PreviousBoltCycle = 0;  
int MessageCount =0;
int BoltStatus = 0;
int Pos ;
 
 
int LockPower =5;        // On Lock power is digital io5 or PD5
int ON = 104; //134        // Time in millisecond to keep the solenoid plunger ON
int OFF = 104 ;//2500;     // Time in millisecond to keep the solenoid plunger OFF
int ProcessingTime = 30;    //Anticipate amount Time for the processing to take place after every solenoid cycle
 
 
 
/////////////////////////
/////////////////////////
 
// for Potentiometer average calculation
 
int TotalforDif = 0;        // for Potentiometer average calculation
int SampleDeleted =0;       // for Potentiometer average calculation
int AverageADf;             // for Potentiometer average calculation
const int numReadings =20;  // for Potentiometer average calculation
int readings[numReadings];  // for Potentiometer average calculation
int AverageDifference [numReadings]; // for Potentiometer average calculation
 
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // for Potentiometer average calculation
 
 
///////////////////
int BoltPosition[24];  // Very Important for autoCal not to populate the array
//int BoltPosition[] = {474,474,474,474,474,474,466,456,447,438,429,419,409,398,389,380,370,359,349,340,330,321,314};  // My test device need to change Initialize previous position
//int BoltPosition[] =   {418,418,418,418,418,418,411,403,394,386,377,367,358,349,340,331,322,314,305,297,288,280,274}; // My lock#3  // need to change Initialize previous position
//int BoltPosition[] = {449,449,449,449,449,418,419,400,423,414,403,393,382,372,363,350,341,332,321,312,301,291,282};  // My Lock#1 on trailer XCF 5393051  // need to change Initialize previous position
//int BoltPosition[] =   {453,453,453,453,453,453,443,433,423,413,403,393,381,370,359,348,337,327,317,306,296,287,279}; // My lock#4  // need to change Initialize previous position
 
 
int WithinPosition=7 ;   //  to determine whether we are close enough to the correct position 
int CurrentPosition ;    // Pot current position  
int PreviousPosition;    // Pot previous position  // need to change Initialize previous position
int StepGap ;            // value difference in 2 consecutive steps // need to change Initialize previous position
 
 
 
 
 
 
 
 
void CheckForBigDrop(){
    
      while ( abs(CurrentPosition - PreviousPosition) < 90  ){ // checking for a significant drop
          
        LockON();
        delay (30);
        PreviousPosition = CurrentPosition;
        CurrentPosition  = readPosition(10);  
       
       // Listening(); 
               /*   
               ++StepCount;   // for debug uncomment          
                if (StepCount >90 ) {    // for debug uncomment
                Serial.println("0,4,23456");  // This mean the bolt is completely stuck
                return;
                }       
               */ 
      }  
 
}
 
void SaveAllStep(){
  
     for (int i=0; i < 24 ; i++){    
        PreviousPosition = CurrentPosition;
        CurrentPosition  = readPosition(10);   
        EEPROM.put( (PosAdress + (i*2)) , CurrentPosition); 
        LockON();        
     }
     
     Serial.println("Auto Calibration");
 
     for (int i= 1; i < 24 ; i++){  // repopulating the array       
           int tt = EEPROM.get( (PosAdress + (i*2)) ,f) ;
           
           delay(2);
           BoltPosition [i-1]= tt; // May need to change it   
           Serial.println( BoltPosition [i-1] ); 
           delay(2);
     }  
     Serial.println ("AutoCal Completed");
}
 
 
 
void AutoCalibration(){  
   CheckForBigDrop();  
   delay(1000);
   SaveAllStep();
}
 
 
 
 
 
void setup() {
    
  // initialize serial:  
 
 
  Serial.begin(115200);     
  inputString.reserve(20);  //20 bytes reserve for string input 
  delay(110);
  
  // Lock Power Up message            
  Serial.println("0,2,23456");  
        
   delay(11);
      
    // Initializing all Pin 
 
    pinMode(GeoZoneInput, INPUT);      
    pinMode(BoostShutDownOut, OUTPUT);
    pinMode(BufferCtrlOut, OUTPUT);
    pinMode(LockCtrlOut, OUTPUT);    
    pinMode(SwitchInput, INPUT);
    pinMode(DE, OUTPUT);
    pinMode(RE, OUTPUT);
    
     // High = Enabled to boost power in case of Input voltage drop
    digitalWrite(BoostShutDownOut,HIGH);   
    
    /*
    // RS 485 Enabled transmitting and disable Serial buffer
    digitalWrite(RE,HIGH);                // LOW Activated  receiving  // High Activated transmission 
    digitalWrite(DE,HIGH);                // LOW Activated  receiving  // High Activated transmission  
    digitalWrite(BufferCtrlOut,LOW);      // High = Enable capable to communicate with the CU via serial TTL    
 
    */
    
      // Enable Serial buffer and disable Serial buffer  // to disable RS 485 RE High DE low; allow TTL serial BufferCtrlOut High
    digitalWrite(RE,HIGH);                // LOW Activated  receiving  // High Activated transmission 
    digitalWrite(DE,LOW);                // LOW Activated  receiving  // High Activated transmission  
    digitalWrite(BufferCtrlOut,HIGH);    // High = Enable capable to communicate with the CU via serial TTL
    
 
    
    
 
 
 
 
 
    if ( EEPROM.get(EverON,f) == -1){   // Checking: is it the first time the device power up          
           AutoCalibration();  // Need to implement later the cyclic check to recalibrate 
           GenerateRandomID();  // Generate random number. This will not be used for lock ID anymore. For it to function we MUST depopulate R37,D1,R35 on the board from J3 which goes to ADC0  
           Serial.println(EEPROM.get(0,f));     // for Debug           
           EEPROM.put( EverON, 111);  // Never Again only if it is re-flashed           
    } 
    
    
    // For Debug
     for (int i= 1; i < 24 ; i++){  // repopulating the array     
           int tt = EEPROM.get( (PosAdress + (i*2)) ,f) ;           
           delay(2);
           BoltPosition [i-1]= tt; // May need to change it   
           Serial.println( BoltPosition [i-1] ); 
           delay(2);
     } 
 
    
    // Move the solenoid Once to solenoid to signal Set up went OK
    CurrentPosition  = readPosition(10);
    LockON();
    PreviousPosition = CurrentPosition;
    CurrentPosition  = readPosition(10);
 
 
    delay(2500); // for debug
    
    // Serial.println(DeviceID, DEC); // To be implemented later 
    //Serial.println(PowerUp);        // // To be implemented later 
    
    Serial.println("0,2,23456");  // Message to signal that the device powered up
    EventTime = millis();
    
}
 
 
void loop() {
    //CallHazlock();   
     //Sendmessage();
     //Listening();
  
    for (int z = 0; z < 1000; z++ )  {
     OpenLock(); 
     Sendmessage();
     delay(2);
    }
    
    while(1){
     delay(1);
    }   
}



void CallHazlock(){
  
      ReadAllCargoInput();      
      
      debounce ();
      
      // Check first to make sure that OutOfDebounce is 0;  // if OutOfDebounce is 1 this switch is too noisy 
      // Geozone Input will be removed for the lock to request the master permission to open  
	     
      if ((OpenRequestValue == 1)&&(PastOpenRequestValue==0 )&&(GeoZoneInputValue==0)&& (OutOfDebounce == 0)) {   // input 2  11 and input 6 Geozone and switch
          OpenLock();       
      }    
      
      else if ((OpenRequestValue == 1)&&(PastOpenRequestValue==0 )&&(GeoZoneInputValue==1)&& (OutOfDebounce == 0) ) {   // If out of the geozone but switch is open 
            // CallBackUp();  // CallBackUp routine Looks for code PIN entry                
      }
      
      MayNeedCalibrate(); 
     // PrintStatus();  // For debug may not need this . altough important to send feedback to GPS
}

////////////////////////////
////////////////////////////

void MayNeedCalibrate(){

    if ( (BoltCycle % CalibrationPeriod) == 0){   // Checking every 999 time
    
           PreviousPosition = CurrentPosition; // Important for AutoCal
           AutoCalibration();  // Need to implement later the cyclic check to recalibrate                     
           BoltCycle ++;       // Important to avoid falling immediately back into this loop again 
    }  
}


/////////////////////////////
//////////////////////////////

 void debounce (){

 
    const int buttonPin = OpenRequest;      
    int buttonState;             
    int lastButtonState = PastOpenRequestValue;   
    
    long lastDebounceTime = millis(); 
    long DebounceTimeOut = millis(); 

     
      if ((OpenRequestValue == 1)&&(PastOpenRequestValue==0 ) ){ //for debug it used to be ((OpenRequestValue == 1)&&(PastOpenRequestValue==0 ) || (OpenRequestValue == 0)&&(PastOpenRequestValue==1 )){     
        
          while (1){
            int reading = digitalRead(buttonPin);
            Listening();
            
            if (reading != lastButtonState) {
              lastDebounceTime = millis();
            } 
            
             if ((millis() - DebounceTimeOut) > debounceTimeOutDelay)
                 if (reading ==1){
                     OutOfDebounce = OutOfDebounce +1 ; //set a flag to signal debounce went out of bound and keep count
                     if (OutOfDebounce == 100) {
                          OutOfDebounce =1;
                          Serial.println ("0,3,23456"); // This means excessive noise //for debug this message is to inform that there has been too many debounce going on switch
                        }
                     return;
              } 
           
            if ((millis() - lastDebounceTime) > debounceDelay) {
                     OpenRequestValue = reading ;                      
                     OutOfDebounce=0;
                     return ;
            }
          
            lastButtonState = reading;
          }
      }
 }

//////////////////////////////////
//////////////////////////////////

// This function open the lock by retracting the bolt, waiting for 45s and releasing the bolt back


void OpenLock(){
    
    StepCount =0; // To keep track of the total Solenoid cycle required to open a lock. This number should normally be 24. If it exceed 90, the lock will quit trying 
    
    PreviousBoltCycle =  BoltCycle;   // recording the previous bolt cycle to compare for a new bolt cycle
     
    int Check6To1= Verify6To1(CurrentStepCount);  // verifying if CurrentStepCount is where it was expected. Return back correct position
	
		
    // At this point, the gear is moving freely. bolt is somwhere below position 6
	
     while ( Check6To1 <7){ // checking whether the gear is rotating freely
		       
        LockON();
                
        Check6To1= Verify6To1(CurrentStepCount);    // verifying if CurrentStepCount is where it was expected. Return back correct position 
        
               ++StepCount;          
                if (StepCount >90 ) {    // If this exceed 90, give up trying
                Serial.println("0,4,23456");  // This mean the bolt is completely stuck
                return;
                }
     }
 
     
    VerifyPosition(CurrentStepCount);
  
  // At this point, the gear is engaged . bolt is in motion. Bolt position is between 7-21
 
  if (CurrentStepCount<21){       // check we are not on the last step
  
       for( ;CurrentStepCount<22; CurrentStepCount++){       
          VerifyPosition(CurrentStepCount);       
          LockON();  

               // Listening();    
               ++StepCount;            
                if (StepCount >90 ) {    /// If this exceed 90, give up trying
                Serial.println("0,4,23456");  // This mean the bolt is completely stuck
                return;
                }                                                   
       }       
  } 
  
    VerifyPosition(CurrentStepCount);


       // At this point, the bolt is near the top Bolt position is somewhere between 22-23
       
   delay(HoldOpen);   //  At this point the bolt is retracted. Typically we wait for bolt to remain open for 45000 ms.
   
      while ( abs(CurrentPosition - PreviousPosition) < abs(BoltPosition[1]-BoltPosition[20])  ){ // checking for a significant drop
       
        delay (30);
        VerifyPosition(CurrentStepCount);    
        LockON();
        CurrentStepCount++;
		
               // Listening();    
               ++StepCount;   //        
                if (StepCount >90 ) {    // for debug uncomment
                Serial.println("0,4,23456");  // This mean the bolt is completely stuck
                return;
                }        
      }

   
     if (CurrentStepCount ==24 ){   // Current Count only goes from 0-24.
         CurrentStepCount=0;
     }
     
        Event =1    ;             // For debug uncomment
        BoltCycle= BoltCycle+1;  // For debug uncomment
        
}



///////////////////////////////
///////////////////////////////
// This function adjust k to appropriate position based on the potentiometer reading and return the corrected k val
// If inconsistent increment the incident count

int  Verify6To1( int k){
  
           
    int NewPosition =k ;
    int PastPosition =k;
      
      
    PreviousPosition = CurrentPosition;      
    CurrentPosition  = readPosition(10); // reading from 10 samples average
   
        
     if ( abs( (BoltPosition[k])-CurrentPosition ) < WithinPosition )   { 
         BoltStatus=0;
         return k;     // Position is where expected. Return back the same integer that was received
     }
      
       
     else{    // Position is not where expected. Go figure out the correct position.
		 
             for ( int i=0; i<24; i++) {      
               
               if ( (abs( (BoltPosition[i])-CurrentPosition )) < ( WithinPosition-1) )   {                     
                      CurrentStepCount=i;
                      NewPosition =i;
                       Event=1;            // Signal that a step was missed
                     
               }            
               }
             
        return NewPosition;      // This is the corrected position
        
     }
  
    PreviousPosition = CurrentPosition;  // need to change move this to the appropriate location
    
}





////////////////////////////
/////////////////////////////
// adjust k to appropriate position based on the pot and return the corrected K
// If inconsistent increment the incident count

int  VerifyPosition( int k){
         
    if (k==24)
        k=0;
        
    int NewPosition =k ;
    int PastPosition =k;
      
    PreviousPosition = CurrentPosition;      
    CurrentPosition  = readPosition(10); // reading from 10 samples average
    
     if ( abs( (BoltPosition[k])-CurrentPosition ) < WithinPosition )   { 
         BoltStatus=0;
         return k;     // Position is where expected
     }
             
     else{    // Position is not where expected. Go figure out the correct position. 
          
             for ( int i=0; i<24; i++){      
               
               if ( (abs( (BoltPosition[i])-CurrentPosition )) < ( WithinPosition-1) )   {                     
                      CurrentStepCount=i;
                      NewPosition =i;
                      Event=1;                  
				}            
               }
               
			           
                if ((NewPosition != PastPosition)&&(NewPosition<8)) {
                                       
					IncidentCount = IncidentCount +1;
                    
					if (IncidentCount == 50000){
						IncidentCount =0; 
					}
                    
					if (NewPosition < PastPosition) // The bolt is getting stuck on the way up
						BoltStatus = 1 ;
          
					if (NewPosition > PastPosition) // The bolt step got push
						BoltStatus = 2 ;          
                }     
             
        return NewPosition;
        
     }
  
    PreviousPosition = CurrentPosition;  // need to change move this to the appropriate location    
}






////////////////////////////
////////////////////////

// This function return an average of the "SampleNumber"

int readPosition(int SampleNumber){
  
 
    total =0;
    
    for (int h = 0; h < SampleNumber; h++){
         
      delay(2);    
      readings[h] =   analogRead(PotInput);     
      // Serial.print ("Reading: "); Serial.print ( readings[h] ,DEC); Serial.print (" : ");  
      total = total + readings[h];    
      //Serial.println(total, DEC);
     
    }   
    
     average = total / SampleNumber; // for debg uncomment
     //Serial.print ("average: "); Serial.print ( average ,DEC); Serial.println (" : ");
   
     
     TotalforDif = 0;
      
     for (int i = 0 ; i < SampleNumber; i++ ){       
       
       AverageDifference[i] = abs(average - readings[i]) ;
       // Serial.print ("AverageDifference : "); Serial.println ( AverageDifference[i] ,DEC);
       
       TotalforDif = TotalforDif + AverageDifference[i];
     }
     
      AverageADf = TotalforDif / SampleNumber;      
       //Serial.print ("Average for AverageDifference: "); Serial.println(AverageADf, DEC);
      
      
       SampleDeleted =0;  
       int upperlimit = average + (AverageADf + 1);
       int lowerlimit =  average - (AverageADf + 1); 

     for (int k = 0; k < SampleNumber; k++ ){    
       
       if ( (readings[k] > upperlimit)  ||  (readings[k] < lowerlimit) ) {
         readings[k] =0;
         SampleDeleted ++;
       }
       
     }

    total =0;
    
    for (int hh = 0; hh < SampleNumber; hh++){
    
     //Serial.print ("New Reading: "); Serial.print ( readings[hh] ,DEC); Serial.print (" : ");  
     total = total + readings[hh];   //Serial.println(total, DEC);
     delay(2); 
    }   
    
     average = total / (SampleNumber - SampleDeleted); // for debg uncomment
     //Serial.println();
     //Serial.print ("final average: "); Serial.print ( average ,DEC); Serial.println (" : ");
 
     return average;     
     
}






///////////////////// for debug
void ShortOpenLock(){
 for (int i =0; i<24; i++) 
      LockON();
}


///////////////////////
///////////////////////
void ReadAllCargoInput(){
  
      // delay (10);   
     PastOpenRequestValue = OpenRequestValue;
     OpenRequestValue = digitalRead(OpenRequest); 
     GeoZoneInputValue = digitalRead( GeoZoneInput);  // Geozone Input will remove for the lock to request the master permission to open lock
   
 }



///////////////////////////
//////////////////////////

   
int  VerifyBoltPush( int k){
         
    if (k==24)
        k=0;
    
    int NewPosition =k ;
    int PastPosition =k;
      
    PreviousPosition = CurrentPosition;      
    CurrentPosition  = readPosition(10); // reading from 10 samples average
       
        
     if ( abs( (BoltPosition[k])-CurrentPosition ) < WithinPosition )   {   // Compare against the known position value
         BoltStatus=0;             
         return k;     // Position is where expected
     }
      
       
     else{       // Position is not matching where it was expected to be. Go figure out the new position
            
             for ( int i=0; i<24; i++){      
               
				   if ( (abs( (BoltPosition[i])-CurrentPosition )) < ( WithinPosition-1) )   {                     
						  CurrentStepCount=i;
						  NewPosition =i;
						  //Event=1;
                      
						if (PreviousBoltCycle == BoltCycle)
							 BoltEvent =1;     //  to signal that 
				   }            
			   }
         
         
            if ((NewPosition != PastPosition)&&(NewPosition<8)) {
                                
					IncidentCount = IncidentCount +1;
                    
					if (IncidentCount == 50000){
						IncidentCount =0; 
					}
                
					if (NewPosition < PastPosition) // The bolt is getting stuck on the way up
						BoltStatus = 1 ;
          
					if (NewPosition > PastPosition) // The bolt step got push
						BoltStatus = 2 ;          
            }     
             
        return NewPosition;  
     }
	 
    PreviousPosition = CurrentPosition;  // need to change move this to the appropriate location   
}




//////////////////////////
//////////////////////////
void Sendmessage() {


  Pos= VerifyBoltPush (CurrentStepCount); 
  
   if (Event == 1){
     PrintStatus();   
      Event=0; // no more event
      EventTime = millis(); // reset the time for the next time driven  
      ++MessageCount;
   } 

  
   if ((millis() - EventTime) > EventTimeDelay){ 
        
        PrintStatusSame();        
        EventTime = millis();
        ++MessageCount;
   }
   
}

//////////////////////////////
//////////////////////////////
void PrintStatus(){
  
  //   Special Event 1
   //  Message Count, Message type, Lock ID, Switch, bolt position, Status, Incident Count, Step Count, Bolt cycle\n
   
   Serial.print(MessageCount,DEC);  Serial.print(',');
   Serial.print('1');  Serial.print(',');
   Serial.print(LockID,DEC);   Serial.print(',');
   Serial.print(OpenRequestValue,DEC);  Serial.print(',');
   Serial.print(CurrentStepCount,DEC);   Serial.print(',');
   //Serial.print(BoltStatus,DEC);   Serial.print(',');  // For the status to be implemented
   Serial.print(IncidentCount,DEC);   Serial.print(',');
   Serial.print(StepCount,DEC);   Serial.print(',');
   Serial.println(BoltCycle,DEC);
       
}


/////////////////////////
/////////////////////////
void PrintStatusSame(){
  //  0  No Event 
  //    Message Count,0,LockID, Switch \n    
   Pos = VerifyPosition(CurrentStepCount);
    
   Serial.print(MessageCount,DEC);  Serial.print(',');
   Serial.print('0');  Serial.print(',');
   Serial.print(LockID,DEC);   Serial.print(',');
   Serial.print(OpenRequestValue,DEC);  Serial.print(','); 
   Serial.print(Pos,DEC);  Serial.print(',');
     Serial.println(IncidentCount,DEC);  
   
}






/////////////////////////////
/////////////////////////////

void LockON (){
    analogWrite(LockPower, 255);
    delay(ON);  //20                // 104 is the original test 
    //digitalWrite(8,HIGH);
    analogWrite(LockPower, 0);
    delay(OFF-ProcessingTime);  //70           //  104 
          
}


//////////////////////////////////////
//////////////////////////////////////

void Listening(){
    if (stringComplete) {
          Serial.println(inputString);     

       if (inputString.substring(0) == stringOne){ 
          Serial.println("In The Zone");   
          GeoZoneInputValue =1;  
       }
  
       if (inputString == stringTwo ){ 
          Serial.println("Out of the Zone");
          GeoZoneInputValue =0;  
       }         
  
      // clear the string:
      inputString = "";
      stringComplete = false;
  } 
}



void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    inputString += inChar;
    
    if (inChar == '\n') {
      stringComplete = true;
    }      
  }
}



////////////////////////
////////////////////////

void GenerateRandomID(){
    randomSeed(analogRead(0));      // Using Analog O to generate random number
    unsigned int randNumber = random(50000);
    EEPROM.put(0, randNumber);
}

/////////////////// // for debug
void ShortLockON (){
    analogWrite(LockPower, 255);
    delay(10);  //20                // 104 is the original test 
    //digitalWrite(8,HIGH);
    analogWrite(LockPower, 0);
    delay(10);  //70           //  104 
    
}   
    
   