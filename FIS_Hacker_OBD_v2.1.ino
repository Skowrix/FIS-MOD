// ==================================  display Boost(ADC), oil Temp(CAN) and data requested by OBD2 on FIS ==========================================
//

/* 
 * 0 - OliPress
 * 1 - STFT 
 * 2 - LTFT
 * 3 - tADV
 * 4 - IAT
 * 5 - MAF
 * 6 - Lambda
 * 7 - EGT
 * 8 - OilTemp
 * 9 - Boost
 * 10 - Coolant temp
*/

/*  FIS Hacker OBD
 *  display engine parameters on FIS 
 *  rememer to update the fimrware version in code! 
 *____________________________________________________________________________________
 *  
 * ---------------------- <<<<<   RELEASE NOTES   >>>>>  -----------------------------
 * ___________________________________________________________________________________
 * 
 * version 2.1  
 * Release changes:
 *  [ ] rebuild OBD readings
 *  - added oil pressure reading
 *  - added alarms
 * 
 * 
 * version 2.0  (HW changed)
 * Release changes:
 * - library change (CAN to mcp_can)
 * - control through multifunction steering wheel!
 * - added 2nd CAN transciever to remove multiplexing (HW change!)
 * - saving settings into EEPROM                                
 * - simple debugging via COM (any chracter sent via COM will turn on/off debug)
 * - fix (test) deactivating the display
 * - fix the calculations
 * - activate EEPROM in SETUP section
 * - fix issue with building/displaying data1 for screen1 (OBD logs https://docs.google.com/spreadsheets/d/1fXjdaKr_RC_L5Tx4Rh_TtjwmZb2dc1k03bhNyoVSwL0/edit#gid=311951710)
 * - OBD Readings only on 2nd screen 
 *  
 *  
 *  version 1.3 
 * Release changes:
 * - boost calculated instatnly (no average calculation)
 * - added OBD2 communication error indicator "---" when no/wrong reply from ECU
 * - changed CAN IDs for FIS display (phone) 
 * 
 * 
 * version 1.2
 * Release changes:
 * - Infotaiment can messages sent by 100ms
 * 
 * 
 * version 1.1
 * Release changes:
 * - boost calculated instatnly (no average calculation)
 * - added OBD2 communication error indicator "---" when no/wrong reply from ECU
 * 
*/
//====================================================================================================================================================

 /*
   * bitrate - bit rate in bits per seconds (bps) (1000E3, 500E3, 250E3, 200E3, 125E3, 100E3, 80E3, 50E3, 40E3, 20E3, 10E3, 5E3) - default was 500E3
   * 500E - Drivetrain CAN works ok
   * 100E - infotainment and komfort
  */
  
#include <mcp_can.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

//-----------------------------  Definicje pinow -----------------------------------
SoftwareSerial mySerial(3, 2); // RX, TX
#define ELM_PORT mySerial

#define BUZZER_PIN A2
#define BOOST_PIN A7
#define OILP_PIN A6
#define CAN_DRIVETRAIN_PIN 9   
#define CAN_INFOTAIMENT_PIN 7         //                                                              <<--------  sprawdzic piny z CANShield!!!!!!!
#define BUTTON_PIN  A0               //                                                              

//-----------------------------  Definicje settinsów -----------------------------------
#define OILP 0
#define STFT 1
#define LTFT 2
#define tADV 3
#define IAT 4
#define MAF 5
#define LBD 6
#define EGT 7
#define OILt 8
#define BST 9
#define TMP 10


//---------------------------  Definicje zmiennych ----------------------------------

//----------- zmienne do obslugi CAN  --------------
unsigned long rxId;
byte len;
byte rxBuf[8];
char data1[8];
char data2[8];
String OBD;
byte data_byte[7];


uint16_t loop_count; 

int16_t boost_ref, oil_adc;  //boost reference value
int16_t boost, oil_temp, coolant_temp, stft, ltft, tadv, iat, maf, lbd, egt, rpm;  //available parameters
int16_t row1_1, row1_10, row1_100, row2_1, row2_10, row2_100; //display values


char liczby[14] = {'0','1','2','3','4','5','6','7','8','9','-',' ','.','-'};
//char screen_char1[4];
//char screen_char2[4];

//0 - OilP, 1 - STFT, 2 - LTFT, 3 - tADV, 4 - IAT, 5 - MAF, 6 - LBD, 7 - EGT, 8 - OILt, 9 - Boost, 10 - Coolant temp, +100 - OFF
uint8_t f_screen1, f_screen2, f_settings;

uint8_t f_debug = 0;  // debugowanie
uint8_t f_OBD_read; //zmienna pomocnicza wskazujÄ…ca na aktywny odczyt z OBD
uint8_t mf_byte1, mf_byte2;  //obsĹ‚uga kierownicy MF

//zmienne do komunikacji z ELM327
byte inData;
char inChar;
String BuildINString="";
String WorkingString="";
int32_t A, B, oil_press1, oil_press2, oil_press3, oil_press4, oil_press5, oil_press6, oil_press;      //zmienione z int16_t (ze wzgl. na wartosci lbd i obliczenia OilT)

//komunikacja przez COM
byte inData_COM;
char inChar_COM;
String COM_String="";

MCP_CAN CAN0(CAN_DRIVETRAIN_PIN);
MCP_CAN CAN1(CAN_INFOTAIMENT_PIN);


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------  setup  ----------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup(){
  Serial.begin(9600);
  while (!Serial);
  
  ELM_PORT.begin(38400);//oryginalnie 115200, moje ELM laczylo sie na 38400

  pinMode(OILP_PIN, INPUT);
  pinMode(BOOST_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CAN_INFOTAIMENT_PIN, OUTPUT);
  pinMode(CAN_DRIVETRAIN_PIN, OUTPUT);
  digitalWrite(BUTTON_PIN, HIGH); //podciagniecie przycisku do VCC
  digitalWrite(BUZZER_PIN, LOW); 
  digitalWrite(CAN_INFOTAIMENT_PIN, LOW);
  digitalWrite(CAN_DRIVETRAIN_PIN, LOW);

  //----------------  Welcome screen and firmware version ----------------------------
  digitalWrite(BUZZER_PIN, HIGH); 
  Serial.println("FIS HACKER OBD v 2.1");
  digitalWrite(BUZZER_PIN, LOW); 

  
  //------------------  pobranie wartosci referencyjnej BOOST ------------------------
  boost_ref = analogRead(BOOST_PIN)/2;
  Serial.print("Reference Boost: ");
  Serial.println(boost_ref);

  //------------------------- aktywuj CAN DRIVETRAIN  --------------------------------
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN Drivetrain Started!");
    CAN0.setMode(MCP_NORMAL);
  }else Serial.println("Starting CAN Drivetrain failed!");


  //------------------------- aktywuj CAN INFOTAIMENT  ------------------------------
  if (CAN1.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN Infotaiment Started!");
    CAN1.setMode(MCP_NORMAL);
  }else Serial.println("Starting CAN Infotaiment failed!");

  //----------------------  pobranie danych z EEPROM  -------------------------------      
  f_settings = 1;
  f_screen1 = EEPROM.read(1);
  if(f_screen1 > 10) f_screen1 = 8;
  f_screen2 = EEPROM.read(2);
  if(f_screen2 > 10) f_screen2 = 9; 
  
  
  //-----------------------------  uruchom ELM ------------------------------------- 
  mySerial.println("ATZ");  //RESET_ALL
  Serial.println("Sending ATZ");
  delay(900);
  ReadData_OBD();
  Serial.println(BuildINString);
  mySerial.println("ATE0");  //ECHO_OFF
  Serial.println("Sending ATE0");
  delay(900);
  ReadData_OBD();
  Serial.println(BuildINString);
  mySerial.println("ATS0");  //PRINTING_SPACES_OFF
  Serial.println("Sending ATS0");
  delay(900);
  ReadData_OBD();
  Serial.println(BuildINString);
  mySerial.println("ATAL");  //ALLOW_LONG_MESSAGES
  Serial.println("Sending ATAL");
  delay(900);
  ReadData_OBD();
  Serial.println(BuildINString);
  mySerial.println("ATSP4");  //ALLOW_LONG_MESSAGES
  Serial.println("Sending ATSP4");
  delay(900);
  ReadData_OBD();
  Serial.println(BuildINString);

  digitalWrite(BUZZER_PIN, HIGH);
  Serial.println("====== Setup Done! ======"); //debug
  digitalWrite(BUZZER_PIN, LOW); 
  delay(70);
  digitalWrite(BUZZER_PIN, HIGH); 
  delay(70);
  digitalWrite(BUZZER_PIN, LOW);   
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------- main loop -----------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop(){
  
  //----------------------------------- odczyt CAN DRIVETRAIN  -------------------------------------
  CAN0.readMsgBuf(&rxId, &len, rxBuf);       // Read data: len = data length, buf = data byte(s)

  //---------------------------------- przeliczenie temp. oleju i plynu chlodniczego -----------------------------------
  if(rxId == 1056){
    if(f_debug == 1) Serial.print("ID 420 / 1056 received ");   
    coolant_temp = (((int)rxBuf[4]-64)*0.75);              
    oil_temp = (((int)rxBuf[3]-64)*0.75);
    if(f_debug == 1){
      //Serial.println(rxBuf); 
      Serial.print("Oil Temp: ");
      Serial.println(oil_temp);
    }    
  }

  //---------------------------------------------- przeliczenie RPM  ----------------------------------------------------
  if(rxId == 640){
    if(f_debug == 1) Serial.print("ID 420 / 1056 received ");   
    rpm = (int)rxBuf[3];              
  }    

  //----------------------------------- odczyt CAN INFOTAIMENT  -------------------------------------                                      
  CAN1.readMsgBuf(&rxId, &len, rxBuf);       // Read data: len = data length, buf = data byte(s)
  if(rxId == 1475){   
    mf_byte1 = rxBuf[0];
    mf_byte2 = rxBuf[1];  
    if(f_debug == 1 || f_debug == 2){                                             
      Serial.print("CAN Infotaiment Msg:");
      //Serial.println(rxBuf);
      Serial.print("bytes:");
      Serial.print(mf_byte1); 
      Serial.println(mf_byte2);  
    }
  }
    

  /*======================================================================================================================================================
   * =============================================== funkcje wykonywane tylko 1 na 1000 petli ============================================================
   * =====================================================================================================================================================
  */
  
  //-------------------------------------  odczyt BOOST z ADC  ----------------------------------------  
  if(loop_count == 5)  boost = ((analogRead(BOOST_PIN)/2)-boost_ref)/2;

  //-------------------------------------  odczyt OilPress z ADC  ----------------------------------------  
  if(loop_count == 450){
    oil_adc = analogRead(OILP_PIN);
    oil_press1 = 50*oil_adc;
    oil_press2 = oil_press1/1023;
    oil_press3 = 50-oil_press2;
    oil_press4 = 5000/oil_press3;
    oil_press5 = oil_press4-100;
    oil_press6 = oil_press5*100;
    oil_press = oil_press6/16;
  }

  //-------------------------------------- wyslanie PID do OBD ----------------------------------------                               <<<<----- DO SPRAWDZENIA!!!!
  if(loop_count == 25){
    if(f_OBD_read == 0){
      OBD = "01";
      /*switch(f_screen1){
        case 1: //STFT
        OBD += "06";
        break;

        case 2: //LTFT
        OBD += "07";
        break;

        case 3: //tADV
        OBD += "0E";       
        break;

        case 4: //IAT
        OBD += "0F";        
        break;

        case 5: //MAF
        OBD += "10";         
        break;

        case 6: //LBD
        OBD += "34";         
        break;

        default:
        break;        
      }
      */
      //if(f_screen2 != f_screen1){
        switch(f_screen2){
          case 1: //STFT  
          OBD += "06";
          break;
  
          case 2: //LTFT
          OBD += "07";
          break;
  
          case 3: //tADV
          OBD += "0E";       
          break;
  
          case 4: //IAT
          OBD += "0F";        
          break;
  
          case 5: //MAF
          OBD += "10";         
          break;

          case 6: //LBD
          OBD += "34";         
          break;
            
          default:
          break;        
        //}
      }
      if(f_screen1 < 7 || f_screen2 <7){
        mySerial.println(OBD);  
        if(f_debug == 1|| f_debug == 4){
          Serial.print("OBD Command sent: "); 
          Serial.println(OBD);    
        }
        f_OBD_read = 1; 
      }
    }
  }

  //------------------------------- odczyt z OBD  --------------------------------------------
  if(loop_count==600){
    ReadData_OBD();
  }

  //------------------------  przeliczenie odczytow z OBD ------------------------------------            <<<<<---------------  PRZEROBIĆ NA OBSLUGE 1 LUB 2 PIDOW  !!!!
  if(loop_count==800){
    if(BuildINString){
      //sprawdzenie poprawnosci paczki danych
      WorkingString = BuildINString.substring(0,2);  
      A = strtol(WorkingString.c_str(),NULL,16);      //convert hex to decimnal
      
      //----------- odebrano poprawnie PID (65 = 41 HEX)  ------------------
      if(A == 65){
        WorkingString = BuildINString.substring(2,4); 
        B = strtol(WorkingString.c_str(),NULL,16);    //convert hex to decimnal     
        WorkingString = BuildINString.substring(4,8); 
        A = strtol(WorkingString.c_str(),NULL,16);    //convert hex to decimnal
        if(f_debug == 1 || f_debug == 4){
          Serial.print("OBD PID: ");
          Serial.print(B);
          Serial.print(" value: ");
          Serial.println(A);                       
        }

        switch(B)
        {
          case 6: //STFT
          stft = ((A/1.28)-100);
          break;
  
  
          case 7: //LTFT
          ltft = (A/1.28)-100;        
          break;
  
  
          case 14: //Timing Adv
          tadv = (A/2)-64;         
          break;
  
  
          case 15: //IAT
          iat = A-40;        
          break;
  
  
          case 16: //MAF
          maf = A/100;
          break;
  
  
          case 52: //Lbd      
          lbd = 200*A/65536;      
          break;
  
          default:
          break;
        }
        f_OBD_read = 47;
        A=0;
        B=0;
        row2_100 = 10;
      }
      /*
      else {
        //nie odebrano poprawnie kodu OBD
        row1_100 = 13;
        row1_10 = 13;
        row1_1 = 13;
        
        row2_100 = 13;
        row2_10 = 13;
        row2_1 = 13;    
        
      }*/
    }
  }

  //--------------------------------  obsluga 1 rzedu wyswietlacza  -----------------------------------
  if(loop_count == 905)
  {
    switch(f_screen1)
    {
      case OILP:
        data1[0]='O';
        data1[1]='I';
        data1[2]='L';
        data1[3]='P';
        if(oil_press<0){
          row1_100 = -1*(oil_press/100);
          row1_10 = -1*((oil_press/10)+(row1_100*10));
          row1_1 = -1*(oil_press+(row1_100*100)+(row1_10*10));
          }
        else{
          row1_100 = oil_press/100;
          row1_10 = (oil_press/10)-(row1_100*10);
          row1_1 = oil_press-(row1_100*100)-(row1_10*10);
        }
        break; 
             
      case STFT:
        data1[0]='S';
        data1[1]='T';
        data1[2]='F';
        data1[3]='T';      
        if(row1_100 == 13) break;
        if(stft<0)
        {
          row1_100 = 10;   //znak -
          row1_10 = -1*(stft/10);
          row1_1 = -1*(stft+(row1_10*10));
        }
        else
        {
          row1_100 = 11; //spacja
          row1_10 = stft/10;
          row1_1 = stft-(row1_10*10);
        }
        break;


      case LTFT:
        data1[0]='L';
        data1[1]='T';
        data1[2]='F';
        data1[3]='T'; 
        if(row1_100 == 13) break;
        if(ltft<0)
        {
          row1_100 = 10;   //znak -
          row1_10 = -1*(ltft/10);
          row1_1 = -1*(ltft+(row1_10*10));
        }
        else
        {
          row1_100 = 11; //spacja
          row1_10 = ltft/10;
          row1_1 = ltft-(row1_10*10);
        }        
        break;


      case tADV:
        data1[0]='t';
        data1[1]='A';
        data1[2]='D';
        data1[3]='V';   
        if(row1_100 == 13) break;
        if(tadv<0)
        {
          row1_100 = 10;   //znak -
          row1_10 = -1*(tadv/10);
          row1_1 = -1*(tadv+(row1_10*10));
        }
        else
        {
          row1_100 = 11; //spacja
          row1_10 = tadv/10;
          row1_1 = tadv-(row1_10*10);
        }          
        break;
        

      case IAT:
        data1[0]='I';
        data1[1]='A';
        data1[2]='T';
        data1[3]=' ';
        if(row1_100 == 13) break; 
        if(iat<0)
        {
          row1_100 = 10;   //znak -
          row1_10 = -1*(iat/10);
          row1_1 = -1*(iat+(row1_10*10));
        }
        else
        {
          row1_100 = 11; //spacja
          row1_10 = iat/10;
          row1_1 = iat-(row1_10*10);
        }          
        break;
        

      case MAF:
        data1[0]='M';
        data1[1]='A';
        data1[2]='F';
        data1[3]=' ';
        if(row1_100 == 13) break;
        row1_100 = maf/100; 
        row1_10 = maf/10-(row1_100*10);
        row1_1 = maf-(row1_100*100)-(row1_10*10);
        break;

  
      case LBD:
        data1[0]='L';
        data1[1]='B';
        data1[2]='D';
        data1[3]=' ';
        if(row1_100 == 13) break;
        row1_100 = lbd/100; 
        row1_10 = (lbd/10)-(row1_100*10);
        row1_1 = lbd-(row1_100*100)-(row1_10*10);       
        break;


      case EGT:
        data1[0]='E';
        data1[1]='G';
        data1[2]='T';
        data1[3]=' ';  
        if(egt<0)
        {
          row1_100 = 10; //znak '-'
          row1_10 = -1*(egt/10);
          row1_1 = -1*(egt+(row1_10*10)); 
        } 
        else
        {
          row1_100 = egt/100; 
          row1_10 = (egt/10)-(row1_100*10);
          row1_1 = egt-(row1_100*100)-(row1_10*10); 
        }
        break; 


      case OILt:
        data1[0]='O';
        data1[1]='I';
        data1[2]='L';
        data1[3]='t';  
        if(oil_temp<0)
        {
          row1_100 = 10; //znak '-'
          row1_10 = -1*(oil_temp/10);
          row1_1 = -1*(oil_temp+(row1_10*10)); 
        } 
        else
        {
          row1_100 = oil_temp/100; 
          row1_10 = (oil_temp/10)-(row1_100*10);
          row1_1 = oil_temp-(row1_100*100)-(row1_10*10); 
        }
        break;        

        
      case BST:
        data1[0]='B';
        data1[1]='S';
        data1[2]='T';
        data1[3]=' ';
        if(boost<0){
          row1_100 = -1*(boost/100);
          row1_10 = -1*((boost/10)+(row1_100*10));
          row1_1 = -1*(boost+(row1_100*100)+(row1_10*10));
          }
        else{
          row1_100 = boost/100;
          row1_10 = (boost/10)-(row1_100*10);
          row1_1 = boost-(row1_100*100)-(row1_10*10);
        }
        break;
        
      case TMP:
        data1[0]='T';
        data1[1]='M';
        data1[2]='P';
        data1[3]=' ';  
        if(coolant_temp<0)
        {
          row1_100 = 10; //znak '-'
          row1_10 = -1*(coolant_temp/10);
          row1_1 = -1*(coolant_temp+(row1_10*10)); 
        } 
        else
        {
          row1_100 = coolant_temp/100; 
          row1_10 = (coolant_temp/10)-(row1_100*10);
          row1_1 = coolant_temp-(row1_100*100)-(row1_10*10); 
        }
        break;        
              
      default:
        break; 
    }
  }
    
  //------------------------------  obsluga 2 rzedu wyswietlacza  ------------------------------------
  if(loop_count == 915)
  {
    switch(f_screen2)
    {
      case OILP:
        data2[0]='O';
        data2[1]='I';
        data2[2]='L';
        data2[3]='P';
        if(oil_press<0){
          row2_100 = -1*(oil_press/100);
          row2_10 = -1*((oil_press/10)+(row2_100*10));
          row2_1 = -1*(oil_press+(row2_100*100)+(row2_10*10));
          }
        else{
          row2_100 = oil_press/100;
          row2_10 = (oil_press/10)-(row2_100*10);
          row2_1 = oil_press-(row2_100*100)-(row2_10*10);
        }
        break; 
              
      case STFT:
        data2[0]='S';
        data2[1]='T';
        data2[2]='F';
        data2[3]='T';       
        if(row2_100 == 13) break;
        if(stft<0)
        {
          row2_100 = 10;   //znak -
          row2_10 = -1*(stft/10);
          row2_1 = -1*(stft+(row2_10*10));
        }
        else
        {
          row2_100 = 11; //spacja
          row2_10 = stft/10;
          row2_1 = stft-(row2_10*10);
        }
        break;


      case LTFT:
        data2[0]='L';
        data2[1]='T';
        data2[2]='F';
        data2[3]='T'; 
        if(row2_100 == 13) break;
        if(ltft<0)
        {
          row2_100 = 10;   //znak -
          row2_10 = -1*(ltft/10);
          row2_1 = -1*(ltft+(row2_10*10));
        }
        else
        {
          row2_100 = 11; //spacja
          row2_10 = ltft/10;
          row2_1 = ltft-(row2_10*10);
        }        
        break;


      case tADV:
        data2[0]='t';
        data2[1]='A';
        data2[2]='D';
        data2[3]='V';      
        if(row2_100 == 13) break;
        if(tadv<0)
        {
          row2_100 = 10;   //znak -
          row2_10 = -1*(tadv/10);
          row2_1 = -1*(tadv+(row2_10*10));
        }
        else
        {
          row2_100 = 11; //spacja
          row2_10 = tadv/10;
          row2_1 = tadv-(row2_10*10);
        }          
        break;
        

      case IAT:
        data2[0]='I';
        data2[1]='A';
        data2[2]='T';
        data2[3]=' '; 
        if(row2_100 == 13) break;
        if(iat<0)
        {
          row2_100 = 10;   //znak -
          row2_10 = -1*(iat/10);
          row2_1 = -1*(iat+(row2_10*10));
        }
        else
        {
          row2_100 = 11; //spacja
          row2_10 = iat/10;
          row2_1 = iat-(row2_10*10);
        }          
        break;
        

      case MAF:
        data2[0]='M';
        data2[1]='A';
        data2[2]='F';
        data2[3]=' ';  
        if(row2_100 == 13) break;
        row2_100 = maf/100; 
        row2_10 = maf/10-(row2_100*10);
        row2_1 = maf-(row2_100*100)-(row2_10*10);
        break;
        

      case LBD:
        data2[0]='L';
        data2[1]='B';
        data2[2]='D';
        data2[3]=' ';  
        if(row2_100 == 13) break;   
        row2_100 = lbd/100; 
        row2_10 = (lbd/10)-(row2_100*10);
        row2_1 = lbd-(row2_100*100)-(row2_10*10);       
        break;


      case EGT:
        data2[0]='E';
        data2[1]='G';
        data2[2]='T';
        data2[3]=' ';  
        if(egt<0)
        {
          row2_100 = 10; //znak '-'
          row2_10 = -1*(egt/10);
          row2_1 = -1*(egt+(row2_10*10)); 
        } 
        else
        {
          row2_100 = egt/100; 
          row2_10 = (egt/10)-(row2_100*10);
          row2_1 = egt-(row2_100*100)-(row2_10*10); 
        }
        break;


      case OILt:
        data2[0]='O';
        data2[1]='I';
        data2[2]='L';
        data2[3]='t';  
        if(oil_temp<0)
        {
          row2_100 = 10; //znak '-'
          row2_10 = -1*(oil_temp/10);
          row2_1 = -1*(oil_temp+(row2_10*10)); 
        } 
        else
        {
          row2_100 = oil_temp/100; 
          row2_10 = (oil_temp/10)-(row2_100*10);
          row2_1 = oil_temp-(row2_100*100)-(row2_10*10); 
        }
        break;


      case BST:
        data2[0]='B';
        data2[1]='S';
        data2[2]='T';
        data2[3]=' ';      
        if(boost<0){
          row2_100 = -1*(boost/100);
          row2_10 = -1*((boost/10)+(row2_100*10));
          row2_1 = -1*(boost+(row2_100*100)+(row2_10*10));
          }
        else{
          row2_100 = boost/100;
          row2_10 = (boost/10)-(row2_100*10);
          row2_1 = boost-(row2_100*100)-(row2_10*10);
        }
        break;

      case TMP:
        data2[0]='T';
        data2[1]='M';
        data2[2]='P';
        data2[3]=' ';  
        if(coolant_temp<0)
        {
          row2_100 = 10; //znak '-'
          row2_10 = -1*(coolant_temp/10);
          row2_1 = -1*(coolant_temp+(row2_10*10)); 
        } 
        else
        {
          row2_100 = coolant_temp/100; 
          row2_10 = (coolant_temp/10)-(row2_100*10);
          row2_1 = coolant_temp-(row2_100*100)-(row2_10*10); 
        }
        break;

      default:
        break; 
    }
  }


  //-------------------------------------  obsluga portu COM (debug) --------------------------------------------
  /* _________________________________________________ 
   *   
   *   Debug modes:
   *   0 - disabled
   *   1 - ALL
   *   2 - MFSW control
   *   3 - FIS display status and messages
   *   4 - OBD Readings   
   *   5 - OilT debugging
   * _______________________________________________
   */
  
  if(loop_count == 990){
    ReadData_COM();
    if(COM_String!=0){
      if(f_debug >= 5) f_debug = 0;     
      else f_debug++;
    }
    COM_String="";
  }

  //---------------------------------------------- aktywacja modulu telefonu ---------------------------------------------
  if(loop_count == 1000){
    if(f_screen1/100 == 0){
      byte data_byte[] = {0x82, 0xb2, 0x00, 0x00, 0x00, 0x00, 0x10};
      CAN1.sendMsgBuf(0x665, 0, 7, data_byte);  //0x665 = 1637
      if(f_debug == 1 || f_debug == 2) Serial.println("Phone active");
    }else{
      byte data_byte[] = {0x0, 0xb2, 0x00, 0x00, 0x00, 0x00, 0x10};
      CAN1.sendMsgBuf(0x665, 0, 7, data_byte);  //turn off Phone module   
      if(f_debug == 1 || f_debug == 2)  Serial.println("Phone deactivated");                
    }
  }
  

  //-------------------------------------------------- wyslanie danych do FIS -----------------------------------------------
  if(loop_count>1100){   

    //--------------  1 linijka -------------------
    if(f_screen1 == OILP){
      data1[4] = liczby[11];  //spacja
      data1[5] = liczby[row1_100];
      data1[6] = liczby[12]; //znak kropki "."
      data1[7] = liczby[row1_10];
    }
    else{
      if(f_screen1 == LBD || f_screen1 == BST){
        data1[4] = liczby[row1_100];
        data1[5] = liczby[12]; //znak kropki "."
        data1[6] = liczby[row1_10];
        data1[7] = liczby[row1_1];
      } 
      else{
        data1[4] = liczby[11];  //spacja
        data1[5] = liczby[row1_100];
        data1[6] = liczby[row1_10];
        data1[7] = liczby[row1_1];
      }
    }
    CAN1.sendMsgBuf(0x265, 0, 8, data1);    

    //--------------  2 linijka ----------------

   if(f_screen2 == OILP){
      data2[4] = liczby[11];  //spacja
      data2[5] = liczby[row2_100];
      data2[6] = liczby[12];  //kropka
      data2[7] = liczby[row2_10];
    }
    else{
     if(f_screen2 == LBD || f_screen2 == BST){
        data2[4] = liczby[row2_100];
        data2[5] = liczby[12];  //kropka
        data2[6] = liczby[row2_10];
        data2[7] = liczby[row2_1];
      }
      else{
        data2[4] = liczby[11];  //spacja
        data2[5] = liczby[row2_100];
        data2[6] = liczby[row2_10];
        data2[7] = liczby[row2_1];
      }
    }
    
    CAN1.sendMsgBuf(0x267, 0, 8, data2);
    
    if(f_debug == 1 || f_debug == 3){
      Serial.print("Screen1: ");
      Serial.println(data1);      
      Serial.print("Screen2: ");
      Serial.println(data2);               
      Serial.print("f_screen values:");
      Serial.print(f_screen1);
      Serial.print(" ");
      Serial.println(f_screen2);           
    }

    //----------------------------------------  ALARMS  -----------------------------------------------
    if(oil_press<100 && rpm>0){
      digitalWrite(BUZZER_PIN, HIGH);
    }
    else digitalWrite(BUZZER_PIN, LOW);
    
    //--------------------------------  obsluga kierownicy MF -----------------------------------------
    
    //---------------  Mode button double press -------------------
    if((mf_byte1 == 57) && (mf_byte2 == 1)){          //39 01 - double press  
      if(f_screen1/100 == 0){
        f_screen1=f_screen1+100;
        byte data_byte[] = {0x0, 0xb2, 0x00, 0x00, 0x00, 0x00, 0x10};
        CAN1.sendMsgBuf(0x665, 0, 7, data_byte);  //turn off Phone module   
        if(f_debug == 1 || f_debug == 2)  Serial.println("Phone deactivated"); 
        delay(500);
      }
      else{
        f_screen1=f_screen1-100;
        delay(200);
      }
      EEPROM.write(1,f_screen1);
      if(f_debug == 1 || f_debug == 2){
        Serial.print("Screen: ");
        Serial.println(f_screen1);
      }
    }

    //---------------  Scroll button single press -------------------
    if(mf_byte2 == 26 || mf_byte2 == 167){ //3A 1A  lub 3B A7 - single press
     if(f_settings == 1) f_settings=2;
     else f_settings =1;
    }

    if(f_screen1/100 == 0){ //if FIS-MOD activated
      //--------  scroll up -------------
      if(mf_byte2 == 2 || mf_byte2 == 11){
       if(f_settings == 1){
         if(f_screen1 < 10) f_screen1++;  
         else f_screen1=7;
         row1_100 = 13;
         row1_10 = 13;
         row1_1 = 13;
         EEPROM.write(1,f_screen1);
         if(f_debug == 1 || f_debug == 2) Serial.println("Scroll UP screen1");
        }
       else if(f_settings ==2){
         if(f_screen2 < 10) f_screen2++;  
         else f_screen2=0;
         row2_100 = 13;
         row2_10 = 13;
         row2_1 = 13;
         EEPROM.write(1,f_screen2);
         if(f_debug == 1 || f_debug == 2) Serial.println("Scroll UP screen2");         
         }
       }
     
      //--------  scroll down -------------
      if(mf_byte2 == 3 || mf_byte2 == 12){
       if(f_settings == 1){
         if(f_screen1 > 7) f_screen1--;
         else f_screen1=10;
         EEPROM.write(2,f_screen1);
         row1_100 = 13;
         row1_10 = 13;
         row1_1 = 13;
         if(f_debug == 1 || f_debug == 2) Serial.println("Scroll DOWN screen1");
       }  
       else if(f_settings == 2){
         if(f_screen2 > 0) f_screen2--;
         else f_screen2=10;
         EEPROM.write(2,f_screen2);
         row2_100 = 13;
         row2_10 = 13;
         row2_1 = 13;
         if(f_debug == 1 || f_debug == 2) Serial.println("Scroll DOWN screen2");
        }     
      }
    }
    
    if(f_debug == 1|| f_debug == 5){
      Serial.print("OILp ADC: "); 
      Serial.println(oil_adc);  
      Serial.print("Oil_press: "); 
      Serial.println(oil_press1);    
      Serial.println(oil_press2); 
      Serial.println(oil_press3);
      Serial.println(oil_press4);  
      Serial.println(oil_press5); 
      Serial.println(oil_press6); 
      Serial.println(oil_press); 
    } 
    //----------- resetowanie licznikow pomocniczych  ----------------
    if(f_OBD_read>0)f_OBD_read++;
    if(f_OBD_read>50) f_OBD_read=0;           
    loop_count=0; 
  }
  
  loop_count++; //licznik 
}
//----------------------------------------------------------  loop end  -----------------------------------------------------------------------


//==================================================================================================================================================
//=======================================================    definicje funkcji   ===================================================================
//==================================================================================================================================================

void ReadData_OBD(){
  BuildINString="";  
  while(mySerial.available() > 0)
  {
    inData=0;
    inChar=0;
    inData = mySerial.read();
    inChar=char(inData);
    BuildINString = BuildINString + inChar;
  }
  if(inData > 0){
    if(f_debug == 1 || f_debug == 4){
      Serial.print("Received from OBD: ");
      Serial.println(BuildINString);
    }
  }
  inData=0;
}


void ReadData_COM(){
  COM_String = "";
  while(Serial.available() > 0)
  {
    inData_COM=0;
    inChar_COM=0;
    inData_COM = Serial.read();
    inChar_COM = char(inData_COM);
    COM_String = COM_String + inChar_COM;
  }
  if(f_debug == 1){
    Serial.print("Received from COM: ");
    Serial.println(COM_String);
  }
}
