//README
//default buffer sizes are too big leading to arduino running out of dynamic memory and misbehaving, to solve the problem change the following files to:
//arduino side file: Arduino/libraries/Rosserial_Arduino_library/src/ros.h ----->  typedef NodeHandle_<ArduinoHardware, 5, 5, 120, 120,FlashReadOutBuffer_> NodeHandle;
//ubuntu ros side file: catkin_ws/src/rosserial/rosserial_arduino/src/ros_lib -----> typedef NodeHandle_<ArduinoHardware, 5, 5, 120, 120> NodeHandle;

//since there will be two arduino button terminals remember to create two serial_node.py in directory /home/monti/catkin_ws/src/rosserial/rosserial_python/nodes,call them
//serial_node.py and serial_node2.py, launch two serial ports using the below command and change the python and port number, also make sure the sketch uploaded to each
//individual arduino calls its topics differently, ie term1_displayScreenX, term2_displayScreenX, term1_buttonPressed,term2_buttonPressed

//How to use:
//connect to arduino via Serial using : rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0
//to read the serial from arduino:      rostopic echo term1_buttonOut
//to send data to the arduino:          rostopic pub term1_screenIn std_msgs/UInt8 1 -1 
//replace first "1" with the unisgned 8 bit integer of your choise 

//Libraries used
#include <U8g2lib.h> //screen library
#include <ros.h>     //ROS libraries        
#include <std_msgs/UInt8.h> 


U8G2_ST7920_128X64_1_HW_SPI u8g2(U8G2_R0, /* CS=*/ 10, /* reset=*/ 8); //screen we are using with hardware SPI in mode 1 for less dynamic memory (aka static RAM = 2Kb) usage 


ros::NodeHandle nh;   
uint8_t rosInputNum = 0; 
void messageCb( const std_msgs::UInt8& msg)
{
  rosInputNum = msg.data;
}
std_msgs::UInt8 test;
ros::Subscriber<std_msgs::UInt8> s("term2_displayScreenX", &messageCb);  //subscribes to topic term1_displayScreenX and calls routine "messageCb" as soon as there are new data
ros::Publisher p("term2_buttonPressed", &test);                          //echoes/publishes out on "term1_buttounPressed" the data in "test"


//SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP 
//SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP 
//SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP--SETUP 

void setup() 
{
  u8g2.begin();   //start screen
  u8g2.setFont(u8g2_font_ncenB14_tr);
  
  pinMode(5,INPUT);  //setup buttons
  pinMode(4,INPUT);
  pinMode(3,INPUT);
  pinMode(2,INPUT);
  
  nh.initNode();      //initialise ROS nodes
  nh.advertise(p);    //sets up publisher
  nh.subscribe(s);    //sets up subscriber
}



//LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP
//LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP
//LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP--LOOP

void loop() 

{
  chooseDisplay(rosInputNum);
  sendButtonPressed();
  nh.spinOnce();
  delay(10);
}

//FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS 
//FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS 
//FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS--FUNCTIONS 

void sendButtonPressed()
{   
  static char previousButtonPressed = '0';
  char buttonPressed = returnButtonPressed();
   
  if (buttonPressed != previousButtonPressed)
  { 
    previousButtonPressed = buttonPressed;
    test.data = buttonPressed;
    p.publish( &test );
  }
}

void chooseDisplay(uint8_t inputNum)
{
  static uint8_t previousNum = 0;
  if (previousNum != inputNum)
  {
    previousNum = inputNum;
    switch(inputNum)
      {
        case 1: printChooseDrinksDisplay();     break;
        case 2: printPouringWaitDisplay();      break;
        case 3: printWantAnotherDrinkDisplay(); break;
        case 4: printEnjoyDrinkDisplay();       break;
      }
  }
    else{return;}
}

void printChooseDrinksDisplay()
{
  u8g2.firstPage();
  do{
      u8g2.setFontMode(1);  // activate transparent font mode 
      u8g2.setDrawColor(1); // color 1 for the box 
      u8g2.drawBox(0, 14, 10, 54);
      u8g2.setFont(u8g2_font_ncenB08_tr); //reduced font 'tr' to save memory space
      u8g2.setDrawColor(2);
      u8g2.drawStr(1, 9, "Choose your drink:");
      u8g2.drawHLine(0,12,128);
      u8g2.drawHLine(0,25,10);
      u8g2.drawHLine(0,38,10);
      u8g2.drawHLine(0,51,10);
      u8g2.drawHLine(0,64,10);
      u8g2.drawStr(1, 23, "1: Stella Artois");
      u8g2.drawStr(1, 36, "2: Heineken");
      u8g2.drawStr(1, 49, "3: Guiness");
      u8g2.drawStr(1, 62, "4: Non-Alcoholic");
    }while ( u8g2.nextPage() );
}

void printPouringWaitDisplay()
{
  u8g2.firstPage();
  do{
      u8g2.setFont(u8g2_font_ncenB18_tr);
      u8g2.drawFrame(4,8,120,48);
      u8g2.drawStr(10, 30, "Pouring");
      u8g2.setFont(u8g2_font_ncenB12_tr);
      u8g2.drawStr(9, 49, "please wait..."); 
    }while (u8g2.nextPage());
  delay(500);
}

void printWantAnotherDrinkDisplay()
{
  u8g2.firstPage();
  do{
      u8g2.setFontMode(1);  // activate transparent font mode 
      u8g2.setDrawColor(1); // color 1 for the box 
      u8g2.drawBox(0, 39, 10, 45);
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.setDrawColor(2);
      u8g2.drawStr(1, 9, "Would you like");
      u8g2.drawStr(1, 20, "another drink?");
      u8g2.drawHLine(0,23,128);
      u8g2.drawHLine(0,51,10);
      u8g2.drawHLine(0,64,10);
      u8g2.drawStr(1, 49, "3: Yes, please");
      u8g2.drawStr(1, 62, "4: No, thank you");
    }while ( u8g2.nextPage() );

}

void printEnjoyDrinkDisplay()
{
  u8g2.firstPage();
  do{
      u8g2.setFont(u8g2_font_ncenB18_tr);
      u8g2.drawFrame(4,8,120,48);
      u8g2.drawStr(27, 30, "Enjoy");
      u8g2.setFont(u8g2_font_ncenB12_tr);
      u8g2.drawStr(14, 49, "responsibly");
      u8g2.sendBuffer();          // transfer internal memory to the display
    }while ( u8g2.nextPage() );
}

char returnButtonPressed()
{
  if (digitalRead(5) == LOW){return 1;}
  else if (digitalRead(4) == LOW){return 2;}
  else if (digitalRead(3) == LOW){return 3;}
  else if (digitalRead(2) == LOW){return 4;}
  else return 0;
}
