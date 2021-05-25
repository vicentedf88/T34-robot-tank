#include <SR04.h> //librería del sensor de ultra sonidos
#include <Servo.h>
#include <SoftwareSerial.h>

#define VEL_CONVERSION_RATIO 0.22 //empiric data

//ISR
#define FrecISR 7812.5 //Hz
float const Sampling_time = 0.5;

//BLUETOOTH
#define RX 2
#define TX 3
SoftwareSerial BT(RX,TX); //RX, TX
volatile char message = 0;
//TANK TURRET
#define TURRET A3
#define SHOT A2
//MOTOR L (B)
#define IN3  7    // Input3 conectada al pin X
#define IN4  6    // Input4 conectada al pin X 
#define ENB  5    // ENB conectada al pin X de Arduino
//MOTOR R (A)
#define IN1  9    // Input1 conectada al pin X
#define IN2  8    // Input2 conectada al pin X 
#define ENA  10    // ENA conectada al pin X de Arduino
//ULTRASONIC SENSOR
#define TRIG_1 11 //front sensor 
#define ECHO_1 12
#define TRIG_2 A1 //rear sensor 
#define ECHO_2 A0
SR04 front_sensor = SR04(ECHO_1, TRIG_1); 
SR04 rear_sensor = SR04(ECHO_2, TRIG_2);
float front_distance = 0;
float rear_distance = 0;
float min_obj_distance = 25; //MIN OBJETC DISTANCE
float front_distances[] = {50, 50, 50, 50, 50};
float rear_distances[] = {50, 50, 50, 50, 50};

int i=0;
float pos_x=0;
float pos_y=0;
float orientation=0;
float B=1; //Parameter: fictitius distance between wheels

//INITIALIZATION AND CONSTANTS TANK TURRET
int const CENTER = 100; //cañon alineado dirección tanque a 100º
int pos_cannon= CENTER;
Servo servo;

//INITIALIZATION AND CONSTANTS MOTOR
int vel_l=0;
int vel_r=0;

bool const FRONT = 0;
bool const REAR = 1;
bool const LEFT = 1;
bool const RIGHT = 0;
int const FAST = 255; //velocidad máxima
int const SLOW = 160; //velocidad mínima (por debajo los motores no funcionan)
int const STOP = 0; //motor parado

int STATE = 0;
volatile bool MANUAL = false;

//TIMER CLASS
class Timer{
  public:
    volatile int count=0;
    int limit_time=0;
    volatile bool count_enable=false;
    
    Timer(float t_sample);
    Timer();
    int segToISR(float seg);
    float ISRToseg(int isr);
    void init(){count_enable=true;}
    void init(float t_sample);
    bool counting();
    float time_() {return ISRToseg(limit_time);}
};

Timer::Timer(float t_sample){
  limit_time=segToISR(t_sample);
}
void Timer:: init(float t_sample){
  count_enable=true;
  limit_time=segToISR(t_sample);
}
int Timer::segToISR(float seg){ 
  return(int(seg/((1/FrecISR)*255)));
}
float Timer::ISRToseg(int isr){ 
  return((float(isr)*255)/FrecISR);
}
bool Timer::counting(){
  if(count_enable){
    count++;
    if(count>limit_time){
      count=0;
      return true;
    }
    else return false;
  } 
  else return false;  
}

//TIMER INSTANTIATION 
Timer timerUS(Sampling_time);
Timer timerBT(0.1);
Timer timerMOV(0.2);



void setup() {
//ISR
 SREG = (SREG & 0b01111111); //Desabilitar interrupciones
 TIMSK2 = TIMSK2|0b00000001; //Habilita la interrupcion por desbordamiento
 TCCR2B = 0b00000111; //Configura preescala para que FT2 sea de 7812.5Hz. T interrupción = T * 255 = 0.125 uS *255= 31.875 uS
 SREG = (SREG & 0b01111111) | 0b10000000; //Habilitar interrupciones //Desabilitar interrupciones

 timerMOV.init();
 timerUS.init();
 timerBT.init();
//TANK TURRET
  servo.attach(TURRET);
  servo.write(pos_cannon);
  pinMode (SHOT, OUTPUT); 
//MOTOR L (B)
   pinMode (ENB, OUTPUT); 
   pinMode (IN3, OUTPUT);
   pinMode (IN4, OUTPUT);
   direction_(FRONT,LEFT);
   velocity(STOP, LEFT);
//MOTOR R (A) 
   pinMode (ENA, OUTPUT); 
   pinMode (IN1, OUTPUT);
   pinMode (IN2, OUTPUT);
   direction_(FRONT,RIGHT);
   velocity(STOP, RIGHT);
//ULTRASONIC
   pinMode(TRIG_1, OUTPUT);
   pinMode(ECHO_1, INPUT_PULLUP);
   pinMode(TRIG_2, OUTPUT);
   pinMode(ECHO_2, INPUT_PULLUP);
   front_distance = front_sensor.Distance();
   rear_distance = rear_sensor.Distance();
   Serial.begin(9600);
//BLUETOOTH
   BT.begin(9600);

   delay(3000);
   digitalWrite(SHOT, HIGH); 
   delay(1000);
   digitalWrite(SHOT, LOW); 
}

void loop() {
   
if(MANUAL) STATE=4;//manual bt control

else{
if(!frontDetection() and !rearDetection()) STATE=0; //none

if(frontDetection() and !rearDetection()) STATE=1;//front

if(!frontDetection() and rearDetection()) STATE=2;//rear

if(frontDetection() and rearDetection()) STATE=3;//both
}


switch(STATE){
  case 0: 
             {   direction_(FRONT,LEFT);
                velocity(FAST, LEFT);
                direction_(FRONT, RIGHT);
                velocity(FAST, RIGHT);}
    break;
  case 1:    {   direction_(REAR,LEFT);
                velocity(FAST, LEFT);
                direction_(REAR, RIGHT);
                velocity(FAST, RIGHT);}
    break;
  case 2:     {  direction_(FRONT,LEFT);
                velocity(FAST, LEFT);
                direction_(FRONT, RIGHT);
                velocity(FAST, RIGHT);}
    break;
  case 3:  {     direction_(REAR,LEFT);
                velocity(FAST, LEFT);
                direction_(FRONT, RIGHT);
                velocity(FAST, RIGHT);}
    break;
  case 4: bt_control();
    break;
  default: Serial.println("Error en el control");
    break;
  
}
  /*
if(pos_x>50){
  velocity(FAST, RIGHT);
  velocity(FAST, LEFT);
 }
else{
  velocity(STOP, RIGHT);
  velocity(STOP, LEFT);
  /*Serial.print("Posición x = ");
  Serial.println();*/
 }
//FUNCTIONS
void direction_(int order, int motor) { // front/rear, left/right
  if(motor) { //LEFT
    if(order) { //FRONT
      digitalWrite (IN3, HIGH);
      digitalWrite (IN4, LOW);
    }
    else { //REAR
      digitalWrite (IN3, LOW);
      digitalWrite (IN4, HIGH);
    }
  }
  else { //RIGHT
    if(order) {//FRONT
      digitalWrite (IN2, HIGH);
      digitalWrite (IN1, LOW);
    }
    else { //REAR
      digitalWrite (IN2, LOW);
      digitalWrite (IN1, HIGH);
    }
  }

}

//
void velocity(int order, int motor) { // fast/slow, left/right
  if(motor) {//LEFT
    analogWrite(ENB,order);
    vel_l=order;
  }
    
  else{  //RIGHT
   analogWrite(ENA,order);
   vel_r=order;
  }

}
//
float vel_conversion (int vel_motor){ //Esta función convierte el valor digital de velociadad del motor (de 0 a 255) en velocidad lineal en cm/s
  return(float(vel_motor)*VEL_CONVERSION_RATIO);
}
//
int micros_(float t){
  return(int(t*1000));
}
//
void movement(){

  pos_x +=((vel_conversion(vel_l)+vel_conversion(vel_r))/2)*timerMOV.time_()*cos(orientation);
  pos_y +=((vel_conversion(vel_l)+vel_conversion(vel_r))/2)*timerMOV.time_()*sin(orientation);
  orientation +=((vel_conversion(vel_l)-vel_conversion(vel_r))/B)*timerMOV.time_();
  /*
  Serial.print("pos x= ");
  Serial.println(pos_x);
  Serial.print("pos y= ");
  Serial.println(pos_y);
  Serial.print("orientation= ");
  Serial.println(orientation);*/
}

//ISR FUNCTIONS
//
int segToISR(float seg){
  return(int(seg/((1/FrecISR)*255)));
}
//INTERRUPTION  
ISR(TIMER2_OVF_vect){
    
    if(timerUS.counting()) { //timer for US sampling
     front_distance = front_sensor.Distance();
     rear_distance = rear_sensor.Distance();
     

     filter (front_distances,&front_distance,250,0);//elimina las medidas fuera de rango
     filter (rear_distances,&rear_distance,250,0);//elimina las medidas fuera de rango
 
     distanceStorage (front_distances,&front_distance);
     distanceStorage (rear_distances,&rear_distance);

    /* Serial.print("Distancia FRONTAL= ");
     Serial.println(front_distance);

     Serial.print("Distancia TRASERA= ");
     Serial.println(rear_distance);*/
    }
    if(timerBT.counting()){
      
     read_bluetooth();
    } 
    if(timerMOV.counting()){
      movement();
    }   
}
//BLUETOOTH
void read_bluetooth(){
if (BT.available())
  {
    message=(char)BT.read();
    Serial.println(message);
    if(message == 'M') MANUAL = true;  
    if(message == 'A') MANUAL = false; 

   

  }
else message = 0;
}

void bt_control(){
     switch(message)
    {  
      case 'M': Serial.println("Modo manual activado");
                break;
      case 'A': Serial.println("Modo manual desactivado");
                break;
      case 'K': shot();
                break;     
      case '1': servo.write(180); //L
                break;
      case '2': servo.write(100);
                break; 
      case '3': servo.write(0);//R
                break;
      case 'L': velocity(STOP, LEFT);
                direction_(FRONT,RIGHT);
                velocity(FAST, RIGHT);
                break;
      case 'R': velocity(STOP, RIGHT);
                direction_(FRONT, LEFT);
                velocity(FAST, LEFT);
                break;
      case 'F': direction_(FRONT,LEFT);
                velocity(FAST, LEFT);
                direction_(FRONT, RIGHT);
                velocity(FAST, RIGHT);
                break;
      case 'B': direction_(REAR,LEFT);
                velocity(FAST, LEFT);
                direction_(REAR, RIGHT);
                velocity(FAST, RIGHT);
                break;
      case 'S': velocity(STOP, LEFT);
                velocity(STOP, RIGHT);
                break;
   
      default: break;
    }
}
//SHOT
void shot(){ //default function
  digitalWrite(SHOT, HIGH);
  delay(500);
  digitalWrite(SHOT, LOW);
}
void shot(int t){ //customizable
  digitalWrite(SHOT, HIGH);
  delay(t);
  digitalWrite(SHOT, LOW);
}
//SENSORS
void distanceStorage (float vector[], float *distancia){
  vector[4]= vector[3];
  vector[3]= vector[2];
  vector[2]= vector[1];
  vector[1]= vector[0];
  vector[0]= *distancia;
}
void filter (float vector[], float *distancia, float lim_sup,float lim_inf){ //la k le da valor a la medida. Si es 0 no la tenemos en cuenta, si es 1 es como una muestra más. También se puede sobreponderar
  if(*distancia>lim_sup){
      float accum=0;
      int n=5;
      for(int i = 0; i<n; i++){
        accum += vector[i];
      }
      *distancia= (accum)/float(n);
    }
  
  if(*distancia<=lim_inf)
  Serial.println("Error en el sensor");
}
bool frontDetection (){
  if(front_distance<40) return true;
   else return false;
  }

 bool rearDetection (){
  if(rear_distance<40) return true;
   else return false;
  }
