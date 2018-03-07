//Este programa ejecuta la estación terrena de DC con el puente H L298N
//Mide la posición de la estación con el Acelerómetro y Giroscopio MPU6050
//Orienta la estación al norte automáticamente con el Magnetómetro MAG3110

/*       Componente ======> Arduino UNO
 * ===MPU6050===AZ        * ===MPU6050===EL
 * 1 vCC -------> 5Vc     * 1 Vcc -------> 5Vc
 * 2 GND -------> GND     * 2 GND -------> GND
 * 3 SCL -------> A5      * 3 SCL -------> A5
 * 4 SDA -------> A4      * 4 SDA -------> A4
 * 5 AD0 -------> *no     * 5 AD0 -------> 5Vc
 * 6 INT -------> 8       * 6 INT -------> 10
 * ===L298N (Pte-H)===    * === MAG3110 ===
 * 1 IN1 -------> D8 AZ   * 1 GND -------> GND
 * 2 IN2 -------> D3 AZ   * 2 Vcc -------> 3V3
 * 3 ENA -------> D5 AZ   * 3 SDA -------> A4
 * 4 IN3 -------> D4 EL   * 4 SCL -------> A5
 * 5 IN4 -------> D7 EL   * 5 INT -------> *no
 * 6 ENB -------> D6 EL
 * ===LEDS INDICADORES===
 * 1 AZ -------> D9
 * 2 EL -------> D11
 * 3 MAG ------> D13
 */

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <SparkFun_MAG3110.h>

/* Definicion del puente H L298N*/
//**********Azimuth**********
#define ENA 5 //Velocidad PWM
#define IN1 8 //1 MOTOR 1
#define IN2 3 //2 MOTOR 1
//*********Elevación*********
#define ENB 6 //Velocidad PWM
#define IN3 4 //1 MOTOR 2
#define IN4 7 //2 MOTOR 2
/* Limites para el control de la señal */
#define outMax 120
#define outMin 35
#define Deadband 1
/* Angulo Maximo para homing scanning */
#define SEEK_MOVE 30
#define MIN_AZ_ANGLE 0
#define MAX_AZ_ANGLE 370
#define MIN_EL_ANGLE 0
#define MAX_EL_ANGLE 110
/* Tiempo para deshabilitar los motores, en milisegundos */
#define T_DELAY 1000
/* Configuracion del puerto serie */
#define BufferSize 256
#define BaudRate 19200
/*Variables globales*/
unsigned long t_DIS = 0; /*Tiempo para deshabilitar Motores*/
/* offset del angulo */
double AZcero[2], offsetAZ; //offset azimuth
/* Encoder */   
//**********Azimuth**********  ==AD0 en alto para 0x069
#define AZ_ADDRESS 0x69
#define INTERRUPT_PIN_AZ 9
#define LED_PIN_AZ 10
//*********Elevación*********
#define EL_ADDRESS 0x68
#define INTERRUPT_PIN_EL 11
#define LED_PIN_EL 12
//==================================================
//**********Azimuth**********
MPU6050 mpu_AZ(AZ_ADDRESS);  //0x68 defalt
//==================================================
bool dmpReady_AZ = false; 
uint8_t mpuIntStatus_AZ;  
uint8_t devStatus_AZ;     
uint16_t packetSize_AZ;   
uint16_t fifoCount_AZ;    
uint8_t fifoBuffer_AZ[64];
//==================================================
//Opreciones vectoriales de la libreria MPU60 
Quaternion q_AZ;           // [w, x, y, z]         quaternion
VectorInt16 aa_AZ;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal_AZ;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld_AZ;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity_AZ;    // [x, y, z]            gravity vector
//*********Elevación*********
MPU6050 mpu_EL(EL_ADDRESS);  //0x69 asignada (conectar AD0 a Vcc)
//==================================================
bool dmpReady_EL = false; 
uint8_t mpuIntStatus_EL;  
uint8_t devStatus_EL;     
uint16_t packetSize_EL;   
uint16_t fifoCount_EL;    
uint8_t fifoBuffer_EL[64];
//==================================================
//Opreciones vectoriales de la libreria MPU60 
Quaternion q_EL;           // [w, x, y, z]         quaternion
VectorInt16 aa_EL;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal_EL;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld_EL;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity_EL;    // [x, y, z]            gravity vector
bool blinkState = false;    
/* MAGNETOMETRO */  
MAG3110 mag = MAG3110();
bool calibrationDone = false;
double encabezado;
#define led_cal 13

/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
//FUNCION INICIAL
void setup() {
    // Puente H
  pinMode(OUTPUT, ENA);
  pinMode(OUTPUT, IN1); //1 MOTOR 1
  pinMode(OUTPUT, IN2); //2 MOTOR 1
  pinMode(OUTPUT, ENB);
  pinMode(OUTPUT, IN3); //1 MOTOR 2
  pinMode(OUTPUT, IN4); //2 MOTOR 2
    // Comuicacion Serial
  Serial.begin(BaudRate);
    // MPU6050
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE    //Agregar bus I2C
      Wire.begin();
      Wire.setClock(400000);                          //400kHz I2C clock
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  Wire.begin();                       //comunicacion I2C
  pinMode(LED_PIN_AZ, OUTPUT);
  pinMode(LED_PIN_EL, OUTPUT);
//**********Azimuth**********
  mpu_AZ.initialize();
  devStatus_AZ = mpu_AZ.dmpInitialize();//DMP
  pinMode(INTERRUPT_PIN_AZ, INPUT);
   // Calibración del sensor
   mpu_AZ.setXGyroOffset(220);
   mpu_AZ.setYGyroOffset(76);
   mpu_AZ.setZGyroOffset(-85);
   mpu_AZ.setZAccelOffset(1788); 
   //verificar que el sensor este activado
   if (devStatus_AZ == 0)
   {
      mpu_AZ.setDMPEnabled(true);
      //attachInterrupt(0, dmpDataReady, RISING);
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_AZ), dmpDataReady, RISING);
      mpuIntStatus_AZ = mpu_AZ.getIntStatus(); //obtener el estado del sensor  
      dmpReady_AZ = true;                           
      packetSize_AZ = mpu_AZ.dmpGetFIFOPacketSize();
   }
//*********Elevación*********
  mpu_EL.initialize();
  devStatus_EL = mpu_EL.dmpInitialize();//DMP
  pinMode(INTERRUPT_PIN_EL, INPUT);
   // Calibración del sensor
   mpu_EL.setXGyroOffset(220);
   mpu_EL.setYGyroOffset(76);
   mpu_EL.setZGyroOffset(-85);
   mpu_EL.setZAccelOffset(1788); 
   //verificar que el sensor este activado
   if (devStatus_EL == 0)
   {
      mpu_EL.setDMPEnabled(true);
      attachInterrupt(0, dmpDataReady, RISING);
      //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_EL), dmpDataReady, RISING);
      mpuIntStatus_EL = mpu_EL.getIntStatus();
      dmpReady_EL = true;                           
      packetSize_EL = mpu_EL.dmpGetFIFOPacketSize();
   }
      //MAGNETOMETRO
  pinMode(led_cal, OUTPUT);                 // Led de calibracion
  mag.initialize();                             // Inicializar MAG3110
  mag.enterCalMode();                           // Inicializar calibracion
}   //Fin de la función inicial

/*********************************************************************************************************/
/*********************************************************************************************************/
//FUNCION PRINCIPAL
void loop() 
{
  double *set_point;      //lleva puntero (se utiliza en varias funciones)
  double curr_pos[2];     //guarda la posicion actual del rotor

  // Leer comandos del puerto Serie
  set_point = cmd_proc();  
  // Mover Motores
  dc_move(set_point);
  // Revisar Tiempo
  if (t_DIS == 0)
    t_DIS = millis();
  // Alinear al norte
  encabezado =  norte(); 
  delay(10);
  get_position(curr_pos);
  delay(10);
  curr_pos[0] -= encabezado;  //double az_corregido=curr_pos[0]-encabezado;
  if (curr_pos[0] < 0)        //if (az_corregido < 0)
    curr_pos[0] += 360;       //az_corregido += 360;
  curr_pos[1]=curr_pos[1];

  //Desactivar motores
  if ( (abs(curr_pos[0]-set_point[0]) < Deadband) || (abs(curr_pos[0]-set_point[0]) >= 358) && millis()-t_DIS > T_DELAY) //detener azimuth
  //if (abs(curr_pos[0]-set_point[0]) < Deadband && millis()-t_DIS > T_DELAY) //detener azimuth
  {
    StopAZ();
  }
  if (abs(curr_pos[1]-set_point[1]) < Deadband && millis()-t_DIS > T_DELAY) //detener elevacion
    StopEL();

//  MOSTRAR la posicion en el monitor serie
//  Serial.print("Az");
//  Serial.print(curr_pos[0],1);
//  Serial.print(" ");
//  Serial.print("EL");
//  Serial.print(curr_pos[1],1);
//  Serial.print("Norte ");
//  Serial.println(encabezado,1);  
//  delay(50);
    
}   //Fin de la función principal

/*********************************************************************************************************/
/*********************************************************************************************************/
//DETENER MOTORES
void StopAZ()
{
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, 0); //Velocidad motor A
  delay(50);
}
void StopEL()
{
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);
  analogWrite (ENB, 0); //Velocidad motor A
}

/*********************************************************************************************************/
/*********************************************************************************************************/
//FUNCION DE INTERRUPCION
volatile bool mpuInterrupt = false;
void dmpDataReady() 
{
   mpuInterrupt = true;
}   //Fin de la interrupción

/*********************************************************************************************************/
/*********************************************************************************************************/
//FUNCION DE POSICION
void get_position(double *new_pos) 
{ 
  double az[2];
  double el[2];
  //obtener azimuth
  get_azimuth(az);
  //obtener elevacion
  get_elevacion(el);
  //Asignacion para la funcion
  new_pos[0]=az[0]; //azimuth de la funcion1
  new_pos[1]=el[1]; //elevacion de la funcion2
}//Fin del get_position

/*********************************************************************************************************/
/*********************************************************************************************************/
//FUNCION DE AZIMUTH
void get_azimuth(double *pos_az) 
//double get_azimuth()
{   
  static double in_pos[]={0,0};
  static double real_pos[]={0,0};
  float euler_AZ[3];         // [psi, theta, phi]    Euler angle container
  float ypr_AZ[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  
   //if (!dmpReady_AZ) return;  //si el sensor no esta activado, saldra de esta funcion
 
   mpuInterrupt = false;                                //funcion de interrupcion (la misma para ambos)
   mpuIntStatus_AZ = mpu_AZ.getIntStatus();                   //registra estado de entrada
   fifoCount_AZ = mpu_AZ.getFIFOCount();                      //contador del buffer FIFO
 
   // Comprobar desbordamiento
   if ((mpuIntStatus_AZ & 0x10) || fifoCount_AZ == 1024)         //si hay desbordamiento
   {
      // Reset FIFO
      mpu_AZ.resetFIFO();
   }
   else if (mpuIntStatus_AZ & 0x02)                          //si no hay desbordamiento
   {
      // Esperar datos
      while (fifoCount_AZ < packetSize_AZ) fifoCount_AZ = mpu_AZ.getFIFOCount();
 
      // Leer paquete de FIFO
      mpu_AZ.getFIFOBytes(fifoBuffer_AZ, packetSize_AZ);
      fifoCount_AZ -= packetSize_AZ;                      //fifoCount = fifoCount - packetSize;
   
      // Calcular angulos euler
      mpu_AZ.dmpGetQuaternion(&q_AZ, fifoBuffer_AZ);           //quaternion
      mpu_AZ.dmpGetGravity(&gravity_AZ, &q_AZ);                //euler (degrees)
      mpu_AZ.dmpGetYawPitchRoll(ypr_AZ, &q_AZ, &gravity_AZ);
      
      // Calcular angulos Azimuth y Elevacion        //Los ° estan de 0.0 a 179.9 y -180.0 a -0.1
      in_pos[0]=ypr_AZ[0] * 180 / M_PI ;   //Azimuth
      in_pos[1]=ypr_AZ[1] * 180 / M_PI ;   //Elevacion
      
      //Ajusta los angulos 
                  //Azimuth de 0.0 a 360.0
      if (in_pos[0] <= -179 && in_pos[0] > -181) in_pos[0] = -180;      //ajustar 180°
      
      if (in_pos[0] < 0) real_pos[0] = in_pos[0] + 360 ;
      else real_pos[0] = in_pos[0];                                //Azimuth
      
      if (in_pos[0] < 1) in_pos[0] = 0;
      else if (in_pos[0] > 359) in_pos[0] = 360;  //0 o 360
      
                  //Elevacion de 0.0 90.0
      if (in_pos[1] < 0) real_pos[1] = 0;
      else if (in_pos[1] > 90) real_pos[1] = 90;
//      else real_pos[1] = in_pos[1];  
//      if (in_pos[1] <= 0) real_pos[1] = in_pos[1] + 360 ;    
//      else real_pos[1] = in_pos[1];                              //Elevación

                        //Elevacion de 0.0 a 180.0
      

      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN_AZ, blinkState); 
   }
      //Asignacion para la funcion
      pos_az[0]=real_pos[0];
      pos_az[1]=real_pos[1];
      //return real_pos[0];
}//Fin get_azimuth

/*********************************************************************************************************/
/*********************************************************************************************************/
//Función de elevación
void get_elevacion(double *pos_el) 
//double get_elevacion()
{   
  static double in_pos[]={0,0};
  static double real_pos[]={0,0};
  float euler_EL[3];         // [psi, theta, phi]    Euler angle container
  float ypr_EL[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  
   //if (!dmpReady_EL) return;  //si el sensor no esta activado, saldra de esta funcion
 
   mpuInterrupt = false;                                //funcion de interrupcion (la misma para ambos)
   mpuIntStatus_EL = mpu_EL.getIntStatus();                   //registra estado de entrada
   fifoCount_EL = mpu_EL.getFIFOCount();                      //contador del buffer FIFO
 
   // Comprobar desbordamiento
   if ((mpuIntStatus_EL & 0x10) || fifoCount_EL == 1024)         //si hay desbordamiento
   {
      // Reset FIFO
      mpu_EL.resetFIFO();
   }
   else if (mpuIntStatus_EL & 0x02)                          //si no hay desbordamiento
   {
      // Esperar datos
      while (fifoCount_EL < packetSize_EL) fifoCount_EL = mpu_EL.getFIFOCount();
 
      // Leer paquete de FIFO
      mpu_EL.getFIFOBytes(fifoBuffer_EL, packetSize_EL);
      fifoCount_EL -= packetSize_EL;                      //fifoCount = fifoCount - packetSize;
   
      // Calcular angulos euler
      mpu_EL.dmpGetQuaternion(&q_EL, fifoBuffer_EL);           //quaternion
      mpu_EL.dmpGetGravity(&gravity_EL, &q_EL);                //euler (degrees)
      mpu_EL.dmpGetYawPitchRoll(ypr_EL, &q_EL, &gravity_EL);
      
      // Calcular angulos Azimuth y Elevacion        //Los ° estan de 0.0 a 179.9 y -180.0 a -0.1
      in_pos[0]=ypr_EL[0] * 180 / M_PI ;   //Azimuth
      in_pos[1]=ypr_EL[1] * 180 / M_PI ;   //Elevacion
      
      //Ajusta los angulos 
                  //Azimuth de 0.0 a 359.9
      if (in_pos[0] <= 0) real_pos[0] = in_pos[0] + 360 ;    
      else real_pos[0] = in_pos[0];                                //Azimuth
                  //Elevacion de 0.0 359.9
//      if (in_pos[1] <= 0) real_pos[1] = in_pos[1] + 360 ;    
//      else real_pos[1] = in_pos[1];                                //Elevación
                  //Elevacion de 0.0 a 90.0
      if (in_pos[1] < 0) real_pos[1] = 0;
      else if (in_pos[1] > 90) real_pos[1] = 90;
      else real_pos[1] = in_pos[1];                                //Elevacion

      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN_EL, blinkState);      
   }
      //Asignacion para la funcion
      pos_el[0]=real_pos[0];
      pos_el[1]=real_pos[1];
      //return real_pos[1];    
}//Fin del get_elevacion


/*********************************************************************************************************/
/*********************************************************************************************************/
//FUNCION DEL MAGNETOMETRO
double norte ()
{
  bool nor = false;
  do
  {
   digitalWrite(led_cal, LOW);   // apaga el led de calibracion
   //static int heading = 0;           // inicializar a 0 el marcador del norte
   /*MODO CALIBRACION*/
   while (mag.isCalibrating())    
    {
       digitalWrite(led_cal, HIGH);         // apaga el led de calibracion
      mag.calibrate();                              // Funcion de calibracion
      //Serial.println("calibrando ...");
    }
    /*CALIBRACION EXITOSA*/
    while (mag.isCalibrated() && !calibrationDone)
    {
      calibrationDone = true;
      //Serial.println("calibrado");
    }
    /*BRUJULA*/
    while (mag.isCalibrated())
    { 
      static int heading = 0;      
      // definir el marcador del norte
      heading = (int)mag.readHeading();       //el eje x del dispositivo debe estar alineado al Azimuth 0 de la estacion
      //medir grados de 0° a 360° (de lo contrario se miden en -180 y 180)
      if (heading < 0)                        //Si el marcador es negativo(valor entre -1 to -180)
        heading *= -1;                        //Se multiplica por -1, para obtener valores de 1 a 180
      else if (heading > 0)
        heading = 360 - heading;              //Si el marcador es positivo se resta a 360 para obtener valores de 181 a 359.9-0
      return heading;
      nor ==true;
    }
  } while (nor == false);
}  //fin del norte

/*********************************************************************************************************/
/*********************************************************************************************************/
//FUNCION DE COMUNICACION - PROTOCOLO EASYCOMM2
double * cmd_proc()   //return set_point[]
{
  static double set_point[] = {0, 0};   //inicializa el vector en ceros
    // Serial
  char buffer[BufferSize];
  char incomingByte;
  char *Data = buffer;
  char *rawData;
  static int BufferCnt = 0;
  char data[100];     //los vectores son de tamaño definido
  double pos[2];

    // Leer del puerto Serial
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    //Obtener posición usando código personalizado SatNOGS
    if (incomingByte == '!')
    {
      get_position(pos);
      //Obtener posición/
      Serial.print("TM");
      Serial.print(1);
      Serial.print(" ");
      Serial.print("AZ");
      Serial.print(10*pos[0], 1);
      Serial.print(" ");
      Serial.print("EL");
      Serial.println(10*pos[1], 1);
    }
    // Nuevo dato
    else if (incomingByte == '\n')
    {
      buffer[BufferCnt] = 0;
      if (buffer[0] == 'A' && buffer[1] == 'Z')
      {
        if (buffer[2] == ' ' && buffer[3] == 'E' && buffer[4] == 'L')
        {
          get_position(pos);
          //Obtener posición
          Serial.print("AZ");
          Serial.print(pos[0], 1);
          Serial.print(" ");
          Serial.print("EL");
          Serial.print(pos[1], 1);
          Serial.println(" ");
        }
        else
        {
          //Obtener valor absoluto del ángulo
          rawData = strtok_r(Data, " " , &Data);
          strncpy(data, rawData+2, 10);
          if (isNumber(data))
          {
            set_point[0] = atof(data);
            if (set_point[0] > MAX_AZ_ANGLE)
              set_point[0] = MAX_AZ_ANGLE;
            else if (set_point[0] < MIN_AZ_ANGLE)
              set_point[0] = MIN_AZ_ANGLE;
          }
          //Obtener valor absoluto del ángulo
          rawData = strtok_r(Data, " " , &Data);
          if (rawData[0] == 'E' && rawData[1] == 'L')
          {
            strncpy(data, rawData+2, 10);
            if (isNumber(data))
            {
              set_point[1] = atof(data);
              if (set_point[1] > MAX_EL_ANGLE)
                set_point[1] = MAX_EL_ANGLE;
              else if (set_point[1] < MIN_EL_ANGLE)
                set_point[1] = MIN_EL_ANGLE;
            }
          }        
        }
      }
      //Detener motores
      else if (buffer[0] == 'S' && buffer[1] == 'A' && buffer[2] == ' ' && buffer[3] == 'S' && buffer[4] == 'E')
      {
        get_position(pos);
        //Obtener posición
        Serial.print("AZ");
        Serial.print(pos[0], 1);
        Serial.print(" ");
        Serial.print("EL");
        Serial.println(pos[1], 1);
        set_point[0] = pos[0];
        set_point[1] = pos[1];
      }
      // Reiniciar el rotor
      else if (buffer[0] == 'R' && buffer[1] == 'E' && buffer[2] == 'S' && buffer[3] == 'E' && buffer[4] == 'T')
      {
        get_position(pos);
        //Obtener posición
        Serial.print("AZ");
        Serial.print(pos[0], 1);
        Serial.print(" ");
        Serial.print("EL");
        Serial.println(pos[1], 1);
        set_point[0] = 0;
        set_point[1] = 0;
      }
      BufferCnt = 0;
      //Reiniciar el tiempo para desactivar motores
      t_DIS = 0;
    }
      else 
      {
        buffer[BufferCnt] = incomingByte;
        BufferCnt++;
      }   
  }
  return set_point;
}   //Fin del cmd_proc

/*********************************************************************************************************/
/*********************************************************************************************************/
//FUNCION DE CONTROL DE MOTORES
void dc_move(double set_point[])      //control PID
{
  double u[2];
  double curr_pos[2];
  static double prev_pos[] = {0, 0};
  double error[2];
  double Iterm[2];
  double Pterm[2];
  double Dterm[2];
  double Kp = 200;
  double Ki = 2;
  double Kd = 0;
  double dt = 0.001; // calculate

  get_position(curr_pos);
  error[0] = set_point[0] - curr_pos[0];
  error[1] = set_point[1] - curr_pos[1];

  Pterm[0] = Kp * error[0];
  Pterm[1] = Kp * error[1];

  Iterm[0] += Ki * error[0] * dt;
  Iterm[1] += Ki * error[1] * dt;
  if (Iterm[0] > outMax) Iterm[0] = outMax;
  else if (Iterm[0] < outMin) Iterm[0] = outMin;
  if (Iterm[1] > outMax) Iterm[1] = outMax;
  else if (Iterm[1] < outMin) Iterm[1] = outMin;

  Dterm[0] = Kd * (curr_pos[0] - prev_pos[0]) / dt;
  prev_pos[0] = curr_pos[0];
  Dterm[1] = Kd * (curr_pos[1] - prev_pos[1]) / dt;
  prev_pos[1] = curr_pos[1];

  //generar pulso PWM
  u[0] = Pterm[0] + Iterm[0] + Dterm[0];    //Azimuth
  u[1] = Pterm[1] + Iterm[1] + Dterm[1];    //Elevacion

  //AZIMUTH
  if (u[0] >= 0)        //HORARIO
  {
    if (u[0] > outMax)
      u[0] = outMax;
    digitalWrite(IN1, HIGH);  //1 MOTOR 1
    digitalWrite(IN2, LOW);   //2 MOTOR 1
    analogWrite(ENA, u[0]);
  }
  else                 //ANTI-HORARIO
  {
    u[0] = -u[0];
    if ( u[0] > outMax)
      u[0] = outMax;
    digitalWrite(IN1, LOW);   //1 MOTOR 1
    digitalWrite(IN2, HIGH);  //2 MOTOR 1
    analogWrite(ENA, u[0]);
  }

  //ELEVACION
  if (u[1] >= 0)        //HORARIO
  {
    if (u[1] > outMax)
      u[1] = outMax;
    digitalWrite(IN3, HIGH);  //1 MOTOR 2
    digitalWrite(IN4, LOW);   //2 MOTOR 3
    analogWrite(ENB, u[1]);
  }
  else                 //ANTI-HORARIO
  {
    u[1] = -u[1];
    if ( u[1] > outMax)
      u[1] = outMax;
    digitalWrite(IN3, LOW);   //1 MOTOR 2
    digitalWrite(IN4, HIGH);  //2 MOTOR 2
    analogWrite(ENB, u[1]);
  }
}   //Fin del dc_move

/*********************************************************************************************************/
/*********************************************************************************************************/
// Revisar si el argumento es un numero
boolean isNumber(char *input)
{
  for (int i = 0; input[i] != '\0'; i++)
  {
    if (isalpha(input[i]))
      return false;
  }
  return true;
}   //Fin del isNumber
