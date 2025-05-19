//pines motores DC
#define M12 32
#define M22 33
#define PWM2 16
#define TOLERANCE 50
const int pinMotor2[3] = {PWM2, M12, M22};

#define M13 19
#define M23 18
#define PWM3 17
#define Maxpulse 1900
const int pinMotor3[3] = {PWM3, M13, M23};

//encoders motores DC
const int ENCA2 = 13;
const int ENCB2 = 14;
volatile long posicion2 = 0;

const int ENCA3 = 22;
const int ENCB3 = 23;
volatile long posicion3 = 0;

//pines MPP
const int dirPin = 25;
const int stepPin = 26;
const int enablePin = 27;

const int steps = 200;
int stepsGrados;
int stepDelay = 2000;
float posicion1 = 0.0; 
int gradosantes = 0;

//posiciones de Matlab para cada motor
float alpha1 = 0.0;
float alpha2 = 0.0;
float alpha3 = 0.0;

//señal de nueva posicion
bool New1 = 0; 


// Variables PID 1
long previousTime2 = 0;
float errorA2 = 0.0;
float errorI2 = 0.0;
float kp2 = 0.3, kd2 = 0, ki2 = 0.1;
float setpoint2 = 0.0;
float output2 = 0.0;

// Variables PID 2
long previousTime3 = 0;
float errorA3 = 0.0;
float errorI3 = 0.0;
float kp3 = 0.2, kd3 = 0, ki3 = 0.1;
float setpoint3 = 0.0;
float output3 = 0.0;

//reducciones
const float red1 = 4.5; 
const float red2 = 1.0; 
const float red3 = 3.25; 

// Conversor ángulo a pulsos
float AngleToPulse(float angle){
    float pulse = round((Maxpulse * angle) / 360.0);
    if (abs(angle) == 360.0) {
        pulse = 0.0;
    }
    return pulse;
}
//Conversor pulsos a ángulos
float PulseToAngle(float pos){
    float angle = round((360.0 * pos) / Maxpulse);
    return angle;
}

// Cálculo PID motor 2
float calculaPID2(int target){
    long now2 = micros();
    float dT2 = ((float)(now2 - previousTime2)) / 1.0e6;
    int error2 = posicion2 - target;
    float errorD2 = (error2 - errorA2) / dT2;
    errorI2 += error2 * dT2;

    // Limitar errorI para evitar windup
    if (errorI2 > 1000) errorI2 = 1000;
    if (errorI2 < -1000) errorI2 = -1000;

    float u;
    // Detener motor si está dentro de la tolerancia
    if (abs(error2) <= TOLERANCE) {
        errorI2 = 0;
        u = 0;
    } else {
        // Calcular PID
        u = kp2 * error2 + kd2 * errorD2 + ki2 * errorI2;
    }

    previousTime2 = now2;
    errorA2 = error2;
    return u;
}

// Cálculo PID motor 3
float calculaPID3(int target){
    long now3 = micros();
    float dT3 = ((float)(now3 - previousTime3)) / 1.0e6;
    int error3 = posicion3 - target;
    float errorD3 = (error3 - errorA3) / dT3;
    errorI3 += error3 * dT3;

    // Limitar errorI para evitar windup
    if (errorI3 > 1000) errorI3 = 1000;
    if (errorI3 < -1000) errorI3 = -1000;

    float u;
    // Detener motor si está dentro de la tolerancia
    if (abs(error3) <= TOLERANCE) {
        errorI3 = 0;
        u = 0;
    } else {
        // Calcular PID
        u = kp3 * error3 + kd3 * errorD3 + ki3 * errorI3;
    }

    previousTime3 = now3;
    errorA3 = error3;
    return u;
}

// Control de velocidad motores dc
void move(const int pinMotor[3], float u){
    float speed = fabs(u);
    float angulo2 = fabs(PulseToAngle(posicion2));
    
    if(speed > 255) speed = 255;

    // Dirección del motor
    if (u < 0 ){
      if(angulo2 < 90){
        digitalWrite(pinMotor[1], HIGH); // Sentido horario
        digitalWrite(pinMotor[2], LOW);
      } else {
        digitalWrite(pinMotor[2], HIGH); // Sentido antihorario
        digitalWrite(pinMotor[1], LOW);
      }

    } else if (u > 0) {
      if(angulo2 > 90){
        digitalWrite(pinMotor[2], HIGH); // Sentido antihorario
        digitalWrite(pinMotor[1], LOW);
      } else{
        digitalWrite(pinMotor[1], HIGH); // Sentido horario
        digitalWrite(pinMotor[2], LOW);
      }
    } else {
        digitalWrite(pinMotor[1], LOW); // Detener
        digitalWrite(pinMotor[2], LOW);
    }
    analogWrite(pinMotor[0], speed);
    
}

//Gira motor paso a paso 
void giraMPP(int alpha) {
    if(alpha!=gradosantes) {
    digitalWrite(enablePin, LOW);
    digitalWrite(dirPin, HIGH);
    
    if((alpha-gradosantes)<0) {
      for (int x = 0; x < abs(steps*(alpha-gradosantes)/360); x++) {
         digitalWrite(stepPin, HIGH);
         delayMicroseconds(stepDelay);
         digitalWrite(stepPin, LOW);
         delayMicroseconds(stepDelay);
         posicion1 = posicion1 + 360/steps; 
         }
         digitalWrite(enablePin, HIGH);
       }
       
    else {
      digitalWrite(enablePin, LOW);
      digitalWrite(dirPin, LOW);
      for (int x = 0; x < abs(steps*(alpha-gradosantes)/360); x++) {
         digitalWrite(stepPin, HIGH);
         delayMicroseconds(stepDelay);
         digitalWrite(stepPin, LOW);
         delayMicroseconds(stepDelay);
         posicion1 = posicion1 - 360/steps;
       }
    digitalWrite(enablePin, HIGH);
    }
  }
}


// Comunicación con MATLAB
void matlabcom(){
    String OP_st;
    String grados1_st;
    String grados2_st;
    String grados3_st;
    
    if (Serial.available()){
      
        OP_st = Serial.readStringUntil(',');
        grados1_st = Serial.readStringUntil(',');
        grados2_st = Serial.readStringUntil(',');
        grados3_st = Serial.readStringUntil('\n');
        int OP = OP_st.toInt();
        float grados1 = grados1_st.toFloat();
        float grados2 = grados2_st.toFloat();
        float grados3 = grados3_st.toFloat();
        New1 = 1;
        
        // Actualizar alpha
        if (OP == 1) {
            alpha1 = grados1;
            alpha2 = grados2;
            alpha3 = red3*grados3;
            stepsGrados = red1*abs(steps*(alpha1-gradosantes)/360);
        }
        else if (OP == 0) {
            alpha1 = 0.0;
            alpha2 = 0.0;
            alpha3 = 0.0;
            stepsGrados = red1*abs(steps*(alpha1-gradosantes)/360);
        }
        else {
            Serial.print(posicion1/red1); // Enviar posición del motor paso a paso
            Serial.print(",");       // Separador
            Serial.print(PulseToAngle(posicion2)/red2); // Enviar posición del motor 2
            Serial.print(",");       // Separador
            Serial.println(PulseToAngle(posicion3)/red3); // Enviar posición del motor 3 con salto de línea

    }
  }
}

void setup() {
    // Inicialización de la comunicación serie a 9600 baudios.
    Serial.begin(9600);
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(ENCA2, INPUT_PULLUP);
    pinMode(ENCB2, INPUT_PULLUP);
    pinMode(PWM2, OUTPUT);
    pinMode(M12, OUTPUT);
    pinMode(M22, OUTPUT);
    pinMode(ENCA3, INPUT_PULLUP);
    pinMode(ENCB3, INPUT_PULLUP);
    pinMode(PWM3, OUTPUT);
    pinMode(M13, OUTPUT);
    pinMode(M23, OUTPUT);
    
    attachInterrupt(digitalPinToInterrupt(ENCA2), SensorA2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB2), SensorB2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCA3), SensorA3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB3), SensorB3, CHANGE);
    

}


void loop() {
    // Leer comando desde MATLAB 
    matlabcom();
    // Convertir ángulo a setpoint de pulsos solo si es un nuevo valor
    if (New1) {
        setpoint2 = AngleToPulse(alpha2);
        setpoint3 = AngleToPulse(alpha3);
        New1 = 0;
    }
 
    // Calcular el PID y mover motores dc
    output2 = calculaPID2(setpoint2);
    output3 = calculaPID3(setpoint3);
    move(pinMotor2, output2);
    move(pinMotor3, output3);

    //Mover paso a paso 
    giraMPP(alpha1);
    gradosantes = alpha1;
     Serial.println(posicion2);
     Serial.println(PulseToAngle(posicion2));
     Serial.println(posicion3);
     Serial.println(PulseToAngle(posicion3));
     delay(100);
}

// Interrupciones encoder motor 2
void SensorA2(){
    if(digitalRead(ENCA2) == digitalRead(ENCB2)) {
        posicion2++;
    } else {
        posicion2--;
    }
}
void SensorB2(){
    if(digitalRead(ENCA2) == digitalRead(ENCB2)) {
        posicion2--;
    } else {
        posicion2++;
    }
}
// Interrupciones encoder motor 3
void SensorA3(){
    if(digitalRead(ENCA3) == digitalRead(ENCB3)) {
        posicion3++;
    } else {
        posicion3--;
    }
}
void SensorB3(){
    if(digitalRead(ENCA3) == digitalRead(ENCB3)) {
        posicion3--;
    } else {
        posicion3++;
    }
}
