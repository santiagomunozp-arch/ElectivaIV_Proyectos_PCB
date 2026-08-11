#include <EEPROM.h>
#include <math.h>
#include <ESP32Servo.h>

// ============================================================
// EEPROM
// ============================================================
#define EEPROM_SIZE  20
#define ADDR_CADERA  0
#define ADDR_HOMBRO  4
#define ADDR_CODO    8
#define ADDR_PITCH   12
#define ADDR_ROLL    16

// ============================================================
// PINES
// ============================================================
#define CADERA_STEP_PIN   14
#define CADERA_DIR_PIN    12
#define CADERA_ENA_PIN    13

#define HOMBRO_STEP_PIN   4
#define HOMBRO_DIR_PIN    2
#define HOMBRO_ENA_PIN    15

#define CODO_STEP_PIN     27
#define CODO_DIR_PIN      25
#define CODO_ENA_PIN      26

#define PITCH_STEP_PIN    32
#define PITCH_DIR_PIN     33
#define PITCH_ENA_PIN     23

#define ROLL_STEP_PIN     18
#define ROLL_DIR_PIN      19
#define ROLL_ENA_PIN      21

#define SERVO_GARRA_PIN   22

// ============================================================
// Garra
// ============================================================

Servo servoGarra;

// Estado y límites de la garra
const int GARRA_ABIERTA  = 180;
const int GARRA_CERRADA  = 50;
const int GARRA_PULSO_MIN = 1000;
const int GARRA_PULSO_MAX = 2000;
const int GARRA_US_ABIERTA = 1800;
const int GARRA_US_CERRADA = 1200;
const int GARRA_CERRADA_REAL = 55;
const int GARRA_ABIERTA_REAL = 170;  // O AJUSTAR (NO usar 180 si no llega)

int anguloActualGarra = 180;

// ============================================================
// CONFIG MOTORES
// ============================================================
const float PASOS_REV    = 1600.0f;
const int   PULSO_US     =500;
// Velocidad hombro
const int   PULSO_HOMBRO  = 400;

// Velocidad exclusiva codo
const int   PULSO_CODO    = 1200;


// Suavidad de rampa trapezoidal
const int RAMPA_HOMBRO = 1000;

const float PASOS_GRADO_CADERA = (PASOS_REV * 24.0f)  / 360.0f;
const float PASOS_GRADO_HOMBRO = (PASOS_REV * 48.0f)  / 360.0f;
const float PASOS_GRADO_CODO   = (PASOS_REV * 18.0f)  / 360.0f;
const float PASOS_GRADO_PITCH  = (PASOS_REV * 11.0f)  / 360.0f;
const float PASOS_GRADO_ROLL   = (PASOS_REV *  5.0f)  / 360.0f;

const bool INV_HOMBRO = false;
const bool INV_CODO   = false;
const bool INV_PITCH  = false;
const bool INV_ROLL   = false;

// Límites en grados
const float LIM_CADERA_MIN = -180.0f, LIM_CADERA_MAX =  180.0f;
const float LIM_HOMBRO_MIN =    0.0f, LIM_HOMBRO_MAX =  180.0f;
const float LIM_CODO_MIN   = -130.0f, LIM_CODO_MAX   =  130.0f;
const float LIM_PITCH_MIN  =  -30.0f, LIM_PITCH_MAX  = 130.0f;
const float LIM_ROLL_MIN   = -180.0f, LIM_ROLL_MAX   =  180.0f;

// ============================================================
// PARÁMETROS DH — índice 1..5 (0 sin uso)
//
// Segmento            | Medida | Campo  | Índice
// --------------------|--------|--------|-------
// Base → hombro       | 19.2   | d[1]   | axial (columna vertical)
// Hombro → codo       | 27.5   | a[2]   | longitud eslabón
// Codo → pitch        | 19.0   | a[3]   | longitud eslabón
// Pitch → roll        | 14.8   | a[4]   | normal común (alpha[5]=π/2)
// Roll → garra        | 10.0   | d[5]   | offset herramienta
//
// ============================================================
//                         [0]     [1]     [2]         [3]     [4]     [5]
const float DH_d[6]    = { 0.0f,  19.2f,   0.0f,       0.0f,   0.0f,  10.0f };
const float DH_a[6]    = { 0.0f,   0.0f,  27.5f,      19.0f,  14.8f,   0.0f };
const float DH_alpha[6]= { 0.0f,   0.0f,  -M_PI/2.0f,  0.0f,   0.0f,  M_PI/2.0f };

// Estado articular (grados)
float anguloActualCadera = 0.0f;
float anguloActualHombro = 0.0f;
float anguloActualCodo   = 0.0f;
float anguloActualPitch  = 0.0f;
float anguloActualRoll   = 0.0f;

// ============================================================
// MATRIZ 4×4
// ============================================================
struct Mat4 {
  float m[4][4];
  Mat4() { for(int i=0;i<4;i++) for(int j=0;j<4;j++) m[i][j]=(i==j)?1.0f:0.0f; }
};

Mat4 mat4Mul(const Mat4 &A, const Mat4 &B) {
  Mat4 C;
  for(int i=0;i<4;i++) for(int j=0;j<4;j++) {
    C.m[i][j]=0;
    for(int k=0;k<4;k++) C.m[i][j]+=A.m[i][k]*B.m[k][j];
  }
  return C;
}

Mat4 mat4InvTransform(const Mat4 &A) {
  Mat4 I;
  for(int i=0;i<3;i++) for(int j=0;j<3;j++) I.m[i][j]=A.m[j][i];
  for(int i=0;i<3;i++){
    I.m[i][3]=0;
    for(int k=0;k<3;k++) I.m[i][3]-=I.m[i][k]*A.m[k][3];
  }
  I.m[3][0]=I.m[3][1]=I.m[3][2]=0; I.m[3][3]=1;
  return I;
}

// ============================================================
// MATRIZ DH ESTÁNDAR
// ============================================================
Mat4 dhMatrix(int i, float theta_rad) {
  float ct  = cosf(theta_rad);
  float st  = sinf(theta_rad);
  float ca  = cosf(DH_alpha[i]);
  float sa  = sinf(DH_alpha[i]);
  float a   = DH_a[i];
  float d   = DH_d[i];

  Mat4 T;
  T.m[0][0] =  ct;    T.m[0][1] = -st;    T.m[0][2] = 0.0f; T.m[0][3] = a;
  T.m[1][0] =  st*ca; T.m[1][1] =  ct*ca; T.m[1][2] = -sa;  T.m[1][3] = -sa*d;
  T.m[2][0] =  st*sa; T.m[2][1] =  ct*sa; T.m[2][2] =  ca;  T.m[2][3] =  ca*d;
  T.m[3][0] =  0.0f;  T.m[3][1] =  0.0f;  T.m[3][2] = 0.0f; T.m[3][3] = 1.0f;
  return T;
}

// ============================================================
// EEPROM
// ============================================================
void guardarPosicionEEPROM() {
  EEPROM.put(ADDR_CADERA, anguloActualCadera);
  EEPROM.put(ADDR_HOMBRO, anguloActualHombro);
  EEPROM.put(ADDR_CODO,   anguloActualCodo);
  EEPROM.put(ADDR_PITCH,  anguloActualPitch);
  EEPROM.put(ADDR_ROLL,   anguloActualRoll);
  EEPROM.commit();
}

void cargarPosicionEEPROM() {
  EEPROM.get(ADDR_CADERA, anguloActualCadera);
  EEPROM.get(ADDR_HOMBRO, anguloActualHombro);
  EEPROM.get(ADDR_CODO,   anguloActualCodo);
  EEPROM.get(ADDR_PITCH,  anguloActualPitch);
  EEPROM.get(ADDR_ROLL,   anguloActualRoll);

  auto clamp = [](float v, float lo, float hi, float def) {
    return (isnan(v) || v < lo || v > hi) ? def : v;
  };
  anguloActualCadera = clamp(anguloActualCadera, LIM_CADERA_MIN, LIM_CADERA_MAX, 0);
  anguloActualHombro = clamp(anguloActualHombro, LIM_HOMBRO_MIN, LIM_HOMBRO_MAX, 0);
  anguloActualCodo   = clamp(anguloActualCodo,   LIM_CODO_MIN,   LIM_CODO_MAX,   0);
  anguloActualPitch  = clamp(anguloActualPitch,  LIM_PITCH_MIN,  LIM_PITCH_MAX,  0);
  anguloActualRoll   = clamp(anguloActualRoll,   LIM_ROLL_MIN,   LIM_ROLL_MAX,   0);
}

// ============================================================
// PROTOTIPOS
// ============================================================
void  inicializarArticulacion(int stepPin, int dirPin, int enaPin);
void  moverMotorPasosSuave(int stepPin, int dirPin, long pasos, bool horario, int pulsoUs, int PULSOS_HOMBRO);
bool  moverCaderaHacia(float ang);
bool  moverHombroHacia(float ang);
bool  moverCodoHacia(float ang);
bool  moverPitchHacia(float ang);
bool  moverRollHacia(float ang);
void  moverCincoAngulos(float c, float h, float e, float p, float r);
void  moverCincoAngulosSimultaneo(float c, float h, float e, float p, float r);
bool  resolverIK(float xObj, float yObj, float zObj, float phi, float theta,
                 float &th1, float &th2, float &th3, float &th4, float &th5);
void  ejecutarIK(float xObj, float yObj, float zObj, float phi, float theta);
void  ejecutarSecuenciaA();
void  ejecutarSaludo();
void  irAPosicionReposo();
void  imprimirAyuda();
void  setPosicionActualComoCero();

void moverGarra(int angulo) {

  angulo = constrain(angulo, 0, 180);

  // Usar GARRA_US_CERRADA / GARRA_US_ABIERTA (microsegundos reales)
  int pulso = map(angulo,
                  0, 180,
                  GARRA_US_CERRADA,
                  GARRA_US_ABIERTA);

  servoGarra.writeMicroseconds(pulso);

  anguloActualGarra = angulo;

  Serial.printf("[GARRA] Input:%d -> Pulso:%d us\n", angulo, pulso);
}

void abrirGarra()  { moverGarra(GARRA_ABIERTA);  Serial.println("[GARRA] Abierta."); }
void cerrarGarra() { moverGarra(GARRA_CERRADA); Serial.println("[GARRA] Cerrada."); }

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoGarra.setPeriodHertz(50);
  servoGarra.attach(
    SERVO_GARRA_PIN,
    1000,
    2000
  );
  moverGarra(GARRA_ABIERTA);
  cargarPosicionEEPROM();

  inicializarArticulacion(CADERA_STEP_PIN, CADERA_DIR_PIN, CADERA_ENA_PIN);
  inicializarArticulacion(HOMBRO_STEP_PIN, HOMBRO_DIR_PIN, HOMBRO_ENA_PIN);
  inicializarArticulacion(CODO_STEP_PIN,   CODO_DIR_PIN,   CODO_ENA_PIN);
  inicializarArticulacion(PITCH_STEP_PIN,  PITCH_DIR_PIN,  PITCH_ENA_PIN);
  inicializarArticulacion(ROLL_STEP_PIN,   ROLL_DIR_PIN,   ROLL_ENA_PIN);

  Serial.println("[SISTEMA] Posicion cargada.");
  imprimirAyuda();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  if (!Serial.available()) return;

  String entrada = Serial.readStringUntil('\n');
  entrada.trim();
  String eu = entrada;
  eu.toUpperCase();

  if      (eu == "A")              ejecutarSecuenciaA();
  else if (eu == "Q")              setPosicionActualComoCero();
  else if (eu == "R")              irAPosicionReposo();
  else if (eu == "P")              ejecutarSaludo();
  else if (eu == "S") { ejecutarPickPlace(); }
  else if (eu == "I") { irAPosicionPickPlace(); }
  else if (eu == "H" || eu == "?") imprimirAyuda();

  else if (eu == "GO") { abrirGarra(); }
  else if (eu == "GC") { cerrarGarra(); }
  else if (eu.startsWith("G") && eu.length() > 1 && isDigit(eu.charAt(1))) {
    int ang = entrada.substring(1).toInt();
    moverGarra(ang);
  }

  // PICK: recoger en coordenadas
// Formato: PICK X20,Y0,Z5,P0,T-30
else if (eu.startsWith("PICK")) {
  int posX=eu.indexOf('X'), posY=eu.indexOf('Y'), posZ=eu.indexOf('Z');
  int posP=eu.indexOf('P'), posT=eu.indexOf('T');
  if (posX==-1||posY==-1||posZ==-1) {
    Serial.println("[ERROR] Formato: PICK X20,Y0,Z5,P0,T-30");
  } else {
    float xv=entrada.substring(posX+1,posY-1).toFloat();
    float yv=entrada.substring(posY+1,posZ-1).toFloat();
    float zv,phiv=0,thetav=0;
    if (posP!=-1) zv=entrada.substring(posZ+1,posP-1).toFloat();
    else          zv=entrada.substring(posZ+1).toFloat();
    if (posP!=-1) phiv=entrada.substring(posP+1,(posT!=-1)?posT-1:(int)entrada.length()).toFloat();
    if (posT!=-1) thetav=entrada.substring(posT+1).toFloat();
    recogerObjeto(xv,yv,zv,phiv*DEG_TO_RAD,thetav*DEG_TO_RAD);
  }
}
// PLACE: depositar en coordenadas
// Formato: PLACE X-20,Y0,Z5,P0,T-30
  else if (eu.startsWith("PLACE")) {
    int posX=eu.indexOf('X'), posY=eu.indexOf('Y'), posZ=eu.indexOf('Z');
    int posP=eu.indexOf('P'), posT=eu.indexOf('T');
    if (posX==-1||posY==-1||posZ==-1) {
      Serial.println("[ERROR] Formato: PLACE X-20,Y0,Z5,P0,T-30");
    } else {
    float xv=entrada.substring(posX+1,posY-1).toFloat();
    float yv=entrada.substring(posY+1,posZ-1).toFloat();
    float zv,phiv=0,thetav=0;
    if (posP!=-1) zv=entrada.substring(posZ+1,posP-1).toFloat();
    else          zv=entrada.substring(posZ+1).toFloat();
    if (posP!=-1) phiv=entrada.substring(posP+1,(posT!=-1)?posT-1:(int)entrada.length()).toFloat();
    if (posT!=-1) thetav=entrada.substring(posT+1).toFloat();
    depositarObjeto(xv,yv,zv,phiv*DEG_TO_RAD,thetav*DEG_TO_RAD);
  }
}

  // IK: X<x>,Y<y>,Z<z>[,P<phi>,T<theta>]
  else if (eu.startsWith("X")) {
    int posX = eu.indexOf('X'), posY = eu.indexOf('Y'),
        posZ = eu.indexOf('Z'), posP = eu.indexOf('P'),
        posT = eu.indexOf('T');

    if (posX == -1 || posY == -1 || posZ == -1) {
      Serial.println("[ERROR] Formato: X10,Y20,Z30  o  X10,Y20,Z30,P0,T0");
    } else {
      float xv = entrada.substring(posX+1, posY-1).toFloat();
      float yv = entrada.substring(posY+1, posZ-1).toFloat();
      float zv, phiv = 0, thetav = 0;

      if      (posP != -1) zv = entrada.substring(posZ+1, posP-1).toFloat();
      else if (posT != -1) zv = entrada.substring(posZ+1, posT-1).toFloat();
      else                 zv = entrada.substring(posZ+1).toFloat();

      if (posP != -1)
        phiv = entrada.substring(posP+1, (posT!=-1)?posT-1:(int)entrada.length()).toFloat();
      if (posT != -1)
        thetav = entrada.substring(posT+1).toFloat();

      ejecutarIK(xv, yv, zv, phiv * DEG_TO_RAD, thetav * DEG_TO_RAD);
    }
  }

  // E<ang> — codo
  else if (eu.startsWith("E") && eu.length() > 1 &&
           (isDigit(eu.charAt(1)) || eu.charAt(1) == '-')) {
    if (!moverCodoHacia(entrada.substring(1).toFloat()))
      Serial.printf("[ERROR] Codo: rango [%.0f, %.0f]\n", LIM_CODO_MIN, LIM_CODO_MAX);
  }

  // C<ang> — cadera
  else if (eu.startsWith("C") && eu.length() > 1 &&
           (isDigit(eu.charAt(1)) || eu.charAt(1) == '-')) {
    if (!moverCaderaHacia(entrada.substring(1).toFloat()))
      Serial.printf("[ERROR] Cadera: rango [%.0f, %.0f]\n", LIM_CADERA_MIN, LIM_CADERA_MAX);
  }

  // W<ang> — pitch
  else if (eu.startsWith("W") && eu.length() > 1 &&
           (isDigit(eu.charAt(1)) || eu.charAt(1) == '-')) {
    if (!moverPitchHacia(entrada.substring(1).toFloat()))
      Serial.printf("[ERROR] Pitch: rango [%.0f, %.0f]\n", LIM_PITCH_MIN, LIM_PITCH_MAX);
  }

  // K<ang> — roll
  else if (eu.startsWith("K") && eu.length() > 1 &&
           (isDigit(eu.charAt(1)) || eu.charAt(1) == '-')) {
    if (!moverRollHacia(entrada.substring(1).toFloat()))
      Serial.printf("[ERROR] Roll: rango [%.0f, %.0f]\n", LIM_ROLL_MIN, LIM_ROLL_MAX);
  }

  // Número sólo → hombro | lista separada por comas → multi-eje
  else {
    int comaPos = entrada.indexOf(',');
    if (comaPos != -1) {
      String partes[5];
      int n = 0, ini = 0;
      for (int i = 0; i <= (int)entrada.length() && n < 5; i++) {
        if (i == (int)entrada.length() || entrada.charAt(i) == ',') {
          partes[n++] = entrada.substring(ini, i);
          ini = i + 1;
        }
      }
      float vals[5] = { anguloActualCadera, anguloActualHombro,
                        anguloActualCodo,   anguloActualPitch, anguloActualRoll };
      for (int i = 0; i < n; i++) vals[i] = partes[i].toFloat();
      moverCincoAngulosSimultaneo(vals[0], vals[1], vals[2], vals[3], vals[4]);
    }
    else if (entrada.length() > 0) {
      float ang = entrada.toFloat();
      if (!moverHombroHacia(ang))
        Serial.printf("[ERROR] Hombro: rango [%.0f, %.0f]\n", LIM_HOMBRO_MIN, LIM_HOMBRO_MAX);
    }
  }
}

// ============================================================
// HARDWARE
// ============================================================
void inicializarArticulacion(int stepPin, int dirPin, int enaPin) {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin,  OUTPUT);
  pinMode(enaPin,  OUTPUT);
  digitalWrite(enaPin, LOW);
}

void moverMotorPasosSuave(int stepPin, int dirPin, long pasos, bool horario, int pulsoUs) {
  if (pasos <= 0) return;
  digitalWrite(dirPin, horario ? HIGH : LOW);

  const int delayIni = 400;
  const int delayMin = pulsoUs;
  long zona = max(50L, pasos / 5);

  for (long i = 0; i < pasos; i++) {
    int d;
    if      (i < zona)          d = delayIni - (int)((delayIni - delayMin) * (float)i / zona);
    else if (i > pasos - zona)  d = delayIni - (int)((delayIni - delayMin) * (float)(pasos - i) / zona);
    else                        d = delayMin;

    digitalWrite(stepPin, HIGH); delayMicroseconds(d);
    digitalWrite(stepPin, LOW);  delayMicroseconds(d);
  }
}

void moverMotorPasosDirecto(int stepPin, int dirPin, long pasos, bool horario, int pulsoUs) {
  if (pasos <= 0) return;

  digitalWrite(dirPin, horario ? HIGH : LOW);

  // ===== VELOCIDAD INDEPENDIENTE =====
  int velocidadMotor = pulsoUs;

  // Si es el motor del codo → usar velocidad propia
  if (stepPin == CODO_STEP_PIN) {
    velocidadMotor = PULSO_CODO;
  }

  for (long i = 0; i < pasos; i++) {

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(velocidadMotor);

    digitalWrite(stepPin, LOW);
    delayMicroseconds(velocidadMotor);
  }
}

// ============================================================
// ARTICULACIONES
// ============================================================
bool moverCaderaHacia(float obj) {
  if (obj < LIM_CADERA_MIN || obj > LIM_CADERA_MAX) return false;
  if (obj == anguloActualCadera) return true;
  if (fabsf(obj - anguloActualCadera) > 180.0f) {
    moverCaderaHacia(0); moverCaderaHacia(obj); return true;
  }
  float delta = obj - anguloActualCadera;
  moverMotorPasosSuave(CADERA_STEP_PIN, CADERA_DIR_PIN,
                       (long)(fabsf(delta)*PASOS_GRADO_CADERA), delta>0, PULSO_US);
  anguloActualCadera = obj;
  guardarPosicionEEPROM();
  return true;
}

bool moverHombroHacia(float obj) {

  // Limites
  if (obj < LIM_HOMBRO_MIN || obj > LIM_HOMBRO_MAX)
    return false;

  // Diferencia angular
  float delta = obj - anguloActualHombro;

  // Sin movimiento
  if (delta == 0)
    return true;

  // Direccion
  bool dir = (delta > 0);

  if (INV_HOMBRO)
    dir = !dir;

  // Cantidad de pasos
  long pasos = (long)(fabsf(delta) * PASOS_GRADO_HOMBRO);

  Serial.printf(
      "[HOMBRO] Delta: %.2f | Pasos: %ld\n",
      delta,
      pasos
  );

  // Configurar direccion
  digitalWrite(HOMBRO_DIR_PIN, dir ? HIGH : LOW);

  // ===== CONTROL TRAPEZOIDAL =====

  const int delayInicio = RAMPA_HOMBRO;
  const int delayFinal  = PULSO_HOMBRO;

  long zona = max(20L, pasos / 5);

  for (long i = 0; i < pasos; i++) {

    int velocidadActual;

    // ACELERACION
    if (i < zona) {

      velocidadActual =
          delayInicio -
          ((delayInicio - delayFinal) * i / zona);

    }

    // DESACELERACION
    else if (i >= (pasos - zona)) {

      velocidadActual =
          delayInicio -
          ((delayInicio - delayFinal) *
          (pasos - i) / zona);

    }

    // VELOCIDAD CONSTANTE
    else {

      velocidadActual = delayFinal;

    }

    // Pulso STEP
    digitalWrite(HOMBRO_STEP_PIN, HIGH);
    delayMicroseconds(velocidadActual);

    digitalWrite(HOMBRO_STEP_PIN, LOW);
    delayMicroseconds(velocidadActual);
  }

  // Guardar posicion
  anguloActualHombro = obj;

  guardarPosicionEEPROM();

  return true;
}

bool moverCodoHacia(float obj) {
  if (obj < LIM_CODO_MIN || obj > LIM_CODO_MAX) return false;

  float delta = obj - anguloActualCodo;

  if (delta == 0) return true;

  bool dir = (delta > 0);

  if (INV_CODO) dir = !dir;

  moverMotorPasosDirecto(
      CODO_STEP_PIN,
      CODO_DIR_PIN,
      (long)(fabsf(delta) * PASOS_GRADO_CODO),
      dir,
      PULSO_US
  );

  anguloActualCodo = obj;

  guardarPosicionEEPROM();

  return true;
}

bool moverPitchHacia(float obj) {
  if (obj < LIM_PITCH_MIN || obj > LIM_PITCH_MAX) return false;
  float delta = obj - anguloActualPitch;
  if (delta == 0) return true;
  bool dir = (delta > 0); if (INV_PITCH) dir = !dir;
  moverMotorPasosSuave(PITCH_STEP_PIN, PITCH_DIR_PIN,
                       (long)(fabsf(delta)*PASOS_GRADO_PITCH), dir, PULSO_US);
  anguloActualPitch = obj;
  guardarPosicionEEPROM();
  return true;
}

bool moverRollHacia(float obj) {
  if (obj < LIM_ROLL_MIN || obj > LIM_ROLL_MAX) return false;
  if (obj == anguloActualRoll) return true;
  if (fabsf(obj - anguloActualRoll) > 180.0f) {
    moverRollHacia(0); moverRollHacia(obj); return true;
  }
  float delta = obj - anguloActualRoll;
  bool dir = (delta > 0); if (INV_ROLL) dir = !dir;
  moverMotorPasosSuave(ROLL_STEP_PIN, ROLL_DIR_PIN,
                       (long)(fabsf(delta)*PASOS_GRADO_ROLL), dir, PULSO_US);
  anguloActualRoll = obj;
  guardarPosicionEEPROM();
  return true;
}

void moverCincoAngulos(float c, float h, float e, float p, float r) {
  if (!moverHombroHacia(h)) { Serial.println("[ERROR] Hombro."); return; }
  if (!moverCodoHacia(e))   { Serial.println("[ERROR] Codo.");   return; }
  if (!moverPitchHacia(p))  { Serial.println("[ERROR] Pitch.");  return; }
  moverCaderaHacia(c);
  moverRollHacia(r);
}

// ============================================================
// MOVIMIENTO SIMULTÁNEO 5D — Bresenham
// ============================================================
void moverCincoAngulosSimultaneo(float c, float h, float e, float p, float r) {

  if (c < LIM_CADERA_MIN || c > LIM_CADERA_MAX) {
    Serial.println("[ERROR] Cadera.");
    return;
  }

  if (h < LIM_HOMBRO_MIN || h > LIM_HOMBRO_MAX) {
    Serial.println("[ERROR] Hombro.");
    return;
  }

  if (e < LIM_CODO_MIN || e > LIM_CODO_MAX) {
    Serial.println("[ERROR] Codo.");
    return;
  }

  if (p < LIM_PITCH_MIN || p > LIM_PITCH_MAX) {
    Serial.println("[ERROR] Pitch.");
    return;
  }

  if (r < LIM_ROLL_MIN || r > LIM_ROLL_MAX) {
    Serial.println("[ERROR] Roll.");
    return;
  }

  long pC = (long)(fabsf(c - anguloActualCadera) * PASOS_GRADO_CADERA);
  long pH = (long)(fabsf(h - anguloActualHombro) * PASOS_GRADO_HOMBRO);
  long pE = (long)(fabsf(e - anguloActualCodo)   * PASOS_GRADO_CODO);
  long pP = (long)(fabsf(p - anguloActualPitch)  * PASOS_GRADO_PITCH);
  long pR = (long)(fabsf(r - anguloActualRoll)   * PASOS_GRADO_ROLL);

  long dom = max(pC, max(pH, max(pE, max(pP, pR))));

  if (dom == 0) return;

  bool dC = (c > anguloActualCadera);

  bool dH = (h > anguloActualHombro);
  if (INV_HOMBRO) dH = !dH;

  bool dE = (e > anguloActualCodo);
  if (INV_CODO) dE = !dE;

  bool dP = (p > anguloActualPitch);
  if (INV_PITCH) dP = !dP;

  bool dR = (r > anguloActualRoll);
  if (INV_ROLL) dR = !dR;

  digitalWrite(CADERA_DIR_PIN, dC ? HIGH : LOW);
  digitalWrite(HOMBRO_DIR_PIN, dH ? HIGH : LOW);
  digitalWrite(CODO_DIR_PIN,   dE ? HIGH : LOW);
  digitalWrite(PITCH_DIR_PIN,  dP ? HIGH : LOW);
  digitalWrite(ROLL_DIR_PIN,   dR ? HIGH : LOW);

  long aC = 0;
  long aH = 0;
  long aE = 0;
  long aP = 0;
  long aR = 0;

  long cC = 0;
  long cH = 0;
  long cE = 0;
  long cP = 0;
  long cR = 0;

  for (long i = 0; i < dom; i++) {

    #define PASO_IF(acc, cnt, total, pin) \
      acc += total; \
      if (acc >= dom && cnt < total) { \
        \
        int velocidadActual = PULSO_US; \
        \
        if (pin == HOMBRO_STEP_PIN) { \
          velocidadActual = PULSO_HOMBRO; \
        } \
        \
        if (pin == CODO_STEP_PIN) { \
          velocidadActual = PULSO_CODO; \
        } \
        \
        digitalWrite(pin, HIGH); \
        delayMicroseconds(velocidadActual); \
        digitalWrite(pin, LOW); \
        delayMicroseconds(velocidadActual); \
        \
        cnt++; \
        acc -= dom; \
      }

    PASO_IF(aC, cC, pC, CADERA_STEP_PIN)
    PASO_IF(aH, cH, pH, HOMBRO_STEP_PIN)
    PASO_IF(aE, cE, pE, CODO_STEP_PIN)
    PASO_IF(aP, cP, pP, PITCH_STEP_PIN)
    PASO_IF(aR, cR, pR, ROLL_STEP_PIN)

    #undef PASO_IF
  }

  anguloActualCadera = c;
  anguloActualHombro = h;
  anguloActualCodo   = e;
  anguloActualPitch  = p;
  anguloActualRoll   = r;

  guardarPosicionEEPROM();
}

// ============================================================
// IK — RRRRR 5-DOF con matrices DH completas
// ============================================================
bool resolverIK(float xObj, float yObj, float zObj,
                float phiObj, float thetaObj,
                float &th1, float &th2, float &th3, float &th4, float &th5) {

  float d5 = DH_d[5];  // offset herramienta: 10 cm

  // --- Centro de muñeca ---
  float wx = xObj - d5 * cosf(thetaObj) * cosf(phiObj);
  float wy = yObj - d5 * cosf(thetaObj) * sinf(phiObj);
  float wz = zObj - d5 * sinf(thetaObj);

  // --- th1 — Cadera ---
  th1 = atan2f(wy, wx) * RAD_TO_DEG;
  float th1r = th1 * DEG_TO_RAD;

  // --- Problema planar 2D para th2 y th3 ---
  float r  = sqrtf(wx*wx + wy*wy);
  float sz = wz - DH_d[1];          // descontar altura de base: 19.2 cm
  float L2 = DH_a[2];               // hombro → codo:  27.5 cm
  float L3 = DH_a[3];               // codo  → pitch:  19.0 cm

  float D = (r*r + sz*sz - L2*L2 - L3*L3) / (2.0f * L2 * L3);
  if (D < -1.0f || D > 1.0f) {
    Serial.println("[IK] Posicion inalcanzable.");
    return false;
  }

  float th3r = atan2f(-sqrtf(1.0f - D*D), D);  // codo arriba
  float th2r = atan2f(sz, r) - atan2f(L3*sinf(th3r), L2 + L3*cosf(th3r));

  // --- T05 (transformación objetivo) ---
  Mat4 T05;
  float cp = cosf(phiObj), sp = sinf(phiObj);
  float ct = cosf(thetaObj), st2 = sinf(thetaObj);

  T05.m[0][0] =  cp*ct;  T05.m[0][1] = -sp*ct;  T05.m[0][2] =  cp*st2; T05.m[0][3] = xObj;
  T05.m[1][0] =  sp*ct;  T05.m[1][1] =  cp*ct;  T05.m[1][2] =  sp*st2; T05.m[1][3] = yObj;
  T05.m[2][0] = -st2;    T05.m[2][1] =  0;       T05.m[2][2] =  ct;     T05.m[2][3] = zObj;
  T05.m[3][0] =  0;      T05.m[3][1] =  0;       T05.m[3][2] =  0;      T05.m[3][3] = 1;

  // --- T03 encadenando DH ---
  Mat4 T01 = dhMatrix(1, th1r);
  Mat4 T12 = dhMatrix(2, th2r);
  Mat4 T23 = dhMatrix(3, th3r);
  Mat4 T03 = mat4Mul(T01, mat4Mul(T12, T23));

  // --- T35 = T03^-1 * T05 ---
  Mat4 T35 = mat4Mul(mat4InvTransform(T03), T05);

  // --- th4 — Pitch ---
  float th4r = -(th2r + th3r) + (M_PI/2.0f); //atan2f(sqrtf(T35.m[0][2]*T35.m[0][2] + T35.m[1][2]*T35.m[1][2]),
                      //T35.m[2][2]);

  // --- th5 — Roll ---
  float th5r = atan2f(T35.m[1][0], T35.m[0][0]);

  th2 = th2r * RAD_TO_DEG;
  th3 = th3r * RAD_TO_DEG;
  th4 = th4r * RAD_TO_DEG;
  th5 = th5r * RAD_TO_DEG;

  Serial.printf(
    "[DEBUG] C=%.1f H=%.1f E=%.1f W=%.1f R=%.1f\n",
    th1,
    th2,
    th3,
    th4,
    th5
  );

  while (th5 >  180.0f) th5 -= 360.0f;
  while (th5 < -180.0f) th5 += 360.0f;

  Serial.printf("[IK] C=%.1f H=%.1f E=%.1f W=%.1f R=%.1f\n",
                th1, th2, th3, th4, th5);

  if (th1 < LIM_CADERA_MIN || th1 > LIM_CADERA_MAX) { Serial.printf("[IK] Cadera fuera: %.1f\n", th1); return false; }
  if (th2 < LIM_HOMBRO_MIN || th2 > LIM_HOMBRO_MAX) { Serial.printf("[IK] Hombro fuera: %.1f\n", th2); return false; }
  if (th3 < LIM_CODO_MIN   || th3 > LIM_CODO_MAX)   { Serial.printf("[IK] Codo fuera: %.1f\n",   th3); return false; }
  if (th4 < LIM_PITCH_MIN  || th4 > LIM_PITCH_MAX)  { Serial.printf("[IK] Pitch fuera: %.1f\n",  th4); return false; }
  if (th5 < LIM_ROLL_MIN   || th5 > LIM_ROLL_MAX)   { Serial.printf("[IK] Roll fuera: %.1f\n",   th5); return false; }

  return true;
}
bool configuracionSegura(float hombro,
                         float codo,
                         float pitch)
{
    if(hombro < 80)
        return false;

    if(codo < -100)
        return false;

    if(codo > 100)
        return false;

    if(pitch > 130)
        return false;

    return true;
}

void ejecutarIK(float xObj, float yObj, float zObj, float phi, float theta) {
  Serial.printf("[IK] -> X=%.2f Y=%.2f Z=%.2f Phi=%.1f° Theta=%.1f°\n",
                xObj, yObj, zObj, phi*RAD_TO_DEG, theta*RAD_TO_DEG);

  // Debug: verificar alcance antes de resolver
  // CORRECCIÓN: L3b usaba DH_d[4] (incorrecto); ahora usa DH_a[3] = 19.0 cm
  float d5b  = DH_d[5];
  float wxb  = xObj - d5b * cosf(theta) * cosf(phi);
  float wyb  = yObj - d5b * cosf(theta) * sinf(phi);
  float wzb  = zObj - d5b * sinf(theta);
  float rb   = sqrtf(wxb*wxb + wyb*wyb);
  float szb  = wzb - DH_d[1];
  float L2b  = DH_a[2];   // 27.5 cm
  float L3b  = DH_a[3];   // 19.0 cm  ← CORREGIDO (antes era DH_d[4] = 14.8)
  float Db   = (rb*rb + szb*szb - L2b*L2b - L3b*L3b) / (2.0f * L2b * L3b);
  Serial.printf("[DBG] wx=%.2f wz=%.2f r=%.2f sz=%.2f D=%.4f\n", wxb, wzb, rb, szb, Db);

  float th1, th2, th3, th4, th5;
  if (!resolverIK(xObj, yObj, zObj, phi, theta, th1, th2, th3, th4, th5)) {
    Serial.println("[IK] Sin solucion.");
    return;
  }
  moverCincoAngulosSimultaneo(th1, th2, th3, th4, th5);
  Serial.println("[IK] OK.");
}


void recogerObjeto(float x, float y, float z, float phi, float theta) {
  float th1, th2, th3, th4, th5;
  const float ALTURA_APROXIMACION = 8.0f; // cm sobre el objeto

  Serial.println("[PICK] Iniciando secuencia de recogida...");

  // FASE 1: Abrir garra antes de moverse
  abrirGarra();
  delay(300);

  // FASE 2: Ir al punto de aproximación (sobre el objeto)
  float zA = z + ALTURA_APROXIMACION;
  if (!resolverIK(x, y, zA, phi, theta, th1, th2, th3, th4, th5)) {
    Serial.println("[PICK] Error: punto de aproximacion inalcanzable.");
    return;
  }
  moverCincoAngulosSimultaneo(th1, th2, th3, th4, th5);
  delay(400);

  // FASE 3: Descender al objeto
  if (!resolverIK(x, y, z, phi, theta, th1, th2, th3, th4, th5)) {
    Serial.println("[PICK] Error: posicion de agarre inalcanzable.");
    return;
  }
  moverCincoAngulosSimultaneo(th1, th2, th3, th4, th5);
  delay(400);

  // FASE 4: Cerrar garra (agarrar)
  cerrarGarra();
  delay(500); // Tiempo para que el servo agarre

  // FASE 5: Retiro vertical (evitar arrastrar el objeto)
  if (!resolverIK(x, y, zA, phi, theta, th1, th2, th3, th4, th5)) {
    Serial.println("[PICK] Error: no puede subir con objeto.");
    cerrarGarra(); // mantener agarre aunque no pueda subir
    return;
  }
  moverCincoAngulosSimultaneo(th1, th2, th3, th4, th5);
  delay(300);

  Serial.println("[PICK] Objeto recogido.");
}

void depositarObjeto(float x, float y, float z, float phi, float theta) {
  float th1, th2, th3, th4, th5;
  const float ALTURA_APROXIMACION = 8.0f;

  Serial.println("[PLACE] Iniciando secuencia de deposito...");

  // FASE 1: Ir sobre el destino
  float zA = z + ALTURA_APROXIMACION;
  if (!resolverIK(x, y, zA, phi, theta, th1, th2, th3, th4, th5)) {
    Serial.println("[PLACE] Error: destino inalcanzable.");
    return;
  }
  moverCincoAngulosSimultaneo(th1, th2, th3, th4, th5);
  delay(400);

  // FASE 2: Descender al punto de deposito
  if (!resolverIK(x, y, z, phi, theta, th1, th2, th3, th4, th5)) {
    Serial.println("[PLACE] Error: posicion de deposito inalcanzable.");
    return;
  }
  moverCincoAngulosSimultaneo(th1, th2, th3, th4, th5);
  delay(400);

  // FASE 3: Abrir garra (soltar)
  abrirGarra();
  delay(400);

  // FASE 4: Retiro vertical
  if (resolverIK(x, y, zA, phi, theta, th1, th2, th3, th4, th5))
    moverCincoAngulosSimultaneo(th1, th2, th3, th4, th5);

  Serial.println("[PLACE] Objeto depositado.");
}

// ============================================================
// CALIBRACIÓN / REPOSO / SALUDO / SECUENCIA A
// ============================================================
void setPosicionActualComoCero() {
  anguloActualCadera = anguloActualHombro = anguloActualCodo =
  anguloActualPitch  = anguloActualRoll   = 0.0f;
  guardarPosicionEEPROM();
  Serial.println("[CAL] Cero establecido.");
}

void irAPosicionReposo() {
  moverCincoAngulos(0, 45, -130, 0, 0);
}

void ejecutarSaludo() {
  Serial.println("[SALUDO] Inicio.");
  moverCaderaHacia(180);
  moverHombroHacia(90);
  moverCodoHacia(45);
  moverPitchHacia(0);
  moverRollHacia(0);
  for (int i = 0; i < 3; i++) {
    moverHombroHacia(120); delay(100);
    moverHombroHacia(70);  delay(100);
  }
  moverHombroHacia(90);
  Serial.println("[SALUDO] Listo.");
}

void ejecutarSecuenciaA() {
  Serial.println("[SEQ-A] Inicio.");

  moverCodoHacia(0);
  moverHombroHacia(0);
  moverCaderaHacia(-180);
  moverPitchHacia(0);
  moverRollHacia(0);

  const long pC = (long)(360L * PASOS_GRADO_CADERA);
  const long pH = (long)(180L * PASOS_GRADO_HOMBRO);
  const long pE = (long)( 90L * PASOS_GRADO_CODO);
  const long pP = (long)( 90L * PASOS_GRADO_PITCH);

  digitalWrite(CADERA_DIR_PIN, HIGH);
  digitalWrite(HOMBRO_DIR_PIN, INV_HOMBRO ? LOW : HIGH);
  digitalWrite(CODO_DIR_PIN,   INV_CODO   ? LOW : HIGH);
  digitalWrite(PITCH_DIR_PIN,  INV_PITCH  ? LOW : HIGH);

  long aH=0, aE=0, aP=0, cH=0, cE=0, cP=0;

  for (long i = 0; i < pC; i++) {
    digitalWrite(CADERA_STEP_PIN, HIGH); delayMicroseconds(PULSO_US);
    digitalWrite(CADERA_STEP_PIN, LOW);  delayMicroseconds(PULSO_US);

    #define BRE(acc, cnt, total, pin, velocidad) \
      acc += total; \
      if (acc >= pC && cnt < total) { \
        digitalWrite(pin, HIGH); delayMicroseconds(velocidad); \
        digitalWrite(pin, LOW);  delayMicroseconds(velocidad); \
        cnt++; acc -= pC; \
  }
    BRE(aH, cH, pH, HOMBRO_STEP_PIN, PULSO_HOMBRO)

    BRE(aE, cE, pE, CODO_STEP_PIN, PULSO_CODO)

    BRE(aP, cP, pP, PITCH_STEP_PIN, PULSO_US)
    #undef BRE
  }

  anguloActualCadera = 180; anguloActualHombro = 180;
  anguloActualCodo   =  90; anguloActualPitch  =  90;
  anguloActualRoll   =   0;
  guardarPosicionEEPROM();

  Serial.println("[SEQ-A] OK. Volviendo a reposo...");
  irAPosicionReposo();
}

void irAPosicionPickPlace() {
  Serial.println("[INIT] Yendo a posicion de inicio pick & place...");
  moverCodoHacia(-105);
  moverPitchHacia(40);
  moverRollHacia(0);
  moverCaderaHacia(-90);
  moverHombroHacia(60);
  moverGarra(130);  // garra semi-abierta, no completa
  Serial.println("[INIT] Listo.");
}

void ejecutarPickPlace() {
  Serial.println("[PP] Iniciando pick & place...");

  // === PASO 1: Aproximacion recogida — garra ya en 130 ===
  ejecutarIKConPitch(0.0, -38.0, 38.0, -1.5708, 0.0873, 40.0, 0.0);
  delay(400);

  // === PASO 2: Bajar a recogida ===
  ejecutarIKConPitch(0.0, -40.0, 32.0, -1.5708, 0.0873, 40.0, 0.0);
  delay(400);

  // === PASO 3: Cerrar garra ===
  moverGarra(20);
  delay(600);

  // === PASO 4: Subir con objeto ===
  ejecutarIKConPitch(0.0, -38.0, 38.0, -1.5708, 0.0873, 40.0, 0.0);
  delay(400);

  // === PASO 5: Transito a dejada ===
  ejecutarIKConPitch(-32.91, -19.00, 38.0, -2.6180, 0.0873, 40.0, -45.0);
  delay(400);

  // === PASO 6: Bajar a dejada ===
  ejecutarIKConPitch(-34.64, -20.00, 32.0, -2.6180, 0.0873, 40.0, -45.0);
  delay(400);

  // === PASO 7: Soltar ===
  moverGarra(130);
  delay(500);

  // === PASO 8: Retirar dejada ===
  ejecutarIKConPitch(-32.91, -19.00, 38.0, -2.6180, 0.0873, 40.0, -45.0);
  delay(400);

  // === PASO 9: Volver a inicio ===
  ejecutarIKConPitch(0.0, -38.0, 38.0, -1.5708, 0.0873, 40.0, 0.0);
  delay(400);

  Serial.println("[PP] Ciclo completo. En posicion de inicio.");
}

void ejecutarIKConPitch(float xObj, float yObj, float zObj, float phi, float theta, float pitchFijo, float rollFijo) {
  float th1, th2, th3, th4, th5;
  if (!resolverIK(xObj, yObj, zObj, phi, theta, th1, th2, th3, th4, th5)) {
    Serial.println("[IK] Sin solucion.");
    return;
  }
  th4 = pitchFijo;
  th5 = rollFijo;
  moverCincoAngulosSimultaneo(th1, th2, th3, th4, th5);
  Serial.println("[IK] OK.");
}

// ============================================================
// AYUDA
// ============================================================
void imprimirAyuda() {
  Serial.println("=== Brazo 5DOF ===");
  Serial.println("C<ang>             cadera  [-180,180]");
  Serial.println("<ang>              hombro  [0,180]");
  Serial.println("E<ang>             codo    [-150,150]");
  Serial.println("W<ang>             pitch   [0,180]");
  Serial.println("K<ang>             roll    [-180,180]");
  Serial.println("<c>,<h>[,<e>,<p>,<r>]  multi-eje (simultaneo)");
  Serial.println("X<x>,Y<y>,Z<z>[,P<phi>,T<theta>]  IK (cm, grados)");
  Serial.println("A  secuencia  | P  saludo  | R  reposo");
  Serial.println("GO             abrir garra");
  Serial.println("GC             cerrar garra");
  Serial.println("G<50-180>      angulo garra manual");
  Serial.println("PICK X,Y,Z,P,T   recoger objeto");
  Serial.println("PLACE X,Y,Z,P,T  depositar objeto");
  Serial.println("I  ir a inicio pick & place");
  Serial.println("Q  calibrar   | H  ayuda");
  Serial.printf("Pos: C=%.1f H=%.1f E=%.1f W=%.1f R=%.1f\n",
                anguloActualCadera, anguloActualHombro,
                anguloActualCodo, anguloActualPitch, anguloActualRoll);
}
