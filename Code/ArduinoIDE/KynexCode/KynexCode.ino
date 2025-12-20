//231859 , 231802 , 231775

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ---------------- CONFIG ----------------
#define SERVO_MIN  50
#define SERVO_MAX  3200
#define SERVO_FREQ 50

int angleToPulse(int angle)
{
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}


// ================= WIFI =================
const char* ssid = "QuadrupedDog";
const char* password = "12345678";

WebServer server(80);


// ---------------- CHANNEL MAP ----------------
#define KNEE_FL 0
#define HIP_FL  1
#define KNEE_FR 2
#define HIP_FR  3
#define KNEE_BL 4
#define HIP_BL  5
#define KNEE_BR 6
#define HIP_BR  7


#define BODY_ROTATE 8
#define NECK_UD     9

#define TRIG_PIN 12
#define ECHO_PIN 14
#define BARK_PIN 27
#define Bark_Full 26

// ---------------- KNEE DIRECTION ----------------
#define KNEE_FL_DIR -1
#define KNEE_FR_DIR +1
#define KNEE_BL_DIR -1
#define KNEE_BR_DIR +1

// ---------------- SITTING ----------------
int sitAngles[8] =
{
  0,   180,
  180, 0,
  180, 0,
  0,   180
};

// ---------------- STANDING ----------------
int standAngles[8] =
{
  98,  148,
  90,  50,
  110, 60,
  100, 150
};

// ---------------- HALFSIT ----------------
int Halfsit[8] =
{
  150, 95,
  35,  90,
  180, 0,
  0,  180
};

// ---------------- GAIT CONTROL ----------------
bool trotEnabled = false;
char turnMode = 'S';   // S=straight, L=left, R=right


int trotStepDelay = 80;   // SPEED CONTROL
int kneeLift      = 12;
int hipRelief     = 3;

// ---------- WEB SPEED CONTROL ----------
int webSpeed = 80;              // default trot speed (ms)

// ===================================================
//                 LOW LEVEL SERVO
// ===================================================
void setServo(int ch, int angle)
{
  pwm.writeMicroseconds(ch, angleToPulse(angle));
  delay(5);
}

// ===================================================
//                    SIT
// ===================================================
void goSit(int stepDelay = 60)
{
  int cur[8];
  for(int i=0;i<8;i++) cur[i]=standAngles[i];
  bool moving=true;

  while(moving)
  {
    moving=false;
    for(int ch : {HIP_FL, HIP_FR, HIP_BL, HIP_BR})
    {
      if(cur[ch]<sitAngles[ch]) cur[ch]++;
      else if(cur[ch]>sitAngles[ch]) cur[ch]--;
      if(cur[ch]!=sitAngles[ch]) moving=true;
      setServo(ch,cur[ch]);
    }
    delay(stepDelay);
  }

  delay(120);

  moving=true;
  while(moving)
  {
    moving=false;
    for(int ch : {KNEE_FL, KNEE_FR, KNEE_BL, KNEE_BR})
    {
      if(cur[ch]<sitAngles[ch]) cur[ch]++;
      else if(cur[ch]>sitAngles[ch]) cur[ch]--;
      if(cur[ch]!=sitAngles[ch]) moving=true;
      setServo(ch,cur[ch]);
    }
    delay(stepDelay);
  }
}

// ===================================================
//                  STAND
// ===================================================
void goStandFrontBack(int stepDelay = 60)
{
  Serial.println("STAND");

  int cur[8];
  for(int i=0;i<8;i++) cur[i]=sitAngles[i];
  bool moving;

  // FRONT HIPS
  moving=true;
  while(moving)
  {
    moving=false;
    for(int ch : {HIP_FL, HIP_FR})
    {
      if(cur[ch]<standAngles[ch]) cur[ch]++;
      else if(cur[ch]>standAngles[ch]) cur[ch]--;
      if(cur[ch]!=standAngles[ch]) moving=true;
      setServo(ch,cur[ch]);
    }
    delay(stepDelay);
  }









  // BACK HIPS
  moving=true;
  while(moving)
  {
    moving=false;
    for(int ch : {HIP_BL, HIP_BR})
    {
      if(cur[ch]<standAngles[ch]) cur[ch]++;
      else if(cur[ch]>standAngles[ch]) cur[ch]--;
      if(cur[ch]!=standAngles[ch]) moving=true;
      setServo(ch,cur[ch]);
    }
    delay(stepDelay);
  }

  // FRONT KNEES
  moving=true;
  while(moving)
  {
    moving=false;
    for(int ch : {KNEE_FL, KNEE_FR})
    {
      if(cur[ch]<standAngles[ch]) cur[ch]++;
      else if(cur[ch]>standAngles[ch]) cur[ch]--;
      if(cur[ch]!=standAngles[ch]) moving=true;
      setServo(ch,cur[ch]);
    }
    delay(stepDelay);
  }

  // BACK KNEES
  moving=true;
  while(moving)
  {
    moving=false;
    for(int ch : {KNEE_BL, KNEE_BR})
    {
      if(cur[ch]<standAngles[ch]) cur[ch]++;
      else if(cur[ch]>standAngles[ch]) cur[ch]--;
      if(cur[ch]!=standAngles[ch]) moving=true;
      setServo(ch,cur[ch]);
    }
    delay(stepDelay);
  }
}

// ===================================================
//                HALF SIT 
// ===================================================
void goHalfSit(int stepDelay = 60)
{
  Serial.println("HALF SIT");

  int cur[8];
  for(int i=0;i<8;i++) cur[i]=standAngles[i];
  bool moving=true;

  // ---- ALL HIPS ----
  while(moving)
  {
    moving=false;
    for(int ch : {HIP_FL, HIP_FR, HIP_BL, HIP_BR})
    {
      if(cur[ch]<Halfsit[ch]) cur[ch]++;
      else if(cur[ch]>Halfsit[ch]) cur[ch]--;
      if(cur[ch]!=Halfsit[ch]) moving=true;
      setServo(ch,cur[ch]);
    }
    delay(stepDelay);
  }

  delay(120);

  // ---- ALL KNEES ----
  moving=true;
  while(moving)
  {
    moving=false;
    for(int ch : {KNEE_FL, KNEE_FR, KNEE_BL, KNEE_BR})
    {
      if(cur[ch]<Halfsit[ch]) cur[ch]++;
      else if(cur[ch]>Halfsit[ch]) cur[ch]--;
      if(cur[ch]!=Halfsit[ch]) moving=true;
      setServo(ch,cur[ch]);
    }
    delay(stepDelay);
  }
}

// ===================================================
//              TROT STEP (WITH HIP RELIEF)
// ===================================================
void trotDiagonal(int kneeA, int kneeB, int hipA, int hipB)
{
  // HIP RELIEF
  setServo(hipA, standAngles[hipA] + hipRelief);
  setServo(hipB, standAngles[hipB] + hipRelief);

  for(int i=0;i<=kneeLift;i++)
  {
    int liftA = (turnMode=='L' && kneeA==KNEE_FR) ? i/2 : i;
    int liftB = (turnMode=='R' && kneeB==KNEE_BL) ? i/2 : i;

    setServo(kneeA, standAngles[kneeA] + liftA);
    setServo(kneeB, standAngles[kneeB] + liftB);
    delay(trotStepDelay);
  }

  // REMOVE HIP RELIEF
  setServo(hipA, standAngles[hipA]);
  setServo(hipB, standAngles[hipB]);

  for(int i=kneeLift;i>=0;i--)
  {
    setServo(kneeA, standAngles[kneeA] + i);
    setServo(kneeB, standAngles[kneeB] + i);
    delay(trotStepDelay);
  }
}

// ===================================================
//                TROT LOOP
// ===================================================
void trotLoop()
{
  if(!trotEnabled) return;

  trotDiagonal(KNEE_FL, KNEE_BR, HIP_FR, HIP_BL);
  trotDiagonal(KNEE_FR, KNEE_BL, HIP_FL, HIP_BR);
}


// ===================================================
//                DANCE 
// ===================================================

void dance(int stepDelay = 60, int loops = 4)
{
    int cur[10];  // 8 legs + body + neck
    for(int i=0;i<8;i++) cur[i] = standAngles[i];
    cur[BODY_ROTATE] = 90;
    cur[NECK_UD] = 90;

    for(int l=0;l<loops;l++)
    {
        // Forward swing hips + body/neck
        for(int step=0; step<=5; step++)
        {
            for(int i : {HIP_FL, HIP_FR, HIP_BL, HIP_BR})
                setServo(i, standAngles[i] + step);
            
            // Knees stay at standing angles
            for(int i : {KNEE_FL, KNEE_FR, KNEE_BL, KNEE_BR})
                setServo(i, standAngles[i]);

            setServo(BODY_ROTATE, 90 + step*9); // 45->135 approx
            setServo(NECK_UD, 90 + step*9);    // 45->135 approx

            delay(stepDelay);
        }

        // Backward swing hips + body/neck
        for(int step=5; step>=-5; step--)
        {
            for(int i : {HIP_FL, HIP_FR, HIP_BL, HIP_BR})
                setServo(i, standAngles[i] + step);

            for(int i : {KNEE_FL, KNEE_FR, KNEE_BL, KNEE_BR})
                setServo(i, standAngles[i]);

            setServo(BODY_ROTATE, 90 + step*9); 
            setServo(NECK_UD, 90 + step*9);

            delay(stepDelay);
        }

        // Return hips to standing
        for(int step=-5; step<=0; step++)
        {
            for(int i : {HIP_FL, HIP_FR, HIP_BL, HIP_BR})
                setServo(i, standAngles[i] + step);

            for(int i : {KNEE_FL, KNEE_FR, KNEE_BL, KNEE_BR})
                setServo(i, standAngles[i]);

            setServo(BODY_ROTATE, 90 + step*9);
            setServo(NECK_UD, 90 + step*9);

            delay(stepDelay);
        }
    }
}
 
// ===================================================
//          ULTRASONIC SENSOR AND DISTANCE 
// ===================================================

void setupUltrasonic()
{
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    digitalWrite(TRIG_PIN, LOW);

    pinMode(BARK_PIN, OUTPUT);
    digitalWrite(BARK_PIN, LOW);

    pinMode(Bark_Full, OUTPUT);
    digitalWrite(Bark_Full, LOW);
}

long readDistanceCM()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
    long distance = duration * 0.034 / 2; // cm
    return distance;
}

//Call this in loop() periodically
void checkObstacle()
{
    long dist = readDistanceCM();
    Serial.print("Distance: ");
    Serial.print(dist);
    Serial.println(" cm");

    if(dist > 0 && dist < 10) // valid measurement <10cm
    {
        Bark(HIGH);
    }
    else
    {
        Bark(LOW);
    }
}

// ===================================================
//                BARK
// ===================================================

void Bark(bool x)
{
  if(x==HIGH)
  {
    digitalWrite(BARK_PIN, HIGH);
    Serial.print("Barking...");
  }
  
  
  else 
  digitalWrite(BARK_PIN, LOW);

}
void Barking()
{
  digitalWrite(Bark_Full, HIGH);
}

// ===================================================
//                Move LEFT / RIGHT
// ===================================================
void MoveLeft(int stepDelay = 30)
{
    int cur = 90; 
    while(cur < 110)
    {
        cur++;
        setServo(BODY_ROTATE, cur);
        delay(stepDelay);
    }
}

void MoveRight(int stepDelay = 30)
{
    int cur = 90; 
    while(cur > 70)
    {
        cur--;
        setServo(BODY_ROTATE, cur);
        delay(stepDelay);
    }
}

void MoveCenter(int stepDelay = 30)
{
    int cur = 0;     
    cur = 90; 

    // smooth transition to 90
    int pos = cur;
    if(pos < 90)
    {
        while(pos < 90)
        {
            pos++;
            setServo(BODY_ROTATE, pos);
            delay(stepDelay);
        }
    }
    else if(pos > 90)
    {
        while(pos > 90)
        {
            pos--;
            setServo(BODY_ROTATE, pos);
            delay(stepDelay);
        }
    }
    else
    {
        setServo(BODY_ROTATE, 90); // already centered
    }
}


// ===================================================
//                  WEBPAGE
// ===================================================
void handleCommand(char cmd)
{
  switch(cmd)
  {
    case '1': goSit(); break;
    case '2': goStandFrontBack(); break;
    case '3': trotEnabled = true; break;
    case '4': trotEnabled = false; break;
    case '5': goHalfSit(); break;

    case 'L': turnMode = 'L'; break;
    case 'R': turnMode = 'R'; break;

    case 'Q': MoveLeft(); break;
    case 'W': MoveRight(); break;
    case 'C': MoveCenter(); break;

    case 'D': dance(); break;
    case 'B': Barking(); break;
  }
}

void setSpeedFromWeb(int value)
{
  webSpeed = constrain(value, 30, 200);   // safe range
  trotStepDelay = webSpeed;
}

// ===================================================
//                  HTML & JAVA
// ===================================================

void handleRoot()
{
  String html =
    "<!DOCTYPE html>\n"
   "<html>\n"
   "<head>\n"
 "  <title>Quadruped Control</title>\n"
  "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
 "  <style>\n"
 "    body { font-family: Arial; text-align: center; background:#111; color:#fff; }\n"
 "    button {\n"
 "      width: 120px; height: 45px;\n"
 "      margin: 6px; font-size: 16px;\n"
 "      border-radius: 8px; border: none;\n"
 "    }\n"
  "  </style>\n"
 "</head>\n"
 "<body>\n"
 "<h2> Quadruped Robot Control</h2>\n"
 "<h3>Posture</h3>\n"
 "<button onclick=\"send('1')\">Sit</button>\n"
 "<button onclick=\"send('2')\">Stand</button>\n"
 "<button onclick=\"send('5')\">Half Sit</button>\n"
 "<h3>Gait</h3>\n"
 "<button onclick=\"send('3')\">Start Trot</button>\n"
 "<button onclick=\"send('4')\">Stop Trot</button>\n"
 "<h3>Turning</h3>\n"
 "<button onclick=\"send('L')\">Left</button>\n"
 "<button onclick=\"send('R')\">Right</button>\n"
 "<h3>Speed Control</h3>\n"
 "<input type=\"range\" min=\"30\" max=\"200\" value=\"80\"\n"
 "       oninput=\"setSpeed(this.value)\">\n"
 "<p>Speed: <span id=\"sp\">80</span> ms</p>\n"
 "<h3>Body</h3>\n"
 "<button onclick=\"send('Q')\">Move Left</button>\n"
 "<button onclick=\"send('W')\">Move Right</button>\n"
 "<button onclick=\"send('C')\">Move Center</button>\n"
 "<h3>Actions</h3>\n"
 "<button onclick=\"send('D')\">Dance</button>\n"
 "<button onclick=\"send('B')\">Bark</button>\n"
 "<script>\n"
 "function send(cmd)\n"
 "{\n"
 "  fetch(\"/cmd?c=\" + cmd);\n"
 "}\n"
 "function setSpeed(v)\n"
 "{\n"
 "  document.getElementById(\"sp\").innerHTML = v;\n"
 "  fetch(\"/speed?v=\" + v);\n"
 "}\n"
 "</script>\n"
  "</body>\n"
  "</html>\n";

  server.send(200, "text/html", html);
}


// ===================================================
//                  SETUP
// ===================================================
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(300);

  for(int ch=0; ch<8; ch++)
    setServo(ch, sitAngles[ch]);

  // -------- SPIFFS --------
  if(!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  // -------- WIFI AP --------
  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // -------- ROOT WEBPAGE --------
  server.on("/", handleRoot);

  // -------- COMMAND API --------
  server.on("/cmd", HTTP_GET, []()
  {
    if(server.hasArg("c"))
      handleCommand(server.arg("c")[0]);

    server.send(200, "text/plain", "OK");
  });

  // -------- SPEED API --------
  server.on("/speed", HTTP_GET, []()
  {
    if(server.hasArg("v"))
      setSpeedFromWeb(server.arg("v").toInt());

    server.send(200, "text/plain", "OK");
  });

  server.begin();

  setupUltrasonic();
  Serial.println("READY");
}


// ===================================================
//                   LOOP
// ===================================================
void loop()
{
  checkObstacle();
  server.handleClient();

if(Serial.available())
  handleCommand(Serial.read());

  trotLoop();
}
