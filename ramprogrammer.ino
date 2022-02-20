


// instruction  pins
const int I0 = 2;
const int I1 = 3;
const int I2 = 4;
const int I3 = 5;

// Data pins
const int D0 = 6;
const int D1 = 7;
const int D2 = 8;
const int D3 = 9;


// control pins
const int CLEAR_PIN = 10;
const int STOP_PIN = 11;
const int LOAD_PIN = 12;



void Load() {
  digitalWrite(LOAD_PIN, LOW);
  delay(500);
  digitalWrite(LOAD_PIN, HIGH);
}

void Clear() {
  digitalWrite(CLEAR_PIN, LOW);
  delay(500);
  digitalWrite(CLEAR_PIN, HIGH);
}


void ProgramRAM(bool I0Val,  bool I1Val,  bool I2Val,  bool I3Val,  bool D0Val,  bool D1Val,  bool D2Val,  bool D3Val) {
  digitalWrite(I0, I0Val);
  digitalWrite(I1, I1Val);
  digitalWrite(I2, I2Val);
  digitalWrite(I3, I3Val);

  digitalWrite(D0, D0Val);
  digitalWrite(D1, D1Val);
  digitalWrite(D2, D2Val);
  digitalWrite(D3, D3Val);
  delay(500);
  Load();
}


void setup() {

  pinMode(I0, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(I3, OUTPUT);

  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);

  pinMode(CLEAR_PIN, OUTPUT);
  pinMode(STOP_PIN, OUTPUT);
  pinMode(LOAD_PIN, OUTPUT);
  digitalWrite(CLEAR_PIN, HIGH);
  digitalWrite(STOP_PIN, HIGH);
  digitalWrite(LOAD_PIN, HIGH);
  

}

void loop() {
  // put your main code here, to run repeatedly:

  delay(1000);
  digitalWrite(STOP_PIN, LOW);
  Clear();
 delay(1000);
    ProgramRAM(1,0,1,0,1,1,1,1);
    
  digitalWrite(STOP_PIN, HIGH);
  

}
