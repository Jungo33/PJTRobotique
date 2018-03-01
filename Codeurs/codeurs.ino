int pinA = 3;   // Le port D2 est associé à l'interruption 0
int pinB = 4;
volatile int pos = 0;  // Position (en nombre de pas) du codeur
 
void setup()  {
   Serial.begin(9600);
   Serial.println("Codeur incremental");
   pinMode(pinB, INPUT);
   attachInterrupt(INT1, front, CHANGE);  // Détection des fronts 
}
 
void loop()   {
   delay(10);
}
 
void front()   {
   int sA = digitalRead(pinA);
   int sB = digitalRead(pinB);
   
   if (sA == sB)   {
      ++pos;
   }
   else   {
      --pos;
   }
   Serial.println(pos);
}
