
//http://www.liutaiomottola.com/formulae/freqtab.htm
const int e=329;
const int f=349;
const int fS=369;
const int g=392;
const int a=440;
const int b=494;
const int c=523;
const int bPin = 3;

int Tempo=(60000/128)*4;

void beep(int note, int duration)
{
 tone(bPin, note,Tempo/duration);
 delay(Tempo/duration);
 noTone(bPin);
 delay(50);
}

void setup() {
 Serial.begin(115200);
 pinMode(bPin, OUTPUT);
}


   void loop()
   {
   firstLine();
   ScndLine();
   ThirdLine() ;
   ForthLine();
   FiftthLine();
   delay(3000);
   }
 


void firstLine()
{
 beep(g, 4);
 beep(b, 16);
 beep(a, 16);    
 beep(b, 4);
 beep(e, 4);
 //-----------------
 delay(Tempo/4);
 delay(Tempo/8);
 beep(c, 16);
 beep(b, 16);
 beep(c, 8);
 beep(b, 8);
 beep(a, 4);
 //-----------------
 delay(Tempo/4);
 delay(Tempo/8);
 beep(c, 16);
 beep(b, 16);    
 beep(c, 4);
 beep(e, 4);
 //-----------------
 beep(fS, 4);
 delay(Tempo/4);
 beep(a, 16);
 beep(g, 16);
 beep(a, 8);
 beep(g, 8);
 beep(fS, 8); //sharp
 beep(a, 8);
 //-----------------
 
}

void ScndLine() {

 
 beep(g, 4);
 delay(Tempo/4);
 beep(b, 16);
 beep(a, 16);    
 beep(b, 4);
 beep(e, 4);
 //-----------------
 delay(Tempo/8);
 delay(Tempo/4);
 beep(c, 16);
 beep(b, 16);
 beep(c, 8);
 beep(b, 8);
 beep(a, 4);
  //-----------------
 delay(Tempo/8);
 delay(Tempo/4);
 beep(b, 16);
 beep(a, 16);    
 beep(b, 4);
 beep(e, 4);
 //-----------------
 beep(fS, 4); //sharp
 delay(Tempo/4);
   beep(a, 16);
 beep(g, 16);    
 beep(a, 8);
 beep(g, 8);
 beep(fS, 8); //sharp
 beep(a, 8);  
}
 
void ThirdLine() {

 beep(g, 3);
 beep(fS, 16);  //sharp
 beep(g, 16);
 beep(a, 3);
 beep(g, 16);
 beep(a, 16);
//-----------------
   beep(b, 8);
 beep(a, 8);    
 beep(g, 8);
 beep(fS, 8); //sharp
 beep(e, 4);
 beep(b, 4);
 //-----------------
 beep(c, 1);
 //-----------------
 //beep(c, 1);
 //-----------------
 beep(g, 4);
 delay(Tempo/4);
 beep(b, 16);
 beep(a, 16);
 beep(b, 4);    
 beep(e, 4);
 //-----------------
 delay(Tempo/8);
 delay(Tempo/4);
 beep(c, 16);
 beep(b, 16);
 beep(c, 8);
 beep(b, 8);
 beep(a, 8);
 //-----------------
}
 
void ForthLine() {
 delay(Tempo/8);
 delay(Tempo/4);
 beep(c, 16);
 beep(b, 16);
 beep(c, 4);
 beep(e, 4);
 //-----------------
 beep(fS, 4);
 delay(Tempo/4);
 beep(a, 16);
 beep(g, 16);
 beep(a, 8);
 beep(g, 8);
 beep(fS, 8); //sharp
 beep(a, 8);
 //-----------------
 beep(g, 4);
 delay(Tempo/4);
 beep(b, 16);
 beep(a, 16);    
 beep(b, 4);
 beep(e, 4);
 //-----------------
 delay(Tempo/8);
 delay(Tempo/4);
 beep(c, 16);
 beep(b, 16);
 beep(c, 8);
 beep(b, 8);
 beep(a, 4);
 //-----------------
 delay(Tempo/8);
 delay(Tempo/4);
  beep(c, 16);
 beep(b, 16);
   beep(c, 4);
   beep(e, 4);
}

void FiftthLine() {
 beep(fS, 4);
 delay(Tempo/4);
 beep(a, 16);
 beep(g, 16);
 beep(a, 8);
 beep(g, 8);
 beep(fS, 8); //sharp
 beep(a, 8);
 //-----------------
   beep(g, 3);
 beep(fS, 16);  //sharp
 beep(g, 16);
 beep(a, 3);
 beep(g, 16);
 beep(a, 16);
//-----------------
   beep(b, 8);
 beep(a, 8);    
 beep(g, 8);
 beep(fS, 8); //sharp
 beep(e, 4);
 beep(b, 4);
 //-----------------
 beep(c, 1);
 //-----------------
}
