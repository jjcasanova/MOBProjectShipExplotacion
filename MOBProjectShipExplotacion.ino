#include "Arduino.h"
#include <VirtualWire.h>
#include <stdlib.h>
#include <stdio.h>

#include <Wire.h>                   // Se importan las librerias
#include <LiquidCrystal_I2C.h>

                    //Addr, En, Rw, Rs, d4, d5, d6, d7, luz de fondo, polaridad
LiquidCrystal_I2C lcd(0x27, 2 , 1 , 0 , 4 , 5 , 6 , 7 ,      3 ,      POSITIVE);

int dato=0;

//double alfa;
//double charlie=0;
//double Lat=0;
double lnt;
double LatDividido;
double LongDividido;
double DiferenciaLongDividido;
double LatsDividido;
double LongsDividido;
double DiferenciaLongsDividido;
double rmobDividido;
double veloDividido;
double LATMOB;
double LONMOB;
double LATSHIP;
double LONSHIP;
double DISTANCIA;
double CURSO;
//double rmob;
double velo;
void setup()
{
 lcd.begin(16,2);
Serial.begin(9600); //Se inicia la comunicación serial
vw_set_ptt_inverted(true);
vw_setup(2000); //Tiempo de espera
vw_set_rx_pin(11); //Pin donde se conecta el Arduino
vw_rx_start(); //Se inicia la recepción de datos
}

void loop()
{
uint8_t buf[VW_MAX_MESSAGE_LEN];         //Tama�o del mensaje
uint8_t buflen = VW_MAX_MESSAGE_LEN;
if (vw_get_message(buf, &buflen))       //Se verifica si hay mensaje disponible para ser leido
{
 int i;
char *matrizAlfa;
String cadenaAlfa;
for (i = 0; i < buflen; i++)               //Se leen todos los caracteres
{
Serial.write(buf[i]);                     //Se guarda la informacion en una matriz
cadenaAlfa.concat((char)buf[i]);          // Se concatena a la cadena de caracteres
}
cadenaAlfa.toCharArray(matrizAlfa,cadenaAlfa.length()+1);
float alfa;
alfa=cadenaAlfa.toFloat();
double charlie;
charlie=(alfa+0);
Serial.println(charlie);                      //saca la variable charlie que es la que llega sin error



if (charlie>0 && charlie <10000)                    // rango de velocidades
{double velo;(velo = charlie);
Serial.print('\n');
veloDividido=velo*(0.01);
Serial.print('\n');                                                              // vuelve a darnos decimales
}
else
if (charlie>10000 && charlie <360000)                    // rango de rumbo
{double rmob;(rmob = charlie);
Serial.print('\n');
rmobDividido=rmob*(0.0001);
Serial.print('\n');                                                              // vuelve a darnos decimales
}
else                                                     // discrimina cual de ellas es longitud y cual latitud
if (charlie>20000000 && charlie <45000000)                    // rango de latitud en nuestra zona
{double Lat;(Lat = charlie);
Serial.print('\n');
LatDividido=Lat*(0.000001);
Serial.print('\n');                                                              // vuelve a darnos decimales
}
else
if (90000000> charlie && charlie>45000000)
{double lnt;(lnt = charlie);
Serial.print('\n');
LongDividido=lnt*(0.000001);                                     // vuelve a darnos decimales
DiferenciaLongDividido=LongDividido-60;             // vuelve a su rango incluyendo negativos, (W)
Serial.print('\n');
  }
else
if (450000000> charlie && charlie >90000000)
{double Lats;(Lats = charlie);
Serial.print('\n');
LatsDividido=Lats*(0.0000001);                    // saca ya las variables latitud y longitud corregidas
double LATSHIP; LATSHIP=LatsDividido;
}
else
if (charlie >450000000)
{double lnts;(lnts = charlie);
Serial.print('\n');
LongsDividido=lnts*(0.0000001);
DiferenciaLongsDividido=LongsDividido-60;
}

double lat1 =LatsDividido;
double long1=DiferenciaLongsDividido;
double lat2=LatDividido;
double long2=DiferenciaLongDividido;
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom); Serial.println("delta  :    "); Serial.println(delta,8);
  DISTANCIA=(delta*6372795);

double tiempo; tiempo = (DISTANCIA/(((veloDividido+11)*1000)/60));  //si no se mueve son unos 5,5kn CORREGIR
Serial.print('\n');
Serial.print("tiempo necesario para volver: "); Serial.print(tiempo,2);
Serial.print('\n');
int miR; miR = (rmobDividido+121);  //si no se mueve le metemos 121 CORREGIR
Serial.print('\n');
Serial.print("mi rumbo: "); Serial.print(miR,0);
Serial.print('\n');



 // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  int curso;
  CURSO = degrees(a2); curso=(CURSO);



Serial.print('\n');
LATMOB=LatDividido;
Serial.print("LATMOB: "); Serial.print(LATMOB,8);
LONMOB=DiferenciaLongDividido;
Serial.print('\n');
Serial.print("LONMOB: "); Serial.print(LONMOB,8);
Serial.print('\n');

Serial.print('\n');
LATSHIP=LatsDividido;
Serial.print("LATSHIP: "); Serial.print(LATSHIP,8);
LONSHIP=DiferenciaLongsDividido;
Serial.print('\n');
Serial.print("LONSHIP: "); Serial.print(LONSHIP,8);
Serial.print('\n');




Serial.print('\n');
Serial.print("distancia:   "); Serial.print(DISTANCIA,0);
Serial.print('\n');
Serial.print("CURSO    :   "); Serial.print(CURSO,0);
Serial.print('\n');
lcd.clear();
lcd.setCursor(10,0);             // Ponemos el cursor en la posicion x=0 e y=0
  lcd.print("d");  // Imprimimos un mensaje
  lcd.setCursor(12,0);
  lcd.print(DISTANCIA);
     lcd.setCursor(0,0);
  lcd.print("MOB::");
    lcd.setCursor(0,1);
  lcd.print("miR::");
      lcd.setCursor(5,0);
      lcd.print(curso);
       lcd.setCursor(5,1);
      lcd.print(miR);
     lcd.setCursor(10,1);
  lcd.print("t");
  lcd.setCursor(12,1);
  lcd.print(tiempo);
delay(2000);

//codigo para encender led,s

}
}

