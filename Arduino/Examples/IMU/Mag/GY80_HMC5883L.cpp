// Programa : Teste HMC5883L - Bussola
// Adaptacoes : Arduino e Cia

#include <Wire.h>

// Define o endereco do HMC5883 - 0x1E ou 30 em decimal
#define address 0x1E 

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  // Inicializa o HMC5883
  Wire.beginTransmission(address);
  // Seleciona o modo
  Wire.write(0x02); 
  // Modo de medicao continuo
  Wire.write(0x00); 
  Wire.endTransmission();
}

void loop()
{
  int x,y,z; //triple axis data
  
  // Indica ao HMC5883 para iniciar a leitura
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
 
  // Le os dados de cada eixo, 2 registradores por eixo
  Wire.requestFrom(address, 6);
  if(6<=Wire.available())
  {
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  
  // Imprime os vaores no serial monitor
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.println(z);
  
  delay(250);
}
