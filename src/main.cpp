#include <AccelStepper.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"

/*   Nome do dispositivo Bluetooth/
     "TCC2024LPF" 
    
//////////////////🍀 Comandos uteis 🍀/////////////////////////////////////////////////////
//                                                                                       //
//    📕"H = Ajuda."                                                                   //
//    🚗"(valor)+m = Valor que incrementa o posicionamneto do caro. exemplo: 100m"    //
//    🏃"(valor)+v = Valor que corespondente a velocidade da base. exemplo: 500v"    //
//    🧼"(valor)+l = Valor corespondente . exemplo: 500M"                           //
//    🧯"@ = Apaga variaveis e desativa motores"                                   //
//    🧑‍💻"D = Ativar/Desativar modo Desenvolvedor"                                 //
//                                                                               //
//////////////////////////////////////////////////////////////////////////////////

*/

//Metodo Comunicacao
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


BluetoothSerial SerialBT;

//Pinos
#define PINO_MOTOR_DC1 23//D1
#define PINO_MOTOR_DC2 22//D2
#define PINO_MOTOR_A 19//D5
#define PINO_MOTOR_B 18//D6//sentido
#define ENABLE 21 //D7
#define PINO_FIM_DE_CURSO_1 33//39//11 //D3
#define PINO_FIM_DE_CURSO_2 35//34//D4
#define PINO_FIM_DE_CURSO_3 32//35//D8
#define PINO_FIM_DE_CURSO_4 34//32//D0//nao esta funcinando
#define INTERNAL_LED 2
//Cria motor 
AccelStepper stepper(1, PINO_MOTOR_A, PINO_MOTOR_B);
int VelocidadeBase;

//Constantes
const long REGIAO_ATUACAO_CARRO = 1000;

//ESTADOS DO CARRO
volatile bool MOVIMENTANDO_CARRO = false;
volatile bool LIMPANDO_PAINEL = false;

//Variaveis globais 
char comando;
bool INDOnaLimpesa = 0;
bool VINDOnaLimpesa = 1;
bool EstadoDaLimpesa = INDOnaLimpesa;
bool IndoLimpando = 0;
bool VindoLimpando = 1;
bool SentidoDeLimpesa = IndoLimpando;
int Esfrega;
bool DEV = LOW;
int AnovaPosicao;
String texto;

//Funcoes
String verifica_seriais();
void imprimi_informacoes();
void menu_serial();
void movimentar_carro();
void limpar_painel();
void limite_de_trabalho();


// //Interrupcoes

void IRAM_ATTR carro_fim_inferior()//(1)Fin de curso Caro Inferior -
{
  /*
  stepper.disableOutputs();
      LIMPANDO_PAINEL = false;
      MOVIMENTANDO_CARRO = false;
      EstadoDaLimpesa = INDOnaLimpesa;
      SentidoDeLimpesa = IndoLimpando;
      digitalWrite(ENABLE, HIGH);
      analogWrite(PINO_MOTOR_DC1,0);
 */
  if(MOVIMENTANDO_CARRO)
  { 
    
    stepper.disableOutputs();
    LIMPANDO_PAINEL = false;
    MOVIMENTANDO_CARRO = false;
    digitalWrite(ENABLE, HIGH);
    analogWrite(PINO_MOTOR_DC1,0);//Desativa motor base
  }
  if(LIMPANDO_PAINEL)
  {
     Esfrega=Esfrega*(-1);
     EstadoDaLimpesa=INDOnaLimpesa;
  }
  
}
void IRAM_ATTR carro_fim_superior() //(2)Fin de curso Caro Superior +
{
  /*
  stepper.disableOutputs();
      LIMPANDO_PAINEL = false;
      MOVIMENTANDO_CARRO = false;
      EstadoDaLimpesa = INDOnaLimpesa;
      SentidoDeLimpesa = IndoLimpando;
      digitalWrite(ENABLE, HIGH);
      analogWrite(PINO_MOTOR_DC1,0);
  */
  if(MOVIMENTANDO_CARRO)
  {  
    SentidoDeLimpesa = IndoLimpando;
    EstadoDaLimpesa=INDOnaLimpesa;
    stepper.disableOutputs();
    LIMPANDO_PAINEL = false;
    MOVIMENTANDO_CARRO = false;
    digitalWrite(ENABLE, HIGH);
    analogWrite(PINO_MOTOR_DC1,0);//Desativa motor base
  }
  else if(LIMPANDO_PAINEL)
  {
    Esfrega=Esfrega*(-1);
     //SentidoDeLimpesa = IndoLimpando;
     //stepper.move(100);
     EstadoDaLimpesa=INDOnaLimpesa;
  }
}
void IRAM_ATTR base_fim_inferior() ///Serial.println("(3)Fin de curso BASE Inferior -");
{
 
  analogWrite(PINO_MOTOR_DC1,0);
}
void IRAM_ATTR base_fim_superior()// Serial.println("(4)Fin de curso BASE Superior +");
{
 
  analogWrite(PINO_MOTOR_DC1,0);
}

void setup()
{
  //PINOS
  pinMode(PINO_MOTOR_DC1,OUTPUT);
  pinMode(PINO_MOTOR_DC2,OUTPUT); 
  //analogWriteFreq(1000);
  pinMode(ENABLE, OUTPUT); 
  //FIM DE CURSOS
  pinMode(PINO_FIM_DE_CURSO_1, INPUT_PULLUP);
  attachInterrupt(PINO_FIM_DE_CURSO_1,carro_fim_inferior, RISING);
  pinMode(PINO_FIM_DE_CURSO_2, INPUT_PULLUP);
  attachInterrupt(PINO_FIM_DE_CURSO_2,carro_fim_superior, RISING);
  pinMode(PINO_FIM_DE_CURSO_3,INPUT_PULLUP );
  attachInterrupt(PINO_FIM_DE_CURSO_3,base_fim_inferior, RISING);
  pinMode(PINO_FIM_DE_CURSO_4, INPUT_PULLUP);
  attachInterrupt(PINO_FIM_DE_CURSO_4,base_fim_superior, RISING);
  //LED AZUL INTERNO
  pinMode(INTERNAL_LED,OUTPUT);
  //CONFIGURA SERIAL
  Serial.begin(9600);
  SerialBT.begin("TCC2024LPF"); // Nome do dispositivo Bluetooth
 
  //CONFIGURA MOTOR PASSO
  stepper.setMaxSpeed(2000); //SPEED = Steps / second
  stepper.setAcceleration(1000); //ACCELERATION = Steps /(second)^2
  //stepper.disableOutputs();
  
  //CONFIGURA VELOCIDADE DO MOTOR BASE A 0
  VelocidadeBase=0;
  
  //LIGA OLED AZUL PARA INDICAR A INICIALIZACAO
 digitalWrite(INTERNAL_LED,HIGH);

  //AGUARDA PARA INICIAR
  delay(1000);

  //DESLIGA OLED AZUL PARA INDICAR o FIM DA INICIALIZACAO
 digitalWrite(INTERNAL_LED,LOW);
}

void loop()
{
  //analogWrite(PINO_MOTOR_DC2,1000);
  
  menu_serial();

  //OPERACOES
  if(MOVIMENTANDO_CARRO)
    movimentar_carro();
  //else
  if(LIMPANDO_PAINEL)
   limpar_painel();

}


void menu_serial()//2,0
{
//menu_serial/////////////////////////////////////////////////////////////////////////
//                                  Serial PC
//////////////////////////////////////////////////////////////////////////////////////
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.endsWith("m")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      MOVIMENTANDO_CARRO = true;
      LIMPANDO_PAINEL = false;
      stepper.enableOutputs();
    
      stepper.moveTo(value);
      Serial.println("Movendo Caro");
      AnovaPosicao = value;
      Serial.println(AnovaPosicao);

    } 
    else if (command.endsWith("v")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      VelocidadeBase = value;
      analogWrite(PINO_MOTOR_DC1,VelocidadeBase);
    
      Serial.println("Movimntando Base");

    } 
    else if (command.endsWith("l")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      Esfrega = value;
      LIMPANDO_PAINEL = true;
      MOVIMENTANDO_CARRO = false;

      stepper.enableOutputs();
      //stepper.moveTo(value);
      Serial.println("limpando");

    }
    else if (command.endsWith("@")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      stepper.disableOutputs();
      LIMPANDO_PAINEL = false;
      MOVIMENTANDO_CARRO = false;
      EstadoDaLimpesa = INDOnaLimpesa;
      SentidoDeLimpesa = IndoLimpando;
      digitalWrite(ENABLE, HIGH);
      analogWrite(PINO_MOTOR_DC1,0);
      Serial.println("==PARANDO==");
      Serial.println("Apagou Variavei");
      Serial.println("Desativando Motores");
    }
    else if (command.endsWith("H")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      Serial.println("H = Ajuda.");
      Serial.println("(valor)+m = Valor que incrementa o posicionamneto do caro. exemplo: 100m");
      Serial.println("(valor)+v = Valor que corespondente a velocidade da base. exemplo: 500v");
      Serial.println("(valor)+l = Valor corespondente . exemplo: 500M");
      Serial.println("@ = Apaga variaveis e desativa motores");
      Serial.println("D = Ativar/Desativar modo Desenvolvedor");
    }
    else if (command.endsWith("D")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      if (DEV == LOW)
      {
        Serial.println("   <<  MODO DESENVOLVEDOR ATIVADO >>");
        digitalWrite(INTERNAL_LED,HIGH);
        DEV =  HIGH;
      }
      else
      {
        Serial.println("   <<  MODO DESENVOLVEDOR DESATIVADO >>");
        digitalWrite(INTERNAL_LED,LOW);
        DEV =  LOW;
      }
      
      
      
     
    }
  }
//menu_serial/////////////////////////////////////////////////////////////////////////
//                                  Serial BT
//////////////////////////////////////////////////////////////////////////////////////
  
  if (SerialBT.available())
  {
    String command = SerialBT.readStringUntil('\n');
    command.trim();
    
    if (command.endsWith("m")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      MOVIMENTANDO_CARRO = true;
      LIMPANDO_PAINEL = false;
      //stepper.distanceToGo(novaPosicao);
      stepper.enableOutputs();
    
      stepper.moveTo(value);
      //stepper.moveTo(novaPosicao);
      SerialBT.println("Movendo Caro");
      AnovaPosicao = value;
      SerialBT.println(AnovaPosicao);

    } 
    else if (command.endsWith("v")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      VelocidadeBase = value;
      analogWrite(PINO_MOTOR_DC1,VelocidadeBase);
    
      SerialBT.println("Movimntando Base");

    } 
    else if (command.endsWith("l")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      Esfrega = value;
      LIMPANDO_PAINEL = true;
      MOVIMENTANDO_CARRO = false;

      stepper.enableOutputs();
      //stepper.moveTo(value);
      SerialBT.println("limpando");

    }
    else if (command.endsWith("@")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      stepper.disableOutputs();
      LIMPANDO_PAINEL = false;
      MOVIMENTANDO_CARRO = false;
      EstadoDaLimpesa = INDOnaLimpesa;
      SentidoDeLimpesa = IndoLimpando;
      digitalWrite(ENABLE, HIGH);
      analogWrite(PINO_MOTOR_DC1,0);
      SerialBT.println("==PARANDO==");
      SerialBT.println("Apagou Variavei");
      SerialBT.println("Desativando Motores");
    }
    else if (command.endsWith("H")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      SerialBT.println("H = Ajuda.");
      SerialBT.println("(valor)+m = Valor que incrementa o posicionamneto do caro. exemplo: 100m");
      SerialBT.println("(valor)+v = Valor que corespondente a velocidade da base. exemplo: 500v");
      SerialBT.println("(valor)+l = Valor corespondente . exemplo: 500M");
      SerialBT.println("@ = Apaga variaveis e desativa motores");
      SerialBT.println("D = Ativar/Desativar modo Desenvolvedor");
    }
    else if (command.endsWith("D")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      if (DEV == LOW)
      {
        SerialBT.println("   <<  MODO DESENVOLVEDOR ATIVADO >>");
        digitalWrite(INTERNAL_LED,HIGH);
        DEV =  HIGH;
      }
      else
      {
        SerialBT.println("   <<  MODO DESENVOLVEDOR DESATIVADO >>");
        digitalWrite(INTERNAL_LED,LOW);
        DEV =  LOW;
      }
    }
  }
}
/*
String verifica_seriais() // verifica as seriais 
{
  if (Serial.available()) {
    SerialBT.write(Serial.read());

  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
 
  }
return "";
}
*/
//////////////////////////////////////////////////////////////////////////////////////
//                             Movimeto do Carro
//////////////////////////////////////////////////////////////////////////////////////

void movimentar_carro() //Testar
{
  if(stepper.currentPosition() == stepper.targetPosition())//(stepper.distanceToGo() == 0)
  {
      int novaPosicao=0;
      stepper.disableOutputs();
      LIMPANDO_PAINEL = false;
      MOVIMENTANDO_CARRO = false;
      digitalWrite(ENABLE, HIGH);
       texto ="";//nada
      
  }
  else
  {
    digitalWrite(ENABLE, LOW);
    stepper.run();

    if(DEV == HIGH)//modo desenvolvedor
    {
      Serial.print("=_");Serial.print(stepper.distanceToGo());Serial.print("_=_");Serial.print(stepper.targetPosition());Serial.print("_=_");Serial.println(stepper.currentPosition());
      SerialBT.print("=_");SerialBT.print(stepper.distanceToGo());SerialBT.print("_=_");SerialBT.print(stepper.targetPosition());SerialBT.print("_=_");SerialBT.println(stepper.currentPosition());
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
//                             Execução da Limpesa
//////////////////////////////////////////////////////////////////////////////////////
void limpar_painel() 
{
  //if (SentidoDeLimpesa==IndoLimpando)// limite de limpeza alcançado
 // {
    if(EstadoDaLimpesa==INDOnaLimpesa)
    {
      if (stepper.distanceToGo() == 0)
      {
      stepper.move(-1 * (Esfrega));
      EstadoDaLimpesa=VINDOnaLimpesa;
      }
      else
      {
      digitalWrite(ENABLE, LOW);
      stepper.run();

      if(DEV == HIGH)//modo desenvolvedor
      {
        Serial.print("Ii=_");Serial.print(stepper.distanceToGo());Serial.print("_=_");Serial.print(stepper.targetPosition());Serial.print("_=_");Serial.println(stepper.currentPosition());
        SerialBT.print("Ii=_");SerialBT.print(stepper.distanceToGo());SerialBT.print("_=_");SerialBT.print(stepper.targetPosition());SerialBT.print("_=_");SerialBT.println(stepper.currentPosition());
      }    
      }
   }
    else if(EstadoDaLimpesa==VINDOnaLimpesa)
    {
      if (stepper.distanceToGo()==0)
      {
        
      stepper.move(Esfrega * 2);
      EstadoDaLimpesa=INDOnaLimpesa;
      }
      else{
      digitalWrite(ENABLE, LOW);
      stepper.run();

      if(DEV == HIGH)//modo desenvolvedor
      {
        Serial.print("Iv=_");Serial.print(stepper.distanceToGo());Serial.print("_=_");Serial.print(stepper.targetPosition());Serial.print("_=_");Serial.println(stepper.currentPosition());
        SerialBT.print("Iv=_");SerialBT.print(stepper.distanceToGo());SerialBT.print("_=_");SerialBT.print(stepper.targetPosition());SerialBT.print("_=_");SerialBT.println(stepper.currentPosition());
      }
      }
    } 

  }

  /*
  if(SentidoDeLimpesa==VindoLimpando){

    if(EstadoDaLimpesa == NDOnaLimpesa)
    {
      if (stepper.distanceToGo() == 0)
      {
      stepper.move(-1*(Esfrega * 2));
      EstadoDaLimpesa=VINDOnaLimpesa;
      }
      else
      {
      digitalWrite(ENABLE, LOW);
      stepper.run();

        if(DEV == HIGH)//modo desenvolvedor
        {
        Serial.print("Vi=_");Serial.print(stepper.distanceToGo());Serial.print("_=_");Serial.print(stepper.targetPosition());Serial.print("_=_");Serial.println(stepper.currentPosition());
        SerialBT.print("Vi=_");SerialBT.print(stepper.distanceToGo());SerialBT.print("_=_");SerialBT.print(stepper.targetPosition());SerialBT.print("_=_");SerialBT.println(stepper.currentPosition());
        }    
      }
    }
    else if(EstadoDaLimpesa==VINDOnaLimpesa)
    {
      if (stepper.distanceToGo()==0)
      {
        
      stepper.move(Esfrega);
      EstadoDaLimpesa=INDOnaLimpesa;
      }
      else{
      digitalWrite(ENABLE, LOW);
      stepper.run();

        if(DEV == HIGH)//modo desenvolvedor
        {
        Serial.print("Vv=_");Serial.print(stepper.distanceToGo());Serial.print("_=_");Serial.print(stepper.targetPosition());Serial.print("_=_");Serial.println(stepper.currentPosition());
        SerialBT.print("Vv=_");SerialBT.print(stepper.distanceToGo());SerialBT.print("_=_");SerialBT.print(stepper.targetPosition());SerialBT.print("_=_");SerialBT.println(stepper.currentPosition());
        }
      }
    } 
  }*/
//}

void limite_de_trabalho(){
// se o limie superior for atinguido (stepper.targetPosition)
    //Sentidodelimpesa muda
    //...
//se...

}


