#include <AccelStepper.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"
#include <neotimer.h>

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

//Timer
Neotimer mytimer = Neotimer(10000); // esta em milisegundos

//Pinos
#define PINO_MOTOR_DC1 23//D1
#define PINO_MOTOR_DC2 22//D2
#define PINO_MOTOR_A 19//D5
#define PINO_MOTOR_B 18//D6//sentido
#define ENABLE 21 //D7
#define PINO_FIM_DE_CURSO_1 33//39//11 //D3
#define PINO_FIM_DE_CURSO_2 35//34//D4
#define PINO_FIM_DE_CURSO_3 32//35//D8
#define PINO_FIM_DE_CURSO_4 34//32//D0
#define INTERNAL_LED 2
//Cria motor 
AccelStepper stepper(1, PINO_MOTOR_A, PINO_MOTOR_B);
int VelocidadeBase;

//Constantes
//////////////////////const long REGIAO_ATUACAO_CARRO = 1000;

//ESTADOS DO CARRO
volatile bool MOVIMENTANDO_CARRO = false;
volatile bool LIMPANDO_PAINEL = false;

//Variaveis globais 
char comando;
bool INDOnaLimpesa = 0;
bool VINDOnaLimpesa = 1;
volatile bool EstadoDaLimpesa = INDOnaLimpesa;
const bool IndoLimpando = 0;
const bool VindoLimpando = 1;
volatile bool SentidoDeLimpesa = IndoLimpando;
int Esfrega;
bool DEV = LOW;
int AnovaPosicao;
String texto;
int contador;

//Variáveis de debounce de interrupção
unsigned long tempo_int_fc1 = 0;
unsigned long tempo_int_fc2 = 0;
//unsigned long tempo_int_fc3 = 0;
//unsigned long tempo_int_fc4 = 0;  
unsigned long ultimo_tempo_int_fc1 = 0;
unsigned long ultimo_tempo_int_fc2 = 0;
//unsigned long ultimo_tempo_int_fc3 = 0;
//unsigned long ultimo_tempo_int_fc4 = 0;



//Funcoes
String verifica_seriais();
void imprimi_informacoes();
void menu_serial();
void movimentar_carro();
void limpar_painel();
void limite_de_trabalho();
void verifica_timer();

// //Interrupcoes

void IRAM_ATTR carro_fim_inferior()//(1)Fin de curso Caro Inferior -
{
  tempo_int_fc1 = millis();
  if (tempo_int_fc1 - ultimo_tempo_int_fc1 > 1000)
  {  
    if(MOVIMENTANDO_CARRO)
    { 
      
      stepper.disableOutputs();
      LIMPANDO_PAINEL = false;
      MOVIMENTANDO_CARRO = false;
      digitalWrite(ENABLE, HIGH);
      analogWrite(PINO_MOTOR_DC1,0);//Desativa motor base
    }
    else//if(LIMPANDO_PAINEL)
    {
    
      //digitalWrite(INTERNAL_LED, HIGH);  
      // digitalWrite(PINO_MOTOR_B, LOW);  
    
      SentidoDeLimpesa = VindoLimpando;
      // EstadoDaLimpesa=INDOnaLimpesa;
    }
  }
  ultimo_tempo_int_fc1 = tempo_int_fc1;
}
void IRAM_ATTR carro_fim_superior() //(2)Fin de curso Caro Superior +
{ 
  tempo_int_fc2 = millis();
  if (tempo_int_fc2 - ultimo_tempo_int_fc2 > 1000)
  { 
    digitalWrite(INTERNAL_LED, HIGH);
    contador++;
    if(MOVIMENTANDO_CARRO)
    {  
      stepper.disableOutputs();
      LIMPANDO_PAINEL = false;
      MOVIMENTANDO_CARRO = false;
      digitalWrite(ENABLE, HIGH);
      analogWrite(PINO_MOTOR_DC1,0);//Desativa motor base
    }
    else// if(LIMPANDO_PAINEL)
    {   
      
      //digitalWrite(INTERNAL_LED, LOW);  
      
      
      SentidoDeLimpesa = IndoLimpando;
      
    }
    ultimo_tempo_int_fc2 = tempo_int_fc2;
  }
  
}
void IRAM_ATTR base_fim_inferior() ///Serial.println("(3)Fin de curso BASE Inferior -");
{
 //contador++;
  analogWrite(PINO_MOTOR_DC1,0);
}
void IRAM_ATTR base_fim_superior()// Serial.println("(4)Fin de curso BASE Superior +");
{
 //contador++;
  analogWrite(PINO_MOTOR_DC1,0);
}

void setup()
{

  mytimer.start();


  //PINOS
  pinMode(PINO_MOTOR_DC1,OUTPUT);
  pinMode(PINO_MOTOR_DC2,OUTPUT); 
  //analogWriteFreq(1000);
  pinMode(ENABLE, OUTPUT); 
  //FIM DE CURSOS
  pinMode(PINO_FIM_DE_CURSO_1, INPUT);
  attachInterrupt(PINO_FIM_DE_CURSO_1,carro_fim_inferior,RISING	 );
  pinMode(PINO_FIM_DE_CURSO_2, INPUT);
  attachInterrupt(PINO_FIM_DE_CURSO_2,carro_fim_superior, RISING	);
  pinMode(PINO_FIM_DE_CURSO_3,INPUT);
  attachInterrupt(PINO_FIM_DE_CURSO_3,base_fim_inferior, RISING	);
  pinMode(PINO_FIM_DE_CURSO_4, INPUT);
  attachInterrupt(PINO_FIM_DE_CURSO_4,base_fim_superior, RISING	);
  //LED AZUL INTERNO
  pinMode(INTERNAL_LED,OUTPUT);
  //CONFIGURA SERIAL
  Serial.begin(9600);
  SerialBT.begin("TCC2024LPF"); // Nome do dispositivo Bluetooth
 
  //CONFIGURA MOTOR PASSO
  stepper.setMaxSpeed(2000); //SPEED = Steps / second
  stepper.setAcceleration(1000); //ACCELERATION = Steps /(second)^2
  //stepper.disableOutputs();
  //stepper.setSpeed(1500); //em tese apenas seta avelocidade da limpesa

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
  
  verifica_timer();
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
      Serial.println(contador);
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
        //digitalWrite(INTERNAL_LED,HIGH);
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
      Serial.println(contador);
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
        //digitalWrite(INTERNAL_LED,HIGH);
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


//////////////////////////////////////////////////////////////////////////////////////
//                             Movimeto do Carro
//////////////////////////////////////////////////////////////////////////////////////

void movimentar_carro() 
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
  verifica_timer();
  if (SentidoDeLimpesa==IndoLimpando)
  {
    stepper.setSpeed(500);
  }
  else{
    stepper.setSpeed(-500);
  }

  digitalWrite(ENABLE, LOW);
  stepper.runSpeed();

    if(DEV == HIGH)//modo desenvolvedor
    {
      if (SentidoDeLimpesa==IndoLimpando)
      {
       Serial.print("Li=_");Serial.print(stepper.distanceToGo());Serial.print("_=_");Serial.print(stepper.targetPosition());Serial.print("_=_");Serial.println(stepper.currentPosition());
      SerialBT.print("Li=_");SerialBT.print(stepper.distanceToGo());SerialBT.print("_=_");SerialBT.print(stepper.targetPosition());SerialBT.print("_=_");SerialBT.println(stepper.currentPosition());
      }
    
      else{
        Serial.print("Lv=_");Serial.print(stepper.distanceToGo());Serial.print("_=_");Serial.print(stepper.targetPosition());Serial.print("_=_");Serial.println(stepper.currentPosition());
      SerialBT.print("Lv=_");SerialBT.print(stepper.distanceToGo());SerialBT.print("_=_");SerialBT.print(stepper.targetPosition());SerialBT.print("_=_");SerialBT.println(stepper.currentPosition());
      }
    }
  

}

void limite_de_trabalho(){
// se o limie superior for atinguido (stepper.targetPosition)
    //Sentidodelimpesa muda
    //...
//se...

}



void verifica_timer(){//timer ta funcionado
  if(mytimer.done()){
    digitalWrite(INTERNAL_LED,LOW);
    Serial.println("Timer finished");Serial.println(contador);// mytimer.start(); para iniciar novamente
  SerialBT.println("Timer finished");SerialBT.println(contador);
  mytimer.start();
  }
}
