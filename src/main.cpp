#include <AccelStepper.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"
#include <neotimer.h>

/*     PROTÓTIPO DE BAIXO CUSTO PARA LIMPEZA DE PAINÉIS FOTOVOLTAICOS
      
      
      ||    
      ////////////////////////////////////////////////////////////////////////////////////////////////
      //                          🍀 Comandos uteis 🍀                                            //
      //                                                                                          //
      //    📕"H = Ajuda."                                                                      //
      //    🚗"(valor)+m = Valor que incrementa o posicionamneto do caro." exemplo: 100m       //
      //    🏃"(valor)+v = Valor que corespondente a velocidade da base." exemplo: 500v       //
      //    🧼"(valor)+l = Valor da velocidade do caro na limpeza ". exemplo: 200l           //
      //    🧯"@ = Apaga variaveis e desativa motores"                                      //
      //    🧑‍💻"D = Ativar/Desativar modo Desenvolvedor"                                    //
      //    🤚"Q = Ativar/Desativar modo Limite de trabalho"                               \\
      //    👆"(valor)+p = Atribui Limite Positivo de trabalho +" exemplo: 500p             \\
      //    👇"(valor)+n = Atribui Limite Negativo de trabalho -" exemplo: 100n              \\
      //    🚶"(valor)+w = Atribui Velocidade da Base na Limpeza" exemplo: 300w               \\
      //    ⏲️"(valor)+t = Timer da Base na Limpeza,em milisegundos" exemplo: 10000t=10segundos\\
      //                                                                                         \\
      //                                                                                          \\
      ////////////////////////////////////////////////////////////////////////////////////////////.\\
      ||
      ||
      ||
      ||
 _    ||    __    _     
|#|__|##|__|##|__|#|                                                                                                         
|__|__|__[]__|__|__|                                               |>  v-v-v-v   |>
|_|__|__|__|__|__|_|                                       ,   ,  /_\  |     |  /_\
 \================/                                        |\_/|  | |'''''''''''| |          |\
  \'._.'.__.'._.'/                                         (q p),-| | ||  _  || | |'-._       ))
   |    .--.    |                                           \_/_(/| |    |#|    | | )  '-.___//
   |    |  |    |      ,-'''''-.:-^-._                   .--w-w---'-'----'-'----'-'----------'--------.
   |    |__|    |     /      '  ( `  _\                 :     Nome do dispositivo Bluetooth/           "                           
   |    ====    |     \      \   _ .,-'         ...-._'             "TCC2024LPF"                       
   |            |      )_\-._-._((_(       ~-~-'         
  OLA o Objetivo Geral deste projeto é desenvolver um protótipo de baixo custo para manter 
de forma autônoma os painéisfotovoltaicos limpos do Instituto Federal de Santa Catarina, 
campus Chapecó.


 



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
#define PINO_FIM_DE_CURSO_4 34//32//D0
#define INTERNAL_LED 2

//Cria motor 
AccelStepper stepper(1, PINO_MOTOR_A, PINO_MOTOR_B);
int VelocidadeBase;

//ESTADOS DO CARRO
volatile bool MOVIMENTANDO_CARRO = false;
volatile bool LIMPANDO_PAINEL = false;
volatile bool LIMITE_DE_TRABALHO_ATIVO = false;
volatile bool VOLTANDO_TERMINADO = false;

//Constantes e Variaveis globais 
char comando;
bool INDOnaLimpesa = 0;
bool VINDOnaLimpesa = 1;
volatile bool EstadoDaLimpesa = INDOnaLimpesa;
const int IndoLimpando = 0;
const int VindoLimpando = 1;
const int MovendoBaseLimpando = 2;
volatile int SentidoDeLimpesa = IndoLimpando;
volatile int AuxSentidoDeLimpesa = IndoLimpando;
int Esfrega;
bool DEV = LOW;
int AnovaPosicao;
String texto;
int contador;
int batidas = 0;
int LimiteNegativo = 0;
int LimitePositivo = 500;
int VelocidadeBaseNaLimpeza = 300;
int TempoTimer =10000;
int TavaDeSeguranca =true;
int base_voltando = LOW;

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
void menu_serial();
void movimentar_carro();
void limpar_painel();
void limite_de_trabalho();
void verifica_timer();
void voltar_origem();

//Timer
Neotimer mytimer = Neotimer(TempoTimer); // esta em milisegundos


//////////////////////////////////////////////////////////////////////////////////////
//                             Interupções
//////////////////////////////////////////////////////////////////////////////////////
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
    batidas++;
      //digitalWrite(INTERNAL_LED, HIGH);  
      // digitalWrite(PINO_MOTOR_B, LOW);  
    
      SentidoDeLimpesa = VindoLimpando;
      AuxSentidoDeLimpesa = VindoLimpando;
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
    //digitalWrite(INTERNAL_LED, HIGH);
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
      batidas++;
      //digitalWrite(INTERNAL_LED, LOW);  
      
      
      SentidoDeLimpesa = IndoLimpando;
      AuxSentidoDeLimpesa = IndoLimpando;
    }
    ultimo_tempo_int_fc2 = tempo_int_fc2;
  }
  
}
void IRAM_ATTR base_fim_inferior() ///Serial.println("(3)Fin de curso BASE Inferior -");
{
 //contador++;
  //analogWrite(PINO_MOTOR_DC1,0);
   VOLTANDO_TERMINADO=HIGH;
   base_voltando=HIGH;
}
void IRAM_ATTR base_fim_superior()// Serial.println("(4)Fin de curso BASE Superior +");
{
 //contador++;
  //analogWrite(PINO_MOTOR_DC1,0);
   VOLTANDO_TERMINADO=HIGH;
  base_voltando=LOW;
}
//////////////////////////////////////////////////////////////////////////////////////
//                             Setup de inicialização
//////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  //inicialisa o timr para o debounce////////////////////////////
 // mytimer.start();


  //PINOS
  pinMode(PINO_MOTOR_DC1,OUTPUT);
  pinMode(PINO_MOTOR_DC2,OUTPUT); 
  //analogWriteFreq(1000);
  pinMode(ENABLE, OUTPUT); 
  //FIM DE CURSOS
  pinMode(PINO_FIM_DE_CURSO_1, INPUT);
  attachInterrupt(PINO_FIM_DE_CURSO_1,carro_fim_inferior, RISING);
  pinMode(PINO_FIM_DE_CURSO_2, INPUT);
  attachInterrupt(PINO_FIM_DE_CURSO_2,carro_fim_superior, RISING);
  pinMode(PINO_FIM_DE_CURSO_3,INPUT);
  attachInterrupt(PINO_FIM_DE_CURSO_3,base_fim_inferior, RISING);
  pinMode(PINO_FIM_DE_CURSO_4, INPUT);
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
  //stepper.setSpeed(1500); //em teste apenas seta avelocidade da limpesa
  digitalWrite(ENABLE, HIGH);
  //CONFIGURA VELOCIDADE DO MOTOR BASE A 0
  VelocidadeBase=0;
  
  //LIGA OLED AZUL PARA INDICAR A INICIALIZACAO
 digitalWrite(INTERNAL_LED,HIGH);

  //AGUARDA PARA INICIAR
  delay(1000);

  //DESLIGA OLED AZUL PARA INDICAR o FIM DA INICIALIZACAO
 digitalWrite(INTERNAL_LED,LOW);
}
//////////////////////////////////////////////////////////////////////////////////////
//                                Loop Principal
//////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  
  //verifica_timer();//sera usado apenas na limpesa
  menu_serial();
  
  //OPERACOES
  if(MOVIMENTANDO_CARRO)
    movimentar_carro();
  //else
  if(LIMPANDO_PAINEL)
   limpar_painel();
  
  if(VOLTANDO_TERMINADO)
   voltar_origem();

}

//////////////////////////////////////////////////////////////////////////////////////
//                                  Menu Serial
//////////////////////////////////////////////////////////////////////////////////////
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
      if(VelocidadeBase>=0)
      analogWrite(PINO_MOTOR_DC1,VelocidadeBase);
    
      if(VelocidadeBase<<0)
      analogWrite(PINO_MOTOR_DC2,VelocidadeBase);

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
      analogWrite(PINO_MOTOR_DC2,0);
      Serial.println("==PARANDO==");
      Serial.println("Apagou Variavei");
      Serial.println("Desativando Motores");
      Serial.println(contador);
    }
    else if (command.endsWith("H")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      Serial.println("H = Ajuda.");
      Serial.println("(valor)+m = Valor que incrementa o posicionamneto do caro .exemplo: 100m");
      Serial.println("(valor)+v = Valor que corespondente a velocidade da base .exemplo: 500v");
      Serial.println("(valor)+l = Valor da velocidade do caro na limpeza . exemplo: 200l");
      Serial.println("@ = Apaga variaveis e desativa motores");
      Serial.println("D = Ativar/Desativar modo Desenvolvedor");
      Serial.println("Q = Ativar/Desativa modo Limite de trabalho");
      Serial.println("(valor)+p = Atribui Limite Positivo de trabalho + .exemplo: 500p");
      Serial.println("(valor)+n = Atribui Limite Negativo de trabalho - .exemplo: 100n");
      Serial.println("(valor)+w = Atribui Velocidade da Base na Limpeza .exemplo: 300w");
      Serial.println("(valor)+t = Timer da Base na Limpeza, em milisegundos .exemplo: 10000t = 10segundos");
    
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
    else if (command.endsWith("Q")) 
    {
        int value = command.substring(0, command.length() - 1).toInt();
        
        if (LIMITE_DE_TRABALHO_ATIVO == LOW)
        {
          Serial.println("   <<  Limite de trabalho ativo. >>");
          LIMITE_DE_TRABALHO_ATIVO = HIGH;
        }
        else
        {
          Serial.println("   <<  Limite de trabalho ativo. >>");
          LIMITE_DE_TRABALHO_ATIVO = LOW;
        }
    }
     else if (command.endsWith("p")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      LimitePositivo=value;
      Serial.println("Atribuido Limite Positivo +");

    } 
    else if (command.endsWith("n")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      LimiteNegativo=value;
      Serial.println("Atribuido Limite Negativo -");

    } 
    else if (command.endsWith("w")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      VelocidadeBaseNaLimpeza=value;
      Serial.println("Atribuido Velocidade da Base na Limpeza");

    } 
    else if (command.endsWith("t")) //tempo do timer de limpeza
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      TempoTimer=value;
      Serial.println("Atribuido Velocidade da Base na Limpeza");

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
      analogWrite(PINO_MOTOR_DC1,VelocidadeBase);//////////////////////////////////// falat arumar................
    
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
      analogWrite(PINO_MOTOR_DC2,0);
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
      SerialBT.println("(valor)+l = Valor da velocidade do caro na limpeza . exemplo: 200l");
      SerialBT.println("@ = Apaga variaveis e desativa motores");
      SerialBT.println("D = Ativar/Desativar modo Desenvolvedor");
      SerialBT.println("Q = Ativar/Desativa modo Limite de trabalho");
      SerialBT.println("(valor)+p = Atribui Limite Positivo de trabalho + .exemplo: 500p");
      SerialBT.println("(valor)+n = Atribui Limite Negativo de trabalho - .exemplo: 100n");
      SerialBT.println("(valor)+w = Atribui Velocidade da Base na Limpeza .exemplo: 300w");
      SerialBT.println("(valor)+t = Timer da Base na Limpeza, em milisegundos .exemplo: 10000t = 10segundos");
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
        else if (command.endsWith("Q")) 
    {
        int value = command.substring(0, command.length() - 1).toInt();
        
        if (LIMITE_DE_TRABALHO_ATIVO == LOW)
        {
          SerialBT.println("   <<  Limite de trabalho ativo. >>");
          LIMITE_DE_TRABALHO_ATIVO = HIGH;
        }
        else
        {
          SerialBT.println("   <<  Limite de trabalho ativo. >>");
          LIMITE_DE_TRABALHO_ATIVO = LOW;
        }
    }
     else if (command.endsWith("p")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      LimitePositivo=value;
      SerialBT.println("Atribuido Limite Positivo +");

    } 
    else if (command.endsWith("n")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      LimiteNegativo=value;
      SerialBT.println("Atribuido Limite Negativo -");

    } 
    else if (command.endsWith("w")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      VelocidadeBaseNaLimpeza=value;
      SerialBT.println("Atribuido Velocidade da Base na Limpeza");

    } 
    else if (command.endsWith("t")) 
    {
      int value = command.substring(0, command.length() - 1).toInt();
      
      TempoTimer=value;
      SerialBT.println("Atribuido timer da Base na limpeza");

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
  if(LIMITE_DE_TRABALHO_ATIVO){
  limite_de_trabalho();
  }

  if (SentidoDeLimpesa==IndoLimpando)
  {
    stepper.setSpeed(Esfrega);
  }
   if(SentidoDeLimpesa==VindoLimpando){
    stepper.setSpeed(-Esfrega);
  }

  if(SentidoDeLimpesa==MovendoBaseLimpando){
    stepper.setSpeed(0);
    //desativar motor
    verifica_timer();
    digitalWrite(ENABLE, HIGH);
  }

  if (batidas >= 4){
  
    SentidoDeLimpesa=MovendoBaseLimpando;
    batidas=0;
    analogWrite(PINO_MOTOR_DC1,VelocidadeBaseNaLimpeza);//moviemta base
    mytimer.start();
    TavaDeSeguranca=false;
    Serial.println("Cabim");
    SerialBT.println("Cabim");
  } 
  if(TavaDeSeguranca==false){
     batidas=0;

      SerialBT.println("seguranca");//////////////////////////
      SentidoDeLimpesa = MovendoBaseLimpando;
  }
  
  if(SentidoDeLimpesa!=MovendoBaseLimpando){// movimenta o caro na limpeza Rum...
  digitalWrite(ENABLE, LOW);
  stepper.runSpeed();
  }

    if(DEV == HIGH)//modo desenvolvedor
    {
      if (SentidoDeLimpesa==IndoLimpando)
      {
        Serial.print("Li=_");Serial.print(stepper.distanceToGo());Serial.print("_=_");Serial.print(stepper.targetPosition());Serial.print("_=_");Serial.println(stepper.currentPosition());Serial.println(batidas);
        SerialBT.print("Li=_");SerialBT.print(stepper.distanceToGo());SerialBT.print("_=_");SerialBT.print(stepper.targetPosition());SerialBT.print("_=_");SerialBT.println(stepper.currentPosition());SerialBT.println(batidas);
      }
    
      if(SentidoDeLimpesa==VindoLimpando)
      {
        Serial.print("Lv=_");Serial.print(stepper.distanceToGo());Serial.print("_=_");Serial.print(stepper.targetPosition());Serial.print("_=_");Serial.println(stepper.currentPosition());Serial.println(batidas);
        SerialBT.print("Lv=_");SerialBT.print(stepper.distanceToGo());SerialBT.print("_=_");SerialBT.print(stepper.targetPosition());SerialBT.print("_=_");SerialBT.println(stepper.currentPosition());SerialBT.println(batidas);
      }
      
    }
  

}

//////////////////////////////////////////////////////////////////////////////////////
//                 Execução da Limpesa com modo Limite de Tabalho
//////////////////////////////////////////////////////////////////////////////////////
void limite_de_trabalho(){
  
// se o limie superior for atinguido 
    if(stepper.currentPosition()>=LimitePositivo)//Sentidodelimpesa muda
      {
        if(MOVIMENTANDO_CARRO)
        { 
          
          stepper.disableOutputs();
          LIMPANDO_PAINEL = false;
          MOVIMENTANDO_CARRO = false;
          digitalWrite(ENABLE, HIGH);
          analogWrite(PINO_MOTOR_DC1,0);//Desativa motor base
          analogWrite(PINO_MOTOR_DC2,0);
        }
        else
        {
        batidas++;
          SentidoDeLimpesa = VindoLimpando;
          AuxSentidoDeLimpesa = VindoLimpando;
          
        }
      }
    if(stepper.currentPosition()<=LimiteNegativo)//Sentidodelimpesa muda
      {
        if(MOVIMENTANDO_CARRO)
        {  
          stepper.disableOutputs();
          LIMPANDO_PAINEL = false;
          MOVIMENTANDO_CARRO = false;
          digitalWrite(ENABLE, HIGH);
          analogWrite(PINO_MOTOR_DC1,0);//Desativa motor base
          analogWrite(PINO_MOTOR_DC2,0);
        }
        else// 
        {   
        batidas++;
          SentidoDeLimpesa = IndoLimpando;
          AuxSentidoDeLimpesa = IndoLimpando;
        }
      } 
    


}


//////////////////////////////////////////////////////////////////////////////////////
//                             Timer para limpiza
//////////////////////////////////////////////////////////////////////////////////////
void verifica_timer(){//timer ta funcionado
  SerialBT.println("timer verificou");

  if(mytimer.done())
  {
    //digitalWrite(INTERNAL_LED,LOW);//desliga led
    Serial.println("Timer Acabou");  Serial.println(batidas);// mytimer.start(); para iniciar novamente
    SerialBT.println("Timer Acabou");  SerialBT.println(batidas);
    //mytimer.start();
    analogWrite(PINO_MOTOR_DC1,0);
    analogWrite(PINO_MOTOR_DC2,0);
    SentidoDeLimpesa = AuxSentidoDeLimpesa;
    digitalWrite(INTERNAL_LED,LOW);
    TavaDeSeguranca=true;
  }

  
}
// add voltar par base quando fim de curso atingido
void voltar_origem(){
  if(base_voltando==LOW){
    analogWrite(PINO_MOTOR_DC1,0);
    analogWrite(PINO_MOTOR_DC2,0);
  }
  if(base_voltando==HIGH){
    analogWrite(PINO_MOTOR_DC1,1000);
    analogWrite(PINO_MOTOR_DC2,0);
  }
  VOLTANDO_TERMINADO=LOW;
}
// add um jeito de girat o moror da base ao contrario
