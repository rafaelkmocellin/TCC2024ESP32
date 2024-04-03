#include <AccelStepper.h>
#include <HardwareSerial.h>
//Metodo Comunicacao
#define ESCOLHA_BT true
#define ESCOLHA_SERIAL false 

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
//int novaPosicao=0;
String texto;
//Funcoes
String verifica_seriais();
void imprimi_informacoes();
void menu_escolha();
void movimentar_carro();
void limpar_painel();
//bool INDO=0;
//bool VINDO=1;
//bool sentidoDElimpesa=INDO;
int AnovaPosicao;


//novo teste steper22222222222222222222222222222222222222222222222222222222222




// //Interrupcoes

void IRAM_ATTR carro_fim_inferior()
{
  //Serial.println("(1)Fin de curso Caro Inferior -");
 
  stepper.disableOutputs();
  LIMPANDO_PAINEL = false;
  MOVIMENTANDO_CARRO = false;
  digitalWrite(ENABLE, HIGH);
  analogWrite(PINO_MOTOR_DC1,0);//Desativa motor base
  
 /*
  if(MOVIMENTANDO_CARRO)
  {  
    stepper.disableOutputs();
    LIMPANDO_PAINEL = false;
    MOVIMENTANDO_CARRO = false;
    digitalWrite(ENABLE, HIGH);
    analogWrite(23,0);//Desativa motor base
  }
  else if(LIMPANDO_PAINEL)
  {
    stepper.moveTo(-1*(novaPosicao));
    Serial.println("Vindo limpando");
  }
*/
}
void IRAM_ATTR carro_fim_superior()
{
  //Serial.println("(2)Fin de curso Caro Superior +");
  
}
void IRAM_ATTR base_fim_inferior()
{
  ///Serial.println("(3)Fin de curso BASE Inferior -");
  
}
void IRAM_ATTR base_fim_superior()
{
 // Serial.println("(4)Fin de curso BASE Superior +");
  
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
  attachInterrupt(PINO_FIM_DE_CURSO_1,carro_fim_inferior, FALLING);
  pinMode(PINO_FIM_DE_CURSO_2, INPUT_PULLUP);
  attachInterrupt(PINO_FIM_DE_CURSO_2,carro_fim_superior, FALLING);
  pinMode(PINO_FIM_DE_CURSO_3, INPUT_PULLUP);
  attachInterrupt(PINO_FIM_DE_CURSO_3,base_fim_inferior, FALLING);
  pinMode(PINO_FIM_DE_CURSO_4, INPUT_PULLUP);
  attachInterrupt(PINO_FIM_DE_CURSO_4,base_fim_superior, FALLING);

  //CONFIGURA SERIAL
  Serial.begin(9600);

 
  //CONFIGURA MOTOR PASSO
  stepper.setMaxSpeed(2000); //SPEED = Steps / second
  stepper.setAcceleration(1000); //ACCELERATION = Steps /(second)^2
  //stepper.disableOutputs();
  
  //CONFIGURA VELOCIDADE DO MOTOR BASE A 0
  VelocidadeBase=0;
  
  //AGUARDA PARA INICIAR
  delay(1000);
}
 









void loop()
{
  //analogWrite(PINO_MOTOR_DC2,1000);
  
  menu_escolha();

  //OPERACOES
  if(MOVIMENTANDO_CARRO)
    movimentar_carro();
  //else
  if(LIMPANDO_PAINEL)
   limpar_painel();

}


void menu_escolha()//2,0
{

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.endsWith("m")) {
      int value = command.substring(0, command.length() - 1).toInt();
      // Adicione aqui o código para incrementar o posicionamento do carro
      // Exemplo: incrementPosition(value);
      MOVIMENTANDO_CARRO = true;
      LIMPANDO_PAINEL = false;
      //stepper.distanceToGo(novaPosicao);
      stepper.enableOutputs();
    
      stepper.moveTo(value);
      //stepper.moveTo(novaPosicao);
      Serial.println("Movendo Caro");
      AnovaPosicao = value;
      Serial.println(AnovaPosicao);

    } else if (command.endsWith("v")) {
      int value = command.substring(0, command.length() - 1).toInt();
      // Adicione aqui o código para ajustar a velocidade da base
      // Exemplo: adjustBaseSpeed(value);
      
      value = VelocidadeBase;
      analogWrite(PINO_MOTOR_DC1,VelocidadeBase);
    
      Serial.println("Movimntando Base");

    } else if (command.endsWith("l")) {
      int value = command.substring(0, command.length() - 1).toInt();
      // Adicione aqui o código para ajustar o valor correspondente
      // Exemplo: adjustCorrespondingValue(value);
      Serial.println("fgsfgsdfgsdfgcvsdshgfs Bfdgase");

    }
  }



}








/*
String verifica_seriais() //testar
{
  if(Serial.available() > 0) //if something comes
  {
    return Serial.readString();
  }
  return "";
}

void imprimi_informacoes() //ajustar
{
  Serial.println(stepper.currentPosition());
  Serial.println(comando);
  Serial.println(stepper.targetPosition());
}

void menu_escolha() //ajustar
{
if (Serial.available()) {
     comando = Serial.read();
  
  //MOVER CARRO
   if (comando =='m' )
  {
    texto.remove('m');
    int novaPosicao = texto.toInt();//trasforma inteiro 
    texto ="";//nada
   //comando="";//nada

    MOVIMENTANDO_CARRO = true;
    LIMPANDO_PAINEL = false;
    //stepper.distanceToGo(novaPosicao);
    stepper.enableOutputs();
    
    stepper.moveTo(novaPosicao);
    //stepper.moveTo(novaPosicao);
     Serial.println("Movendo Caro");
     AnovaPosicao = novaPosicao;
     Serial.println(AnovaPosicao);
  }
  
  //MOVER BASE
 if (comando =='v' )
  {
    texto.remove('v');
    VelocidadeBase = texto.toInt();//trasforma inteiro 
    texto ="";//nada
   
    analogWrite(PINO_MOTOR_DC1,VelocidadeBase);
    
     Serial.println("Movimntando Base");
  }

  //OPERACAO LIMPEZA
 else if (comando == 'l')
  {
    texto.remove('l');
    int novaPosicao = texto.toInt();//trasforma inteiro 
    texto ="";//nada
    
    LIMPANDO_PAINEL = true;
    MOVIMENTANDO_CARRO = false;

    stepper.enableOutputs();
    stepper.moveTo(novaPosicao);
     Serial.println("limpando");
  }
  else if (comando == '@')
  {
      imprimi_informacoes();
      
      texto ="";//nada
      
      stepper.disableOutputs();
      LIMPANDO_PAINEL = false;
      MOVIMENTANDO_CARRO = false;
      digitalWrite(ENABLE, HIGH);
      analogWrite(PINO_MOTOR_DC1,0);
      Serial.println("==PARANDO==");
      Serial.println("Apagou Variavei");
      Serial.println("Desativando Motores");
  }
  else if (comando == '!')
  {
      //imprimi_informacoes();
      texto ="";//nada
      stepper.disableOutputs();
      LIMPANDO_PAINEL = false;
      MOVIMENTANDO_CARRO = false;
      digitalWrite(ENABLE, HIGH);
      Serial.println("Bugatison");
     
  }
  else if (comando == 'H')
  {
      texto ="";//nada
      Serial.println("H = Ajuda.");
      Serial.println("(valor)+m = Valor que incrementa o posicionamneto do caro. exemplo: 100m");
      Serial.println("(valor)+v = Valor que corespondente a velocidade da base. exemplo: 500v");
      Serial.println("(valor)+l = Valor corespondente . exemplo: 500M");
      Serial.println("@ = Apaga variaveis e desativa motores");
  }
  else {
      texto.concat(comando);//concatenando os valores
      Serial.print("comando==");Serial.println(comando);
      Serial.print("texto====");Serial.println(texto);
    }
}
}
*/


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
    Serial.print("=_");Serial.print(stepper.distanceToGo());Serial.print("_=_");Serial.print(stepper.targetPosition());Serial.print("_=_");Serial.println(stepper.currentPosition());
    //Serial.println(AnovaPosicao);
    stepper.run();
  }
}


void limpar_painel() //add LIMPANDO_PAINEL = false ao chegar no endstop
{
  /*
  if(sentidoDElimpesa==INDO)
  {
    if (stepper.distanceToGo() == 0)
    {
    stepper.move(-1 * (50));
    sentidoDElimpesa=VINDO;
    }
    else
    {
    digitalWrite(ENABLE, LOW);
    //Serial.println(" rum ");
    stepper.run();
    }
  }
  else if(sentidoDElimpesa==VINDO)
  {
    if (stepper.distanceToGo()==0)
    {
      
    stepper.move(100);
    sentidoDElimpesa=INDO;
    }
    else{
    digitalWrite(ENABLE, LOW);
    //Serial.println(" rum ");
    stepper.run();
    }
  } 
 */
  if(!LIMPANDO_PAINEL)
  {
    Serial.println(" - LIMBANDO");
      digitalWrite(ENABLE, HIGH);
      stepper.moveTo(-1 * (stepper.currentPosition()));
      MOVIMENTANDO_CARRO = true;
  }
  
  /*
  sentidoDElimpesa for + = ele vai 
  se nao ele volta

  fim de curso caro, inverte sentidoDElimpesa
  
  para mudar sentido presisa mudae novaposisaode l para negativo
  
  
  */
}


