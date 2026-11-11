
#include <ACAN_ESP32.h>
#include "mensagensCan.h"

//Definição dos pinos
#define sinalDrive      GPIO_NUM_25     // pino da seleção de drive 
#define sinalFreio      GPIO_NUM_13     // sensor de pedal do freio
#define sinalAcel1      GPIO_NUM_14     // pino analógico acelerador/potenciometro 1
#define sinalAcel2      GPIO_NUM_27     // pino analógico acelerador/potenciometro 2
//#define sinalReset    GPIO_NUM_32     // Botão reset
#define comPreCharge    GPIO_NUM_18     // Saida do Pre Charge TESTE correto: 27
#define comContator     GPIO_NUM_21     // Saida dos Contatores TESTE correto 14
//#define comSilentCAN  GPIO_NUM_5      // controla o modo silent da CAN (nao usado)
#define comEstadoDrive  GPIO_NUM_26     // pino led painel
#define CANTX           GPIO_NUM_33     //Pino TX do transdutor CAN
#define CANRX           GPIO_NUM_32     //Pino RX do transdutor CAN
#define LED_BUILTIN     GPIO_NUM_2      //Pino do LED integrado


int pedal_min = 470;  //ajustado no carro
int pedal_max = 1150; //ajustado no carro
int tolerancePedalcurve = 700; //ajustado no carro, pendente parametrização

//Parametros Limites de aplicação
#define limiteTorqueMax   55
#define limiteTorqueMin   -1.0
#define limiteCorrenteMax 85
#define limiteCorrenteMin -5
#define limiteTensaoMax   54
#define limiteTensaoMin   32
#define resBateria        0.05
#define FILTER_SIZE 5
#define READ_INTERVAL 10  // Lê a cada 10ms
#define FILTER_WINDOW 50  // Janela de 50ms (5 amostras × 10ms)


//Criar maquina de estados com enum class para os estados do powertrain


enum class EstadoPowertrain : uint8_t {
   Desligado = 0,
   PreCharge = 1,
   Neutro = 2,
   DriveAcelerando = 3,
   Reiniciando = 5  
};


//variáveis globais
CANMessage bufferRx;
CANMessage bufferTx;
unsigned long millis_ant        = millis();
unsigned long millis_ant_print  = millis();
unsigned int periodo_mensagem   = 9;
unsigned int periodo_print      = 1000;
unsigned int temp_janela        = 0;
EstadoPowertrain estadoPowertrain = EstadoPowertrain::Neutro; //verificar se antes ja ligou e ocorreu o precharger

// Buffers circulares (diferente do original)
int bufferPedal1[FILTER_SIZE] = {0};
int bufferPedal2[FILTER_SIZE] = {0};
int indexBuffer = 0;
unsigned long lastPedalReadTime = 0;
bool bufferInitialized = false;


////  ----   Declara as mensagens a enviar pro Motor   ----   ////
EemEdrvLim1  msgLimites;
PtEdrvDes1   msgControle;
PtVeh1       msgVeiculo1;

////  ----   Declara as mensagens a receber do motor   ----   ////
EdrvAct1  msgEstadoAtual;
EdrvDern1 msgDeratingAtual;


//Protótipo das funções
void printaMsg (unsigned char stringMsg[], int tamanho);
void sequenciaLigacao();
void reiniciaMotor();
float leituraAcelerador();
void carregaMsgPadrao();
void enviaMsgControle();
void enviaMsgLimites();
void enviaMsgVeiculo1();
bool adcPedalvalid(int a, int b, int tolerance) {
    return abs(a - b) <= tolerance;
}
int calculateBufferAverage(int buffer[], int size) { //calcular média do buffer
    long sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

void updatePedalReadings() { // atualizar leituras a cada 10ms
    if (millis() - lastPedalReadTime >= READ_INTERVAL) {
        // Captura nova amostra 
        bufferPedal1[indexBuffer] = analogRead(sinalAcel1);
        bufferPedal2[indexBuffer] = analogRead(sinalAcel2);
        
        // Avança o índice circular
        indexBuffer = (indexBuffer + 1) % FILTER_SIZE;
        lastPedalReadTime = millis();
        
        // Marca buffer como inicializado após primeira volta completa
        if (indexBuffer == 0) bufferInitialized = true;
    }
}

void setup() 
{
  Serial.begin(115200);

  //pinMode(sinalReset,     INPUT_PULLUP);
  pinMode(sinalDrive,     INPUT_PULLUP);
  pinMode(sinalAcel1,     INPUT);
  pinMode(sinalAcel2,     INPUT);
  pinMode(sinalFreio,     INPUT_PULLUP);
  pinMode(comEstadoDrive, OUTPUT);
  pinMode(comPreCharge,   OUTPUT);
  pinMode(comContator,    OUTPUT);
  //pinMode(comSilentCAN,   OUTPUT);

  //debug com led
  pinMode(LED_BUILTIN,    OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  //digitalWrite(comSilentCAN, LOW);

  ACAN_ESP32_Settings settings (500E3);

  settings.mTxPin = CANTX ;
  settings.mRxPin = CANRX ;

  
  while(1)
  {    
    int codErro = ACAN_ESP32::can.begin (settings) ;
    if(!codErro)
    {
      /*Serial.print ("Bit Rate prescaler: ") ;
      Serial.println (settings.mBitRatePrescaler) ;
      Serial.print ("Time Segment 1:     ") ;
      Serial.println (settings.mTimeSegment1) ;
      Serial.print ("Time Segment 2:     ") ;
      Serial.println (settings.mTimeSegment2) ;
      Serial.print ("RJW:                ") ;
      Serial.println (settings.mRJW) ;
      Serial.print ("Triple Sampling:    ") ;
      Serial.println (settings.mTripleSampling ? "yes" : "no") ;
      Serial.print ("Actual bit rate:    ") ;
      Serial.print (settings.actualBitRate ()) ;
      Serial.println (" bit/s") ;
      Serial.print ("Exact bit rate ?    ") ;
      Serial.println (settings.exactBitRate () ? "yes" : "no") ;
      Serial.print ("Distance            ") ;
      Serial.print (settings.ppmFromDesiredBitRate ()) ;
      Serial.println (" ppm") ;
      Serial.print ("Sample point:       ") ;
      Serial.print (settings.samplePointFromBitStart ()) ;
      Serial.println ("%") ;*/
      Serial.println ("CAN Inicializada! OK!");
      break;
    }
    else
    {
      Serial.print ("Configuration error 0x") ;
      Serial.println (codErro, HEX) ;
    }
    Serial.print("Erro na inicialização, tentando novamente");
    delay(400);
    Serial.print(".");
    delay(400);
    Serial.print(".");
    delay(400);
    Serial.println(".");
    delay(400);
  }

  delay(10);
  //Carrega valores padrão das mensagens
  carregaMsgPadrao();
  
  sequenciaLigacao();  
  
  millis_ant = millis();
}

void loop() 
{

  updatePedalReadings(); //atualiza leitura dos pedais a cada 10ms

  //Gatilho de Reset via Serial
  if (Serial.available() > 0) {
    char comando = Serial.read();
    if (comando == 'r' || comando == 'R') {
      Serial.println("\n*** COMANDO DE RESET RECEBIDO ***");
      estadoPowertrain = EstadoPowertrain::Reiniciando;
    }
  }





  //Leitura Sinal Drive substituicao por switch case 

switch (estadoPowertrain)
  {
    case EstadoPowertrain::Neutro:  // se estado atual é Neutro
      digitalWrite(comEstadoDrive, LOW);  // apaga o led paionel
       msgControle.pt_st_oper_mod_des_edrv = ModoOperacao::Idle; // motor em Idle
       msgControle.pt_tq_des_edrv = 0; // torque == 0

       // só entra em DriveAcelerando se freio E drive pressionados simultaneamente
       if (!digitalRead(sinalFreio)  &&  !digitalRead(sinalDrive))
       {
         estadoPowertrain = EstadoPowertrain::DriveAcelerando;
       }   
      break;



      
    case EstadoPowertrain::DriveAcelerando: // se estado atual é Acelerando
      digitalWrite(comEstadoDrive, HIGH); // acende o led painel

       // libera so se o freio nao estiver pressionado
      if(digitalRead(sinalFreio))
      {
      float torqueDesejado = leituraAcelerador(); // lê o acelerador
      msgControle.pt_tq_des_edrv = torqueDesejado; // manda torque para o motor
      msgControle.pt_st_oper_mod_des_edrv = ModoOperacao::Torque; // motor em modo torque
      msgControle.pt_tq_min_edrv = limiteTorqueMin; // define limite mínimo de torque
      msgControle.pt_tq_max_edrv = limiteTorqueMax; // define limite máximo de torque
    }
    else // freio  pressionado
    {
      msgControle.pt_st_oper_mod_des_edrv = ModoOperacao::Idle; // motor em Idle
      msgControle.pt_tq_des_edrv = 0; // zera o torque
    }




    // se o botão de drive foi solto, volta para Neutro
    if (digitalRead(sinalDrive)) estadoPowertrain = EstadoPowertrain::Neutro;
    break; 
    
    case EstadoPowertrain::Reiniciando:

      
      reiniciaMotor();
      estadoPowertrain = EstadoPowertrain::Neutro;
     
      break;

    default:
      break;
  }


  //Envia mensagens
  if (millis() - millis_ant > periodo_mensagem) //janela 10ms
  {
    enviaMsgControle();      
    temp_janela++;
    if (temp_janela >= 10) //janela 100ms
    {
      enviaMsgLimites();
      temp_janela = 0;
    }
    millis_ant = millis();    
  }
  
  //Verifica mensagem a receber
  if (ACAN_ESP32::can.receive (bufferRx))
  {
    if(!bufferRx.rtr && !bufferRx.ext)
    {
      switch (bufferRx.id)
      {
        case 0xA3:
          msgEstadoAtual    = converteMsgEstadoAtual(bufferRx.data);
          break;
        case 0xA4:
          digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN));
          msgDeratingAtual  = converteMsgDerating(bufferRx.data);
          break;
        default:
          break;
      }   
    }
  }
  
  //Printa as mensagens
  if (millis() - millis_ant_print > periodo_print)
  {
    Serial.println("--------------------------------");
    Serial.print("Estado Powertrain : "); Serial.println((uint8_t)estadoPowertrain);
    Serial.println("--------------------------------");
    Serial.println("Controle Atual (0xA6):");
    Serial.print("Modo Op       : "); Serial.println((uint8_t)msgControle.pt_st_oper_mod_des_edrv);
    Serial.print("Torque Des    : "); Serial.println(msgControle.pt_tq_des_edrv);
    Serial.print("Torque Min    : "); Serial.println(msgControle.pt_tq_min_edrv);
    Serial.print("Torque Max    : "); Serial.println(msgControle.pt_tq_max_edrv);
    Serial.println();
    Serial.println("Estado Atual (0xA3):");
    Serial.print("Estado Operacao : "); Serial.println((uint8_t)msgEstadoAtual.edrv_st_oper_mod_act);
    Serial.print("Torque Atual    : "); Serial.println(msgEstadoAtual.edrv_tq_act);
    Serial.print("Velocidade Atual: "); Serial.println(msgEstadoAtual.edrv_n_act);
    Serial.print("Tensao Atual    : "); Serial.println(msgEstadoAtual.edrv_u_act);
    Serial.print("Corrente Atual  : "); Serial.println(msgEstadoAtual.edrv_i_act);
    Serial.println();
    Serial.println("Derating do Motor (0xA4):");
    Serial.print("Torque Min Disp   : "); Serial.println(msgDeratingAtual.edrv_tq_min_avl);
    Serial.print("Torque Max Disp   : "); Serial.println(msgDeratingAtual.edrv_tq_max_avl);
    Serial.print("Estado de Derating: "); Serial.print(msgDeratingAtual.edrv_st_act_dern);
    Serial.print(" ("); Serial.print(textoDerating(msgDeratingAtual.edrv_st_act_dern)); Serial.println(")");
    Serial.print("ID de Falha       : "); Serial.println(msgDeratingAtual.edrv_st_flt_id);
    Serial.print("Temp. Motor       : "); Serial.print(msgDeratingAtual.edrv_rat_t_crit_mac); Serial.println(" %");
    Serial.print("Temp. Inversor    : "); Serial.print(msgDeratingAtual.edrv_rat_t_crit_ivtr); Serial.println(" %");
    Serial.println();
    millis_ant_print = millis();
  }
}

////////////////////////////////////////////////
/////             Corpo das                /////
/////              funções                 /////
////////////////////////////////////////////////

//Imprime o hexa de cada mensagem
void printaMsg (uint8_t* buffer, int tamanho)
{
  for (int i = 0; i<tamanho; i++)
  {
    Serial.printf("%02X ", buffer[i]);
  }
  Serial.println();
  return;
}


//Função de envio de mensagem de controle
//Já trata o alive counter e calcula checksum
void enviaMsgControle()
{
  //Serial.println("Entrou controle");
  //Mensagem Pt_Edrv_Des_1
  bufferTx.id  = 0xA6;
  bufferTx.len = msgControle.tamanho;  
  converteMsgControle(msgControle, bufferTx.data);
  calculateCrc8(bufferTx.data, bufferTx.len, bufferTx.id);
  if (ACAN_ESP32::can.tryToSend(bufferTx))
    msgControle.pt_nr_alv_cntr_des = (msgControle.pt_nr_alv_cntr_des+1)%16;
  //Serial.println("Enviou pacote");
}

//Função de envio de mensagem de limites
//Já trata o alive counter e calcula checksum
void enviaMsgLimites()
{
  //Serial.println("Entrou limite");
  //Mensagem Eem_Edrv_Lim_1
  bufferTx.id  = 0xA5;
  bufferTx.len = msgLimites.tamanho;  
  converteMsgLimites(msgLimites, bufferTx.data);
  calculateCrc8(bufferTx.data, bufferTx.len, bufferTx.id);
  if (ACAN_ESP32::can.tryToSend(bufferTx))
    msgLimites.eem_nr_alv_cntr_lim = (msgLimites.eem_nr_alv_cntr_lim+1)%16;  
  //Serial.println("Enviou pacote");  
}

//Função de envio de mensagem veh_1
//Já trata o alive counter e calcula checksum
void enviaMsgVeiculo1()
{
  //Mensagem Pt_Veh_1
  //Serial.println("Entrou Veiculo");
  bufferTx.id  = 0xA7;
  bufferTx.len = msgVeiculo1.tamanho;  
  converteMsgVeiculo1(msgVeiculo1, bufferTx.data);
  calculateCrc8(bufferTx.data, bufferTx.len, bufferTx.id);
  if (ACAN_ESP32::can.tryToSend(bufferTx))
    msgVeiculo1.pt_nr_alv_cntr_veh1 = (msgVeiculo1.pt_nr_alv_cntr_veh1+1)%16;
  //Serial.println("Enviou pacote");
}

void sequenciaLigacao()
{
  Serial.println("Aguardando motor."); 
  delay(2000);
  Serial.println("Mandando os limites de elétricos."); 
  enviaMsgLimites();
  delay(10);

  Serial.println("Mandando requisição de modo idle.");
  enviaMsgControle();

  Serial.println("Mandando estados veiculo 1.");
  enviaMsgVeiculo1();
  delay(10);

  Serial.println("Ligando ignição.");
  msgControle.pt_st_ign_swt = EstadoIgnicao::On;
  enviaMsgControle();

  // Liga os contatores
  digitalWrite(comPreCharge, HIGH);
  delay(2000);
  digitalWrite(comContator, HIGH);
  digitalWrite(comPreCharge, LOW);
  
  return;
}

void reiniciaMotor()
{
  Serial.println("Reinicialização requisitada.");
  delay(100);

  Serial.println("Desligando ignição.");
  msgControle.pt_st_ign_swt = EstadoIgnicao::Off;
  enviaMsgControle();
  delay(200);

  digitalWrite(comContator, LOW); //desliga contator
  Serial.print("Aguardando.");delay(2000);Serial.print(".");delay(2000);Serial.println(".");delay(2000);

  sequenciaLigacao();

  Serial.println("Reinicio concluido.");
  return;    
}

float leituraAcelerador() //Função de leitura do acelerador (usa média dos últimos 50ms)
{
    if (!bufferInitialized) {// Se buffer ainda não foi preenchido, retorna 0 
        return 0.0;
    }
    
    // média dos últimos 50ms
    int adcPedal10 = calculateBufferAverage(bufferPedal1, FILTER_SIZE);
    int adcPedal20 = calculateBufferAverage(bufferPedal2, FILTER_SIZE);
    
    int adcPedal0;
    
    if (adcPedalvalid(adcPedal10, adcPedal20, tolerancePedalcurve)) { // Valida divergência entre sensores
        adcPedal0 = (adcPedal10 + adcPedal20) / 2; // Média dos dois sensores
    } else {
        adcPedal0 = 0; // Falha de segurança
        Serial.println("AVISO: Divergência entre sensores do pedal!");
    }
    
    if (adcPedal0 < pedal_min) adcPedal0 = pedal_min;     // limites de parametrização
    else if (adcPedal0 > pedal_max) adcPedal0 = pedal_max;

    // Calcula porcentagem e torque desejado
    float porcentTorque = (float)(adcPedal0 - pedal_min) / (float)(pedal_max - pedal_min);
    float torqueDesejado = porcentTorque * limiteTorqueMax;

    return torqueDesejado;
}


void carregaMsgPadrao()
{
  //valores padrao controle: 7bytes, 0, 0, 0, 0 Nm, -50 Nm, 0 RPM
  msgControle = {7, 0xFF, 0, ModoOperacao::Idle, 0, limiteTorqueMin, EstadoIgnicao::Off, limiteTorqueMax};
  //valores padrao Lim: 8bytes, 52v, 36v, 0.02 Ohm, 0a, 250a, -250a
  msgLimites  = {8, 0xFF, 0, limiteCorrenteMin, limiteCorrenteMax, limiteTensaoMin, limiteTensaoMax, resBateria, 50};
  //valores padrao veiculo: 8bytes, 0, 0, 1, 0RPM, 0 km/h, 30C, 0min, 0
  msgVeiculo1 = {7, 0xFF, 0, 40, 90, 0};
}
