
// ============================================================================
// teste_ecu_mesa.ino
// TESTE DE BANCADA - MONITORAMENTO DA ECU E REGENERACAO EM MODO SHADOW
//
// Hardware:
//   - ESP32 conectado ao barramento CAN do inversor
//   - Acelerador 1: GPIO 14
//   - Acelerador 2: GPIO 27
//   - Seletor Drive: GPIO 25 (ativo em LOW)
//   - Freio: simulado pela Serial USB (tecla 'b')
//
// SEGURANCA:
//   - Nenhuma mensagem CAN é transmitida.
//   - Nenhum torque é enviado ao inversor.
//   - Não aciona pré-carga, contator ou ignição.
//   - O cálculo de regeneração é somente para observação.
// ============================================================================

#include <ACAN_ESP32.h>
#include <Arduino.h>

#include "mensagensCan.h"
#include "regen.h"

// ============================================================================
// PINOS
// ============================================================================

#define sinalDrive GPIO_NUM_25
#define sinalAcel1 GPIO_NUM_14
#define sinalAcel2 GPIO_NUM_27

#define CANTX GPIO_NUM_33
#define CANRX GPIO_NUM_32

// ============================================================================
// CONFIGURACOES
// ============================================================================

const uint32_t CAN_BAUD = 500E3;

const uint32_t INTERVALO_PEDAL = 10;
const uint32_t INTERVALO_PRINT = 1000;

const uint8_t FILTER_SIZE = 5;

const int PEDAL_MIN = 470;
const int PEDAL_MAX = 1150;
const int PEDAL_TOLERANCE = 700;

const float TORQUE_MAX_TRACAO = 55.0f;

// ============================================================================
// REGENERACAO
// ============================================================================

RegenParams regenParams;
RegenState regenState;
RegenIn regenInput;
RegenOut regenOutput;

// ============================================================================
// MENSAGENS CAN
// ============================================================================

CANMessage bufferRx;

EdrvAct1 msgEstadoAtual = {};
EdrvDern1 msgDeratingAtual = {};

bool a3Recebido = false;
bool a4Recebido = false;

uint32_t ultimoA3_ms = 0;
uint32_t ultimoA4_ms = 0;

// ============================================================================
// FILTRO DOS PEDAIS
// ============================================================================

int bufferPedal1[FILTER_SIZE] = {};
int bufferPedal2[FILTER_SIZE] = {};

uint8_t indexBuffer = 0;
uint8_t amostrasPedal = 0;

uint32_t ultimaLeituraPedal_ms = 0;
uint32_t ultimoPrint_ms = 0;
uint32_t ultimoStepRegen_ms = 0;

// ============================================================================
// ESTADO DA ECU DE TESTE
// ============================================================================

enum class EstadoTeste : uint8_t { Neutro = 0, Drive = 1 };

EstadoTeste estadoTeste = EstadoTeste::Neutro;

bool freioSimulado = false;

// ============================================================================
// MEDIA DO FILTRO
// ============================================================================

int mediaBuffer(const int* buffer, uint8_t tamanho) {
  long soma = 0;

  for (uint8_t i = 0; i < tamanho; i++) {
    soma += buffer[i];
  }

  return (int)(soma / tamanho);
}

// ============================================================================
// LEITURA DOS PEDAIS
// ============================================================================

void atualizarPedais() {
  uint32_t agora = millis();

  if (agora - ultimaLeituraPedal_ms < INTERVALO_PEDAL) {
    return;
  }

  bufferPedal1[indexBuffer] = analogRead(sinalAcel1);
  bufferPedal2[indexBuffer] = analogRead(sinalAcel2);

  indexBuffer = (indexBuffer + 1) % FILTER_SIZE;

  if (amostrasPedal < FILTER_SIZE) {
    amostrasPedal++;
  }

  ultimaLeituraPedal_ms = agora;
}

// ============================================================================
// CALCULO DO TORQUE DO ACELERADOR
// ============================================================================

float calcularTorqueAcelerador() {
  if (amostrasPedal < FILTER_SIZE) {
    return 0.0f;
  }

  int p1 = mediaBuffer(bufferPedal1, FILTER_SIZE);
  int p2 = mediaBuffer(bufferPedal2, FILTER_SIZE);

  if (abs(p1 - p2) > PEDAL_TOLERANCE) {
    return 0.0f;
  }

  int adc = (p1 + p2) / 2;

  adc = constrain(adc, PEDAL_MIN, PEDAL_MAX);

  float percentual = (float)(adc - PEDAL_MIN) / (float)(PEDAL_MAX - PEDAL_MIN);

  return percentual * TORQUE_MAX_TRACAO;
}

// ============================================================================
// COMANDOS SERIAL
// ============================================================================

void processarSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == 'b' || c == 'B') {
      freioSimulado = !freioSimulado;

      Serial.print("\nFREIO SIMULADO: ");

      Serial.println(freioSimulado ? "PRESSIONADO" : "SOLTO");

    }

    else if (c == 'h' || c == 'H' || c == '?') {
      Serial.println("\nCOMANDOS:");
      Serial.println("b = alternar freio simulado");
      Serial.println("h = ajuda");
      Serial.println("r = reset logico");

    }

    else if (c == 'r' || c == 'R') {
      estadoTeste = EstadoTeste::Neutro;

      regenReset(&regenState);

      Serial.println("\nRESET LOGICO: ECU voltou para NEUTRO.");
    }
  }
}

// ============================================================================
// RECEPCAO CAN
// ============================================================================

void receberCAN() {
  while (ACAN_ESP32::can.receive(bufferRx)) {
    if (bufferRx.rtr || bufferRx.ext || bufferRx.len != 8) {
      continue;
    }

    uint32_t agora = millis();

    if (bufferRx.id == 0xA3) {
      msgEstadoAtual = converteMsgEstadoAtual(bufferRx.data);

      ultimoA3_ms = agora;
      a3Recebido = true;

    }

    else if (bufferRx.id == 0xA4) {
      msgDeratingAtual = converteMsgDerating(bufferRx.data);

      ultimoA4_ms = agora;
      a4Recebido = true;
    }
  }
}

// ============================================================================
// VALIDACAO DE RECEPCAO CAN
// ============================================================================

bool a3Atual(uint32_t agora) {
  return a3Recebido && (agora - ultimoA3_ms <= regenParams.timeout_a3_ms);
}

bool a4Atual(uint32_t agora) {
  return a4Recebido && (agora - ultimoA4_ms <= regenParams.timeout_a4_ms);
}

// ============================================================================
// MAQUINA DE ESTADOS
// ============================================================================

void atualizarEstadoDrive() {
  bool driveAcionado = (digitalRead(sinalDrive) == LOW);

  // Entrada em Drive exige Drive físico + freio pressionado.

  if (estadoTeste == EstadoTeste::Neutro) {
    if (driveAcionado && freioSimulado) {
      estadoTeste = EstadoTeste::Drive;

      regenReset(&regenState);

      Serial.println("\nESTADO: DRIVE");
    }
  }

  // Em Drive, soltar o freio não tira a ECU de Drive.
  // Isso permite testar a regeneração por lift-off.

  else if (!driveAcionado) {
    estadoTeste = EstadoTeste::Neutro;

    regenReset(&regenState);

    Serial.println("\nESTADO: NEUTRO");
  }
}

// ============================================================================
// PREENCHIMENTO DAS ENTRADAS DA REGENERACAO
// ============================================================================

void preencherEntradaRegen(uint32_t agora) {
  bool emDrive = (estadoTeste == EstadoTeste::Drive);

  bool pedalOk = (amostrasPedal >= FILTER_SIZE);

  int p1 = pedalOk ? mediaBuffer(bufferPedal1, FILTER_SIZE) : 0;

  int p2 = pedalOk ? mediaBuffer(bufferPedal2, FILTER_SIZE) : 0;

  regenInput.em_drive = emDrive;

  // IMPORTANTE:
  // Proxy de disponibilidade baseado em A3/A4 recentes.
  // Não é confirmação física do contator.

  regenInput.sistema_pronto = a3Atual(agora) && a4Atual(agora);

  regenInput.freio = freioSimulado;

  regenInput.pedal_filtro_ok = pedalOk;

  regenInput.pedal1 = p1;
  regenInput.pedal2 = p2;

  regenInput.tq_acel = emDrive ? calcularTorqueAcelerador() : 0.0f;

  regenInput.rpm = (float)msgEstadoAtual.edrv_n_act;

  regenInput.tensao = msgEstadoAtual.edrv_u_act;

  regenInput.tq_min_avl = msgDeratingAtual.edrv_tq_min_avl;

  float tempMotor = (float)msgDeratingAtual.edrv_rat_t_crit_mac;

  float tempInversor = (float)msgDeratingAtual.edrv_rat_t_crit_ivtr;

  regenInput.term = (tempMotor > tempInversor) ? tempMotor : tempInversor;

  regenInput.flt_id = msgDeratingAtual.edrv_st_flt_id;

  regenInput.a3_recebido = a3Recebido;
  regenInput.a4_recebido = a4Recebido;

  regenInput.idade_a3_ms = a3Recebido ? agora - ultimoA3_ms : UINT32_MAX;

  regenInput.idade_a4_ms = a4Recebido ? agora - ultimoA4_ms : UINT32_MAX;
}

// ============================================================================
// NOME DOS ESTADOS DA REGENERACAO
// ============================================================================

const char* nomeEstadoRegen(RegenEstado estado) {
  switch (estado) {
    case RegenEstado::Desabilitado:
      return "Desabilitado";

    case RegenEstado::Pronto:
      return "Pronto";

    case RegenEstado::Tracao:
      return "Tracao";

    case RegenEstado::Regenerando:
      return "Regenerando";

    case RegenEstado::Limitado:
      return "Limitado";

    case RegenEstado::Falha:
      return "Falha/Corte";

    default:
      return "Desconhecido";
  }
}

// ============================================================================
// IMPRESSAO DOS MOTIVOS
// ============================================================================

void imprimirMotivos(uint16_t m) {
  if (m == 0) {
    Serial.println("Nenhum");
    return;
  }

  bool separador = false;

#define MOTIVO(bit, nome)               \
  if (m & bit) {                        \
    if (separador) Serial.print(" | "); \
    Serial.print(nome);                 \
    separador = true;                   \
  }

  MOTIVO(RM_NAO_PRONTO, "Sistema nao pronto");
  MOTIVO(RM_TIMEOUT_A3, "Timeout A3");
  MOTIVO(RM_TIMEOUT_A4, "Timeout A4");
  MOTIVO(RM_FALHA_BRM, "Falha BRM");
  MOTIVO(RM_TRAVA_SOBRETENSAO, "Trava sobretensao");
  MOTIVO(RM_DADO_INVALIDO, "Dado invalido");
  MOTIVO(RM_PEDAL_FALHA, "Falha pedal");
  MOTIVO(RM_ROTACAO_BAIXA, "Rotacao baixa");
  MOTIVO(RM_LIM_TENSAO, "Limitacao tensao");
  MOTIVO(RM_LIM_MOTOR, "Limitacao motor");
  MOTIVO(RM_LIM_BATERIA, "Limitacao bateria");
  MOTIVO(RM_LIM_TERMICO, "Limitacao termica");

#undef MOTIVO

  Serial.println();
}

// ============================================================================
// DIAGNOSTICO
// ============================================================================

void imprimirDiagnostico() {
  uint32_t agora = millis();

  bool driveFisico = (digitalRead(sinalDrive) == LOW);

  bool a3Ok = a3Atual(agora);
  bool a4Ok = a4Atual(agora);

  int p1 = (amostrasPedal == FILTER_SIZE)
               ? mediaBuffer(bufferPedal1, FILTER_SIZE)
               : 0;

  int p2 = (amostrasPedal == FILTER_SIZE)
               ? mediaBuffer(bufferPedal2, FILTER_SIZE)
               : 0;

  Serial.println("\n================================================");
  Serial.println(" ECU - TESTE DE BANCADA / REGEN SHADOW");
  Serial.println("================================================");

  Serial.print("Estado ECU: ");

  Serial.println(estadoTeste == EstadoTeste::Drive ? "DRIVE" : "NEUTRO");

  Serial.print("Drive fisico: ");
  Serial.println(driveFisico ? "ACIONADO" : "SOLTO");

  Serial.print("Freio simulado: ");
  Serial.println(freioSimulado ? "PRESSIONADO" : "SOLTO");

  Serial.print("Filtro pedal: ");
  Serial.println(amostrasPedal == FILTER_SIZE ? "PRONTO" : "ENCHENDO");

  Serial.print("Pedal 1 ADC: ");
  Serial.println(p1);

  Serial.print("Pedal 2 ADC: ");
  Serial.println(p2);

  Serial.print("Torque acelerador: ");
  Serial.print(calcularTorqueAcelerador(), 2);
  Serial.println(" Nm");

  // ------------------------------------------------
  // CAN
  // ------------------------------------------------

  Serial.println("\n--- RECEPCAO CAN ---");

  Serial.print("A3: ");
  Serial.println(a3Ok ? "ATUAL" : "AUSENTE/TIMEOUT");

  Serial.print("A4: ");
  Serial.println(a4Ok ? "ATUAL" : "AUSENTE/TIMEOUT");

  Serial.print("Idade A3: ");
  Serial.print(a3Recebido ? agora - ultimoA3_ms : UINT32_MAX);
  Serial.println(" ms");

  Serial.print("Idade A4: ");
  Serial.print(a4Recebido ? agora - ultimoA4_ms : UINT32_MAX);
  Serial.println(" ms");

  // ------------------------------------------------
  // A3
  // ------------------------------------------------

  if (a3Recebido) {
    Serial.println("\n--- INVERSOR / A3 ---");

    Serial.print("Estado run: ");
    Serial.println(msgEstadoAtual.edrv_st_run_act);

    Serial.print("Modo operacao: ");
    Serial.println((uint8_t)msgEstadoAtual.edrv_st_oper_mod_act);

    Serial.print("Torque atual: ");
    Serial.print(msgEstadoAtual.edrv_tq_act, 2);
    Serial.println(" Nm");

    Serial.print("RPM: ");
    Serial.println(msgEstadoAtual.edrv_n_act);

    Serial.print("Tensao DC reportada: ");
    Serial.print(msgEstadoAtual.edrv_u_act, 2);
    Serial.println(" V");

    Serial.print("Corrente atual: ");
    Serial.print(msgEstadoAtual.edrv_i_act);
    Serial.println(" A");

  } else {
    Serial.println("\nA3 ainda nao recebida.");
  }

  // ------------------------------------------------
  // A4
  // ------------------------------------------------

  if (a4Recebido) {
    Serial.println("\n--- DERATING / A4 ---");

    Serial.print("Torque minimo disponivel: ");
    Serial.print(msgDeratingAtual.edrv_tq_min_avl, 2);
    Serial.println(" Nm");

    Serial.print("Torque maximo disponivel: ");
    Serial.print(msgDeratingAtual.edrv_tq_max_avl, 2);
    Serial.println(" Nm");

    Serial.print("Estado derating: ");
    Serial.println(msgDeratingAtual.edrv_st_act_dern);

    Serial.print("Temperatura/uso critico inversor: ");
    Serial.print(msgDeratingAtual.edrv_rat_t_crit_ivtr);
    Serial.println(" %");

    Serial.print("Temperatura/uso critico motor: ");
    Serial.print(msgDeratingAtual.edrv_rat_t_crit_mac);
    Serial.println(" %");

    Serial.print("Falha BRM ID: ");
    Serial.println(msgDeratingAtual.edrv_st_flt_id);

  } else {
    Serial.println("\nA4 ainda nao recebida.");
  }

  // ------------------------------------------------
  // REGENERACAO
  // ------------------------------------------------

  Serial.println("\n--- REGENERACAO SHADOW ---");

  Serial.print("Estado: ");
  Serial.println(nomeEstadoRegen(regenOutput.estado));

  Serial.print("Torque regenerativo calculado: ");
  Serial.print(regenOutput.torque, 3);
  Serial.println(" Nm");

  Serial.print("Torque alvo limitado: ");
  Serial.print(regenOutput.alvo, 3);
  Serial.println(" Nm");

  Serial.print("Idle: ");
  Serial.println(regenOutput.idle ? "SIM" : "NAO");

  Serial.print("Motivos (HEX): 0x");
  Serial.println(regenOutput.motivos, HEX);

  Serial.print("Detalhes: ");
  imprimirMotivos(regenOutput.motivos);

  // ------------------------------------------------
  // TENSAO
  // ------------------------------------------------

  Serial.println("\n--- TENSAO UTILIZADA NO REGEN ---");

  Serial.print("Tensao recebida em A3: ");
  Serial.print(regenInput.tensao, 2);
  Serial.println(" V");

  Serial.println("Campo utilizado: edrv_u_act.");

  Serial.println(
      "Confirme no DBC/manual se representa a tensao DC do barramento.");

  Serial.println("Nao confundir com tensao de fase U/V/W do motor.");

  // ------------------------------------------------
  // SEGURANCA
  // ------------------------------------------------

  Serial.println("\n--- SEGURANCA ---");

  Serial.println("CAN TX: DESABILITADO");
  Serial.println("Torque enviado ao inversor: NAO");
  Serial.println("Pre-carga: NAO acionada");
  Serial.println("Contator: NAO acionado");
  Serial.println("Ignicao: NAO acionada");

  Serial.println(
      "Sistema pronto: proxy de A3/A4 recentes, nao feedback do contator.");

  Serial.println("================================================");
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);

  delay(500);

  pinMode(sinalDrive, INPUT_PULLUP);
  pinMode(sinalAcel1, INPUT);
  pinMode(sinalAcel2, INPUT);

  analogReadResolution(12);

  // Parametros reais padrao do regen.cpp

  regenParamsPadrao(&regenParams);

  regenParams.pedal_min = PEDAL_MIN;
  regenParams.pedal_max = PEDAL_MAX;
  regenParams.pedal_tolerancia = PEDAL_TOLERANCE;

  regenReset(&regenState);

  Serial.println("\nInicializando CAN...");

  ACAN_ESP32_Settings settings(CAN_BAUD);

  settings.mTxPin = CANTX;
  settings.mRxPin = CANRX;

  while (true) {
    int erro = ACAN_ESP32::can.begin(settings);

    if (erro == 0) {
      Serial.println("CAN inicializada.");
      break;
    }

    Serial.print("Erro CAN: 0x");
    Serial.println(erro, HEX);

    delay(1000);
  }

  Serial.println("\nTESTE PRONTO.");

  Serial.println("Monitor Serial: 115200 baud.");

  Serial.println("b = alternar freio simulado");
  Serial.println("h = ajuda");
  Serial.println("r = reset logico");

  Serial.println(
      "Para entrar em Drive: Drive fisico acionado + freio pressionado.");

  Serial.println("Depois solte o freio para testar lift-off.");

  Serial.println("Solte Drive para retornar ao Neutro.");

  Serial.println("ATENCAO: este firmware nao transmite CAN.");

  ultimoPrint_ms = millis();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  atualizarPedais();

  processarSerial();

  receberCAN();

  atualizarEstadoDrive();

  uint32_t agora = millis();

  if (agora - ultimoStepRegen_ms >= INTERVALO_PEDAL) {
    preencherEntradaRegen(agora);

    regenStep(&regenParams, &regenState, &regenInput, agora, &regenOutput);

    ultimoStepRegen_ms = agora;
  }

  if (agora - ultimoPrint_ms >= INTERVALO_PRINT) {
    imprimirDiagnostico();

    ultimoPrint_ms = agora;
  }
}
