
#pragma once
#include <stdint.h>

// regen.h - Decisão de torque (tração / regeneração / zero)
// Não depende de Arduino, Serial, millis() nem de variáveis globais.
// Tudo entra por parâmetro, então dá para testar no PC.

// Estados de regeneração

enum class RegenEstado : uint8_t {
  Desabilitado = 0,  // Fora do Drive ou sistema não pronto
  Pronto = 1,        // Em Drive, torque zero
  Tracao = 2,        // Torque positivo
  Regenerando = 3,   // Torque negativo sem limitação
  Limitado = 4,      // Torque negativo com algum limite ativo
  Falha = 5          // Falha e bloqueio por um corte
};

// enum de motivos de corte (bits em RegenOut.motivos)

enum RegenMotivo : uint16_t {
  RM_NAO_PRONTO = 0x0001,
  RM_TIMEOUT_A3 = 0x0002,
  RM_TIMEOUT_A4 = 0x0004,
  RM_FALHA_BRM = 0x0008,
  RM_TRAVA_SOBRETENSAO = 0x0010,
  RM_DADO_INVALIDO = 0x0020,
  RM_PEDAL_FALHA = 0x0040,
  RM_ROTACAO_BAIXA = 0x0080,
  RM_LIM_TENSAO = 0x0100,
  RM_LIM_MOTOR = 0x0200,
  RM_LIM_BATERIA = 0x0400,
  RM_LIM_TERMICO = 0x0800
};

// calibração

struct RegenParams {
  // Tipos de regeneração

  float torque_freio;    // Regen ao pisar no freio
  float torque_lift;     // Regen ao soltar o acelerador
  float torque_min_cmd;  // Limite negativo base

  // bateria

  float i_carga_max;  // Corrente máxima de carga
  float v_reduz;      // Começa a reduzir a regen
  float v_corte;      // Trava e corta
  float v_religa;     // Destrava abaixo deste valor

  // rotação

  float rpm_sinal;  // +1 ou -1 com o carro indo para frente
  float rpm_zero;   // Abaixo disso a regen é zero
  float rpm_pleno;  // Acima disso a regen é máxima

  // validação

  float v_plausivel_min;  // Abaixo disso a tensão é considerada leitura errada

  //  temperatura

  float term_ini;
  float term_fim;

  // falha brm

  bool usar_flt_id;
  uint16_t flt_id_ok;  // BRM manda quando não há falha

  // timeouts das mensagens recebidas

  uint32_t timeout_a3_ms;
  uint32_t timeout_a4_ms;

  // Lift-off

  float lift_entra_nm;
  float lift_sai_nm;

  // Rampas em Nm/s

  float taxa_regen_sobe;
  float taxa_regen_solta;
  float taxa_tracao_sobe;
  float taxa_tracao_desce;

  // pedal

  int pedal_min;
  int pedal_max;
  int pedal_tolerancia;
  int pedal_margem_baixa;
  int pedal_margem_alta;
};

// ---------- Entradas de cada ciclo ----------

struct RegenIn {
  bool em_drive;
  bool sistema_pronto;   // Contator fechado
  bool freio;            // Freio pressionado
  bool pedal_filtro_ok;  // Filtro do pedal já encheu
  int pedal1;            // Média do ADC, sensor 1
  int pedal2;            // Média do ADC, sensor 2
  float tq_acel;         // Torque de tração pedido pelo acelerador (Nm, >= 0)
  float rpm;             // 0xA3
  float tensao;          // 0xA3
  float tq_min_avl;      // 0xA4 (Nm, negativo)
  float term;            // Maior uso térmico entre motor e inversor (%)
  uint16_t flt_id;       // 0xA4
  bool a3_recebido;
  bool a4_recebido;
  uint32_t idade_a3_ms;  // tempo chegou o último 0xA3
  uint32_t idade_a4_ms;
};

// Memória entre ciclos

struct RegenState {
  float ultimo_torque;
  bool trava_tensao;
  bool lift_ativo;
  bool iniciado;
  uint32_t t_anterior;
};

// saída

struct RegenOut {
  float torque;  // Torque a enviar (Nm). Positivo = tração, negativo = regen
  float alvo;    // Alvo antes da rampa
  bool idle;     // True = pode pedir modo Idle (regen pedida com torque zero)
  RegenEstado estado;  // Estado da regeneração
  uint16_t motivos;    // Bits RM_*
};

// Funções

void regenParamsPadrao(RegenParams* params);
void regenReset(RegenState* state);
void regenStep(const RegenParams* params, RegenState* state,
               const RegenIn* input, uint32_t agora_ms, RegenOut* output);
