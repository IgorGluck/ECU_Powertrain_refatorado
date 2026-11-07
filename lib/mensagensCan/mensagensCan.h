//-----------------------------------------------------------------------------------------//
//--- Cabeçalho com estruturas das mensagens CAN e prototipos das funções de conversão  ---//
//                                                                                         //
//    Cada mensagem tem uma struct correpondente, com o tamanho da mensagem e os campos    //
//        que podem ser alterados pelo programa. Tambem tem as funções de conversão        //
//           que transformam a struct em um vetor de bytes (unsigned char) pra ser         //
//              enviada pela CAN. funções de print serial para debug das                   //
//                         mensagens também estão implementadas                            //
//-----------------------------------------------------------------------------------------//

////////////////////////////////////////////////
/////       Mensagens enviadas para        /////
/////               o motor.               /////
////////////////////////////////////////////////

#pragma once
#include <stdint.h>

// Modo de operação
enum class ModoOperacao : uint8_t { 
  Idle = 0x0, 
  Torque = 0x1 
};

// Estado da ignição
enum class EstadoIgnicao : uint8_t { 
  Lock = 0x0, 
  Off = 0x1, 
  On = 0x4 
};


// Mensagem principal de controle, "Des" é "desired"
struct PtEdrvDes1
{
  uint8_t       tamanho;                 // tamanho da mensagem CAN
  uint8_t       pt_nr_chks_des;          // Checksum (CRC8)
  uint8_t       pt_nr_alv_cntr_des;      // Alive Counter
  ModoOperacao  pt_st_oper_mod_des_edrv; // Modo de operação (ModoOperacao::Idle ou ModoOperacao::Torque)
  float         pt_tq_des_edrv;          // Requisição de Torque (-200 a 209,5 Nm, resolucao 0,1 Nm)
  float         pt_tq_min_edrv;          // Limite de torque negativo (-204,7 a 0 Nm, resolucao 0,1 Nm)
  EstadoIgnicao pt_st_ign_swt;           // Estado da Chave de Ignição (EstadoIgnicao::Lock, Off ou On)
  float         pt_tq_max_edrv;          // Limite de torque positivo (0 a 204,7 Nm, resolucao 0,1 Nm)
};

struct EemEdrvLim1
{
  uint8_t  tamanho;
  uint8_t  eem_nr_chks_lim;      // Checksum
  uint8_t  eem_nr_alv_cntr_lim;  // Alive Counter
  float    eem_i_min_edrv;       // Entre -510 A e 0 A, resolução 2 A
  float    eem_i_max_edrv;       // Entre 0 A e 510 A, resolução 2 A
  float    eem_u_min_edrv;       // Entre 10 V e 61 V, resolução 0,2 V
  float    eem_u_max_edrv;       // Entre 10 V e 61 V, resolução 0,2 V
  float    eem_r_mvb_edrv;       // Entre 5 mOhm e 1,275 Ohm, resolucao 5 mOhm
  float    eem_u_des_edrv;       // Entre 30 V e 55,5 V, resolução 0,1 A
};

struct PtVeh1
{
  uint8_t  tamanho;
  uint8_t  pt_nr_chks_veh1;      // Checksum
  uint8_t  pt_nr_alv_cntr_veh1;  // Alive Counter
  int8_t   pt_t_amb_edrv;
  uint16_t pt_d_air_pres_edrv;
  uint32_t pt_dst_odo;            // odometro precisão de 32 bits pq nn sei a faixa de valores
};

struct PtVeh2
{
  uint8_t  tamanho;
  uint8_t  pt_nr_chks_veh2;      // Checksum
  uint8_t  pt_nr_alv_cntr_veh2;  // Alive Counter
  uint32_t pt_ti_abs;             // timestamp precisão de 32 bits
};


////////////////////////////////////////////////
/////       Mensagens recebidas do         /////
/////               motor.                 /////
////////////////////////////////////////////////


// Mensagem de Estado Atual
struct EdrvAct1
{
  uint8_t       tamanho;               // Tamanho da mensagem CAN
  uint8_t       edrv_nr_chks_act;      // Checksum
  uint8_t       edrv_nr_alv_cntr_act;  // Alive Counter
  uint8_t       edrv_st_run_act;       // Estado de funcionamento (bool ou estado curto, cabe em 1 byte)
  ModoOperacao  edrv_st_oper_mod_act;  // Modo de operação (ModoOperacao::Idle ou ModoOperacao::Torque)
  float         edrv_tq_act;           // Torque atual (-100 a 104,7 Nm, resolucao 0,1 Nm)
  int16_t       edrv_n_act;            // Rotação atual (-24000 a 25146 RPM, cabe perfeitamente em 2 bytes com sinal)
  float         edrv_u_act;            // Tensão atual (0 V a 63,9375 V, resolucao 0,0625 V)
  int16_t       edrv_i_act;            // Corrente atual (-500 A a 523 A, cabe perfeitamente em 2 bytes com sinal)
};

struct EdrvDern1
{
  uint8_t  tamanho;                // Tamanho da mensagem CAN
  uint8_t  edrv_nr_chks_dern;      // Checksum
  uint8_t  edrv_nr_alv_cntr_dern;  // Alive Counter
  float    edrv_tq_min_avl;        // Torque mínimo disponível (-90 a 12,3 Nm, resolucao 0,1 Nm)
  uint8_t  edrv_st_act_dern;       // Estado de Derating (ID curto, cabe em 1 byte)
  float    edrv_tq_max_avl;        // Torque máximo disponível (-12,3 e 90 Nm, resolucao 0,1 Nm)
  uint8_t  edrv_rat_t_crit_ivtr;   // Uso térmico do inversor (0 a 126%, cabe em 1 byte)
  uint8_t  edrv_rat_t_crit_mac;    // Uso térmico da máquina (0 a 126%, cabe em 1 byte)
  uint16_t edrv_st_flt_id;         // Código de Falha do BRM (IDs de falha podem passar de 255, 2 bytes é o ideal)
};

////////////////////////////////////////////////
/////       Funcoes para converter         /////
/////        as structs para array         /////
////////////////////////////////////////////////

// Converte a mensagem PtEdrvDes1 para enviar na CAN
void converteMsgControle(const PtEdrvDes1& msg, uint8_t* buffer);

// Converte a mensagem EemEdrvLim1 para enviar na CAN
void converteMsgLimites(const EemEdrvLim1& msg, uint8_t* buffer);

// Converte a mensagem PtVeh1 para enviar na CAN
void converteMsgVeiculo1(const PtVeh1& msg, uint8_t* buffer);

// Converte a mensagem PtVeh2 para enviar na CAN ( ainda nao existe verificar)
// void converteMsgVeiculo2(const PtVeh2& msg, uint8_t* buffer);


////////////////////////////////////////////////
/////         Funções de arrays            /////
/////           para as structs            /////
////////////////////////////////////////////////

// Converte o buffer CAN recebido preenchendo e retornando a struct correspondente
EdrvAct1  converteMsgEstadoAtual(const uint8_t* buffer);
EdrvDern1 converteMsgDerating(const uint8_t* buffer);

// Texto do estado de derating, ex.: 2 -> "Corrente DC" (estado alterado de int para uint8_t)
const char* textoDerating(uint8_t estado);


////////////////////////////////////////////////
/////             Proteção E2E             /////
/////        Cálculo de Checksum           /////
////////////////////////////////////////////////

void calculateCrc8(const uint8_t* buffer, uint8_t tamanho, uint32_t msgId);
