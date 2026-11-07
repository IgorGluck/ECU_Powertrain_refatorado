#include "mensagensCan.h"

float limites(float a, float min, float max)
{
  if (a > max) return max;
  if (a < min) return min;
  return a;
};

////////////////////////////////////////////////
/////       Funcoes para converter         /////
/////        as structs para array         /////
////////////////////////////////////////////////

//Converte a mensagem PtEdrvDes1 para enviar na CAN
//Mensagem 0xA6
//tamanho 7
//conversao de valor fisico para inteiro:
void converteMsgControle(const PtEdrvDes1& msg, uint8_t* buffer)
{
  
  int torque_min = (int) limites((msg.pt_tq_min_edrv + 204.6)/0.1, 0, 2047); 
  int torque_des = (int) limites((msg.pt_tq_des_edrv + 200  )/0.1, 0, 4095);
  int torque_max = (int) limites((msg.pt_tq_max_edrv + 0    )/0.1, 0, 2047);
  
  buffer[6] = ((torque_max & 0x3) << 6) + ((uint8_t)msg.pt_st_ign_swt << 3);
  buffer[5] = (torque_max >> 2 ) & 0xFF;
  buffer[4] = (torque_min << 1) + (torque_max >> 10);
  buffer[3] = ((torque_des & 0xF) << 4) + (torque_min >> 7);
  buffer[2] = torque_des >> 4;
  buffer[1] = ((uint8_t)msg.pt_st_oper_mod_des_edrv << 4) + msg.pt_nr_alv_cntr_des;
  buffer[0] = msg.pt_nr_chks_des;
};

//Converte a mensagem EemEdrvLim1 para enviar na CAN
void converteMsgLimites(const EemEdrvLim1& msg, uint8_t* buffer)
{
  //Mensagem 0xA5
  int correnteMin   = (int) limites((msg.eem_i_min_edrv +  510)/2     ,  0, 255);
  int correnteMax   = (int) limites((msg.eem_i_max_edrv + 0)/2       ,  0, 255);
  int tensaoDes     = (int) limites((msg.eem_u_des_edrv -   30)/0.1   ,  0, 255);
  int resBateria    = (int) limites((msg.eem_r_mvb_edrv -0.005)/0.005 ,  0, 255);
  int tensaoMin     = (int) limites((msg.eem_u_min_edrv -   10)/0.2   ,  0, 255);
  int tensaoMax     = (int) limites((msg.eem_u_max_edrv -   10)/0.2   ,  0, 255);
  
  buffer[7] = tensaoDes;
  buffer[6] = resBateria;
  buffer[5] = tensaoMax;
  buffer[4] = tensaoMin;
  buffer[3] = correnteMax;
  buffer[2] = correnteMin;
  buffer[1] = msg.eem_nr_alv_cntr_lim; 
  buffer[0] = msg.eem_nr_chks_lim;
};

//Converte a mensagem Pt_Veh_1 para enviar na CAN
void converteMsgVeiculo1(const PtVeh1& msg, uint8_t* buffer)
{
  //Mensagem 0xA7
  buffer[6] = msg.pt_dst_odo & 0xFF;
  buffer[5] = (msg.pt_dst_odo >> 8) & 0xFF;
  buffer[4] = msg.pt_dst_odo >> 16;
  buffer[3] = msg.pt_d_air_pres_edrv;
  buffer[2] = msg.pt_t_amb_edrv;
  buffer[1] = msg.pt_nr_alv_cntr_veh1;
  buffer[0] = msg.pt_nr_chks_veh1;
};



////////////////////////////////////////////////
/////          Funcoes de arrays           /////
/////           para as structs            /////
////////////////////////////////////////////////

EdrvAct1 converteMsgEstadoAtual(const uint8_t* buffer)
{
  EdrvAct1 a;
  unsigned int torqueAtual;
  unsigned int velocidadeAtual;
  unsigned int tensaoAtual;
  unsigned int correnteAtual;

  a.tamanho = 8;
  a.edrv_nr_chks_act    = buffer[0];
  a.edrv_nr_alv_cntr_act = 0x0F & buffer [1];
  a.edrv_st_run_act     = (0xE0 & buffer [1]) >> 5;
  a.edrv_st_oper_mod_act = (ModoOperacao)((0xF0 & buffer [2]) >> 4) ;
  

  // CORREÇÃO: Adicionada a máscara (&) e trocado + por |
  torqueAtual         = ((buffer[2] & 0x0F) << 7) | (buffer[3] >> 1);
  velocidadeAtual     = ((buffer[3] & 0x01) << 12) | (buffer[4] << 4) | (buffer[5] >> 4);
  tensaoAtual         = ((buffer[5] & 0x0F) << 6) | (buffer[6] >> 2);
  correnteAtual       = ((buffer[6] & 0x03) << 8) | buffer[7];

  a.edrv_tq_act  = (float) torqueAtual    *0.1    - 100;
  a.edrv_n_act   = (int) velocidadeAtual  *6      - 24000;
  a.edrv_u_act   = (float) tensaoAtual    *0.0625 - 0;
  a.edrv_i_act   = (int)   correnteAtual  *1      - 500;
  return a;
}

// Texto do estado de derating (edrv_st_act_dern): qual restrição está limitando a BRM
const char* textoDerating(uint8_t estado)
{
  switch (estado)
  {
    case 0:  return "Sem derating";
    case 1:  return "Tensao DC";
    case 2:  return "Corrente DC";
    case 3:  return "Torque";
    case 4:  return "Rotacao";
    case 5:  return "Inversor";
    case 6:  return "Temperatura do motor";
    case 7:  return "Corrente de fase";
    default: return "Desconhecido";
  }
}

EdrvDern1 converteMsgDerating(const uint8_t* buffer)
{
  EdrvDern1 a;
  int torqueMinDisp;
  int torqueMaxDisp;

  a.tamanho = 8;
  a.edrv_nr_chks_dern     = buffer[0];
  a.edrv_nr_alv_cntr_dern  = buffer[1] & 0x0F;
  
  // CORREÇÃO: Trocado + por | para resolver o bug do >> (precedência)
  torqueMinDisp         = (buffer[2] << 2) | (buffer[3] >> 6);
  torqueMaxDisp         = ((buffer[3] & 0x3F) << 4) | (buffer[4] >> 4);
  a.edrv_st_act_dern      = buffer[4] & 0xF;

  // As temperaturas (ratTCritIvtr / ratTCritMac) têm 6 bits cada (0 a 63 -> 0 a 126%),
  // e não 8: os 2 bits restantes do byte pertencem a outros sinais. Por isso é
  // preciso deslocar (TCRIT_DESLOC) e mascarar com 0x3F antes de converter. 
  // Resolução de 2% por unidade: multiplica por 2 para obter a porcentagem.
  static const uint8_t TCRIT_DESLOC = 2; // descarta os dois bit de outro sinal(verificar os bits no brm)

  a.edrv_rat_t_crit_ivtr   = ((buffer[5] >> TCRIT_DESLOC) & 0x3F) * 2; 
  a.edrv_rat_t_crit_mac    = ((buffer[6] >> TCRIT_DESLOC) & 0x3F) * 2; 
  a.edrv_st_flt_id        = buffer[7];     // Isola o byte correto de falha

  a.edrv_tq_max_avl     = (float) torqueMaxDisp *0.1 - 12.3;
  a.edrv_tq_min_avl     = (float) torqueMinDisp *0.1 - 90;
  return a;
}
////////////////////////////////////////////////
/////             Proteção E2E             /////
/////        Cálculo de Checksum           /////
////////////////////////////////////////////////

// static para nao vazar a tabela de lookup para outros arquivos

static const uint8_t CRCLookUpTable[256]={
0x00	,0x2F	,0x5E	,0x71	,0xBC	,0x93	,0xE2	,0xCD	,
0x57	,0x78	,0x09	,0x26	,0xEB	,0xC4	,0xB5	,0x9A	,
0xAE	,0x81	,0xF0	,0xDF	,0x12	,0x3D	,0x4C	,0x63	,
0xF9	,0xD6	,0xA7	,0x88	,0x45	,0x6A	,0x1B	,0x34	,
0x73	,0x5C	,0x2D	,0x02	,0xCF	,0xE0	,0x91	,0xBE	,
0x24	,0x0B	,0x7A	,0x55	,0x98	,0xB7	,0xC6	,0xE9	,
0xDD	,0xF2	,0x83	,0xAC	,0x61	,0x4E	,0x3F	,0x10	,
0x8A	,0xA5	,0xD4	,0xFB	,0x36	,0x19	,0x68	,0x47	,
0xE6	,0xC9	,0xB8	,0x97	,0x5A	,0x75	,0x04	,0x2B	,
0xB1	,0x9E	,0xEF	,0xC0	,0x0D	,0x22	,0x53	,0x7C	,
0x48	,0x67	,0x16	,0x39	,0xF4	,0xDB	,0xAA	,0x85	,
0x1F	,0x30	,0x41	,0x6E	,0xA3	,0x8C	,0xFD	,0xD2	,
0x95	,0xBA	,0xCB	,0xE4	,0x29	,0x06	,0x77	,0x58	,
0xC2	,0xED	,0x9C	,0xB3	,0x7E	,0x51	,0x20	,0x0F	,
0x3B	,0x14	,0x65	,0x4A	,0x87	,0xA8	,0xD9	,0xF6	,
0x6C	,0x43	,0x32	,0x1D	,0xD0	,0xFF	,0x8E	,0xA1	,
0xE3	,0xCC	,0xBD	,0x92	,0x5F	,0x70	,0x01	,0x2E	,
0xB4	,0x9B	,0xEA	,0xC5	,0x08	,0x27	,0x56	,0x79	,
0x4D	,0x62	,0x13	,0x3C	,0xF1	,0xDE	,0xAF	,0x80	,
0x1A	,0x35	,0x44	,0x6B	,0xA6	,0x89	,0xF8	,0xD7	,
0x90	,0xBF	,0xCE	,0xE1	,0x2C	,0x03	,0x72	,0x5D	,
0xC7	,0xE8	,0x99	,0xB6	,0x7B	,0x54	,0x25	,0x0A	,
0x3E	,0x11	,0x60	,0x4F	,0x82	,0xAD	,0xDC	,0xF3	,
0x69	,0x46	,0x37	,0x18	,0xD5	,0xFA	,0x8B	,0xA4	,
0x05	,0x2A	,0x5B	,0x74	,0xB9	,0x96	,0xE7	,0xC8	,
0x52	,0x7D	,0x0C	,0x23	,0xEE	,0xC1	,0xB0	,0x9F	,
0xAB	,0x84	,0xF5	,0xDA	,0x17	,0x38	,0x49	,0x66	,
0xFC	,0xD3	,0xA2	,0x8D	,0x40	,0x6F	,0x1E	,0x31	,
0x76	,0x59	,0x28	,0x07	,0xCA	,0xE5	,0x94	,0xBB	,
0x21	,0x0E	,0x7F	,0x50	,0x9D	,0xB2	,0xC3	,0xEC	,
0xD8	,0xF7	,0x86	,0xA9	,0x64	,0x4B	,0x3A	,0x15	,
0x8F	,0xA0	,0xD1	,0xFE	,0x33	,0x1C	,0x6D	,0x42 };

// Lookup do PDF E2E protection
static const uint8_t BNZCntrLookUp2[16] = {
  0x01, 0x03, 0x05, 0x08, 0x17, 0xC2, 0x07, 0x17,
  0xFF, 0x09, 0x68, 0x01, 0x18, 0x92, 0xbd, 0x4f };

// Lookup da planilha E2E protection
static const uint8_t BNZCntrLookUp[16] = {
  0x89, 0x11, 0xF3, 0xC3, 0x9D, 0x5E, 0x67, 0x3B,
  0xC9, 0x01, 0xBB, 0xB7, 0x0B, 0x9F, 0x8F, 0x92 };

void calculateCrc8(uint8_t* buffer, uint8_t tamanho, uint32_t msgId)
{
  int i = 0;
  uint8_t aliveCounter = buffer[1] & 0xF;

  buffer[0] = 0xFF;

  for(i = 1; i < tamanho; i++)
  {
    buffer[0] = CRCLookUpTable[(buffer[0] ^ buffer[i])];
  }

  // Multiplexador E2E baseado no CAN ID
  if (msgId == 0xA3) {
    buffer[0] = ~CRCLookUpTable[(buffer[0] ^ BNZCntrLookUp2[aliveCounter])];
  } else {
    buffer[0] = ~CRCLookUpTable[(buffer[0] ^ BNZCntrLookUp[aliveCounter])];
  }
}