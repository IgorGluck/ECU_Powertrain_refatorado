#include "regen.h"
#include <math.h>
#include <stdlib.h>
// funões de inicialização

namespace
{

  float limites(float a, float min, float max)
  {
    if (a > max)
      return max;
    if (a < min)
      return min;
    return a;
  };

  bool numeroValido(float a)
  {
    return isfinite(a);
  }

  void adicionaMotivo(uint16_t *motivos, uint16_t motivo)
  {
    *motivos |= motivo;
  }

  // faz descer em funcao afim
  float fatorDescendo(float x, float ini, float fim)
  {
    if (x <= ini)
      return 1.0f;
    if (x >= fim)
      return 0.0f;
    return (fim - x) / (fim - ini);
  }

  // verificar se o pedal ta OK
  bool pedalPlausivel(const RegenParams *params, const RegenIn *input)
  {
    if (!input->pedal_filtro_ok)
      return false; // filtro ainda não encheu

    int diferenca = input->pedal1 - input->pedal2;
    if (diferenca < 0)
      diferenca = -diferenca; // valor absoluto
    if (diferenca > params->pedal_tolerancia)
      return false; // sensores discordam

    int baixo = params->pedal_min - params->pedal_margem_baixa;
    int alto = params->pedal_max + params->pedal_margem_alta;
    if (input->pedal1 < baixo || input->pedal1 > alto)
      return false;
    if (input->pedal2 < baixo || input->pedal2 > alto)
      return false;

    return true;
  }

}

// Inicialização dos parâmetros padrão

void regenParamsPadrao(RegenParams *params)
{

  // proteção contra ponteiros nulos
  if (params == nullptr)
  {
    return;
  }

  params->torque_freio = -1.0f;
  params->torque_lift = -0.5f;
  params->torque_min_cmd = -2.0f;

  params->i_carga_max = 4.0f;
  params->v_reduz = 51.5f;
  params->v_corte = 53.0f;
  params->v_religa = 51.0f;

  params->rpm_sinal = 1.0f;
  params->rpm_zero = 500.0f;
  params->rpm_pleno = 1000.0f;

  params->v_plausivel_min = 20.0f;

  params->term_ini = 75.0f;
  params->term_fim = 100.0f;

  params->usar_flt_id = true;
  params->flt_id_ok = 0;

  params->timeout_a3_ms = 200;
  params->timeout_a4_ms = 500;

  params->lift_entra_nm = 0.5f;
  params->lift_sai_nm = 1.0f;

  params->taxa_regen_sobe = 50.0f;
  params->taxa_regen_solta = 100.0f;
  params->taxa_tracao_sobe = 0.0f;
  params->taxa_tracao_desce = 0.0f;

  params->pedal_min = 470;
  params->pedal_max = 1150;
  params->pedal_tolerancia = 100;
  params->pedal_margem_baixa = 150;
  params->pedal_margem_alta = 150;
}

// Reset dos estados

void regenReset(RegenState *state)
{

  // proteção contra ponteiros nulos
  if (state == nullptr)
  {
    return;
  }

  state->ultimo_torque = 0.0f;
  state->trava_tensao = false;
  state->lift_ativo = false;
  state->iniciado = false;
  state->t_anterior = 0;
}

// verifica se os parâmetros são válidos

void regenStep(
    const RegenParams *params,
    RegenState *state,
    const RegenIn *input,
    uint32_t agora_ms,
    RegenOut *output)
{

  if (params == nullptr || state == nullptr || input == nullptr || output == nullptr)
  {
    return;
  }

  //  ver se passou 10ms para retornar o looop( se passou de 0.5s usar dt = 0.5s)

  float dt = 0.0f;
  if (state->iniciado)
  {
    uint32_t passou = agora_ms - state->t_anterior;
    dt = (passou > 50) ? 0.05f : (float)passou / 1000.0f;
  }
  state->iniciado = true;
  state->t_anterior = agora_ms;

  // zera tudo

  uint16_t m = 0; // motivos
  output->torque = 0.0f;
  output->alvo = 0.0f;
  output->idle = true;
  output->estado = RegenEstado::Desabilitado;
  output->motivos = 0;

  // se os dados Ok

  bool dadosOk = numeroValido(input->rpm) &&
                 numeroValido(input->tensao) &&
                 numeroValido(input->tq_min_avl) &&
                 numeroValido(input->term) &&
                 (input->tensao) >= (params->v_plausivel_min);

  // proteção contra tensao inválida

  if (dadosOk)
  {
    if (input->tensao >= params->v_corte)
      state->trava_tensao = true;
    else if (input->tensao < params->v_religa)
      state->trava_tensao = false;
  }

  // se não tiver no drive zera tudo e retorna nao pronto

  if (!input->em_drive || !input->sistema_pronto)
  {
    state->ultimo_torque = 0.0f;
    state->lift_ativo = false;
    if (input->em_drive)
      adicionaMotivo(&m, RM_NAO_PRONTO);
    output->motivos = m;
    return;
  }

  // validar o pedal

  bool pedalOk = pedalPlausivel(params, input);
  float acel = 0.0f;

  if (pedalOk)
  {
    acel = (input->tq_acel > 0.0f) ? input->tq_acel : 0.0f;
    if (!state->lift_ativo && acel <= params->lift_entra_nm)
      state->lift_ativo = true;
    else if (state->lift_ativo && acel > params->lift_sai_nm)
      state->lift_ativo = false;
  }
  else
  {
    state->lift_ativo = false;
    adicionaMotivo(&m, RM_PEDAL_FALHA);
  }

  // verificar intenção do piloto
  float alvo = 0.0f;
  bool regen = false;

  if (input->freio)
  {
    alvo = params->torque_freio;
    regen = alvo < 0.0f;
  }
  else if (state->lift_ativo)
  {
    alvo = params->torque_lift;
    regen = alvo < 0.0f;
  }

  //  condições de corte

  uint16_t corte = 0;

  if (!input->a3_recebido || input->idade_a3_ms > params->timeout_a3_ms)
    adicionaMotivo(&corte, RM_TIMEOUT_A3);
  if (!input->a4_recebido || input->idade_a4_ms > params->timeout_a4_ms)
    adicionaMotivo(&corte, RM_TIMEOUT_A4);
  if (params->usar_flt_id && input->flt_id != params->flt_id_ok)
    adicionaMotivo(&corte, RM_FALHA_BRM);
  if (!dadosOk)
    adicionaMotivo(&corte, RM_DADO_INVALIDO);
  if (state->trava_tensao)
    adicionaMotivo(&corte, RM_TRAVA_SOBRETENSAO);

  if (corte != 0)
  {
    state->ultimo_torque = 0.0f;
    output->torque = 0.0f;
    output->alvo = 0.0f;
    output->idle = regen;
    output->estado = regen ? RegenEstado::Falha : RegenEstado::Pronto;
    output->motivos = m | corte;
    return;
  }

  // ve se o pioto pede regen ou o torque anterior ainda é negativo

  bool regenCtx = regen || (state->ultimo_torque < 0.0f);

  // se não estiver em regen, faz a rampa de torque positivo (tração) e retorna
  //  curva de torque positivo (protecao pra nao zerar instantaneamente e nao dar tranco)

  if (!regenCtx)
  {
    float t = acel;

    if (params->taxa_tracao_sobe > 0.0f && t > state->ultimo_torque)
    {
      float teto = state->ultimo_torque + params->taxa_tracao_sobe * dt;
      if (t > teto)
        t = teto;
    }
    if (params->taxa_tracao_desce > 0.0f && t < state->ultimo_torque)
    {
      float piso = state->ultimo_torque - params->taxa_tracao_desce * dt;
      if (t < piso)
        t = piso;
    }

    state->ultimo_torque = t;
    output->torque = t;
    output->alvo = acel;
    output->idle = false;
    output->estado = (t > 0.0f) ? RegenEstado::Tracao : RegenEstado::Pronto;
    output->motivos = m;
    return;
  }

  //  conta do maximo de torque negtivo to copiando do codigo do temotheo
  //  verificar contas

  uint16_t lim = 0;

  float rpmEf = input->rpm * params->rpm_sinal;
  float faixa = params->rpm_pleno - params->rpm_zero;
  if (faixa < 1.0f)
    faixa = 1.0f;

  float fRot = limites((rpmEf - params->rpm_zero) / faixa, 0.0f, 1.0f);
  float fTens = fatorDescendo(input->tensao, params->v_reduz, params->v_corte);
  float fTerm = fatorDescendo(input->term, params->term_ini, params->term_fim);

  if (fRot < 1.0f)
    adicionaMotivo(&lim, RM_ROTACAO_BAIXA);
  if (fTens < 1.0f)
    adicionaMotivo(&lim, RM_LIM_TENSAO);
  if (fTerm < 1.0f)
    adicionaMotivo(&lim, RM_LIM_TERMICO);

  float capBase = params->torque_min_cmd * fRot * fTens * fTerm;

  float capMotor = (input->tq_min_avl > 0.0f) ? 0.0f : input->tq_min_avl;
  if (capMotor > alvo)
    adicionaMotivo(&lim, RM_LIM_MOTOR);

  float omega = rpmEf * 0.10471976f;
  float capBat = (omega > 1.0f) ? -(params->i_carga_max * input->tensao / omega) : -1.0e9f;
  if (capBat > alvo)
    adicionaMotivo(&lim, RM_LIM_BATERIA);

  float cap = capBase;
  if (capMotor > cap)
    cap = capMotor;
  if (capBat > cap)
    cap = capBat;

  //  curva de torque negativo (protecao pra nao zerar instantaneamente e nao dar tranco)

  float r;
  if (regen)
  {
    if (alvo < state->ultimo_torque)
    {
      r = state->ultimo_torque - params->taxa_regen_sobe * dt;
      if (r < alvo)
        r = alvo;
    }
    else
    {
      r = state->ultimo_torque + params->taxa_regen_solta * dt;
      if (r > alvo)
        r = alvo;
    }
  }
  else
  {
    r = state->ultimo_torque + params->taxa_regen_solta * dt;
    if (r > 0.0f)
      r = 0.0f;
  }
  if (r < cap)
    r = cap;

  //  output

  state->ultimo_torque = r;
  output->torque = r;
  output->alvo = regen ? ((alvo > cap) ? alvo : cap) : 0.0f;
  output->idle = regen && (r == 0.0f);

  if (regen)
    m |= lim;

  if (r < 0.0f)
    output->estado = (lim != 0) ? RegenEstado::Limitado : RegenEstado::Regenerando;
  else
    output->estado = RegenEstado::Pronto;

  output->motivos = m;
}
