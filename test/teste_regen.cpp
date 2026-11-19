#include <math.h>
#include <stdio.h>

#include "regen.h"

int falhas = 0;

RegenIn entradaBase() {
  RegenIn in = {};
  in.em_drive = true;
  in.sistema_pronto = true;
  in.pedal_filtro_ok = true;
  in.pedal1 = 470;
  in.pedal2 = 470;
  in.rpm = 1500;
  in.tensao = 48;
  in.tq_min_avl = -20;
  in.term = 40;
  in.flt_id = 0;
  in.a3_recebido = true;
  in.a4_recebido = true;
  in.idade_a3_ms = 0;
  in.idade_a4_ms = 0;
  return in;
}

void verifica(bool ok, const char* descricao) {
  printf("[%s] %s\n", ok ? "OK" : "FALHOU", descricao);
  if (!ok) {
    falhas++;
  }
}

const char* textoEstado(RegenEstado estado) {
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
      return "Falha";
  }
  return "Desconhecido";
}

void imprimeDiagnosticoMotivos(uint16_t motivos) {
  const struct {
    uint16_t bit;
    const char* descricao;
  } diagnosticos[] = {
      {RM_NAO_PRONTO, "Sistema nao pronto"},
      {RM_TIMEOUT_A3, "Timeout da mensagem A3"},
      {RM_TIMEOUT_A4, "Timeout da mensagem A4"},
      {RM_FALHA_BRM, "Falha informada pelo BRM"},
      {RM_TRAVA_SOBRETENSAO, "Sobretensao: regeneracao bloqueada"},
      {RM_DADO_INVALIDO, "Dado invalido, incluindo tensao implausivel"},
      {RM_PEDAL_FALHA, "Falha ou divergencia nos pedais"},
      {RM_ROTACAO_BAIXA, "Rotacao abaixo da faixa de regeneracao"},
      {RM_LIM_TENSAO, "Regeneracao limitada pela tensao"},
      {RM_LIM_MOTOR, "Regeneracao limitada pelo motor"},
      {RM_LIM_BATERIA, "Regeneracao limitada pela bateria"},
      {RM_LIM_TERMICO, "Regeneracao limitada pela temperatura"},
  };
  bool encontrouMotivo = false;
  for (size_t i = 0; i < sizeof(diagnosticos) / sizeof(diagnosticos[0]); ++i) {
    if ((motivos & diagnosticos[i].bit) != 0) {
      printf("  - %s\n", diagnosticos[i].descricao);
      encontrouMotivo = true;
    }
  }
  if (!encontrouMotivo) {
    printf("  - Nenhum\n");
  }
}

void imprimeResultado(const char* cenario, uint32_t t, const RegenIn& in,
                      const RegenOut& out) {
  printf("Teste: %s (t=%u ms)\n", cenario, (unsigned)t);
  printf("Entrada: tensao=%.2f V, freio=%s, acelerador=%.1f Nm\n", in.tensao,
         in.freio ? "pressionado" : "solto", in.tq_acel);
  printf("--------------------------------\n");
  printf("Regenerativo (sombra):\n");
  printf("--------------------------------\n");
  printf("Torque calculado : %.2f Nm\n", out.torque);
  printf("Alvo             : %.2f Nm\n", out.alvo);
  printf("Estado           : %u (%s)\n", (unsigned)(uint8_t)out.estado,
         textoEstado(out.estado));
  printf("Motivos (hex)    : 0x%04X\n", (unsigned)out.motivos);
  printf("Diagnostico      :\n");
  imprimeDiagnosticoMotivos(out.motivos);
  printf("\n");
}

void testaAcelerandoDepoisFreia() {
  RegenParams params;
  RegenState state;
  RegenOut out = {};
  RegenIn in = entradaBase();
  regenParamsPadrao(&params);
  regenReset(&state);

  in.tq_acel = 15.0f;
  in.pedal1 = 655;
  in.pedal2 = 655;

  for (uint32_t t = 0; t <= 400; t += 10) {
    in.freio = (t >= 50);
    regenStep(&params, &state, &in, t, &out);
    if (t % 50 == 0 || t == 370) {
      imprimeResultado("acelerando->freia", t, in, out);
    }
    if (t == 50) {
      verifica(
          fabsf(out.torque - 14.5f) < 0.01f,
          "freio inicia em 50 ms; a rampa atual ainda deixa torque positivo");
    }
  }

  verifica(fabsf(out.torque - params.torque_freio) < 0.01f,
           "torque chega ao alvo de freio");
}

void testaAcelerandoDepoisSolta() {
  RegenParams params;
  RegenState state;
  RegenOut out = {};
  RegenIn in = entradaBase();
  regenParamsPadrao(&params);
  regenReset(&state);

  for (uint32_t t = 0; t <= 400; t += 10) {
    in.freio = false;
    if (t < 50) {
      in.tq_acel = 15.0f;
      in.pedal1 = 655;
      in.pedal2 = 655;
    } else {
      in.tq_acel = 0.0f;
      in.pedal1 = params.pedal_min;
      in.pedal2 = params.pedal_min;
    }

    regenStep(&params, &state, &in, t, &out);
    if (t % 50 == 0 || t == 350) {
      imprimeResultado("acelerando->solta", t, in, out);
    }
  }

  verifica(fabsf(out.torque - params.torque_lift) < 0.01f,
           "lift-off chega ao alvo de torque");
}

void testaFalha(const char* nome, uint16_t motivoEsperado, uint8_t caso) {
  RegenParams params;
  RegenState state;
  RegenOut out = {};
  RegenIn in = entradaBase();
  regenParamsPadrao(&params);
  regenReset(&state);
  in.freio = true;

  switch (caso) {
    case 0:
      in.idade_a3_ms = params.timeout_a3_ms + 1;
      break;
    case 1:
      in.idade_a4_ms = params.timeout_a4_ms + 1;
      break;
    case 2:
      in.flt_id = 1;
      break;
    case 3:
      in.tensao = params.v_corte;
      break;
    case 4:
      in.freio = false;
      in.pedal2 = params.pedal_min + params.pedal_tolerancia + 1;
      break;
  }

  regenStep(&params, &state, &in, 0, &out);
  imprimeResultado(nome, 0, in, out);

  verifica((out.motivos & motivoEsperado) != 0, nome);
  if (caso != 4) {
    verifica(fabsf(out.torque) < 0.01f, "falha de seguranca zera o torque");
  } else {
    verifica(fabsf(out.torque) < 0.01f,
             "pedais discordantes bloqueiam o lift-off");
  }
}

int main() {
  testaAcelerandoDepoisFreia();
  testaAcelerandoDepoisSolta();
  testaFalha("timeout A3", RM_TIMEOUT_A3, 0);
  testaFalha("timeout A4", RM_TIMEOUT_A4, 1);
  testaFalha("falha BRM", RM_FALHA_BRM, 2);
  testaFalha("sobretensao", RM_TRAVA_SOBRETENSAO, 3);
  testaFalha("pedais discordantes", RM_PEDAL_FALHA, 4);

  printf("\nResultado: %d falha(s) no teste.\n", falhas);
  return falhas == 0 ? 0 : 1;
}
