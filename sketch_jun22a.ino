
#include "modbus.h"
/* 
 * configuração dos parametros da porta serial.
 *
 * baud: taxa de transmissão em bps (valores típicos entre 9600, 19200... 115200)
 * parity: seta o modo de paridade: 
 *         'n' sem paridade (8N1); 'e' paridede impar (8E1), 'o' paridade par (8O1).
 * tx_en_pin: pino do arduino que controla a transmissão/recepção em uma linha RS485.
 *        0 or 1 desliga esta função (para rede RS232)
 *        >2 para uma rede multiponto.
 */
void configure_mb_slave(long baud, char parity, char txenpin);

/*
 * verifica se há qualquer pedido válido do mestre modbus. Se houver, executa a ação solicitada
 *
 * slave_id: endereço do escravo (arduino) (1 to 127)
 * regs: uma matriz com os holding registers. Eles começam no endereço 1 (mestre ponto de vista)
 * Regs_size: número total de holding registers.
 * Retorna: 0 se não houver pedido do mestre,
 * NO_REPLY (-1) se nenhuma resposta é enviada para o mestre
 * Caso um código de exceção (1 a 4) em algumas exceções de modbus
 * O número de bytes enviados como resposta (> 4) se OK.
 */
int update_mb_slave(unsigned char slave, int *regs, unsigned int regs_size);

/* Parâmetros Modbus RTU de comunicação, o Mestre e os escravos devem usar os mesmos parâmetros */
enum {
        COMM_BPS = 9600,  /* baud rate */
        MB_SLAVE = 1,     /* endereço do escravo modbus */
        PARITY = 'n'      /* paridade */
};

/* registros do escravo (holding registers)
* Aqui ficam ordenados todos os registros de leitura e escrita da comunicação entre o mestre e o escravo (SCADA e arduino) */

enum {
        modoOperacao,     /* Offset = 0, Controle do Led no pino  13 (desliga=0 liga=1). Esse registrador é utilizado como chave seletora, que alterna o modo Automatico e Manual */
        bomba1Regs,       /* Offset = 1, Registrador da Bomba 1 */
        bomba2Regs,       /* Offset = 2, Registrador da Bomba 2 */
        bomba3Regs,       /* Offset = 3, Registrador da Bomba 3 */
        sensor1Regs,      /* Offset = 4, Registrador do sensor 1 */
        sensor2Regs,      /* Offset = 5, Registrador do sensor 2 */
        sensor3Regs,      /* Offset = 6, Registrador do sensor 3 */
        MB_REGS           /* número total de registros do escravo */
};

int regs[MB_REGS];
int ledPin13  =  13;
int bomba1 = 5;
int bomba2 = 6;
int bomba3 = 7;
int sensor1 = 8;
int sensor2 = 9;
int sensor3 = 10;
int sensorInit = sensor1; /* sensorInit é inicializada com sensor1 como condição de Start do Sistema. O reservatório 1 desse começar cheio. */
unsigned long millisAnterior = 0;

int ligarBomba(int bomba) {
  digitalWrite(bomba, LOW);
  return 1;
}

int desligarBomba(int bomba) {
  digitalWrite(bomba, HIGH);
  return 0;
}

int verificacaoDeNivel(int sensor) {
  int media = 0;
  while (digitalRead(sensor)) {
    media++;
    if (media <= 3) {
      return 1; /* nivel alto (Reservatorio cheio) */
    }
    update_mb_slave(MB_SLAVE, regs, MB_REGS);
  }
  return 0; /* nivel baixo (Reservatorio baixo) */
}

int tempoDeExecucao(unsigned long tempo) {
  /* dois minutos para encher um reservatório */
  unsigned long mili = millis();
  if (mili >= tempo) {
    if ((mili - tempo) < 120000) {
      return 1;
    }
    return 0;
  }
  else {
    if ((tempo - mili) < 120000) {
      return 1;
    }
    return 0;
  }
}


void setup() {

        configure_mb_slave(COMM_BPS, PARITY, 2); /* configura comunicação modbus 9600 bps, 8N1, RS485 network */
        pinMode(ledPin13,  OUTPUT);
        pinMode(bomba1, OUTPUT);
        pinMode(bomba2, OUTPUT);
        pinMode(bomba3, OUTPUT);
        pinMode(sensor1, INPUT);
        pinMode(sensor2, INPUT);
        pinMode(sensor3, INPUT);
}

void automatizar(int sensorRegs, int sensor, int bombaRegs, int bomba) {
    digitalWrite(ledPin13, regs[modoOperacao]);
    regs[bomba1Regs] = desligarBomba(bomba1);
    regs[bomba2Regs] = desligarBomba(bomba2);
    regs[bomba3Regs] = desligarBomba(bomba3);
    regs[sensorRegs] = verificacaoDeNivel(sensor);
    update_mb_slave(MB_SLAVE, regs, MB_REGS);
    millisAnterior = millis();
    while ((sensorInit == sensor) and regs[modoOperacao] and !regs[sensorRegs] and tempoDeExecucao(millisAnterior)) {
         regs[bombaRegs] = ligarBomba(bomba);
         regs[sensorRegs] = verificacaoDeNivel(sensor);
         update_mb_slave(MB_SLAVE, regs, MB_REGS);
    }
    sensorInit++;
}

void manual(int sensorRegs, int sensor, int bombaRegs, int bomba) {
    digitalWrite(ledPin13, regs[modoOperacao]);
    regs[bomba1Regs] = desligarBomba(bomba1);
    regs[bomba2Regs] = desligarBomba(bomba2);
    regs[bomba3Regs] = desligarBomba(bomba3);
    regs[sensorRegs] = verificacaoDeNivel(sensor);
    update_mb_slave(MB_SLAVE, regs, MB_REGS);
    millisAnterior = millis();
    while (regs[bombaRegs] and !regs[sensorRegs] and !regs[modoOperacao] and tempoDeExecucao(millisAnterior)) {
         regs[bombaRegs] = ligarBomba(bomba);
         regs[sensorRegs] = verificacaoDeNivel(sensor);
         update_mb_slave(MB_SLAVE, regs, MB_REGS);
    }
}

void loop() {
      
      switch (regs[modoOperacao]) { 
          case 1: { // Modo Automatico
            automatizar(sensor1Regs, sensor1, bomba3Regs, bomba3);
            automatizar(sensor2Regs, sensor2, bomba1Regs, bomba1);
            automatizar(sensor3Regs, sensor3, bomba2Regs, bomba2);            
            sensorInit = sensor1;
            break;
          }
          default: {  // Modo Manual
            manual(sensor2Regs, sensor2, bomba1Regs, bomba1);
            manual(sensor3Regs, sensor3, bomba2Regs, bomba2);
            manual(sensor1Regs, sensor1, bomba3Regs, bomba3);
            break;
          }
      }
}

/*
void loop() {

      regs[bomba1Regs] = desligarBomba(bomba1);
      regs[bomba2Regs] = desligarBomba(bomba2);
      regs[bomba3Regs] = desligarBomba(bomba3);
      update_mb_slave(MB_SLAVE, regs, MB_REGS);
      
      switch (regs[modoOperacao]) { // Modo Automatico
          case 1: {
            digitalWrite(ledPin13, regs[modoOperacao]); /// Liga o Led 13, informando o modo automatico
            regs[sensor1Regs] = verificacaoDeNivel(sensor1);
            millisAnterior = millis();
            while ((sensorInit == sensor1) and regs[modoOperacao] and !regs[sensor1Regs] and tempoDeExecucao(millisAnterior)) {
              regs[bomba3Regs] = ligarBomba(bomba3);
              regs[bomba1Regs] = desligarBomba(bomba1);
              regs[bomba2Regs] = desligarBomba(bomba2);
              regs[sensor1Regs] = verificacaoDeNivel(sensor1);
              update_mb_slave(MB_SLAVE, regs, MB_REGS);
            }
            sensorInit++;
            regs[sensor2Regs] = verificacaoDeNivel(sensor2);
            millisAnterior = millis();
            while ((sensorInit == sensor2) and regs[modoOperacao] and !regs[sensor2Regs] and tempoDeExecucao(millisAnterior)) {
              regs[bomba1Regs] = ligarBomba(bomba1);
              regs[bomba2Regs] = desligarBomba(bomba2);
              regs[bomba3Regs] = desligarBomba(bomba3);
              regs[sensor2Regs] = verificacaoDeNivel(sensor2);
              update_mb_slave(MB_SLAVE, regs, MB_REGS);
            }
            sensorInit++;
            regs[sensor3Regs] = verificacaoDeNivel(sensor3);
            millisAnterior = millis();
            while ((sensorInit == sensor3) and regs[modoOperacao] and !regs[sensor3Regs] and tempoDeExecucao(millisAnterior)) {
              regs[bomba2Regs] = ligarBomba(bomba2);
              regs[bomba1Regs] = desligarBomba(bomba1);
              regs[bomba3Regs] = desligarBomba(bomba3);
              regs[sensor3Regs] = verificacaoDeNivel(sensor3);
              update_mb_slave(MB_SLAVE, regs, MB_REGS);
            }
            sensorInit = sensor1;
            break;
          }
          default: {  // Modo Manual
            digitalWrite(ledPin13, regs[modoOperacao]); // Desliga o Led 13, informando o modo manual
            regs[sensor2Regs] = verificacaoDeNivel(sensor2);
            millisAnterior = millis();
            while (regs[bomba1Regs] and !regs[sensor2Regs] and !regs[modoOperacao] and tempoDeExecucao(millisAnterior)) {
              regs[bomba1Regs] = ligarBomba(bomba1);
              regs[bomba2Regs] = desligarBomba(bomba2);
              regs[bomba3Regs] = desligarBomba(bomba3);
              regs[sensor2Regs] = verificacaoDeNivel(sensor2);
              update_mb_slave(MB_SLAVE, regs, MB_REGS);
            }
            regs[sensor3Regs] = verificacaoDeNivel(sensor3);
            millisAnterior = millis();
            while (regs[bomba2Regs] and !regs[sensor3Regs] and !regs[modoOperacao] and tempoDeExecucao(millisAnterior)) {
              regs[bomba1Regs] = desligarBomba(bomba1);
              regs[bomba2Regs] = ligarBomba(bomba2);
              regs[bomba3Regs] = desligarBomba(bomba3);
              regs[sensor3Regs] = verificacaoDeNivel(sensor3);
              update_mb_slave(MB_SLAVE, regs, MB_REGS);
            }
            regs[sensor1Regs] = verificacaoDeNivel(sensor1);
            millisAnterior = millis();
            while (regs[bomba3Regs] and !regs[sensor1Regs] and !regs[modoOperacao] and tempoDeExecucao(millisAnterior)) {
              regs[bomba1Regs] = desligarBomba(bomba1);
              regs[bomba2Regs] = desligarBomba(bomba2);
              regs[bomba3Regs] = ligarBomba(bomba3);
              regs[sensor1Regs] = verificacaoDeNivel(sensor1);
              update_mb_slave(MB_SLAVE, regs, MB_REGS);
            }
            break;
          }
      }
}
*/

