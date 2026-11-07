# ICM — Instrument Cluster Module com Trampoline RTOS

Cluster de instrumentos automotivo rodando **Trampoline RTOS (OSEK/VDX)** num **STM32F411 Blackpill**, recebendo os dados do veículo por **CAN (500 kbit/s)** de um Raspberry Pi que simula o veículo.

Projeto 2 da especialização em Sistemas Embarcados Automotivos — UTFPR, 2026.

## Estrutura

```
icm/
  app/        Aplicação final: icm.c + icm.oil
  bringup/    Programas de bring-up usados em cada etapa (LEDs, ADC, OLED, CAN, esqueleto RT, resource, ICM v1)
  raspberry/  Veículo simulado (sim_vehicle/), painel web e capturas do analisador lógico (capturas/)
  test/       Testes no host com Google Test
  docs/       Requisitos, signal dictionary, design e relatório
```

O port do Trampoline para o STM32F411 fica fora desta pasta, onde o Trampoline espera:

- `machines/cortex-m/armv7em/stm32f411/` (placa em `blackpill/`)
- `goil/templates/config|code|linker/.../stm32f411/`

## Hardware

| Item | Uso |
|---|---|
| STM32F411CEU6 WeAct Blackpill (84 MHz) | ECU do cluster |
| MCP2515 + TJA1050 (5 V) via conversor de nível HW-209 | Controlador CAN em SPI1, INT no PB1 (EXTI1) |
| Display OLED SH1106 128×64 I²C (0x3C) | Tela do cluster em I2C1 (PB6/PB7) |
| ADKeyboard (5 botões, saída analógica) | Setas, pisca-alerta, troca de tela, injeção de falha (PA1) |
| LEDs e buzzer | Setas, warning, falha CAN, tic-tac |
| Raspberry Pi Zero 2 W + Waveshare 2-CH CAN HAT | Veículo simulado (can0) e logger (can1) |
| ST-Link V2 | Gravação e debug via SWD |

## Como compilar e gravar

Pré-requisitos: Arm GNU Toolchain (`arm-none-eabi-gcc`), `goil` no PATH, `stlink` (`st-flash`), Python 3.

```sh
cd icm/app
goil --target=cortex-m/armv7em/stm32f411/blackpill --templates=../../goil/templates/ icm.oil
python make.py
st-flash write icm_exe.bin 0x08000000
```

Os programas de `bringup/` compilam da mesma forma, trocando `icm.oil` pelo `.oil` desejado.

## Arquitetura de tempo real (icm.oil)

| Objeto | Tipo / prioridade | Ativação | Função |
|---|---|---|---|
| `ISR_CAN` | ISR categoria 2 | Borda de descida no INT do MCP2515 (EXTI1) | Ativa `Task_CAN_RX` |
| `Task_CAN_RX` | Task, prioridade 6 | Pela ISR | Lê o frame via SPI e decodifica em `ICM_Data` |
| `Acquisition` | Task, prioridade 5 | Alarm de 10 ms | Teclado (ADC), setas e pisca-alerta |
| `Display` | Task, prioridade 3 | Alarm de 50 ms | Tela, LEDs e buzzer |
| `Diagnostics` | Task, prioridade 2 | Alarm de 100 ms | Timeout por mensagem (3× o período) |
| `heartbeat` | Task, prioridade 1 | Alarm de 250 ms | LED da placa |
| `ICM_Resource` | Resource | — | Protege `ICM_Data` (priority ceiling) |

## Mensagens CAN

500 kbit/s, IDs de 11 bits, little-endian. Timeout = 3× o período.

| Mensagem | ID | Período | Sinais |
|---|---|---|---|
| M1_Engine | 0x100 | 500 ms | EngineSpeed (0–8000 rpm), M1_Counter |
| M2_Vehicle | 0x200 | 1000 ms | VehicleSpeed (0–250 km/h, fator 0,01), Odometer, M2_Counter |
| M3_Fluids | 0x300 | 1000 ms | FuelLevel (0–100 %, fator 0,5), CoolantTemp (−40–150 °C) |
| M4_Status | 0x400 | 1000 ms | WarningFlags (óleo, bateria, freio, check engine) |

## Veículo simulado (Raspberry Pi)

```sh
cd icm/raspberry/sim_vehicle
python3 sim_vehicle_test.py      # cenários: normal, timeout, range, frozen
python3 web/web_panel.py         # painel web
```

`can0` transmite como veículo e `can1` registra o barramento.

## Testes

Os testes de lógica rodam no host com Google Test (em `test/`, em construção). As evidências de timing vêm das capturas do analisador lógico em `raspberry/capturas/`.

## Como contribuir

1. `git switch main && git pull`
2. `git switch -c feat/<assunto>`
3. Faça a mudança, compile e teste (na placa ou com o Google Test).
4. `git push -u <remote> feat/<assunto>` e abra um Pull Request para a `main`.
5. Outro integrante revisa e aprova antes do merge.

Mensagens de commit citam o requisito, por exemplo: `ICM-CAN-05: timeout por mensagem`.