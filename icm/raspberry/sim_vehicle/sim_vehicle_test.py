#!/usr/bin/env python3
"""
Teste do veiculo simulado no Raspberry Pi (2-CH CAN HAT).

- can0 faz o papel do VEICULO: envia M1..M4 nos periodos do signal dictionary.
- can1 faz o papel do CLUSTER: recebe, decodifica e checa periodo, timeout,
  alive counter e faixa fisica (o mesmo que a Task_Diagnostics vai fazer no Blackpill).

Requisitos: python3-can (sudo apt install python3-can), can0/can1 UP a 500 kbps
e os dois canais do HAT ligados entre si (H com H, L com L, terminacao ligada).

Exemplos:
  python3 vehicle_sim_test.py                       # cenario normal, 20 s
  python3 vehicle_sim_test.py --scenario timeout    # M2 some por 5 s
  python3 vehicle_sim_test.py --scenario range      # rpm e temperatura fora da faixa
  python3 vehicle_sim_test.py --scenario frozen     # alive counter do M1 congela
"""

import argparse
import math
import struct
import threading
import time

import can

# ---------------------------------------------------------------------------
# Signal dictionary (mesmo da especificacao)
# ---------------------------------------------------------------------------
MESSAGES = {
    0x100: {"name": "M1_Engine",  "period": 0.5},
    0x200: {"name": "M2_Vehicle", "period": 1.0},
    0x300: {"name": "M3_Fluids",  "period": 1.0},
    0x400: {"name": "M4_Status",  "period": 1.0},
}
TIMEOUT_FACTOR = 3  # timeout = 3x o periodo

LIMITS = {
    "EngineSpeed":  (0, 8000),     # rpm
    "VehicleSpeed": (0, 250),      # km/h
    "FuelLevel":    (0, 100),      # %
    "CoolantTemp":  (-40, 150),    # graus C
}

WARNING_BITS = {0: "oleo", 1: "bateria", 2: "freio", 3: "check_engine"}


# ---------------------------------------------------------------------------
# Codificacao (veiculo -> bytes), little-endian (Intel)
# ---------------------------------------------------------------------------
def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def encode_m1(rpm, counter):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, clamp(round(rpm), 0, 0xFFFF))  # fator 1
    data[7] = counter & 0x0F                                        # bit 56, 4 bits
    return data


def encode_m2(speed_kmh, odometer_km, counter):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, clamp(round(speed_kmh / 0.01), 0, 0xFFFF))
    struct.pack_into("<I", data, 2, clamp(round(odometer_km / 0.1), 0, 0xFFFFFFFF))
    data[7] = counter & 0x0F
    return data


def encode_m3(fuel_pct, coolant_c):
    data = bytearray(8)
    data[0] = clamp(round(fuel_pct / 0.5), 0, 255)
    data[1] = clamp(round(coolant_c + 40), 0, 255)
    return data


def encode_m4(flags):
    data = bytearray(8)
    data[0] = flags & 0xFF
    return data


# ---------------------------------------------------------------------------
# Decodificacao (bytes -> valores fisicos), o que o cluster faz
# ---------------------------------------------------------------------------
def decode(arb_id, data):
    if arb_id == 0x100:
        return {"EngineSpeed": struct.unpack_from("<H", data, 0)[0],
                "Counter": data[7] & 0x0F}
    if arb_id == 0x200:
        return {"VehicleSpeed": struct.unpack_from("<H", data, 0)[0] * 0.01,
                "Odometer": struct.unpack_from("<I", data, 2)[0] * 0.1,
                "Counter": data[7] & 0x0F}
    if arb_id == 0x300:
        return {"FuelLevel": data[0] * 0.5,
                "CoolantTemp": data[1] - 40}
    if arb_id == 0x400:
        active = [name for bit, name in WARNING_BITS.items() if data[0] & (1 << bit)]
        return {"WarningFlags": ",".join(active) if active else "nenhum"}
    return {}


# ---------------------------------------------------------------------------
# Modelo simples do veiculo
# ---------------------------------------------------------------------------
class Vehicle:
    def __init__(self):
        self.rpm = 800.0
        self.speed = 0.0
        self.odometer = 12345.0
        self.fuel = 60.0
        self.coolant = 20.0
        self.flags = 0
        self.cnt_m1 = 0
        self.cnt_m2 = 0

    def update(self, t, dt):
        # aceleracao e desaceleracao suaves num ciclo de 20 s
        self.speed = 60 + 50 * math.sin(2 * math.pi * t / 20)
        self.rpm = 800 + self.speed * 35
        self.odometer += self.speed * dt / 3600
        self.fuel = max(0.0, self.fuel - 0.02 * dt)           # tanque esvaziando
        self.coolant = min(90.0, self.coolant + 2.0 * dt)     # motor aquecendo
        self.flags = 1 << 3 if self.coolant < 40 else 0       # check engine no frio


# ---------------------------------------------------------------------------
# Lado CLUSTER: recebe em can1 e verifica tudo
# ---------------------------------------------------------------------------
def monitor(bus, stop, t0):
    last_rx = {}
    last_cnt = {}
    in_timeout = set()

    while not stop.is_set():
        msg = bus.recv(timeout=0.05)
        now = time.monotonic()

        # checagem de timeout para cada mensagem conhecida
        for arb_id, info in MESSAGES.items():
            if arb_id in last_rx and arb_id not in in_timeout:
                if now - last_rx[arb_id] > TIMEOUT_FACTOR * info["period"]:
                    in_timeout.add(arb_id)
                    print(f"{now - t0:7.3f}  !!! TIMEOUT {info['name']} "
                          f"(sem mensagem ha {now - last_rx[arb_id]:.2f} s) -> valor seguro")

        if msg is None or msg.arbitration_id not in MESSAGES:
            continue

        arb_id = msg.arbitration_id
        info = MESSAGES[arb_id]
        values = decode(arb_id, msg.data)
        alerts = []

        if arb_id in in_timeout:
            in_timeout.discard(arb_id)
            alerts.append("RECUPERADO")

        dt_ms = (now - last_rx[arb_id]) * 1000 if arb_id in last_rx else 0.0
        last_rx[arb_id] = now

        if "Counter" in values:
            if arb_id in last_cnt and values["Counter"] == last_cnt[arb_id]:
                alerts.append("COUNTER CONGELADO")
            last_cnt[arb_id] = values["Counter"]

        for signal, (lo, hi) in LIMITS.items():
            if signal in values and not lo <= values[signal] <= hi:
                alerts.append(f"FORA DE FAIXA {signal}={values[signal]:.1f}")

        fields = "  ".join(
            f"{k}={v:.1f}" if isinstance(v, float) else f"{k}={v}"
            for k, v in values.items())
        alert_txt = f"  <-- {', '.join(alerts)}" if alerts else ""
        print(f"{now - t0:7.3f}  {info['name']:<11} dt={dt_ms:7.1f} ms  {fields}{alert_txt}")


# ---------------------------------------------------------------------------
# Lado VEICULO: envia em can0 conforme o cenario
# ---------------------------------------------------------------------------
def run(scenario, duration):
    tx = can.Bus(channel="can0", interface="socketcan")
    rx = can.Bus(channel="can1", interface="socketcan")
    stop = threading.Event()
    t0 = time.monotonic()

    mon = threading.Thread(target=monitor, args=(rx, stop, t0), daemon=True)
    mon.start()

    car = Vehicle()
    next_due = {arb_id: t0 for arb_id in MESSAGES}
    last_t = t0
    print(f"Cenario: {scenario}  |  duracao: {duration} s  |  can0 -> can1\n")

    try:
        while (now := time.monotonic()) - t0 < duration:
            t = now - t0
            car.update(t, now - last_t)
            last_t = now

            for arb_id in MESSAGES:
                if now < next_due[arb_id]:
                    continue
                next_due[arb_id] += MESSAGES[arb_id]["period"]

                # cenario: M2 some entre 5 s e 10 s
                if scenario == "timeout" and arb_id == 0x200 and 5 <= t < 10:
                    continue

                if arb_id == 0x100:
                    rpm = 9500 if scenario == "range" and 5 <= t < 8 else car.rpm
                    if not (scenario == "frozen" and t >= 5):
                        car.cnt_m1 += 1
                    data = encode_m1(rpm, car.cnt_m1)
                elif arb_id == 0x200:
                    car.cnt_m2 += 1
                    data = encode_m2(car.speed, car.odometer, car.cnt_m2)
                elif arb_id == 0x300:
                    coolant = 200 if scenario == "range" and 5 <= t < 8 else car.coolant
                    data = encode_m3(car.fuel, coolant)
                else:
                    data = encode_m4(car.flags)

                tx.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))

            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        time.sleep(0.2)  # deixa o monitor imprimir as ultimas mensagens
        stop.set()
        mon.join()
        tx.shutdown()
        rx.shutdown()
        print("\nFim do teste.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Veiculo simulado: can0 envia, can1 verifica.")
    parser.add_argument("--scenario", choices=["normal", "timeout", "range", "frozen"],
                        default="normal")
    parser.add_argument("--time", type=float, default=20.0, help="duracao em segundos")
    args = parser.parse_args()
    run(args.scenario, args.time)
