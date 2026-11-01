import asyncio, json, can
from aiohttp import web
import subprocess, os, signal


sim_proc = None
SIM_SCRIPT = "/home/rasp/sim_vehicle/sim_vehicle_test.py"


state = {"rpm":0,"speed":0,"fuel":0,"temp":0,"warn":0,"online":False}
clients = set()

def decode(msg):
    d = msg.data
    if msg.arbitration_id == 0x100 and len(d)>=2:
        state["rpm"] = d[0] | (d[1]<<8)
    elif msg.arbitration_id == 0x200 and len(d)>=2:
        state["speed"] = (d[0] | (d[1]<<8)) / 100.0
    elif msg.arbitration_id == 0x300 and len(d)>=2:
        state["fuel"] = d[0] * 0.5
        state["temp"] = d[1] - 40
    elif msg.arbitration_id == 0x400 and len(d)>=1:
        state["warn"] = d[0]
    state["online"] = True

async def can_reader():
    bus = can.interface.Bus(channel="can1", bustype="socketcan")
    loop = asyncio.get_event_loop()
    while True:
        msg = await loop.run_in_executor(None, bus.recv, 1.0)
        if msg is not None:
            decode(msg)
        else:
            state["online"] = False

async def ws_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    clients.add(ws)
    try:
        async for _ in ws:
            pass
    finally:
        clients.discard(ws)
    return ws

async def broadcaster():
    while True:
        if clients:
            data = json.dumps(state)
            for ws in list(clients):
                try: await ws.send_str(data)
                except: clients.discard(ws)
        await asyncio.sleep(0.1)   # 10 Hz

async def index(request):
    return web.FileResponse("/home/rasp/sim_vehicle/web/panel.html")

async def start_bg(app):
    app["can"] = asyncio.create_task(can_reader())
    app["bc"]  = asyncio.create_task(broadcaster())

def stop_sim():
    global sim_proc
    if sim_proc and sim_proc.poll() is None:
        sim_proc.terminate()
        try: sim_proc.wait(timeout=2)
        except: sim_proc.kill()
    sim_proc = None

async def scenario_handler(request):
    global sim_proc
    sc = request.match_info["name"]
    stop_sim()
    if sc != "stop":
        # roda por tempo longo (600s) pra nao parar sozinho na demo
        sim_proc = subprocess.Popen(
            ["python3", SIM_SCRIPT, "--scenario", sc, "--time", "600"])
    return web.json_response({"scenario": sc})


app = web.Application()
app.router.add_get("/", index)
app.router.add_get("/ws", ws_handler)
app.router.add_get("/scenario/{name}", scenario_handler)
app.on_startup.append(start_bg)

if __name__ == "__main__":
    web.run_app(app, host="0.0.0.0", port=8080)
