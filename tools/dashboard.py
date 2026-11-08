"""Live bench dashboard (for the demo video and manual testing).

  python tools/dashboard.py [--port 8765]
  then open http://127.0.0.1:8765

Owns both USB consoles while it runs (close idf.py monitor / bench.py first). It switches
the controller to 'stream on' (a TLM line every 100 ms), shows tanks, actuator command vs
actual state, a 60 s chart of levels and flow, link counters and the event log of both
boards, and has buttons for the operator and fault-injection commands. Every line from
both boards is also written to logs/<UTC>_dashboard.log.

Standard library + pyserial only. The page shows simulated values: the liquid, pump,
valve and sensors are modelled on the ESP32-C3.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import queue
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
from bench import LOGDIR, find_ports  # noqa: E402

KV = re.compile(r"(\w+)=([^\s|]+)")


class Hub:
    def __init__(self) -> None:
        self.ports = find_ports()
        self.ser: dict[str, serial.Serial] = {}
        self.clients: list[queue.Queue] = []
        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        LOGDIR.mkdir(exist_ok=True)
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.log = open(LOGDIR / f"{stamp}_dashboard.log", "w", encoding="utf-8")
        for name, dev in self.ports.items():
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = dev, 115200, 0.05
            s.dtr = s.rts = False
            s.open()
            self.ser[name] = s
            threading.Thread(target=self.reader, args=(name,), daemon=True).start()
        self.send("ctl", "stream on")
        threading.Thread(target=self.poll_status, daemon=True).start()

    def publish(self, msg: dict) -> None:
        data = json.dumps(msg)
        with self.lock:
            for q in list(self.clients):
                try:
                    q.put_nowait(data)
                except queue.Full:
                    pass

    def send(self, board: str, text: str) -> None:
        self.log_line("bench", f"--> {board}: {text}")
        self.ser[board].write((text + "\n").encode())

    def log_line(self, src: str, text: str) -> None:
        self.log.write(f"[{time.monotonic() - self.t0:9.3f}] {src:<5} {text}\n")
        self.log.flush()

    def poll_status(self) -> None:
        """Ask both boards for STAT now and then, for counters and peer info."""
        while True:
            time.sleep(2.0)
            for b in ("ctl", "plant"):
                try:
                    self.ser[b].write(b"status\n")
                except serial.SerialException:
                    pass

    def reader(self, board: str) -> None:
        buf = b""
        while True:
            try:
                buf += self.ser[board].read(512)
            except serial.SerialException:
                time.sleep(0.3)
                try:
                    self.ser[board].close()
                    self.ser[board].open()
                    if board == "ctl":
                        self.send("ctl", "stream on")
                except serial.SerialException:
                    pass
                continue
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").rstrip("\r")
                self.log_line(board, line)
                self.handle(board, line)

    def handle(self, board: str, line: str) -> None:
        if line.startswith("TLM "):
            d = dict(KV.findall(line))
            self.publish({"type": "tlm", **d})
        elif line.startswith("STAT"):
            self.publish({"type": "stat", "board": board, "text": line, **dict(KV.findall(line))})
        elif line.startswith(("EVT", "ERR", "VERSION", "FAULTLOG", "BLACKBOX", "BB ")):
            self.publish({"type": "evt", "board": board, "text": line})
        if "BOOT controller" in line:
            self.send("ctl", "stream on")


HUB: Hub | None = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args) -> None:  # keep the console quiet
        pass

    def do_GET(self) -> None:
        if self.path == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q: queue.Queue = queue.Queue(maxsize=2000)
            with HUB.lock:
                HUB.clients.append(q)
            try:
                while True:
                    try:
                        data = q.get(timeout=5)
                        self.wfile.write(f"data: {data}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            finally:
                with HUB.lock:
                    HUB.clients.remove(q)
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        if self.path != "/cmd":
            self.send_error(404)
            return
        n = int(self.headers.get("Content-Length", "0"))
        try:
            req = json.loads(self.rfile.read(n))
            board, text = req["board"], str(req["text"]).strip()
            if board not in ("ctl", "plant") or not text or len(text) > 60 or "\n" in text:
                raise ValueError("bad command")
            HUB.send(board, text)
            self.send_response(204)
            self.end_headers()
        except (ValueError, KeyError, json.JSONDecodeError):
            self.send_error(400)


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Transfer bench</title>
<style>
:root{--bg:#0f1419;--panel:#171e26;--line:#2a3542;--text:#dce3ea;--dim:#8a97a6;
--src:#4aa3ff;--dst:#3ccf91;--flow:#f2b84b;--bad:#ff5d5d;--warn:#f2b84b;--ok:#3ccf91;--blue:#4aa3ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);
font:14px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{display:flex;gap:16px;align-items:center;padding:12px 18px;border-bottom:1px solid var(--line);flex-wrap:wrap}
h1{font-size:16px;margin:0;font-weight:600}.sim{color:var(--dim);font-size:12px}
.badge{padding:4px 10px;border-radius:6px;font-weight:700;letter-spacing:.5px}
main{display:grid;grid-template-columns:minmax(300px,380px) 1fr;gap:14px;padding:14px}
@media(max-width:900px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px}
.card h2{font-size:12px;text-transform:uppercase;letter-spacing:1px;color:var(--dim);margin:0 0 8px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 12px;font-variant-numeric:tabular-nums}
.kv span:nth-child(odd){color:var(--dim)}
#log{height:260px;overflow:auto;font:12px/1.35 ui-monospace,Consolas,monospace;white-space:pre-wrap}
#log .ctl{color:#9cc9ff}#log .plant{color:#a5e8c8}#log .fault{color:var(--bad);font-weight:600}
.btns{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:8px}
button{background:#22303d;color:var(--text);border:1px solid #33475a;border-radius:6px;padding:6px 10px;cursor:pointer;font:inherit}
button:hover{background:#2b3d4e}button.danger{border-color:#7a3434}button.go{border-color:#2f7a55}
input{background:#0f1419;color:var(--text);border:1px solid var(--line);border-radius:6px;padding:6px;width:100%;font:inherit}
canvas{width:100%;height:260px;display:block}
.legend{display:flex;gap:14px;color:var(--dim);font-size:12px;margin-top:4px}
.legend i{display:inline-block;width:12px;height:3px;margin-right:5px;vertical-align:middle}
#detail{color:var(--bad);min-height:1.4em}
</style></head><body>
<header>
 <h1>Two-board transfer bench</h1>
 <span id="state" class="badge">-</span><span id="phase" class="sim"></span>
 <span id="link" class="badge">link ?</span>
 <span class="sim">Controller: real ESP32-S3 &middot; Plant: real ESP32-C3 running a <b>simulated</b> tank/pump/valve model</span>
</header>
<main>
 <div>
  <div class="card"><h2>Plant (simulated)</h2>
   <svg id="tanks" viewBox="0 0 340 214" width="100%">
    <text x="45" y="16" fill="#8a97a6" font-size="12" text-anchor="middle">SOURCE</text>
    <text x="295" y="16" fill="#8a97a6" font-size="12" text-anchor="middle">DESTINATION</text>
    <rect x="10" y="22" width="70" height="150" rx="4" fill="none" stroke="#3a4a5a" stroke-width="2"/>
    <rect x="260" y="22" width="70" height="150" rx="4" fill="none" stroke="#3a4a5a" stroke-width="2"/>
    <rect id="srcfill" x="12" y="170" width="66" height="0" fill="var(--src)" opacity=".8"/>
    <rect id="dstfill" x="262" y="170" width="66" height="0" fill="var(--dst)" opacity=".8"/>
    <line x1="260" y1="30.5" x2="330" y2="30.5" stroke="var(--bad)" stroke-dasharray="4 3"/>
    <text x="336" y="34" fill="var(--bad)" font-size="9" text-anchor="end">950</text>
    <path id="pipe" d="M80 160 H260" stroke="#3a4a5a" stroke-width="8"/>
    <path id="flowline" d="M80 160 H260" stroke="var(--flow)" stroke-width="3" stroke-dasharray="6 6" opacity="0"/>
    <circle id="pump" cx="130" cy="160" r="15" fill="#22303d" stroke="#5a6b7c" stroke-width="2"/>
    <text x="130" y="165" fill="#dce3ea" font-size="13" text-anchor="middle">P</text>
    <rect id="valve" x="195" y="146" width="28" height="28" rx="3" fill="#22303d" stroke="#5a6b7c" stroke-width="2"/>
    <text x="209" y="165" fill="#dce3ea" font-size="13" text-anchor="middle">V</text>
    <text id="srctxt" x="45" y="192" fill="#dce3ea" font-size="14" text-anchor="middle">-</text>
    <text id="dsttxt" x="295" y="192" fill="#dce3ea" font-size="14" text-anchor="middle">-</text>
    <text id="pumptxt" x="170" y="192" fill="#8a97a6" font-size="11" text-anchor="middle">pump cmd - / act -</text>
    <text id="valvetxt" x="170" y="206" fill="#8a97a6" font-size="11" text-anchor="middle">valve cmd - / act -</text>
    <text id="flowtxt" x="170" y="140" fill="var(--flow)" font-size="12" text-anchor="middle"></text>
   </svg>
  </div>
  <div class="card" style="margin-top:14px"><h2>Status</h2>
   <div id="detail"></div>
   <div class="kv" id="kv"></div>
  </div>
 </div>
 <div>
  <div class="card"><h2>Levels and flow, last 60 s</h2>
   <canvas id="chart" width="900" height="260"></canvas>
   <div class="legend"><span><i style="background:var(--src)"></i>source mL</span>
   <span><i style="background:var(--dst)"></i>destination mL</span>
   <span><i style="background:var(--flow)"></i>flow mL/s (x10)</span>
   <span>band: controller state</span></div>
  </div>
  <div class="card" style="margin-top:14px"><h2>Operator (controller)</h2>
   <div class="btns">
    <button class="go" data-b="ctl" data-c="start 500">start 500</button>
    <button class="go" data-b="ctl" data-c="start 900">start 900</button>
    <button class="danger" data-b="ctl" data-c="stop">stop</button>
    <button data-b="ctl" data-c="reset">reset</button>
    <button data-b="ctl" data-c="faults">fault log</button>
    <button data-b="ctl" data-c="blackbox">black box</button>
    <button data-b="ctl" data-c="version">version</button>
   </div>
   <h2>Fault injection (plant simulation)</h2>
   <div class="btns">
    <button data-b="plant" data-c="set src 800">src 800</button>
    <button data-b="plant" data-c="set dst 100">dst 100</button>
    <button data-b="plant" data-c="fault valve_stuck 1">valve stuck</button>
    <button data-b="plant" data-c="fault pump_dead 1">pump dead</button>
    <button data-b="plant" data-c="fault leak 150">leak 150</button>
    <button data-b="plant" data-c="fault dst_frozen 1">dst sensor frozen</button>
    <button data-b="plant" data-c="fault dst_inflow 100">dst inflow 100</button>
    <button data-b="plant" data-c="noise 3000">plant noise 0.3%</button>
    <button data-b="ctl" data-c="noise 3000">ctl noise 0.3%</button>
    <button data-b="plant" data-c="mute 2000">plant mute 2 s</button>
    <button data-b="plant" data-c="proto 9">plant proto v9</button>
    <button class="danger" data-b="plant" data-c="hang">plant hang (WDT)</button>
    <button class="go" data-b="plant" data-c="fault clear">clear faults</button>
    <button class="go" data-b="plant" data-c="noise 0">plant noise off</button>
    <button class="go" data-b="ctl" data-c="noise 0">ctl noise off</button>
    <button class="go" data-b="plant" data-c="proto 2">plant proto v2</button>
   </div>
   <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px">
    <input id="cmdctl" placeholder="controller command, Enter">
    <input id="cmdplant" placeholder="plant command, Enter">
   </div>
  </div>
  <div class="card" style="margin-top:14px"><h2>Events (both boards)</h2><div id="log"></div></div>
 </div>
</main>
<script>
const STATE_COLORS={IDLE:'#5a6b7c',CHECK:'#f2b84b',TRANSFER:'#4aa3ff',COMPLETE:'#3ccf91',FAULT:'#ff5d5d'};
const hist=[];const kv={};let last=null;
function send(board,text){fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({board,text})});}
document.querySelectorAll('button[data-c]').forEach(b=>b.onclick=()=>send(b.dataset.b,b.dataset.c));
for(const [id,board] of [['cmdctl','ctl'],['cmdplant','plant']]){const el=document.getElementById(id);
 el.addEventListener('keydown',e=>{if(e.key==='Enter'&&el.value.trim()){send(board,el.value.trim());el.value='';}});}
function logLine(board,text){const d=document.getElementById('log');const p=document.createElement('div');
 p.className=board+(/FAULT cause|FAILSAFE|INCOMPATIBLE|TASK_WDT/.test(text)?' fault':'');
 p.textContent=(board==='ctl'?'S3  ':'C3  ')+text;d.appendChild(p);while(d.childNodes.length>400)d.removeChild(d.firstChild);
 d.scrollTop=d.scrollHeight;}
function setKV(){const el=document.getElementById('kv');el.innerHTML='';
 for(const [k,v] of Object.entries(kv)){const a=document.createElement('span');a.textContent=k;const b=document.createElement('span');b.textContent=v;el.append(a,b);}}
function onTlm(m){last=m;const now=Date.now()/1000;
 hist.push({t:now,src:+m.src,dst:+m.dst,flow:+m.flow,st:m.st});while(hist.length&&hist[0].t<now-60)hist.shift();
 const st=document.getElementById('state');st.textContent=m.st;st.style.background=STATE_COLORS[m.st]||'#444';
 document.getElementById('phase').textContent=m.ph!=='-'?m.ph:'';
 const ln=document.getElementById('link');const ok=+m.age<=300;ln.textContent=ok?'link OK':'link DOWN';
 ln.style.background=ok?'#1f5c40':'#7a2a2a';
 const H=146;const sh=Math.min(H,H*+m.src/1000),dh=Math.min(H,H*+m.dst/1000);
 const sf=document.getElementById('srcfill');sf.setAttribute('y',170-sh);sf.setAttribute('height',sh);
 const df=document.getElementById('dstfill');df.setAttribute('y',170-dh);df.setAttribute('height',dh);
 document.getElementById('srctxt').textContent=m.src+' mL';document.getElementById('dsttxt').textContent=m.dst+' mL';
 document.getElementById('pump').setAttribute('fill',+m.pa?'#2f7a55':'#22303d');
 document.getElementById('valve').setAttribute('fill',+m.va?'#2f7a55':'#5a2a2a');
 document.getElementById('pumptxt').textContent='pump  cmd '+(+m.pc?'ON':'off')+' / act '+(+m.pa?'ON':'off');
 document.getElementById('valvetxt').textContent='valve  cmd '+(+m.vc?'OPEN':'shut')+' / act '+(+m.va?'OPEN':'shut');
 const fl=document.getElementById('flowline');fl.setAttribute('opacity',+m.flow>0?1:0);
 document.getElementById('flowtxt').textContent=+m.flow>0?m.flow+' mL/s':'';
 kv['target mL']=m.tgt;kv['link age ms']=m.age;kv['cmd ack']=+m.ack?'OK':'STALE';kv['cmd RTT ms']=m.rtt;
 kv['plant flags']='0x'+(+m.fl).toString(16).padStart(2,'0')+' '+flagNames(+m.fl);
 if(m.st!=='FAULT')document.getElementById('detail').textContent='';else if(!document.getElementById('detail').textContent)document.getElementById('detail').textContent='FAULT '+m.fault;
 setKV();}
function flagNames(f){const n=[];if(f&1)n.push('FAILSAFE');if(f&2)n.push('FAULT_INJ');if(f&4)n.push('NOT_ARMED');if(f&8)n.push('RESTARTED');if(f&16)n.push('PEER_BAD');return n.join(' ');}
function onStat(m){if(m.board==='ctl'&&m.text.startsWith('STAT t=')){kv['ctl rx ok/crc/len']=m.rx_ok+' / '+m.crc_err+' / '+m.len_err;kv['ctl seq gaps']=m.seq_gaps;}
 if(m.board==='ctl'&&m.text.startsWith('STAT fault_detail')){document.getElementById('detail').textContent='FAULT: '+m.text.split(': ').slice(1).join(': ');}
 if(m.board==='ctl'&&m.text.startsWith('STAT diag')){kv['controller boot #']=m.boot;kv['ctl TX corrupted']=m.tx_corrupted;}
 if(m.board==='ctl'&&m.text.startsWith('STAT peer')){kv['peer (plant)']=m.text.split(' ')[2]+' proto '+(m.proto||'?');kv['firmware']=m.fw||'?';}
 if(m.board==='plant'&&m.text.startsWith('STAT t=')){kv['plant rx ok/crc']=m.rx_ok+' / '+m.crc_err;kv['plant armed']=m.armed;kv['plant TX corrupted']=m.tx_corrupted;}
 setKV();}
function draw(){const c=document.getElementById('chart');const r=c.getBoundingClientRect();c.width=r.width*devicePixelRatio;c.height=r.height*devicePixelRatio;
 const g=c.getContext('2d');g.scale(devicePixelRatio,devicePixelRatio);const W=r.width,H=r.height-18,now=Date.now()/1000;
 g.fillStyle='#171e26';g.fillRect(0,0,W,r.height);g.strokeStyle='#2a3542';g.lineWidth=1;
 g.fillStyle='#8a97a6';g.font='11px system-ui';
 for(let v=0;v<=1000;v+=250){const y=H-(v/1000)*(H-10);g.beginPath();g.moveTo(34,y);g.lineTo(W,y);g.stroke();g.fillText(v,2,y+4);}
 const x=t=>34+(W-34)*(1-(now-t)/60);
 for(let i=1;i<hist.length;i++){g.fillStyle=STATE_COLORS[hist[i].st]||'#444';g.fillRect(x(hist[i-1].t),H+6,x(hist[i].t)-x(hist[i-1].t)+1,10);}
 for(const [key,col,scale] of [['src','#4aa3ff',1],['dst','#3ccf91',1],['flow','#f2b84b',10]]){g.strokeStyle=col;g.lineWidth=2;g.beginPath();
  hist.forEach((p,i)=>{const y=H-(Math.min(1000,p[key]*scale)/1000)*(H-10);i?g.lineTo(x(p.t),y):g.moveTo(x(p.t),y);});g.stroke();}
 requestAnimationFrame(draw);}
requestAnimationFrame(draw);
const es=new EventSource('/events');es.onmessage=e=>{const m=JSON.parse(e.data);
 if(m.type==='tlm')onTlm(m);else if(m.type==='stat')onStat(m);else if(m.type==='evt'&&!m.text.startsWith('BB '))logLine(m.board,m.text);};
</script></body></html>
"""


def main() -> int:
    global HUB
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    HUB = Hub()
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    print(f"dashboard: http://127.0.0.1:{a.port}  (boards: {HUB.ports}; log: {HUB.log.name})",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        HUB.send("ctl", "stream off")
    return 0


if __name__ == "__main__":
    sys.exit(main())
