/* Browser port of your ESP32 ambient pad
   - WebAudio AudioWorklet (render in 128-sample blocks)
   - Emulated "TFT" on <canvas>, encoder via keys/wheel, button via Enter/click
   - No external libs, no hardware
*/
const SR = 44100;
const CHUNK = 128; // smaller block for responsiveness
const TAU = Math.PI * 2;

// ---------------- AudioWorklet: inline via Blob ----------------
const workletCode = `
const SR = sampleRate|0;
const TAU = Math.PI*2;
const CHUNK = 128;  // match main
// ---- Utilities ----
function clamp(x,a,b){ return x<a?a:(x>b?b:x); }
function lerp(a,b,t){ return a + (b-a)*t; }
function smoothstep(t){ t = clamp(t,0,1); return t*t*(3-2*t); }
function expLerp(aHz,bHz,t){ const la=Math.log(aHz), lb=Math.log(bHz); return Math.exp(lerp(la,lb,t)); }
function softClip(x){ return Math.tanh(1.8*x); }

// RNG
let rng32 = 0xA3C59AC3>>>0;
function xr(){ let x=rng32|0; x^=x<<13; x^=x>>>17; x^=x<<5; rng32=x>>>0; return rng32; }
function rand01(){ return xr() * (1/4294967296.0); }

// One-pole LP
class OnePoleLP{
  constructor(){ this.a=0; this.y=0; }
  set(cut){ this.a = 1 - Math.exp(-(TAU*cut)/SR); }
  run(x){ this.y += this.a*(x - this.y); return this.y; }
}

// DC blocker
class DCBlock{
  constructor(){ this.y1=0; this.x1=0; this.R=0.995; }
  run(x){ const y = x - this.x1 + this.R*this.y1; this.x1 = x; this.y1 = y; return y; }
}

// Osc: sine/triangle blend
class Osc{ constructor(){ this.phase=0; this.freqHz=0; this.triMix=0.3; } step(){
  this.phase += TAU*(this.freqHz/SR); if (this.phase>=TAU) this.phase-=TAU;
  const s = Math.sin(this.phase);
  let u = (this.phase/TAU); u = u - Math.floor(u);
  const tri = 4*Math.abs(u-0.5)-1;
  const m = this.triMix; return (1-m)*s + m*tri;
}}

// Pad synth (3 osc + amp LFO + LPF)
class Pad{
  constructor(){
    this.v1=new Osc(); this.v2=new Osc(); this.v3=new Osc();
    this.baseHz=140; this.det=0; this.triMix=0.3;
    this.ampLfoPhase=0; this.ampLfoRateHz=0.08; this.ampLfoDepth=0.25;
    this.lp=new OnePoleLP(); this.lpfCutHz=800; this.inited=false;
  }
  init(){
    this.v1.freqHz=this.baseHz;
    this.v2.freqHz=this.baseHz*(1-this.det);
    this.v3.freqHz=this.baseHz*(1-this.det);
    this.v1.triMix=this.v2.triMix=this.v3.triMix=this.triMix;
    this.lp.set(this.lpfCutHz);
    this.inited=true;
  }
  step(){
    if(!this.inited) this.init();
    this.ampLfoPhase += TAU*(this.ampLfoRateHz/SR);
    if(this.ampLfoPhase>=TAU) this.ampLfoPhase-=TAU;
    const lfo = 0.5 + 0.5*Math.sin(this.ampLfoPhase);
    const amp = (1-this.ampLfoDepth) + this.ampLfoDepth*lfo;
    let s = (this.v1.step()+this.v2.step()+this.v3.step())*(1/3);
    s *= amp;
    s = Math.tanh(0.6*s);
    s = this.lp.run(s);
    return s*0.8;
  }
}

// Pink noise (Paul Kellet approx) + band limit
class Pink{
  constructor(){ this.b0=0; this.b1=0; this.b2=0; this.hp=new OnePoleLP(); this.lp=new OnePoleLP(); this.init=false; }
  ensure(){ if(this.init) return; this.hp.set(150); this.lp.set(5000); this.init=true; }
  step(){
    this.ensure();
    const w = (xr()|0) * (1/2147483648.0);
    this.b0 = 0.99765*this.b0 + 0.0990460*w;
    this.b1 = 0.96300*this.b1 + 0.2965164*w;
    this.b2 = 0.57000*this.b2 + 1.0526913*w;
    const y = (this.b0 + this.b1 + this.b2 + 0.1848*w)*0.05;
    const low = this.hp.run(y); const high = y - low; const air = this.lp.run(high); return air;
  }
}

// Tiny pluck pool
function midiToHz(m){ return 440*Math.pow(2, (m-69)/12); }
class ADSR{ constructor(){ this.a=0.003; this.d=0.080; this.s=0.0; this.r=0.060; this.env=0; this.state=0; this.gate=false; }
  step(){
    switch(this.state){
      case 1:{ const inc= (this.a<=0)?1:(1/(this.a*SR)); this.env+=inc; if(this.env>=1){ this.env=1; this.state=2; } } break;
      case 2:{ const dec= (this.d<=0)?1:(1/(this.d*SR)); this.env -= dec*(1-this.s); if(this.env<=this.s){ this.env=this.s; this.state=3; } } break;
      case 3:{ if(!this.gate) this.state=4; } break;
      case 4:{ const dec= (this.r<=0)?1:(1/(this.r*SR)); this.env -= dec*this.env; if(this.env<=0.0008){ this.env=0; this.state=0; } } break;
      default: this.env=0; break;
    }
    return this.env;
  }
}
class Voice{ constructor(){ this.active=false; this.midi=60; this.freq=261.63; this.phase=0; this.vel=0.7; this.eg=new ADSR(); }
  step(){
    const env=this.eg.step(); if(this.eg.state===0){ this.active=false; return 0; }
    this.phase += TAU*(this.freq/SR); if(this.phase>=TAU) this.phase-=TAU;
    const sSin = Math.sin(this.phase);
    const duty = 0.30;
    const frac = (this.phase/TAU)%1;
    const sSq = (frac < duty) ? 1 : -1;
    const s = 0.80*sSin + 0.20*sSq;
    return s * env * this.vel;
  }
}
class PluckPool{ constructor(){ this.v=[new Voice(), new Voice(), new Voice(), new Voice()]; this.gain=0.35; }
  noteOn(midi, vel){
    let idx = this.v.findIndex(v=>!v.active);
    if(idx<0){
      let min=1e9; for(let i=0;i<4;i++){ const e=this.v[i].eg.env; if(e<min){ min=e; idx=i; } }
    }
    const v=this.v[idx]; v.active=true; v.midi=midi; v.freq=midiToHz(midi); v.phase=0; v.vel=Math.max(0,Math.min(1,vel));
    v.eg.env=0; v.eg.state=1; v.eg.gate=false;
  }
  mix(){
    let s=0;
    for(let i=0;i<4;i++){ const v=this.v[i]; if(!v.active) continue; const sv=v.step(); if(!v.active) continue; s += sv; }
    return s * this.gain * 0.80;
  }
}

// Simple comb/allpass reverb
class Comb{ constructor(size){ this.buf=new Float32Array(size); this.size=size|0; this.w=0; this.fb=0.62; this.damp=0.22; this.filt=0; }
  step(x){
    const y = this.buf[this.w];
    this.filt += this.damp*(y - this.filt);
    let wr = x + this.fb*this.filt;
    wr = Math.max(-1, Math.min(1, wr));
    this.buf[this.w] = wr;
    if(++this.w >= this.size) this.w=0;
    return y;
  }
}
class Allpass{ constructor(size){ this.buf=new Float32Array(size); this.size=size|0; this.w=0; this.g=0.5; }
  step(x){
    const z = this.buf[this.w];
    const y = -x + z;
    let wr = x + this.g*y; wr = Math.max(-1, Math.min(1, wr));
    this.buf[this.w]=wr; if(++this.w>=this.size) this.w=0;
    return y;
  }
}
class ReverbLite{
  constructor(){
    this.c=[new Comb(1117), new Comb(1189), new Comb(1277), new Comb(1361)];
    this.ap1=new Allpass(225); this.ap2=new Allpass(341);
    this.baseSend=0; this.baseWet=0; this.send=0; this.wet=0;
  }
  setSpace(spacePct){
    const t = clamp(spacePct,0,100)*0.01;
    const k = Math.sqrt(t);
    const fb = Math.min(0.75, 0.45 + 0.27*k);
    const damp = 0.15 + 0.20*k;
    for(let i=0;i<4;i++){ this.c[i].fb=fb; this.c[i].damp=damp; }
    this.baseSend = 0.70*k; this.baseWet=0.35*k;
    this.send=this.baseSend; this.wet=this.baseWet;
  }
  step(dry){
    const xin = this.send*dry;
    let y = 0;
    for(let i=0;i<4;i++) y += this.c[i].step(xin);
    y *= 0.25;
    y = this.ap1.step(y);
    y = this.ap2.step(y);
    return y;
  }
}

// Delay
function delaySamplesFromBpm(bpm, mode){ bpm=Math.max(30,Math.min(240,bpm|0)); const qnote = (SR*60)/bpm; const want = (mode===0)? qnote*0.5 : qnote; return Math.max(1, Math.min(32767, want|0)); }
class Delay{
  constructor(){
    this.size=32768; this.buf=new Float32Array(this.size); this.w=0; this.dSamp=22050;
    this.tone=new OnePoleLP(); this.baseSend=0; this.baseWet=0; this.send=0; this.wet=0; this.fb=0; this.mode=1; this.bpmCached=104;
    this.tone.set(2800);
  }
  updateTempo(bpm){ if(bpm!==this.bpmCached){ this.bpmCached=bpm; this.dSamp=delaySamplesFromBpm(bpm,this.mode); } }
  setMode(m){ this.mode=m; this.dSamp=delaySamplesFromBpm(this.bpmCached,this.mode); }
  setSpace(spacePct){
    const t = clamp(spacePct,0,100)*0.01; const k = Math.sqrt(t);
    this.baseSend = 0.75*k; this.baseWet = 0.55*k; this.fb = Math.min(0.6, 0.55*k);
    this.send = this.baseSend; this.wet = this.baseWet;
  }
  step(dry){
    const r = (this.w + this.size - this.dSamp) % this.size;
    const delayed = this.buf[r];
    const fbSample = this.tone.run(delayed);
    let toWrite = dry*this.send + fbSample*this.fb; toWrite = Math.max(-1, Math.min(1,toWrite));
    this.buf[this.w] = toWrite;
    if(++this.w>=this.size) this.w=0;
    return delayed;
  }
}

// Tone tilt around ~6k
class TiltEQ{
  constructor(){ this.split=new OnePoleLP(); this.split.set(6000); this.gL=1; this.gH=1; }
  set(tonePct){
    let u = clamp(tonePct,0,100)*0.01;
    let tiltDb = 6*(u-0.5); // -3..+3
    let gH = Math.pow(10, tiltDb/20); let gL = 1/gH;
    gH = clamp(gH, 0.67, 1.5); gL = clamp(gL, 0.67, 1.5);
    this.gH=gH; this.gL=gL;
  }
  run(x){
    const low = this.split.run(x); const high = x - low;
    return this.gL*low + this.gH*high;
  }
}

// Scale stuff
const SCALE = { C_MINOR:0, A_MINOR:1, D_DORIAN:2, C_LYDIAN:3 };
const SCALE_DEF = [
  {root:0, deg:[0,2,4,6,7,9,11], n:7},  // C minor (per your table)
  {root:9, deg:[0,2,3,5,7,8,10], n:7},  // A minor
  {root:2, deg:[0,2,3,5,7,9,10], n:7},  // D dorian
  {root:0, deg:[0,2,4,6,7,9,11], n:7},  // C lydian
];
function quantizeMidi(m, s){
  const sd = SCALE_DEF[s]; const inPC = ((m%12)+12)%12;
  let bestDiff=127, bestAbs=128;
  for(let i=0;i<sd.n;i++){
    let targetPC = (sd.root + sd.deg[i])%12;
    let d = targetPC - inPC;
    if (d>6) d-=12; if (d<-6) d+=12;
    let ad = Math.abs(d);
    if (ad<bestAbs || (ad===bestAbs && d>bestDiff)){ bestAbs=ad; bestDiff=d; }
  }
  let out = m + bestDiff; return clamp(out,0,127);
}
let lastAmbient = 60;
function pickAmbientNote(s){
  const sd = SCALE_DEF[s]; const pool=[];
  for(let oct=2; oct<=6; oct++){
    const base = oct*12 + sd.root;
    for(let i=0;i<sd.n;i++){
      const m = base + sd.deg[i]; if (m>=36 && m<=96) pool.push(m);
    }
  }
  let cand = pool[xr()%pool.length];
  if (rand01()<0.10) return lastAmbient;
  if (rand01()<0.15) cand += (xr()&1)? +7 : -5;
  if (Math.abs(cand-lastAmbient)<3) cand += (xr()&1)? +7 : -5;
  cand = clamp(cand,36,96); cand = quantizeMidi(cand, s);
  lastAmbient = cand; return cand;
}

// Clock 1/16
function sptFromBpm(bpm){ bpm = clamp(bpm,30,240)|0; return Math.round((SR*60)/(bpm*4)); }
class Clock16{
  constructor(){ this.smpNow=0n; this.nextTick=0n; this.spt=sptFromBpm(104); this.step16=0; this.bpm=104; }
  init(bpm){ this.bpm=bpm; this.spt=sptFromBpm(bpm); this.smpNow=0n; this.nextTick=BigInt(this.spt); this.step16=0; }
  update(bpm){ if(bpm!==this.bpm){ this.bpm=bpm; this.spt=sptFromBpm(bpm); this.nextTick=this.smpNow + BigInt(this.spt); } }
  advance(block){
    const end = this.smpNow + BigInt(block);
    while(this.nextTick <= end){
      this.onStep(this.step16);
      this.step16 = (this.step16 + 1) & 0x0F;
      this.nextTick += BigInt(this.spt);
    }
    this.smpNow = end;
  }
  onStep(step){} // set externally
}

// Energy & Gestures (subset faithful)
class Energy{
  constructor(){ this.pStrong=0.40; this.pEven=0.22; this.pOdd=0.08; this.ampLfoRateHz=0.08; this.ampLfoDepth=0.25; this.gestureLambdaHz=1/12; this.nextGestureAt=0n; }
  set(pad, energyPct){
    const u = Math.sqrt(clamp(energyPct,0,100)*0.01);
    this.pStrong = clamp(0.40*(0.7+1.2*u), 0.10, 0.85);
    this.pEven   = clamp(0.22*(0.7+1.2*u), 0.05, 0.55);
    this.pOdd    = clamp(0.08*(0.7+1.2*u), 0.02, 0.30);
    this.ampLfoRateHz = 0.04 + (0.18-0.04)*u;
    this.ampLfoDepth  = 0.15 + (0.32-0.15)*u;
    pad.ampLfoRateHz = this.ampLfoRateHz; pad.ampLfoDepth = this.ampLfoDepth;
    this.gestureLambdaHz = (1/30) + ((1/6)-(1/30))*u;
  }
}
function samplesFromExp(lambdaHz){ if(lambdaHz<=0) lambdaHz=1e-6; let r=1-rand01(); if(r<1e-6) r=1e-6; const sec=-Math.log(r)/lambdaHz; const s=sec*SR; return Math.max(1, s|0); }

// Gesture overlay
class Gesture{
  constructor(){ this.lambdaHz = 1/15; this.nextAt=0n; this.active=false; this.t0=0n; this.tAtkEnd=0n; this.tRelStart=0n; this.tEnd=0n; this.env=0; this.cutBoostOct=0.5; this.fxSendBoost=0.5; this.fxWetBoost=0.35; }
  update(nowSmp, pad, delay, reverb){
    this.lambdaHz = Math.max(this.lambdaHz, 1/15);
    if(this.nextAt===0n) this.nextAt = nowSmp + BigInt(samplesFromExp(this.lambdaHz));
    if(!this.active && nowSmp >= this.nextAt){
      const atk = 0.5 + 1.0*rand01(), rel = 2.5 + 2.0*rand01();
      const aS=BigInt(Math.max(1,(atk*SR)|0)), rS=BigInt(Math.max(1,(rel*SR)|0));
      this.t0=nowSmp; this.tAtkEnd=this.t0+aS; this.tRelStart=this.tAtkEnd; this.tEnd=this.tRelStart+rS;
      this.env=0; this.active=true;
      this.cutBoostOct = 0.25 + 0.55*rand01();
      this.fxSendBoost = 0.30 + 0.40*rand01();
      this.fxWetBoost  = 0.20 + 0.30*rand01();
      this.nextAt = nowSmp + BigInt(samplesFromExp(this.lambdaHz));
    }
    // envelope
    let u=0;
    if(this.active){
      if(nowSmp < this.tAtkEnd){
        const num = Number(nowSmp - this.t0), den = Number(this.tAtkEnd - this.t0); u = den? num/den : 1;
      }else if(nowSmp < this.tEnd){
        const num = Number(nowSmp - this.tRelStart), den = Number(this.tEnd - this.tRelStart); const d= den? num/den : 1; u = 1 - d;
      }else{ this.active=false; this.env=0; }
    }
    const alpha = 1 - Math.exp(-CHUNK/(SR*0.05));
    this.env += alpha*(u - this.env);
    const e = this.env;
    if(e>0.0005){
      const eff = clamp(pad.lpfCutHz * Math.pow(2, this.cutBoostOct*e), 400, 8000);
      pad.lp.set(eff);
      delay.send  = delay.baseSend  * (1 + this.fxSendBoost*e);
      delay.wet   = delay.baseWet   * (1 + this.fxWetBoost*e);
      reverb.send = reverb.baseSend * (1 + this.fxSendBoost*e);
      reverb.wet  = reverb.baseWet  * (1 + this.fxWetBoost*e);
    }else{
      delay.send=delay.baseSend; delay.wet=delay.baseWet;
      reverb.send=reverb.baseSend; reverb.wet=reverb.baseWet;
    }
  }
}

// Scene scheduler/xfade
class SceneSch{ constructor(){ this.inited=false; this.nextAt=0n; this.minS=(120*SR)|0; this.maxS=(240*SR)|0; } }
class Scene{ constructor(){ this.active=false; this.t0=0n; this.t1=0n; this.start={}; this.target={}; } }

class State {
  constructor(){
    this.params = { volume:70, tempo:104, space:45, mood:50, tone:42, energy:50 };
    this.fade=0; this.volTarget=70; this.volSmooth=0; this.masterGain=1.35;
    this.dcL=new DCBlock(); this.dcR=new DCBlock();
    this.pad=new Pad(); this.pink=new Pink(); this.tone=new TiltEQ();
    this.delay=new Delay(); this.reverb=new ReverbLite();
    this.pluck=new PluckPool();
    this.scale = 1; // A minor
    this.clock = new Clock16(); this.clock.onStep = (st)=>this.onStep16(st);
    this.energy=new Energy(); this.gesture=new Gesture();
    this.scene=new Scene(); this.sceneSch=new SceneSch();

    this.pitchTarget=160; this.pitchSmooth=160; this.nextPitchAt=0;
    this.outMeter=0; this.cpuMs=0;

    this.delay.setSpace(this.params.space);
    this.reverb.setSpace(this.params.space);
    this.tone.set(this.params.tone);
    this.energy.set(this.pad, this.params.energy);
    this.clock.init(this.params.tempo);
  }

  onStep16(step){
    // vary probabilities per bar
    if((step&15)===0){ this._wob = 0.9 + 0.2*rand01(); }
    const wob = this._wob || 1;
    let p = ((step&3)===0)? this.energy.pStrong*wob : ((step&1)===0)? this.energy.pEven*wob : this.energy.pOdd*wob;
    p = clamp(p, 0.02, 0.95);
    if(rand01() < p){
      const note = pickAmbientNote(this.scale);
      const vel = 0.45 + 0.50*rand01();
      this.pluck.noteOn(note, vel);
    }
  }

  triggerScene(now){
    if(!this.sceneSch.inited){
      this.sceneSch.inited=true;
      const jitter = (this.sceneSch.maxS - this.sceneSch.minS) * rand01();
      this.sceneSch.nextAt = BigInt(this.sceneSch.minS + jitter) + BigInt(now);
    }
    // start immediately
    this.sceneSch.nextAt = BigInt(now);
  }

  sceneUpdate(now){
    if(!this.sceneSch.inited){
      this.sceneSch.inited=true;
      const jitter = (this.sceneSch.maxS - this.sceneSch.minS) * rand01();
      this.sceneSch.nextAt = BigInt(this.sceneSch.minS + jitter) + BigInt(now);
    }
    // start new scene?
    if(!this.scene.active && BigInt(now) >= this.sceneSch.nextAt){
      this.scene.start = {
        scale:this.scale, triMix:this.pad.triMix, lpfCutHz:this.pad.lpfCutHz,
        space:this.params.space, delayMode:this.delay.mode, tempo:this.params.tempo
      };
      const r = Math.floor(3*rand01());
      const tgtScale = (r===0)?1:(r===1)?2:3; // A minor / D dorian / C lydian
      this.scene.target = {
        scale:tgtScale,
        triMix: 0.15 + 0.50*rand01(),
        lpfCutHz: 400 * Math.pow(20, rand01()),
        space: 10 + Math.round(60*rand01()),
        delayMode: (rand01()<0.5)?0:1,
        tempo: clamp(Math.round(this.params.tempo*(0.95 + 0.10*rand01())),30,240)
      };
      const durSec = 10 + 10*rand01();
      const durS = Math.max(1, (durSec*SR)|0);
      this.scene.t0 = BigInt(now);
      this.scene.t1 = BigInt(now + durS);
      this.scene.active=true;

      // schedule next
      const jitter = (this.sceneSch.maxS - this.sceneSch.minS) * rand01();
      this.sceneSch.nextAt = BigInt(this.sceneSch.minS + jitter) + BigInt(now);
      this._sceneBanner = true; // UI hint
      this.port.postMessage({type:'banner', text:'Scene...fading'});
    }

    if(this.scene.active){
      const t = (Number(BigInt(now) - this.scene.t0) / Number(this.scene.t1 - this.scene.t0));
      const e = smoothstep(t);
      // triMix
      const tri = lerp(this.scene.start.triMix, this.scene.target.triMix, e);
      this.pad.triMix = tri; this.pad.v1.triMix = tri; this.pad.v2.triMix = tri; this.pad.v3.triMix=tri;
      // LPF
      const cut = expLerp(this.scene.start.lpfCutHz, this.scene.target.lpfCutHz, e);
      this.pad.lpfCutHz = cut; this.pad.lp.set(cut);
      // Space
      const sp = Math.round(lerp(this.scene.start.space, this.scene.target.space, e));
      if(sp !== this.params.space){ this.params.space = sp; this.delay.setSpace(sp); this.reverb.setSpace(sp); }
      // Tempo
      const bpm = Math.round(lerp(this.scene.start.tempo, this.scene.target.tempo, e));
      if(bpm !== this.params.tempo) this.params.tempo = bpm;
      // Scale switch early
      if(e >= 0.2) this.scale = this.scene.target.scale;
      // Delay mode late
      if(e >= 0.8 && this.delay.mode !== this.scene.target.delayMode){ this.delay.setMode(this.scene.target.delayMode); }
      // finish
      if(t >= 1){
        this.scene.active=false;
        this.port.postMessage({type:'banner', text:''});
      }
    }
  }

  process(outputs, parameters){
    const outL = outputs[0][0];
    const outR = outputs[0][1];
    const t0 = currentTime;

    // smooth volume
    const target = Math.pow(this.params.volume*0.01, 2);
    const alphaV = 1 - Math.exp(-CHUNK/(SR*0.7));
    this.volSmooth += alphaV*(target - this.volSmooth);
    if(this.fade < 1){ this.fade += CHUNK/(SR*0.8); if(this.fade>1) this.fade=1; }

    // tempo tracking
    this.clock.update(this.params.tempo);
    this.delay.updateTempo(this.params.tempo);

    // slow drone pitch random walk
    if(this._msNow === undefined) this._msNow=0;
    this._msNow += (CHUNK*1000/SR);
    if(this.nextPitchAt===0 || this._msNow >= this.nextPitchAt){
      const step = (rand01()*4) - 2; this.pitchTarget = clamp(this.pitchTarget + step, 100, 220);
      this.nextPitchAt = this._msNow + (3000 + 4000*rand01());
    }
    const alphaP = 1 - Math.exp(-CHUNK/(SR*15));
    this.pitchSmooth += alphaP*(this.pitchTarget - this.pitchSmooth);
    this.pad.baseHz = this.pitchSmooth;
    this.pad.v1.freqHz=this.pad.baseHz;
    this.pad.v2.freqHz=this.pad.baseHz*(1-this.pad.det);
    this.pad.v3.freqHz=this.pad.baseHz*(1-this.pad.det);

    let outPeak = 0;

    for(let i=0;i<CHUNK;i++){
      const padMono = this.pad.step();
      const plMono = this.pluck.mix();

      const n = this._noiseDryGain? this.pink.step()*this._noiseDryGain : 0;
      const nfx= this.pink.step()*0.12;

      const CTR = 0.70710678; // -3dB mono center
      let dryMono = padMono*CTR*0.3 + plMono + n;

      let dryFX = this.tone.run(dryMono + nfx);
      const wetTap = this.delay.step(dryFX);
      const wetRev = this.reverb.step(dryFX);
      const wetSum = this.delay.wet*wetTap + this.reverb.wet*wetRev;

      let x = (dryMono + wetSum*CTR) * this.volSmooth * this.fade * this.masterGain;
      let xL = softClip(this.dcL.run(x));
      let xR = softClip(this.dcR.run(x));

      outL[i]=xL; outR[i]=xR;
      const ap = Math.max(Math.abs(xL), Math.abs(xR)); if(ap>outPeak) outPeak=ap;
    }

    // bookkeeping at block boundaries
    this.clock.advance(CHUNK);
    // gestures & scenes at block-rate
    this.gesture.lambdaHz = Math.max(this.energy.gestureLambdaHz, 1/15);
    this.gesture.update(Number(this.clock.smpNow), this.pad, this.delay, this.reverb);
    this.sceneUpdate(Number(this.clock.smpNow));
    // maybe trigger simple gesture schedule on separate ctrl
    if(this.energy.nextGestureAt===0n) this.energy.nextGestureAt = this.clock.smpNow + BigInt(samplesFromExp(this.energy.gestureLambdaHz));
    if(this.clock.smpNow >= this.energy.nextGestureAt){ this.energy.nextGestureAt = this.clock.smpNow + BigInt(samplesFromExp(this.energy.gestureLambdaHz)); }

    this.outMeter = 0.9*this.outMeter + 0.1*outPeak;
    // pseudo CPU estimate
    const cpu = (performance.now() - (this._tStart||performance.now())) * (SR/CHUNK) * 0.001; // fraction
    this.port.postMessage({type:'meters', out:this.outMeter, cpu: clamp(cpu,0,1) });
    return true;
  }
}

class AmbientProcessor extends AudioWorkletProcessor {
  constructor(){
    super();
    this.state = new State();
    this.port = this.port; // for clarity
    this.state.port = this.port;
    this.port.onmessage = (e)=>{
      const m = e.data||{};
      if(m.type==='params'){ Object.assign(this.state.params, m.params||{}); 
        if('space' in m.params){ this.state.delay.setSpace(this.state.params.space); this.state.reverb.setSpace(this.state.params.space); }
        if('tone'  in m.params){ this.state.tone.set(this.state.params.tone); }
        if('energy'in m.params){ this.state.energy.set(this.state.pad, this.state.params.energy); }
        if('tempo' in m.params){ /* clock/delay updated during process() */ }
        if('volume'in m.params){ /* smoothed */ }
      } else if(m.type==='scale'){ this.state.scale = m.scale|0; }
      else if(m.type==='scene'){ this.state.triggerScene(Number(this.state.clock.smpNow)); }
      else if(m.type==='panic'){ this.state.volSmooth=0; this.state.params.volume=0; }
      else if(m.type==='seed'){ rng32 = m.seed>>>0; }
    };
  }
  process(_inputs, outputs, _parameters){
    this.state._tStart = performance.now();
    const ok = this.state.process(outputs, {});
    return ok;
  }
}
registerProcessor('ambient-processor', AmbientProcessor);
`;

const blob = new Blob([workletCode], { type: 'application/javascript' });
const workletURL = URL.createObjectURL(blob);

// ---------------- UI & App ----------------
const tft = document.getElementById('tft');
const ctx = tft.getContext('2d');

const Page = { VOLUME:0, TEMPO:1, SPACE:2, MOOD:3, TONE:4, ENERGY:5, SCENE:6, COUNT:7 };
const pageName = ['Volume','Tempo','Space','Mood','Tone','Energy','Scene'];

const state = {
  page: Page.VOLUME,
  params: { volume:70, tempo:104, space:45, mood:50, tone:42, energy:50 },
  last: { page:-1, value:Infinity, banner:'' },
  running:false,
  audio:null,
  node:null,
};

// draw the emulated TFT
function drawTFT(){
  const w=tft.width, h=tft.height;
  ctx.fillStyle = '#000'; ctx.fillRect(0,0,w,h);
  // banner
  if(state.last.banner){
    ctx.fillStyle='#000'; ctx.fillRect(0,0,w,20);
    ctx.fillStyle='#0ff'; ctx.font='12px monospace';
    const text = state.last.banner;
    const tw = ctx.measureText(text).width;
    ctx.fillText(text, (w - tw)/2, 14);
  }

  // big center text
  ctx.fillStyle = '#fff';
  ctx.font = '14px monospace';
  const label = `PAGE: ${pageName[state.page]}`;
  let val = state.params.volume;
  if(state.page===Page.TEMPO) val = state.params.tempo;
  else if(state.page===Page.SPACE) val = state.params.space;
  else if(state.page===Page.MOOD)  val = state.params.mood;
  else if(state.page===Page.TONE)  val = state.params.tone;
  else if(state.page===Page.ENERGY)val = state.params.energy;
  else if(state.page===Page.SCENE) val = 0;

  const str = (state.page===Page.TEMPO) ? `${label}  VAL: ${val} BPM`
                                        : (state.page===Page.SCENE) ? `${label}  press → trigger`
                                        : `${label}  VAL: ${val}`;
  const tw=ctx.measureText(str).width;
  ctx.fillText(str, (w-tw)/2, h/2);

  // volume big
  ctx.font='20px monospace';
  const v = `VOL ${String(state.params.volume).padStart(3,' ')}%`;
  const vw = ctx.measureText(v).width;
  ctx.fillText(v, (w - vw)/2, h - 16);
}

function sendParams(patch){
  Object.assign(state.params, patch);
  if(state.node) state.node.port.postMessage({type:'params', params: patch});
  drawTFT();
}

function nextPage(){
  state.page = (state.page + 1) % Page.COUNT;
  drawTFT();
}

function applyDelta(delta){
  if(delta===0) return;
  const d = Math.sign(delta);
  if(state.page===Page.VOLUME)       sendParams({volume: clamp(state.params.volume + d, 0,100)});
  else if(state.page===Page.TEMPO)   sendParams({tempo:  clamp(state.params.tempo  + d, 30,240)});
  else if(state.page===Page.SPACE)   sendParams({space:  clamp(state.params.space  + d, 0,100)});
  else if(state.page===Page.MOOD)    sendParams({mood:   clamp(state.params.mood   + d, 0,100)});
  else if(state.page===Page.TONE)    sendParams({tone:   clamp(state.params.tone   + d, 0,100)});
  else if(state.page===Page.ENERGY)  sendParams({energy: clamp(state.params.energy + d, 0,100)});
  else if(state.page===Page.SCENE) { triggerScene(); }
}

function triggerScene(){
  if(state.node) state.node.port.postMessage({type:'scene'});
}

function clamp(x,a,b){ return x<a?a:(x>b?b:x); }

// audio start/stop
async function startAudio(){
  if(state.running) return;
  const ac = new AudioContext({ sampleRate: SR, latencyHint:'interactive' });
  await ac.audioWorklet.addModule(workletURL);
  const node = new AudioWorkletNode(ac, 'ambient-processor', { outputChannelCount:[2] });
  node.port.onmessage = (e)=>{
    const m = e.data||{};
    if(m.type==='meters'){
      const pct = Math.round((m.out||0)*100);
      document.getElementById('out').style.width = Math.min(100,pct) + '%';
      document.getElementById('outText').textContent = `${pct}%`;
      const cpuPct = Math.round((m.cpu||0)*100);
      document.getElementById('cpu').style.width = Math.min(100,cpuPct) + '%';
      document.getElementById('cpuText').textContent = `${cpuPct}%`;
    } else if(m.type==='banner'){
      state.last.banner = m.text||'';
      drawTFT();
    }
  };
  node.connect(ac.destination);
  // initial params
  node.port.postMessage({type:'params', params: state.params});
  node.port.postMessage({type:'seed', seed: (Math.random()*0xffffffff)>>>0 });

  state.audio = ac;
  state.node = node;
  state.running = true;
  document.getElementById('start').textContent = 'Pause Audio';
  drawTFT();
}

function stopAudio(){
  if(!state.running) return;
  state.node.disconnect();
  state.audio.close();
  state.running=false; state.audio=null; state.node=null;
  document.getElementById('start').textContent = 'Start Audio';
  drawTFT();
}

// UI wiring
document.getElementById('start').onclick = async ()=>{
  if(!state.running) await startAudio(); else stopAudio();
};
document.getElementById('scene').onclick = ()=>triggerScene();
document.getElementById('panic').onclick = ()=> state.node?.port.postMessage({type:'panic'});

tft.addEventListener('click', ()=>nextPage());
tft.addEventListener('contextmenu', e=>e.preventDefault());
tft.addEventListener('wheel', (e)=>{ e.preventDefault(); applyDelta(Math.sign(e.deltaY)<0?+1:-1); }, {passive:false});

window.addEventListener('keydown', (e)=>{
  if(e.key==='Enter'){ nextPage(); }
  else if(e.key==='ArrowRight'){ applyDelta(+1); }
  else if(e.key==='ArrowLeft'){ applyDelta(-1); }
  else if(e.key===' '){ e.preventDefault(); if(!state.running) startAudio(); else stopAudio(); }
});

function drawLoop(){
  drawTFT();
  requestAnimationFrame(drawLoop);
}
drawLoop();
