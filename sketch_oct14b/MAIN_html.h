#pragma once
const char MAIN_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Monitoring Pasieki IoT</title>
<style>
body{margin:0; font-family:'Segoe UI',Arial,sans-serif; background:#f2f5ed; color:#333;}
header{background:linear-gradient(135deg,#6B8E23,#3a5515); color:white; padding:20px 30px; box-shadow:0 2px 6px rgba(0,0,0,0.2);}
header h1{margin:0; font-size:24px;}
.grid{display:grid; grid-template-columns:repeat(auto-fit,minmax(290px,1fr)); gap:20px; padding:20px;}
.card{background:white; border-radius:10px; box-shadow:0 2px 5px rgba(0,0,0,0.1); padding:18px; transition:transform .2s;}
.card:hover{transform:scale(1.02);}
.card h2{font-size:18px; color:#3a5515; border-bottom:1px solid #ccc; padding-bottom:5px;}
.value{font-size:22px; font-weight:bold; color:#222;}
.label{font-size:13px; color:#666;}
.remote{background:#fafafa; border:1px solid #dcdcdc; border-radius:8px; padding:10px; margin-top:10px;}
.remote.active{border-left:5px solid #6B8E23;}
.btn-group{display:flex; gap:8px; margin-top:8px;}
.btn{padding:6px 12px; border:none; border-radius:5px; cursor:pointer; font-size:12px; transition:background .2s;}
.btn-history{background:#6B8E23; color:white;}
.btn-history:hover{background:#3a5515;}
.btn-chart{background:#3498db; color:white;}
.btn-chart:hover{background:#2980b9;}
footer{text-align:center; font-size:12px; color:#666; padding:10px;}

/* Modal */
.modal{display:none; position:fixed; z-index:100; left:0; top:0; width:100%; height:100%; background:rgba(0,0,0,0.6);}
.modal-content{background:#fff; margin:3% auto; padding:20px; width:90%; max-width:800px; border-radius:12px; box-shadow:0 8px 32px rgba(0,0,0,0.3);}
.close{float:right; font-size:28px; font-weight:bold; color:#aaa; cursor:pointer;}
.close:hover{color:#000;}
canvas{display:block; margin:20px auto; border:1px solid #ddd; border-radius:8px;}
.history-item{padding:10px; border-bottom:1px solid #eee;}
</style>
</head>
<body>
<header>
  <h1>📡 Monitoring Pasieki IoT – Ule i Otoczenie</h1>
  <p>Podgląd w czasie rzeczywistym: <b>http://ule.local</b></p>
</header>

<div class="grid">
  <div class="card">
    <h2>Dane środowiskowe – Pasieka</h2>
    <div><span class="label">Temperatura:</span> <span id="base-temp" class="value">--</span> °C</div>
    <div><span class="label">Wilgotność:</span> <span id="base-hum" class="value">--</span> %</div>
    <div><span class="label">Ciśnienie:</span> <span id="base-pressure" class="value">--</span> hPa</div>
    <div><span class="label">Wysokość:</span> <span id="base-alt" class="value">--</span> m</div>
  </div>

  <div class="card">
    <h2>Jakość powietrza</h2>
    <div><span class="label">PM1.0:</span> <span id="base-pm1" class="value">--</span> µg/m³</div>
    <div><span class="label">PM2.5:</span> <span id="base-pm25" class="value">--</span> µg/m³</div>
    <div><span class="label">PM10:</span> <span id="base-pm10" class="value">--</span> µg/m³</div>
  </div>

  <div class="card">
    <h2>Czujniki w ulach</h2>
    <div id="remotes-container"></div>
  </div>
</div>

<!-- Modal Historia -->
<div id="modalHistory" class="modal">
  <div class="modal-content">
    <span class="close" onclick="closeHistory()">&times;</span>
    <h2 id="historyTitle">Historia Ula</h2>
    <div id="historyList"></div>
  </div>
</div>

<!-- Modal Wykres -->
<div id="modalChart" class="modal">
  <div class="modal-content">
    <span class="close" onclick="closeChart()">&times;</span>
    <h2 id="chartTitle">Wykres Ula</h2>
    <canvas id="chartCanvas" width="700" height="350"></canvas>
  </div>
</div>

<footer>🐝 Projekt pasieki IoT – monitorowanie mikroklimatu uli i otoczenia • ESP32 & Wi-Fi</footer>

<script>
function updatePage(data){
  document.getElementById("base-temp").innerText=data.base.temperature.toFixed(1);
  document.getElementById("base-hum").innerText=data.base.humidity.toFixed(1);
  document.getElementById("base-pressure").innerText=data.base.pressure.toFixed(1);
  document.getElementById("base-alt").innerText=data.base.altitude;
  document.getElementById("base-pm1").innerText=data.base.pm1_0;
  document.getElementById("base-pm25").innerText=data.base.pm2_5;
  document.getElementById("base-pm10").innerText=data.base.pm10;

  // Dynamiczne generowanie tylko aktywnych uli
  let container = document.getElementById('remotes-container');
  container.innerHTML = '';
  
  let activeCount = 0;
  for(let i=0; i<data.remotes.length; i++){
    let n = data.remotes[i];
    if(!n || !n.active) continue; // Pomiń nieaktywne
    activeCount++;
    
    let div = document.createElement('div');
    div.className = 'remote active';
    div.innerHTML = `<b>Ul ${n.id}</b> – aktywny<br>
      🌡️ Temp: ${n.temperature.toFixed(1)} °C<br>
      💧 Wilgotność: ${n.humidity.toFixed(1)} %
      <div class="btn-group">
        <button class="btn btn-history" onclick="showHistory(${n.id})">📋 Historia</button>
        <button class="btn btn-chart" onclick="showChart(${n.id})">📊 Wykres</button>
      </div>`;
    container.appendChild(div);
  }
  
  if(activeCount === 0){
    container.innerHTML = '<p style="color:#999;text-align:center;">Brak aktywnych uli</p>';
  }
}

function getData(){ 
  fetch("/data.json").then(r=>r.json()).then(updatePage);
}

setInterval(getData,2000);
window.onload=getData;

// --- HISTORIA ---
function showHistory(ulId){
  fetch("/history?id=" + ulId)
    .then(r => r.json())
    .then(data => {
      document.getElementById("historyTitle").innerText = "Historia Ula " + ulId;
      let list = document.getElementById("historyList");
      list.innerHTML = "";
      
      if(!data.history || data.history.length === 0){
        list.innerHTML = '<p style="text-align:center;color:#999;">Brak danych</p>';
      } else {
        data.history.forEach(item => {
          let time = formatTime(item.secondsAgo);
          let div = document.createElement("div");
          div.className = "history-item";
          div.innerHTML = `<b>${time}</b><br>🌡️ ${item.temperature.toFixed(1)}°C | 💧 ${item.humidity.toFixed(1)}%`;
          list.appendChild(div);
        });
      }
      
      document.getElementById("modalHistory").style.display = "block";
    })
    .catch(err => console.error("Błąd:", err));
}

function closeHistory(){
  document.getElementById("modalHistory").style.display = "none";
}

function formatTime(sec){
  if(sec < 60) return sec + " sek. temu";
  let min = Math.floor(sec / 60);
  if(min < 60) return min + " min. temu";
  let hr = Math.floor(min / 60);
  if(hr < 24) return hr + " godz. temu";
  return Math.floor(hr / 24) + " dni temu";
}

// --- WYKRESY ---
function showChart(ulId){
  document.getElementById("chartTitle").innerText = "Wykres Ula " + ulId;
  document.getElementById("modalChart").style.display = "block";
  
  fetch("/history?id=" + ulId)
    .then(r => r.json())
    .then(data => {
      drawChart(data.history);
    })
    .catch(err => console.error("Błąd:", err));
}

function closeChart(){
  document.getElementById("modalChart").style.display = "none";
}

function drawChart(history){
  const canvas = document.getElementById("chartCanvas");
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  
  ctx.clearRect(0, 0, w, h);
  
  if(!history || history.length === 0){
    ctx.fillStyle = "#888";
    ctx.font = "16px Arial";
    ctx.textAlign = "center";
    ctx.fillText("Brak danych historycznych", w/2, h/2);
    return;
  }
  
  const temps = history.map(p => p.temperature);
  const hums = history.map(p => p.humidity);
  const labels = history.map(p => p.secondsAgo);
  
  const minTemp = Math.min(...temps) - 2;
  const maxTemp = Math.max(...temps) + 2;
  const minHum = Math.min(...hums) - 5;
  const maxHum = Math.max(...hums) + 5;
  
  const padding = {left: 60, right: 60, top: 40, bottom: 70};
  const chartW = w - padding.left - padding.right;
  const chartH = h - padding.top - padding.bottom;
  
  // Tło
  ctx.fillStyle = "#f9f9f9";
  ctx.fillRect(padding.left, padding.top, chartW, chartH);
  
  // Siatka pozioma
  ctx.strokeStyle = "#e0e0e0";
  ctx.lineWidth = 1;
  for(let i=0; i<=10; i++){
    let y = padding.top + (i/10) * chartH;
    ctx.beginPath();
    ctx.moveTo(padding.left, y);
    ctx.lineTo(w - padding.right, y);
    ctx.stroke();
  }
  
  // Osie
  ctx.strokeStyle = "#333";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(padding.left, padding.top);
  ctx.lineTo(padding.left, h - padding.bottom);
  ctx.lineTo(w - padding.right, h - padding.bottom);
  ctx.stroke();
  
  function mapY(val, min, max){
    return h - padding.bottom - ((val - min) / (max - min)) * chartH;
  }
  
  // Oś Y lewa - Temperatura
  ctx.fillStyle = "#e74c3c";
  ctx.font = "bold 11px Arial";
  ctx.textAlign = "right";
  ctx.fillText("Temp [°C]", padding.left - 5, padding.top - 10);
  for(let i=0; i<=5; i++){
    let val = minTemp + (maxTemp - minTemp) * (i/5);
    let y = h - padding.bottom - (i/5) * chartH;
    ctx.fillText(val.toFixed(1), padding.left - 8, y + 4);
  }
  
  // Oś Y prawa - Wilgotność
  ctx.fillStyle = "#3498db";
  ctx.textAlign = "left";
  ctx.fillText("Wilg [%]", w - padding.right + 5, padding.top - 10);
  for(let i=0; i<=5; i++){
    let val = minHum + (maxHum - minHum) * (i/5);
    let y = h - padding.bottom - (i/5) * chartH;
    ctx.fillText(val.toFixed(0), w - padding.right + 8, y + 4);
  }
  
  // Oś X - Czas
  ctx.fillStyle = "#666";
  ctx.font = "10px Arial";
  ctx.textAlign = "center";
  
  let step = Math.max(1, Math.floor(history.length / 6));
  for(let i=0; i<labels.length; i+=step){
    const x = padding.left + (i / (labels.length-1)) * chartW;
    ctx.fillText(labels[i] + "s", x, h - padding.bottom + 20);
  }
  
  // Opis osi X
  ctx.fillStyle = "#333";
  ctx.font = "bold 12px Arial";
  ctx.fillText("Czas [sekundy wstecz]", w/2, h - 10);
  
  // Linia temperatury
  ctx.strokeStyle = "#e74c3c";
  ctx.lineWidth = 2;
  ctx.beginPath();
  for(let i=0; i<temps.length; i++){
    const x = padding.left + (i / (temps.length-1)) * chartW;
    const y = mapY(temps[i], minTemp, maxTemp);
    if(i===0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
  
  // Punkty temperatury
  ctx.fillStyle = "#e74c3c";
  for(let i=0; i<temps.length; i++){
    const x = padding.left + (i / (temps.length-1)) * chartW;
    const y = mapY(temps[i], minTemp, maxTemp);
    ctx.beginPath();
    ctx.arc(x, y, 3, 0, 2*Math.PI);
    ctx.fill();
  }
  
  // Linia wilgotności
  ctx.strokeStyle = "#3498db";
  ctx.lineWidth = 2;
  ctx.beginPath();
  for(let i=0; i<hums.length; i++){
    const x = padding.left + (i / (hums.length-1)) * chartW;
    const y = mapY(hums[i], minHum, maxHum);
    if(i===0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
  
  // Punkty wilgotności
  ctx.fillStyle = "#3498db";
  for(let i=0; i<hums.length; i++){
    const x = padding.left + (i / (hums.length-1)) * chartW;
    const y = mapY(hums[i], minHum, maxHum);
    ctx.beginPath();
    ctx.arc(x, y, 3, 0, 2*Math.PI);
    ctx.fill();
  }
  
  // Legenda
  ctx.fillStyle = "#e74c3c";
  ctx.fillRect(w - 170, 20, 12, 10);
  ctx.fillStyle = "#333";
  ctx.font = "11px Arial";
  ctx.textAlign = "left";
  ctx.fillText("Temperatura", w - 153, 28);
  
  ctx.fillStyle = "#3498db";
  ctx.fillRect(w - 170, 35, 12, 10);
  ctx.fillStyle = "#333";
  ctx.fillText("Wilgotność", w - 153, 43);
}
</script>
</body>
</html>
)rawliteral";
