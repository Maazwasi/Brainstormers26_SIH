// Edge AI Navigation Dashboard Frontend Logic

let map = null;
let vehicleMarker = null;
let fusedPolyline = null;
let rawMarkersGroup = null;
let idrPolyline = null;

const refLat = 28.6139;
const refLon = 77.2090;

function initMap() {
  if (typeof L === 'undefined') {
    document.getElementById('map').innerHTML = '<div style="padding:40px;color:#8c9ba5;text-align:center;">Local Map Viewer Active (Offline Canvas Mode)</div>';
    return;
  }

  try {
    map = L.map('map', {
      zoomControl: false,
      attributionControl: false
    }).setView([refLat, refLon], 18);

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 20
    }).addTo(map);

    fusedPolyline = L.polyline([], { color: '#00d2ff', weight: 4, opacity: 0.9 }).addTo(map);
    idrPolyline = L.polyline([], { color: '#ffb300', weight: 3, dashArray: '5, 8', opacity: 0.8 }).addTo(map);
    rawMarkersGroup = L.layerGroup().addTo(map);

    // Custom glowing vehicle dot icon
    const vehicleIcon = L.divIcon({
      className: 'vehicle-marker',
      html: '<div style="width:14px; height:14px; background:#00e676; border:2px solid #fff; border-radius:50%; box-shadow: 0 0 10px #00e676;"></div>',
      iconSize: [14, 14],
      iconAnchor: [7, 7]
    });

    vehicleMarker = L.marker([refLat, refLon], { icon: vehicleIcon }).addTo(map);
  } catch (e) {
    console.error("Map initialization warning:", e);
  }
}

async function fetchTelemetry() {
  try {
    const res = await fetch('/api/telemetry');
    if (!res.ok) return;
    const data = await res.json();
    const out = data.output;
    const frame = data.frame;

    if (!out || !out.est_lat) return;

    // Update Badges
    const badge = document.getElementById('system-badge');
    badge.className = 'badge';
    if (out.mode === 'GNSS_FIX') {
      badge.textContent = '● GNSS ACTIVE';
      badge.classList.add('badge-green');
    } else if (out.mode === 'GNSS_DEGRADED') {
      badge.textContent = '⚠ GNSS DEGRADED';
      badge.classList.add('badge-yellow');
    } else if (out.mode === 'IDR_ACTIVE') {
      badge.textContent = '⚡ IDR ACTIVE (OUTAGE)';
      badge.classList.add('badge-red');
    } else if (out.mode === 'FUSION_CONVERGING') {
      badge.textContent = '🔄 FUSION CONVERGING';
      badge.classList.add('badge-cyan');
    }

    // Update AI condition
    const aiCond = document.getElementById('ai-condition');
    const aiBar = document.getElementById('ai-bar');
    aiCond.textContent = out.ai_condition;
    const confPct = (out.ai_confidence * 100).toFixed(1) + '%';
    document.getElementById('ai-confidence').textContent = confPct;
    document.getElementById('ai-latency').textContent = out.latency_ms.toFixed(2) + ' ms';
    aiBar.style.width = confPct;

    if (out.ai_condition === 'OPEN_SKY_NORMAL') {
      aiCond.style.color = 'var(--accent-green)';
      aiBar.style.background = 'var(--accent-green)';
    } else if (out.ai_condition === 'URBAN_CANYON_WEAK') {
      aiCond.style.color = 'var(--accent-yellow)';
      aiBar.style.background = 'var(--accent-yellow)';
    } else if (out.ai_condition === 'TUNNEL_OUTAGE') {
      aiCond.style.color = 'var(--accent-red)';
      aiBar.style.background = 'var(--accent-red)';
    }

    // Update Metrics
    document.getElementById('val-lat').textContent = out.est_lat.toFixed(7);
    document.getElementById('val-lon').textContent = out.est_lon.toFixed(7);
    document.getElementById('val-speed').textContent = `${out.est_speed.toFixed(1)} m/s (${(out.est_speed * 3.6).toFixed(1)} km/h)`;
    document.getElementById('val-heading').textContent = `${out.est_heading.toFixed(1)}°`;

    document.getElementById('val-sats').textContent = out.gnss_sats;
    document.getElementById('val-hdop').textContent = out.gnss_hdop.toFixed(1);
    
    const gnssStatus = document.getElementById('val-gnss-status');
    if (frame && frame.gnss && frame.gnss.valid) {
      gnssStatus.textContent = 'LOCKED';
      gnssStatus.style.color = 'var(--accent-green)';
    } else {
      gnssStatus.textContent = 'OUTAGE';
      gnssStatus.style.color = 'var(--accent-red)';
    }

    document.getElementById('val-outage-time').textContent = `${out.outage_duration.toFixed(1)} s`;
    document.getElementById('val-drift').textContent = `${out.accumulated_drift.toFixed(2)} m`;
    document.getElementById('val-zupt').textContent = out.zupt_active ? 'YES' : 'NO';

    if (frame && frame.imu) {
      document.getElementById('val-imu-accel').textContent = 
        `Ax: ${frame.imu.ax >= 0 ? '+' : ''}${frame.imu.ax.toFixed(2)} | Ay: ${frame.imu.ay >= 0 ? '+' : ''}${frame.imu.ay.toFixed(2)} | Az: ${frame.imu.az >= 0 ? '+' : ''}${frame.imu.az.toFixed(2)}`;
      document.getElementById('val-imu-gyro').textContent = 
        `Gx: ${frame.imu.gx >= 0 ? '+' : ''}${frame.imu.gx.toFixed(3)} | Gy: ${frame.imu.gy >= 0 ? '+' : ''}${frame.imu.gy.toFixed(3)} | Gz: ${frame.imu.gz >= 0 ? '+' : ''}${frame.imu.gz.toFixed(3)}`;
    }

    // Update Map
    if (map && data.trails) {
      const fusedCoords = data.trails.fused.map(pt => [pt.lat, pt.lon]);
      const idrCoords = data.trails.idr.map(pt => [pt.lat, pt.lon]);
      
      fusedPolyline.setLatLngs(fusedCoords);
      idrPolyline.setLatLngs(idrCoords);
      
      if (vehicleMarker) {
        vehicleMarker.setLatLng([out.est_lat, out.est_lon]);
        map.panTo([out.est_lat, out.est_lon], { animate: true, duration: 0.1 });
      }
    }
  } catch (err) {
    // Polling retry
  }
}

async function triggerOutage() {
  await fetch('/api/control', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'cut_gnss' })
  });
}

async function restoreGNSS() {
  await fetch('/api/control', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'restore_gnss' })
  });
}

window.addEventListener('DOMContentLoaded', () => {
  initMap();
  setInterval(fetchTelemetry, 100);
});
