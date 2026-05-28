'use strict';

const TAU = Math.PI * 2;
const DEG = 180 / Math.PI;

const els = {
  svg: document.getElementById('mechanismSvg'),
  grid: document.getElementById('gridLayer'),
  support: document.getElementById('supportLayer'),
  angle: document.getElementById('angleLayer'),
  link: document.getElementById('linkLayer'),
  joint: document.getElementById('jointLayer'),
  label: document.getElementById('labelLayer'),
  canvasCard: document.getElementById('canvasCard'),
  sliders: {
    g: document.getElementById('gSlider'),
    a: document.getElementById('aSlider'),
    b: document.getElementById('bSlider'),
    f: document.getElementById('fSlider'),
  },
  values: {
    g: document.getElementById('gValue'),
    a: document.getElementById('aValue'),
    b: document.getElementById('bValue'),
    f: document.getElementById('fValue'),
  },
  speed: document.getElementById('speedSlider'),
  play: document.getElementById('playButton'),
  openBranch: document.getElementById('openBranch'),
  crossBranch: document.getElementById('crossBranch'),
  angleReadout: document.getElementById('angleReadout'),
  gIndex: document.getElementById('gIndex'),
  vIndex: document.getElementById('vIndex'),
  t1Index: document.getElementById('t1Index'),
  t2Index: document.getElementById('t2Index'),
  t3Index: document.getElementById('t3Index'),
  grashofBadge: document.getElementById('grashofBadge'),
  stateBadge: document.getElementById('stateBadge'),
  validityBadge: document.getElementById('validityBadge'),
};

const state = {
  g: 22,
  a: 10,
  b: 18,
  f: 26,
  alpha: Math.PI / 4,
  direction: 1,
  playing: false,
  branch: 1,
  speed: 1,
  lastTime: null,
};

const NS = 'http://www.w3.org/2000/svg';

function svgElement(tag, attrs = {}, textContent = '') {
  const el = document.createElementNS(NS, tag);
  Object.entries(attrs).forEach(([k, v]) => el.setAttribute(k, String(v)));
  if (textContent !== '') el.textContent = textContent;
  return el;
}

function fmt(x, digits = 1) {
  if (!Number.isFinite(x)) return '—';
  return Number(x).toFixed(digits);
}

function normAngle(angle) {
  let a = angle % TAU;
  if (a < 0) a += TAU;
  return a;
}

function sortedLengths() {
  return [state.g, state.a, state.b, state.f].slice().sort((x, y) => x - y);
}

function grashofIndex() {
  const [s, p, q, l] = sortedLengths();
  return s + l - p - q;
}

function validityIndex() {
  const [s, p, q, l] = sortedLengths();
  return l - s - p - q;
}

function isGloballyValid() {
  return validityIndex() <= 1e-9;
}

function isGrashof() {
  return grashofIndex() <= 1e-9;
}

function excessValues() {
  const { g, a, b, f } = state;
  return {
    t1: g + f - b - a,
    t2: b + g - f - a,
    t3: f + b - g - a,
  };
}

function solvePose(alpha = state.alpha, branch = state.branch) {
  const { g, a, b, f } = state;
  const A = { x: 0, y: 0 };
  const B = { x: g, y: 0 };
  const C = { x: a * Math.cos(alpha), y: a * Math.sin(alpha) };
  const dx = B.x - C.x;
  const dy = B.y - C.y;
  const d = Math.hypot(dx, dy);
  const eps = 1e-8;

  if (d < eps || d > f + b + eps || d < Math.abs(f - b) - eps) return null;

  const x = (f * f - b * b + d * d) / (2 * d);
  const h2 = f * f - x * x;
  if (h2 < -eps) return null;

  const h = Math.sqrt(Math.max(0, h2));
  const ux = dx / d;
  const uy = dy / d;
  const px = C.x + x * ux;
  const py = C.y + x * uy;
  const sign = branch >= 0 ? 1 : -1;
  const D = { x: px - sign * uy * h, y: py + sign * ux * h };
  const beta = Math.atan2(D.y - B.y, D.x - B.x);

  return { A, B, C, D, alpha, beta };
}

function sampleAngles() {
  const samples = [];
  const n = 720;
  for (let i = 0; i <= n; i += 1) {
    const alpha = (i / n) * TAU;
    const pose = solvePose(alpha, state.branch);
    if (pose) samples.push({ alpha, beta: normAngle(pose.beta) });
  }
  return samples;
}

function circularRange(values) {
  if (values.length < 2) return 0;
  const sorted = values.map(normAngle).sort((a, b) => a - b);
  let largestGap = 0;
  for (let i = 0; i < sorted.length; i += 1) {
    const current = sorted[i];
    const next = sorted[(i + 1) % sorted.length] + (i + 1 === sorted.length ? TAU : 0);
    largestGap = Math.max(largestGap, next - current);
  }
  return TAU - largestGap;
}

function meanAngle(values) {
  if (!values.length) return 0;
  const sx = values.reduce((acc, a) => acc + Math.cos(a), 0);
  const sy = values.reduce((acc, a) => acc + Math.sin(a), 0);
  return normAngle(Math.atan2(sy, sx));
}

function rockerDetail(values) {
  if (!values.length) return 'rocker';
  const center = meanAngle(values);
  const nearZero = Math.min(center, TAU - center);
  const nearPi = Math.abs(center - Math.PI);
  if (nearZero < Math.PI / 3) return '0-rocker';
  if (nearPi < Math.PI / 3) return 'pi-rocker';
  return 'rocker';
}

function stateClassification() {
  if (!isGloballyValid()) return { compact: 'invalid geometry', detailed: 'invalid geometry' };
  const samples = sampleAngles();
  if (samples.length < 4) return { compact: 'locked / no assembly', detailed: 'locked / no assembly' };

  const alphaRange = circularRange(samples.map(s => s.alpha));
  const betaRange = circularRange(samples.map(s => s.beta));
  const input = alphaRange > 1.86 * Math.PI ? 'crank' : rockerDetail(samples.map(s => s.alpha));
  const output = betaRange > 1.86 * Math.PI ? 'crank' : rockerDetail(samples.map(s => s.beta));
  const compactInput = input === 'crank' ? 'crank' : 'rocker';
  const compactOutput = output === 'crank' ? 'crank' : 'rocker';
  return {
    compact: `${compactInput} - ${compactOutput}`,
    detailed: `${input} - ${output}`,
  };
}

function updateValues() {
  ['g', 'a', 'b', 'f'].forEach(k => {
    state[k] = Number(els.sliders[k].value);
    els.values[k].textContent = `${fmt(state[k])} cm`;
  });
  state.speed = Number(els.speed.value);
}

function setBadge(el, text, mode) {
  el.textContent = text;
  el.className = `status-badge ${mode}`;
}

function updateMetrics() {
  const G = grashofIndex();
  const V = validityIndex();
  const { t1, t2, t3 } = excessValues();
  const cls = stateClassification();

  els.gIndex.textContent = fmt(G, 2);
  els.vIndex.textContent = fmt(V, 2);
  els.t1Index.textContent = fmt(t1, 2);
  els.t2Index.textContent = fmt(t2, 2);
  els.t3Index.textContent = fmt(t3, 2);

  setBadge(
    els.grashofBadge,
    isGrashof() ? 'Grashof mechanism' : 'Non-Grashof mechanism',
    isGrashof() ? 'ok' : 'warn'
  );

  setBadge(
    els.validityBadge,
    isGloballyValid() ? 'valid assemblable linkage' : 'invalid: longest link too long',
    isGloballyValid() ? 'ok' : 'bad'
  );

  setBadge(
    els.stateBadge,
    `${cls.compact}  ·  ${cls.detailed}`,
    cls.compact.includes('invalid') ? 'bad' : 'neutral'
  );
}

function getCanvasSize() {
  const box = els.canvasCard.getBoundingClientRect();
  const width = Math.max(620, Math.floor(box.width));
  const height = Math.max(440, Math.floor(box.height));
  return { width, height };
}

function fixedGroundTransform(width, height) {
  const maxSwing = Math.max(state.a, state.b, state.f);
  const totalWorldW = state.g + 2 * maxSwing + 16;
  const totalWorldH = 2 * maxSwing + 18;
  // A slightly more aggressive scale keeps the four-bar mechanism visually central
  // and larger inside the fixed-height animation pane without changing label sizes.
  const scale = Math.max(5, Math.min((width * 0.92) / totalWorldW, (height * 0.86) / totalWorldH, 24));
  const groundPixels = state.g * scale;
  const baseX = (width - groundPixels) / 2;
  const baseY = height * 0.55;

  return {
    scale,
    toScreen: p => ({ x: baseX + p.x * scale, y: baseY - p.y * scale }),
  };
}

function clearLayers() {
  [els.grid, els.support, els.angle, els.link, els.joint, els.label].forEach(layer => layer.replaceChildren());
}

function drawGrid(width, height) {
  const spacing = 48;
  for (let x = spacing; x < width; x += spacing) {
    els.grid.appendChild(svgElement('line', { class: 'grid-line', x1: x, y1: 0, x2: x, y2: height }));
  }
  for (let y = spacing; y < height; y += spacing) {
    els.grid.appendChild(svgElement('line', { class: 'grid-line', x1: 0, y1: y, x2: width, y2: y }));
  }
}

function addLine(layer, cls, p1, p2) {
  layer.appendChild(svgElement('line', { class: cls, x1: p1.x, y1: p1.y, x2: p2.x, y2: p2.y }));
}

function midpoint(p1, p2, offsetX = 0, offsetY = 0) {
  return { x: (p1.x + p2.x) / 2 + offsetX, y: (p1.y + p2.y) / 2 + offsetY };
}

function addText(layer, cls, p, text, anchor = 'middle') {
  layer.appendChild(svgElement('text', { class: cls, x: p.x, y: p.y, 'text-anchor': anchor }, text));
}

function drawGroundPivot(label, p) {
  const baseY = p.y + 42;
  const support = svgElement('path', {
    class: 'pivot-support',
    d: `M ${p.x - 34} ${baseY} L ${p.x + 34} ${baseY} L ${p.x} ${p.y + 9} Z`,
  });
  els.support.appendChild(support);
  for (let i = -30; i <= 22; i += 11) {
    els.support.appendChild(svgElement('line', {
      class: 'pivot-hatch',
      x1: p.x + i,
      y1: baseY + 8,
      x2: p.x + i + 12,
      y2: baseY + 22,
    }));
  }
  addText(els.label, 'point-label', { x: p.x, y: p.y - 28 }, label);
}

function drawJoint(label, p) {
  els.joint.appendChild(svgElement('circle', { class: 'joint-outer', cx: p.x, cy: p.y, r: 14 }));
  els.joint.appendChild(svgElement('circle', { class: 'joint-inner', cx: p.x, cy: p.y, r: 5 }));
  addText(els.label, 'point-label', { x: p.x + 25, y: p.y - 20 }, label, 'start');
}

function polar(center, radius, angle) {
  return { x: center.x + radius * Math.cos(angle), y: center.y - radius * Math.sin(angle) };
}

function arcPath(center, radius, startAngle, endAngle) {
  let end = endAngle;
  while (end < startAngle) end += TAU;
  const largeArc = end - startAngle <= Math.PI ? 0 : 1;
  const start = polar(center, radius, startAngle);
  const finish = polar(center, radius, end);
  return `M ${start.x} ${start.y} A ${radius} ${radius} 0 ${largeArc} 0 ${finish.x} ${finish.y}`;
}

function drawAngles(pose, screen, scale) {
  const A = screen.A;
  const B = screen.B;
  const r = Math.max(34, Math.min(76, scale * 2.2));
  const alpha = normAngle(pose.alpha);
  const beta = normAngle(pose.beta);

  els.angle.appendChild(svgElement('path', { class: 'angle-path-alpha', d: arcPath(A, r, 0, alpha) }));
  els.angle.appendChild(svgElement('path', { class: 'angle-path-beta', d: arcPath(B, r, 0, beta) }));

  const alphaLabelPoint = polar(A, r + 36, alpha / 2);
  const betaLabelPoint = polar(B, r + 40, beta / 2);
  addText(els.angle, 'angle-label alpha', alphaLabelPoint, `ALPHA = ${fmt(alpha * DEG)} deg`);
  addText(els.angle, 'angle-label beta', betaLabelPoint, `BETA = ${fmt(pose.beta * DEG)} deg`);
}

function drawInvalid(width, height) {
  clearLayers();
  drawGrid(width, height);
  els.link.appendChild(svgElement('rect', {
    class: 'invalid-overlay', x: 46, y: 46, width: width - 92, height: height - 92, rx: 30,
  }));
  els.label.appendChild(svgElement('text', {
    class: 'invalid-text', x: width / 2, y: height / 2 - 12,
  }, 'No real assembly for current input angle'));
  els.label.appendChild(svgElement('text', {
    class: 'invalid-text', x: width / 2, y: height / 2 + 36,
    style: 'font-size:20px;fill:#475569;'
  }, 'Change link lengths, switch branch, or press PLAY to search a valid pose.'));
}

function render() {
  updateValues();
  updateMetrics();

  const { width, height } = getCanvasSize();
  els.svg.setAttribute('viewBox', `0 0 ${width} ${height}`);
  const bg = els.svg.querySelector('.svg-bg');
  bg.setAttribute('width', width);
  bg.setAttribute('height', height);

  const pose = solvePose();
  if (!pose) {
    drawInvalid(width, height);
    els.angleReadout.textContent = 'ALPHA = no assembly · BETA = no assembly';
    return;
  }

  clearLayers();
  drawGrid(width, height);

  // This transform is intentionally independent of alpha.
  // Therefore A and B stay fixed on screen, and ground link g cannot rotate or drift during animation.
  const transform = fixedGroundTransform(width, height);
  const screen = {
    A: transform.toScreen(pose.A),
    B: transform.toScreen(pose.B),
    C: transform.toScreen(pose.C),
    D: transform.toScreen(pose.D),
  };

  drawGroundPivot('A', screen.A);
  drawGroundPivot('B', screen.B);

  addLine(els.link, 'ground-link', screen.A, screen.B);
  addLine(els.link, 'input-link', screen.A, screen.C);
  addLine(els.link, 'output-link', screen.B, screen.D);
  addLine(els.link, 'floating-link', screen.C, screen.D);

  drawAngles(pose, screen, transform.scale);

  drawJoint('A', screen.A);
  drawJoint('B', screen.B);
  drawJoint('C', screen.C);
  drawJoint('D', screen.D);

  addText(els.label, 'link-label', midpoint(screen.A, screen.B, 0, 44), 'g');
  addText(els.label, 'link-label', midpoint(screen.A, screen.C, 0, -26), 'a');
  addText(els.label, 'link-label', midpoint(screen.B, screen.D, 0, -26), 'b');
  addText(els.label, 'link-label', midpoint(screen.C, screen.D, 0, -26), 'f');

  els.angleReadout.textContent = `ALPHA = ${fmt(normAngle(pose.alpha) * DEG)}° · BETA = ${fmt(pose.beta * DEG)}°`;
}

function nearestValidAlpha(startAlpha) {
  const step = TAU / 360;
  for (let k = 0; k < 360; k += 1) {
    const plus = normAngle(startAlpha + k * step);
    const minus = normAngle(startAlpha - k * step);
    if (solvePose(plus)) return plus;
    if (solvePose(minus)) return minus;
  }
  return startAlpha;
}

function onSliderChange() {
  updateValues();
  if (!solvePose(state.alpha)) state.alpha = nearestValidAlpha(state.alpha);
  render();
}

function setPlaying(value) {
  state.playing = value;
  els.play.textContent = state.playing ? 'PAUSE' : 'PLAY';
  els.play.classList.toggle('pause', state.playing);
}

function animationFrame(time) {
  if (state.lastTime === null) state.lastTime = time;
  const dt = Math.min(0.05, (time - state.lastTime) / 1000);
  state.lastTime = time;

  if (state.playing) {
    const candidate = normAngle(state.alpha + state.direction * state.speed * dt * 1.25);
    if (solvePose(candidate)) {
      state.alpha = candidate;
    } else {
      state.direction *= -1;
      const bounce = normAngle(state.alpha + state.direction * state.speed * dt * 1.25);
      if (solvePose(bounce)) state.alpha = bounce;
      else state.alpha = nearestValidAlpha(state.alpha);
    }
    render();
  }
  requestAnimationFrame(animationFrame);
}

function bindEvents() {
  Object.values(els.sliders).forEach(slider => slider.addEventListener('input', onSliderChange));
  els.speed.addEventListener('input', () => { state.speed = Number(els.speed.value); });
  els.play.addEventListener('click', () => setPlaying(!state.playing));

  els.openBranch.addEventListener('click', () => {
    state.branch = 1;
    els.openBranch.classList.add('active');
    els.crossBranch.classList.remove('active');
    if (!solvePose(state.alpha)) state.alpha = nearestValidAlpha(state.alpha);
    render();
  });

  els.crossBranch.addEventListener('click', () => {
    state.branch = -1;
    els.crossBranch.classList.add('active');
    els.openBranch.classList.remove('active');
    if (!solvePose(state.alpha)) state.alpha = nearestValidAlpha(state.alpha);
    render();
  });

  window.addEventListener('resize', render);
  new ResizeObserver(render).observe(els.canvasCard);
}

function init() {
  bindEvents();
  render();
  requestAnimationFrame(animationFrame);
}

document.addEventListener('DOMContentLoaded', init);
