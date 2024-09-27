
var container = document.getElementById('magrot-card');
//var size = Math.min(container.offsetWidth, container.offsetHeight);
var size = container.offsetWidth;

var gaugeMagRot = new RadialGauge({
  renderTo: 'gauge-magrot',
  width: size,
  height: size,
//    units: "degrees",
  minValue: -90,
  maxValue: 90,
  startAngle: 90,
  ticksAngle: 180,
//    colorValueBoxRect: "#049faa",
//    colorValueBoxRectEnd: "#049faa",
//    colorValueBoxBackground: "#f1fbfc",
  valueInt: 2,
  valueBox: true,
  majorTicks: [
      "-90",
      "0",
      "90"
  ],
  minorTicks: 4,
  strokeTicks: true,
  highlights: [
      {
          "from": -50,
          "to": 50,
          "color": "#D3D3D3"
      }
  ],
  colorPlate: "#fff",
  borderShadowWidth: 0,
  borders: false,
  needleType: "arrow",
  colorNeedle: "#0A1128",
  colorNeedleEnd: "#D3D3D3",
  needleWidth: 3,
  needleCircleSize: 5,
  colorNeedleCircleOuter: "#D3D3D3",
  needleCircleOuter: true,
  needleCircleInner: false,
  animationDuration: 10,
  animationRule: "linear",
  title: "Mast"
}).draw();
