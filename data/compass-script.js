const compassCircle = document.querySelector(".compass-circle");
const myPoint = document.querySelector(".my-point");
const startBtn = document.querySelector(".start-btn");

window.addEventListener('load', getReadings);

function init() {
  window.addEventListener("deviceorientationabsolute", handler, true);
}

function handler(e) {
  compass = e.webkitCompassHeading || Math.abs(e.alpha - 360);
  compassCircle.style.transform = `translate(-50%, -50%) rotate(${-compass}deg)`;
}

// Function to get current readings on the webpage when it loads for the first time
function getReadings(){
    var xhr = new XMLHttpRequest();
    // we've created a request which we'll send to server://readings to get back the sensor/compass readings
    // but first, we will attach a function to the request, to be executed when 'onreadystatechange'
    // is triggered
    // we're looking for the XHR object's readyState property to be ==4, which indicates that the request
    // finished and we got a response
    xhr.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        var myObj = JSON.parse(this.responseText);
        console.log(myObj);
        var bearing = myObj.bearing.toFixed(1);
        // we know the JSON object has a 'bearing' property because we set it in esp32 getSensorReadings()
        compassCircle.style.transform = `translate(-50%, -50%) rotate(${-bearing}deg)`;
        let imageElement = document.getElementById("calibration");
        switch (myObj.calstatus) {
          case 0:
            imageElement.src = "red.png";
            break;
          case 1:
            imageElement.src = "orange.png";
            break;
          case 2:
            imageElement.src = "yellow.png";
            break;
          case 3:
            imageElement.src = "green.png";
            break;
          default:
            imageElement.src = "";
            break;
        }
      }
    }; 
    xhr.open("GET", "/readings", true);
    xhr.send();
  }

  if (!!window.EventSource) {
    var source = new EventSource('/events');
    
    source.addEventListener('open', function(e) {
      console.log("Events Connected");
    }, false);
  
    source.addEventListener('error', function(e) {
      if (e.target.readyState != EventSource.OPEN) {
        console.log("Events Disconnected");
      }
    }, false);
    
    source.addEventListener('message', function(e) {
      console.log("message", e.data);
    }, false);
    
    source.addEventListener('new_readings', function(e) {
      console.log("new_readings", e.data);
      var myObj = JSON.parse(e.data);
      console.log(myObj);
      var bearing = myObj.bearing.toFixed(1);
      compassCircle.style.transform = `translate(-50%, -50%) rotate(${-bearing}deg)`;
      document.getElementById('bearing').innerHTML = bearing;
    }, false);
  }