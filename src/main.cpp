#include <Arduino.h>
#include <Wire.h>
#include "PID/PID.h"
#include <AS5600.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <kf.h>

// WiFi credentials - CHANGE THESE TO YOUR NETWORK
const char* ssid = "Rob-Arm";         // Replace with your WiFi name
const char* password = "mypasswordisgood"; // Replace with your WiFi password

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Safety timeout variables
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 2000; // 2 seconds timeout
bool safetyActive = false;

// I2C synchronization variables
SemaphoreHandle_t i2cMutex;
bool i2cBusLocked = false;

#define ARM_SENSOR_SDA GPIO_NUM_16
#define ARM_SENSOR_SCL GPIO_NUM_17
#define ARM_WIRE Wire.begin(ARM_SENSOR_SDA,ARM_SENSOR_SCL);

#define END Wire.end();

#define WRIST_SENSOR_SDA GPIO_NUM_21
#define WRIST_SENSOR_SCL GPIO_NUM_22
#define WRIST_WIRE Wire.begin(WRIST_SENSOR_SDA,WRIST_SENSOR_SCL);


#define Motor0 GPIO_NUM_25
#define Motor0A1 GPIO_NUM_32
#define Motor0A2 GPIO_NUM_33

#define Motor1 GPIO_NUM_26
#define Motor1A1 GPIO_NUM_18
#define Motor1A2 GPIO_NUM_27

//#define Motor1 GPIO_NUM_17

#define I2C_Speed 1e5

#define freq 5000 // Hz
#define resolution 8 // bits

unsigned long DT,CT,PT,ET; // Loop time in ms

PID m0(0,0,0,&DT);
PID m1(0,0,0,&DT);

AS5600 Arm;
AS5600 Wrist;

float armOffset = -33.0f;
float wristOffset = 0.0f;

static int wristRotations{};

KF KFArm(0,0,0.5);

// Safe I2C reading functions with mutex protection
float readArmAngleSafe() {
  float angle = 0;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    ARM_WIRE
    if (Arm.detectMagnet()) {
      angle = fmod((Arm.readAngle() / 4096.0f * 360.0f), 360.0f) - 180.0f;

      Serial.println("Angle Before KF: ");
      Serial.println(angle);

      KFArm.predict(DT);
      KFArm.update(angle,0.5);

      angle = KFArm.pos();

      Serial.println("Angle After KF: ");
      Serial.println(angle);

    }
    Wire.end();
    xSemaphoreGive(i2cMutex);
  } else {
    Serial.println("Failed to acquire I2C mutex for ARM reading");
  }
  return angle;
}

float readWristAngleSafe() {
  float angle = 0;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    WRIST_WIRE
    if (Wrist.detectMagnet()) {
      angle =(Wrist.getCumulativePosition() / 4096.0f * 360.0f)/4.5f;
    }
    Wire.end();
    xSemaphoreGive(i2cMutex);
  } else {
    Serial.println("Failed to acquire I2C mutex for WRIST reading");
  }
  return angle;
}

// HTML page content
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>Motor PID Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="utf-8">
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&display=swap');
    
    * {
      box-sizing: border-box;
    }
    
    body {
      font-family: 'Orbitron', monospace;
      text-align: center; 
      margin: 0;
      padding: 20px;
      min-height: 100vh;
      color: #E0F2FE;
      background: linear-gradient(135deg, #1E3A8A 0%, #3B82F6 50%, #06B6D4 100%);
    }
    
    .container {
      max-width: 800px;
      margin: 0 auto;
      background: rgba(0, 0, 0, 0.2);
      border-radius: 20px;
      padding: 30px;
      backdrop-filter: blur(10px);
      border: 1px solid rgba(255, 255, 255, 0.1);
    }
    
    h1 {
      color: #FFF;
      margin-bottom: 30px;
      font-weight: 900;
      text-shadow: 0 0 20px #00FFFF;
    }
    
    .motor-section {
      background: rgba(255, 255, 255, 0.1);
      border-radius: 15px;
      padding: 25px;
      margin: 20px 0;
      border: 1px solid rgba(255, 255, 255, 0.2);
    }
    
    .motor-title {
      font-size: 1.5em;
      font-weight: 700;
      margin-bottom: 20px;
      color: #00FFFF;
    }
    
    .control-group {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 20px;
      margin: 20px 0;
    }
    
    .pid-group {
      background: rgba(0, 0, 0, 0.3);
      border-radius: 10px;
      padding: 15px;
    }
    
    label {
      display: block;
      margin-bottom: 8px;
      font-weight: 700;
      color: #FFF;
    }
    
    input[type="number"], input[type="range"] {
      width: 100%;
      padding: 8px;
      border: none;
      border-radius: 5px;
      background: rgba(255, 255, 255, 0.9);
      color: #000;
      font-family: 'Orbitron', monospace;
    }
    
    .angle-control {
      grid-column: span 3;
      background: rgba(0, 100, 200, 0.3);
      border-radius: 10px;
      padding: 20px;
      margin: 20px 0;
    }
    
    .slider-container {
      margin: 15px 0;
    }
    
    input[type="range"] {
      height: 8px;
      background: linear-gradient(90deg, #FF0000, #FFFF00, #00FF00, #00FFFF, #0000FF, #FF00FF, #FF0000);
      border-radius: 5px;
      outline: none;
    }
    
    input[type="range"]::-webkit-slider-thumb {
      appearance: none;
      width: 20px;
      height: 20px;
      border-radius: 50%;
      background: #FFF;
      cursor: pointer;
      box-shadow: 0 0 10px rgba(0, 255, 255, 0.5);
    }
    
    .angle-display {
      font-size: 1.2em;
      font-weight: 700;
      color: #00FFFF;
      margin: 10px 0;
    }
    
    .button {
      background: linear-gradient(45deg, #00FFFF, #0080FF);
      color: #000;
      border: none;
      padding: 12px 25px;
      border-radius: 25px;
      cursor: pointer;
      font-family: 'Orbitron', monospace;
      font-weight: 700;
      font-size: 1em;
      margin: 10px;
      transition: all 0.3s ease;
      box-shadow: 0 4px 15px rgba(0, 255, 255, 0.3);
    }
    
    .button:hover {
      transform: translateY(-2px);
      box-shadow: 0 6px 20px rgba(0, 255, 255, 0.5);
    }
    
    .status {
      background: rgba(0, 0, 0, 0.5);
      border-radius: 10px;
      padding: 15px;
      margin: 20px 0;
    }
    
    .current-values {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 20px;
      margin: 20px 0;
    }
    
    .value-display {
      background: rgba(0, 0, 0, 0.3);
      border-radius: 8px;
      padding: 10px;
    }
    
    @media (max-width: 768px) {
      .control-group {
        grid-template-columns: 1fr;
      }
      .current-values {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>🤖 Motor PID Control System</h1>
    
    <!-- ARM Motor Section -->
    <div class="motor-section">
      <div class="motor-title">🦾 ARM Motor Control</div>
      
      <div class="control-group">
        <div class="pid-group">
          <label for="armP">P Gain:</label>
          <input type="number" id="armP" step="0.1" value="20" min="0" max="100">
        </div>
        <div class="pid-group">
          <label for="armI">I Gain:</label>
          <input type="number" id="armI" step="0.1" value="15" min="0" max="100">
        </div>
        <div class="pid-group">
          <label for="armD">D Gain:</label>
          <input type="number" id="armD" step="0.1" value="0" min="0" max="100">
        </div>
      </div>
      
      <div class="angle-control">
        <label for="armAngle">Target Angle:</label>
        <div class="slider-container">
          <input type="range" id="armAngle" min="-180" max="180" value="0" oninput="updateAngleDisplay('arm', this.value)">
        </div>
        <div class="angle-display" id="armAngleDisplay">0°</div>
        <button class="button" onclick="applyArmSettings()">Apply ARM Settings</button>
      </div>
    </div>
    
    <!-- WRIST Motor Section -->
    <div class="motor-section">
      <div class="motor-title">🤏 WRIST Motor Control</div>
      
      <div class="control-group">
        <div class="pid-group">
          <label for="wristP">P Gain:</label>
          <input type="number" id="wristP" step="0.1" value="2" min="0" max="100">
        </div>
        <div class="pid-group">
          <label for="wristI">I Gain:</label>
          <input type="number" id="wristI" step="0.1" value="0" min="0" max="100">
        </div>
        <div class="pid-group">
          <label for="wristD">D Gain:</label>
          <input type="number" id="wristD" step="0.1" value="0" min="0" max="100">
        </div>
      </div>
      
      <div class="angle-control">
        <label for="wristAngle">Target Angle:</label>
        <div class="slider-container">
          <input type="range" id="wristAngle" min="-180" max="180" value="0" oninput="updateAngleDisplay('wrist', this.value)">
        </div>
        <div class="angle-display" id="wristAngleDisplay">0°</div>
        <button class="button" onclick="applyWristSettings()">Apply WRIST Settings</button>
      </div>
    </div>
    
    <!-- Status Section -->
    <div class="status">
      <h3>📊 Current Status</h3>
      <div class="current-values">
        <div class="value-display">
          <h4>ARM Motor</h4>
          <div style="font-size: 1.5em; color: #00FFFF; font-weight: bold;">Current: <span id="currentArmAngle">--</span>°</div>
          <div>Target: <span id="targetArmAngle">--</span>°</div>
          <div style="font-size: 0.8em; color: #888;">Last update: <span id="armLastUpdate">--</span></div>
        </div>
        <div class="value-display">
          <h4>WRIST Motor</h4>
          <div style="font-size: 1.5em; color: #00FFFF; font-weight: bold;">Current: <span id="currentWristAngle">--</span>°</div>
          <div>Target: <span id="targetWristAngle">--</span>°</div>
          <div style="font-size: 0.8em; color: #888;">Last update: <span id="wristLastUpdate">--</span></div>
        </div>
      </div>
      <div style="margin-top: 15px; padding: 10px; background: rgba(255,255,255,0.1); border-radius: 5px;">
        <div>Connection Status: <span id="connectionStatus" style="color: #00FF00;">Connecting...</span></div>
        <div>Updates Received: <span id="updateCount">0</span></div>
        <button onclick="emergencyStop()" style="width: 100%; margin-top: 10px; padding: 10px; background: linear-gradient(45deg, #DC2626, #EF4444); color: white; border: none; border-radius: 5px; font-weight: bold; cursor: pointer;">🚨 EMERGENCY STOP 🚨</button>
      </div>
    </div>
  </div>

  <script>
    let updateCount = 0;
    
    function updateAngleDisplay(motor, value) {
      document.getElementById(motor + 'AngleDisplay').innerText = value + '°';
    }
    
    function applyArmSettings() {
      const p = document.getElementById('armP').value;
      const i = document.getElementById('armI').value;
      const d = document.getElementById('armD').value;
      const angle = document.getElementById('armAngle').value;
      
      console.log('Sending ARM settings:', {p, i, d, angle});
      
      // Disable button during request
      const button = event.target;
      button.disabled = true;
      button.textContent = 'Applying...';
      
      // Create abort controller for timeout
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 8000);
      
      fetch('/setArmPID', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: `p=${p}&i=${i}&d=${d}&angle=${angle}`,
        signal: controller.signal
      })
      .then(response => {
        clearTimeout(timeoutId);
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        return response.text();
      })
      .then(data => {
        document.getElementById('targetArmAngle').innerText = angle;
        console.log('ARM response:', data);
        
        // Show success feedback
        button.style.background = 'linear-gradient(45deg, #10B981, #34D399)';
        button.textContent = 'Success!';
        setTimeout(() => {
          button.style.background = '';
          button.textContent = 'Apply ARM Settings';
        }, 2000);
      })
      .catch(error => {
        clearTimeout(timeoutId);
        console.error('Error:', error);
        
        let errorMsg = 'Connection error. ';
        if (error.name === 'AbortError') {
          errorMsg = 'Request timeout. ';
        }
        errorMsg += 'Check connection and try again.';
        
        // Show error feedback
        button.style.background = 'linear-gradient(45deg, #DC2626, #EF4444)';
        button.textContent = 'Error!';
        setTimeout(() => {
          button.style.background = '';
          button.textContent = 'Apply ARM Settings';
        }, 3000);
        
        alert(errorMsg);
      })
      .finally(() => {
        // Re-enable button
        button.disabled = false;
      });
    }
    
    function applyWristSettings() {
      const p = document.getElementById('wristP').value;
      const i = document.getElementById('wristI').value;
      const d = document.getElementById('wristD').value;
      const angle = document.getElementById('wristAngle').value;
      
      console.log('Sending WRIST settings:', {p, i, d, angle});
      
      // Disable button during request
      const button = event.target;
      button.disabled = true;
      button.textContent = 'Applying...';
      
      // Create abort controller for timeout
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 8000);
      
      fetch('/setWristPID', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: `p=${p}&i=${i}&d=${d}&angle=${angle}`,
        signal: controller.signal
      })
      .then(response => {
        clearTimeout(timeoutId);
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        return response.text();
      })
      .then(data => {
        document.getElementById('targetWristAngle').innerText = angle;
        console.log('WRIST response:', data);
        
        // Show success feedback
        button.style.background = 'linear-gradient(45deg, #10B981, #34D399)';
        button.textContent = 'Success!';
        setTimeout(() => {
          button.style.background = '';
          button.textContent = 'Apply WRIST Settings';
        }, 2000);
      })
      .catch(error => {
        clearTimeout(timeoutId);
        console.error('Error:', error);
        
        let errorMsg = 'Connection error. ';
        if (error.name === 'AbortError') {
          errorMsg = 'Request timeout. ';
        }
        errorMsg += 'Check connection and try again.';
        
        // Show error feedback
        button.style.background = 'linear-gradient(45deg, #DC2626, #EF4444)';
        button.textContent = 'Error!';
        setTimeout(() => {
          button.style.background = '';
          button.textContent = 'Apply WRIST Settings';
        }, 3000);
        
        alert(errorMsg);
      })
      .finally(() => {
        // Re-enable button
        button.disabled = false;
      });
    }
    
    // Update current angles every second
    function updateAngles() {
      // Create abort controller for timeout
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 3000);
      
      fetch('/getAngles', {
        method: 'GET',
        signal: controller.signal
      })
        .then(response => {
          clearTimeout(timeoutId);
          if (!response.ok) {
            throw new Error(`Network response was not ok: ${response.status}`);
          }
          return response.json();
        })
        .then(data => {
          console.log('Received angle data:', data);
          
          // Validate data
          if (typeof data.armAngle === 'number' && typeof data.wristAngle === 'number') {
            updateCount++;
            
            document.getElementById('currentArmAngle').innerText = data.armAngle.toFixed(1);
            document.getElementById('currentWristAngle').innerText = data.wristAngle.toFixed(1);
            document.getElementById('armLastUpdate').innerText = new Date().toLocaleTimeString();
            document.getElementById('wristLastUpdate').innerText = new Date().toLocaleTimeString();
            document.getElementById('updateCount').innerText = updateCount;
            
            // Update connection status
            document.getElementById('connectionStatus').innerText = 'Connected';
            document.getElementById('connectionStatus').style.color = '#10B981';
          } else {
            throw new Error('Invalid data format received');
          }
        })
        .catch(error => {
          clearTimeout(timeoutId);
          console.error('Error fetching angles:', error);
          
          if (error.name === 'AbortError') {
            document.getElementById('connectionStatus').innerText = 'Request Timeout';
          } else {
            document.getElementById('connectionStatus').innerText = 'Connection Error';
          }
          document.getElementById('connectionStatus').style.color = '#EF4444';
        });
    }
    
    // Emergency stop function
    function emergencyStop() {
      console.log('EMERGENCY STOP ACTIVATED!');
      
      fetch('/emergency', {
        method: 'GET'
      })
      .then(response => {
        if (response.ok) {
          document.getElementById('connectionStatus').innerText = 'Emergency Stop Active';
          document.getElementById('connectionStatus').style.color = '#EF4444';
          alert('Emergency stop activated! Motors stopped.');
        }
      })
      .catch(error => {
        console.error('Emergency stop error:', error);
        alert('Emergency stop request failed! Check connection.');
      });
    }
    
    // Start updating angles immediately and then every second
    updateAngles();
    setInterval(updateAngles, 1000);
    
    // Test connection on load
    window.onload = function() {
      console.log('Page loaded, testing connection...');
      updateAngles();
    };
  </script>
</body>
</html>
)rawliteral";

void scan_4_I2C(){
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("Done\n");
  }
}

void setup() {
 Serial.begin(115200);
 delay(100);

 // Create I2C mutex for thread safety
 i2cMutex = xSemaphoreCreateMutex();
 if (i2cMutex == NULL) {
   Serial.println("Failed to create I2C mutex!");
   while(1); // halt if mutex creation fails
 }

 // Initialize WiFi
 WiFi.mode(WIFI_AP);
 WiFi.softAP(ssid, password);

 Serial.println("");
 Serial.println("WiFi Setup Complete.");
 Serial.println("Access Point Created");
 Serial.print("SSID: ");
 Serial.println(ssid);
 Serial.print("Password: ");
 Serial.println(password);
 Serial.print("Website URL: http://");
 Serial.println(WiFi.softAPIP());
 Serial.println("Connect your device to the WiFi network above and navigate to the URL");

 delay(3000);

 ARM_WIRE
 Serial.print("\nScanning ArmWire | Pins: ");
 Serial.print(ARM_SENSOR_SDA);
 Serial.print(" ");
 Serial.println(ARM_SENSOR_SCL);
 scan_4_I2C();
 END

 delay(1000);

 WRIST_WIRE
 Serial.print("\nScanning WristWire | Pins: ");
 Serial.print(WRIST_SENSOR_SDA);
 Serial.print(" ");
 Serial.println(WRIST_SENSOR_SCL);
 scan_4_I2C();
 END

 delay(1000);
 
 WRIST_WIRE
 //TODO: Check if sensor remembers assigned address. It Does not.
 Arm.begin();
 Arm.setDirection(AS5600_COUNTERCLOCK_WISE);  //  default, just be explicit.
 END

 ARM_WIRE
 Wrist.begin();
 Wrist.setDirection(AS5600_COUNTERCLOCK_WISE);  //  default, just be explicit.
 END
  
 ARM_WIRE
 if (!Arm.detectMagnet()) Serial.println("Arm Magnet Not Detected, Check if magnet is too far away or missing");
 if (Arm.magnetTooWeak()) Serial.println("Arm Magnet too weak, move it closer");
 if (Arm.magnetTooStrong()) Serial.println("Arm Magnet too strong, move it away");
  Arm.setOffset(armOffset);
 END

 WRIST_WIRE
 if (!Wrist.detectMagnet()) Serial.println("Wrist Magnet Not Detected, Check if magnet is too far away or missing");
 if (Wrist.magnetTooWeak()) Serial.println("Wrist Magnet too weak, move it closer");
 if (Wrist.magnetTooStrong()) Serial.println("Wrist Magnet too strong, move it away");

 Wrist.setOffset(wristOffset);
 Wrist.resetCumulativePosition();
 END

 pinMode(Motor0,OUTPUT);
 pinMode(Motor0A1, OUTPUT);
 pinMode(Motor0A2, OUTPUT);

 digitalWrite(Motor0A1,HIGH);
 digitalWrite(Motor0A2,LOW);
 
 pinMode(Motor1,OUTPUT);
 pinMode(Motor1A1, OUTPUT);
 pinMode(Motor1A2, OUTPUT);

 digitalWrite(Motor1A1,HIGH);
 digitalWrite(Motor1A2,LOW);

 ledcSetup(0,freq,resolution);
 ledcAttachPin(Motor0,0);
 ledcSetup(1,freq,resolution);
 ledcAttachPin(Motor1,1);

  
  // ARM_WIRE
  // float armAngle = Arm.readAngle()*ang2deg;
  // m0.setSetpoint(armAngle);
  // Wire.end();

  // WRIST_WIRE
  // float wristAngle = Wrist.readAngle()*ang2deg;
  // m1.setSetpoint(wristAngle);
  // Wire.end();

  // delay(10000);

  // Setup web server routes
  // Serve the main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Web page requested!");
    request->send(200, "text/html", index_html);
  });

  // Handle ARM PID settings
  server.on("/setArmPID", HTTP_POST, [](AsyncWebServerRequest *request){
    Serial.println("ARM PID request received");
    
    if (request->hasParam("p", true) && request->hasParam("i", true) && 
        request->hasParam("d", true) && request->hasParam("angle", true)) {
      
      float p = request->getParam("p", true)->value().toFloat();
      float i = request->getParam("i", true)->value().toFloat();
      float d = request->getParam("d", true)->value().toFloat();
      float angle = request->getParam("angle", true)->value().toFloat();
      
      // Validate ranges
      p = constrain(p, 0, 1000);
      i = constrain(i, 0, 1000);
      d = constrain(d, 0, 1);
      angle = constrain(angle, -180, 180);
      
      // // Add delay to prevent I2C conflicts during PID updates
      // delay(10);
      
      m0.setP(p);
      m0.setI(i);
      m0.setD(d);
      m0.setSetpoint(angle);
      m0.reset();
      
      Serial.printf("ARM PID updated: P=%.2f, I=%.2f, D=%.2f, Angle=%.2f\n", p, i, d, angle);
      
      // Small delay before responding
      // delay(5);
      request->send(200, "text/plain", "ARM settings applied successfully");
    } else {
      Serial.println("ARM PID update failed: Missing parameters");
      request->send(400, "text/plain", "Missing parameters");
    }
  });

  // Handle WRIST PID settings
  server.on("/setWristPID", HTTP_POST, [](AsyncWebServerRequest *request){
    Serial.println("WRIST PID request received");
    
    if (request->hasParam("p", true) && request->hasParam("i", true) && 
        request->hasParam("d", true) && request->hasParam("angle", true)) {
      
      float p = request->getParam("p", true)->value().toFloat();
      float i = request->getParam("i", true)->value().toFloat();
      float d = request->getParam("d", true)->value().toFloat();
      float angle = request->getParam("angle", true)->value().toFloat();
      
      // Validate ranges
      p = constrain(p, 0, 100);
      i = constrain(i, 0, 100);
      d = constrain(d, 0, 100);
      angle = constrain(angle, -180, 180);
      
      // Add delay to prevent I2C conflicts during PID updates
      // delay(10);
      
      m1.setP(p);
      m1.setI(i);
      m1.setD(d);
      m1.setSetpoint(angle);
      m1.reset();
      
      Serial.printf("WRIST PID updated: P=%.2f, I=%.2f, D=%.2f, Angle=%.2f\n", p, i, d, angle);
      
      // Small delay before responding
      // delay(5);
      request->send(200, "text/plain", "WRIST settings applied successfully");
    } else {
      Serial.println("WRIST PID update failed: Missing parameters");
      request->send(400, "text/plain", "Missing parameters");
    }
  });

  // Get current angles endpoint
  server.on("/getAngles", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Angles requested via web!");
    
    // Use static variables to cache readings and prevent excessive I2C access
    static float lastArmAngle = 0;
    static float lastWristAngle = 0;
    static unsigned long lastReadTime = 0;
    
    // Rate limit to prevent overwhelming I2C bus - only read every 200ms
    if (millis() - lastReadTime > 200) {
      // Use thread-safe I2C reading functions
      lastArmAngle = readArmAngleSafe();
      // delay(10); // Small delay between I2C operations
      lastWristAngle = readWristAngleSafe();
      
      lastReadTime = millis();
      Serial.println("Sensor readings updated safely via web request");
    }
    
    String json = "{\"armAngle\":" + String(lastArmAngle, 2) + 
                  ",\"wristAngle\":" + String(lastWristAngle, 2) + 
                  ",\"safetyActive\":false}";
    Serial.print("Sending JSON: ");
    Serial.println(json);
    
    // Set proper headers for JSON response
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", json);
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("Cache-Control", "no-cache");
    request->send(response);
  });

  // Emergency stop endpoint (manual stop only)
  server.on("/emergency", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("EMERGENCY STOP ACTIVATED!");
    
    // Stop both motors immediately
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    
    // Reset PID controllers to stop any integration buildup
    m0.reset();
    m1.reset();
    
    request->send(200, "text/plain", "Emergency stop activated");
  });

  // Start server
  server.begin();
  Serial.println("Web server started!");
  Serial.print("Open http://");
  Serial.print(WiFi.softAPIP());
  Serial.println(" in your browser");

}

void loop() {
  CT=micros();
  DT=CT-PT;
  PT=CT;

  // Safety timeout disabled - continuous operation mode
  // Note: Safety timeout system has been disabled per user request
  // Motors will run continuously based on PID setpoints

  // Add a small delay to prevent overwhelming the I2C bus
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead < 20) {  // Limit sensor reads to 50Hz max
    // delay(1);
    return;
  }
  lastSensorRead = millis();

  // Use thread-safe I2C reading functions
  float armAngle = readArmAngleSafe();
  // Serial.print("Arm Angle : ");
  // Serial.println(armAngle,2);

  // delay(5);

  float wristAngle = readWristAngleSafe();
  Serial.print("Wrist Angle : ");
  // Serial.println(wristAngle,2);

  // Apply motor control without safety checks
  // Normal operation - compute PID and control motors
  float m0_corr = m0.compute(armAngle);
  m0_corr = constrain(m0_corr,-255, 255);
  if (m0_corr > 0){
    digitalWrite(Motor0A1,LOW);
    digitalWrite(Motor0A2,HIGH);
    //ClockWise
  }
  else {
    digitalWrite(Motor0A1,HIGH);
    digitalWrite(Motor0A2,LOW);
    //Counter Clock Wise
  }
  ledcWrite(0,(int)abs(m0_corr));

  float m1_corr = m1.compute(wristAngle);
  m1_corr = constrain(m1_corr,-255, 255);
  if (m1_corr > 0){
    digitalWrite(Motor1A1,LOW);
    digitalWrite(Motor1A2,HIGH);
    //Counter Clock Wise
  }
  else {
    digitalWrite(Motor1A1,HIGH);
    digitalWrite(Motor1A2,LOW);
    //ClockWise
  }
  ledcWrite(1,(int)abs(m1_corr));
  
  delay(10); // Small delay to prevent system overload
}