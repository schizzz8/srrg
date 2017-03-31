// orazio JS simple javascript thing
var ws = new WebSocket('ws://'+self.location.host+'/','orazio-robot-protocol');
refresh_toggled=true;

ws.onmessage = function(event) {
    if (refresh_toggled==true){
        document.getElementById('msgBox').innerHTML = event.data;
	document.getElementById('outMsg').value='';
    }
}

function send()
{
    ws.send(document.getElementById('outMsg').value);
    refresh_toggled=true;
}

function stopRefresh(){
    refresh_toggled=false;
}

function sendMotorMode()
{
    ws.send("setMotorMode "+document.getElementById('motor_mode').value);
    refresh_toggled=true;
}

function sendWatchdogCycles()
{
    ws.send("setWatchdogCycles "+document.getElementById('watchdog_cycles').value);
    refresh_toggled=true;
}

function sendCommCycles()
{
    ws.send("setCommCycles "+document.getElementById('comm_cycles').value);
    refresh_toggled=true;
}


function sendKp0()
{
    ws.send("setKp 0 "+document.getElementById('kp_0').value);
    refresh_toggled=true;
}

function sendKi0()
{
    ws.send("setKi 0  "+document.getElementById('ki_0').value);
    refresh_toggled=true;
}

function sendKd0()
{
    ws.send("setKd 0  "+document.getElementById('kd_0').value);
    refresh_toggled=true;
}

function sendMaxSpeed0()
{
    ws.send("setMaxSpeed 0  "+document.getElementById('max_speed_0').value);
    refresh_toggled=true;
}

function sendMinPWM0()
{
    ws.send("setMinPWM 0  "+document.getElementById('min_pwm_0').value);
    refresh_toggled=true;
}

function sendMaxPWM0()
{
    ws.send("setMaxPWM 0  "+document.getElementById('max_pwm_0').value);
    refresh_toggled=true;
}

function sendSlope0()
{
    ws.send("setSlope 0  "+document.getElementById('slope_0').value);
    refresh_toggled=true;
}

function sendKp1()
{
    ws.send("setKp 1 "+document.getElementById('kp_1').value);
    refresh_toggled=true;
}

function sendKi1()
{
    ws.send("setKi 1  "+document.getElementById('ki_1').value);
    refresh_toggled=true;
}

function sendKd1()
{
    ws.send("setKd 1  "+document.getElementById('kd_1').value);
    refresh_toggled=true;
}

function sendMaxSpeed1()
{
    ws.send("setMaxSpeed 1  "+document.getElementById('max_speed_1').value);
    refresh_toggled=true;
}

function sendMinPWM1()
{
    ws.send("setMinPWM 1  "+document.getElementById('min_pwm_1').value);
    refresh_toggled=true;
}

function sendMaxPWM1()
{
    ws.send("setMaxPWM 1  "+document.getElementById('max_pwm_1').value);
    refresh_toggled=true;
}

function sendSlope1()
{
    ws.send("setSlope 1  "+document.getElementById('slope_1').value);
    refresh_toggled=true;
}

function sendRightMotorIndex(){
    ws.send("setRightMotorIndex "+document.getElementById('right_motor_index').value);
    refresh_toggled=true;
}

function sendLeftMotorIndex(){
    ws.send("setLeftMotorIndex "+document.getElementById('left_motor_index').value);
    refresh_toggled=true;
}

function sendKr(){
    ws.send("setKr "+document.getElementById('kr').value);
    refresh_toggled=true;
}

function sendKl(){
    ws.send("setKl "+document.getElementById('kl').value);
    refresh_toggled=true;
}

function sendInverseKr(){
    var kl=1.0/document.getElementById('ikr').value;
    ws.send("setKr "+kl);
    refresh_toggled=true;
}

function sendInverseKl(){
    var kl=1.0/document.getElementById('ikl').value;
    ws.send("setKl "+kl);
    refresh_toggled=true;
}

function sendBartoloKl(){
    var kl=2*Math.PI/65536.0*document.getElementById('bart_kl').value;
    ws.send("setKl "+kl);
    refresh_toggled=true;
}

function sendBartoloKr(){
    var kr=2*Math.PI/65536.0*document.getElementById('bart_kr').value;
    ws.send("setKr "+kr);
    refresh_toggled=true;
}

function sendBaseline(){
    ws.send("setBaseline "+document.getElementById('baseline').value);
    refresh_toggled=true;
}

function sendMaxTV(){
    ws.send("setMaxTranslationalVelocity "+document.getElementById('max_tv').value);
    refresh_toggled=true;
}

function sendMaxTA(){
    ws.send("setMaxTranslationalAcceleration "+document.getElementById('max_ta').value);
    refresh_toggled=true;
}

function sendMaxTBrake(){
    ws.send("setMaxTranslationalBrake "+document.getElementById('max_tb').value);
    refresh_toggled=true;
}

function sendMaxRV(){
    ws.send("setMaxRotationalVelocity "+document.getElementById('max_rv').value);
    refresh_toggled=true;
}

function sendMaxRA(){
    ws.send("setMaxRotationalAcceleration "+document.getElementById('max_ra').value);
    refresh_toggled=true;
}

function sendPWM(){
    ws.send("setJointsPWM "+document.getElementById('joints_pwm').value);
    refresh_toggled=true;
}
function sendSpeed(){
    ws.send("setJointsSpeed "+document.getElementById('joints_speed').value);
    refresh_toggled=true;  
}
function sendBaseVelocities(){
    ws.send("setBaseVelocities "+document.getElementById('base_velocities').value);
    refresh_toggled=true;  
}
function sendStop(){
    ws.send("setPWMSpeed 0 0");
    refresh_toggled=true;  
}

function saveParams(){
    ws.send("pushParams 0");
    ws.send("pushParams 1");
    ws.send("pushParams 2");
    ws.send("saveParams 0");
    ws.send("saveParams 1");
    ws.send("saveParams 2");
    refresh_toggled=true;  
}
