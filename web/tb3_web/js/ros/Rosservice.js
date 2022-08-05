function sleep(delay) {
    var start = new Date().getTime();
    while (new Date().getTime() < start + delay);
}

function SendMsgss(Msg,color) 
{ 
  let  d = new Date();
  let n = d.getHours();
  let m = d.getMinutes();
  let s = d.getSeconds();

  //var MSg= $("msg").value; 
  var newFontElem = document.createElement("font");
  if(color == "red"){
    newFontElem.style.color = color;
  }
  if(color == "blue"){
    newFontElem.style.color = color;
  }
  newFontElem.innerHTML = '['+n+':'+m+':'+s+'] '+Msg+"<br/>"; 
  document.getElementById("info").appendChild(newFontElem); 

  document.getElementById("info").scrollTop= document.getElementById("info").scrollHeight;//每次收到消息滾動條拉到最下面
} 
 


var paramClient = new ROSLIB.Service({
  ros : ros,
  name : '/tb3/save',
  serviceType : 'std_srvs/Empty'
});

var param_request = new ROSLIB.ServiceRequest({
});

function savecall(){
  paramClient.callService(param_request,
    function(param_request) {
      console.log('成功儲存參數');
      SendMsgss('成功儲存參數');
      //callback(result.action_servers);
    },
    function(message){
      console.log(message);
      SendMsgss('儲存參數失敗',"red");
    }
  );
}
//=====================================
var paramstrategyClient = new ROSLIB.Service({
  ros : ros,
  name : '/tb3/strategy/save',
  serviceType : 'std_srvs/Empty'
});

var paramstrategy_request = new ROSLIB.ServiceRequest({
});

function strategy_savecall(){
  paramstrategyClient.callService(paramstrategy_request,
    function(paramstrategy_request) {
      console.log('更新策略參數成功');
      SendMsgss('更新策略參數成功');
      //callback(result.action_servers);
    },
    function(message){
      console.log(message);
      SendMsgss('更新策略參數失敗',"red");
    }
  );
}
//=====================================

var runClient = new ROSLIB.Service({
  ros : ros,
  name : '/tb3/connect',
  serviceType : 'std_srvs/Empty'
});

var run_request = new ROSLIB.ServiceRequest({
});

let connected = false;

function run(){
  setTimeout(run, 3000);
  view_checked = document.getElementById('Camera-View').checked;
  if(view_checked){
    runClient.callService(run_request,
      function(run_request) {
        if(connected == false){
          document.getElementById('Red_disable').checked = false;
          document.getElementById('Blue_disable').checked = false;
          document.getElementById('Yellow_disable').checked = false;
          //document.getElementById('White_disable').checked = false;
          //document.getElementById('Black_disable').checked = false;
          console.log('程式連接成功');
          SendMsgss('程式連接成功',"blue");
          connected = true;
        }
        //callback(result.action_servers);
      },
      function(message){
        if(connected == true){
          console.log('程式連接中斷');
          SendMsgss('程式連接中斷',"red");
          //console.log(message);
          connected = false;
        }
        else{
          console.log('無法連接程式端');
          SendMsgss('無法連接程式端',"red");
        }
      }
    );
  }
}
