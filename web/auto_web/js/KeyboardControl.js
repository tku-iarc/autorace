window.addEventListener("keydown", keysdown, false);
window.addEventListener("keyup", keyuped, false);

var keys = [];
var start = false;
var vec3 = new ROSLIB.Message({
        x: 0,
        y: 0,
        z: 0
    });

function KeyboardState(state) {
    start = state;
}


function keysdown(e) {
   
    if (start == true) {
        var speed = document.getElementById("SpeedInput").value;
        keys[e.keyCode] = true;

        
        //RobotControl
        //w+d
        if (keys[87] && keys[68]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed/100),
                y: 0,
                z: -parseFloat(speed*1.5/100)
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
            document.getElementById('direction').innerText = "右前";
        } 
        //w+a
        else if (keys[87] && keys[65]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed/100),
                y: 0,
                z: parseFloat(speed*1.5/100)
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
            document.getElementById('direction').innerText = "左前";
        } 
        //s+d
        else if (keys[83] && keys[68]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed/100),
                y: 0,
                z: -parseFloat(speed*1.5/100)
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
            document.getElementById('direction').innerText = "右後";
        } 
        //s+a
        else if (keys[83] && keys[65]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed/100),
                y: 0,
                z: parseFloat(speed*1.5/100)
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
            document.getElementById('direction').innerText = "左後";
        } 
        //w
        else if (keys[87]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed/100),
                y: 0,
                z: 0
            });
            PublishTopicCmdVel(vec3);
           // PublishTopicCmdVel(vec3);
           document.getElementById('direction').innerText = "前進";
        } 
        //d
        else if (keys[68]) {
            vec3 = new ROSLIB.Message({
                x: 0,
                y: 0,
                z: -parseFloat(speed*1.5/100)
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
            document.getElementById('direction').innerText = "右轉";
        }
        //s
        else if (keys[83]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed/100),
                y: 0,
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
            document.getElementById('direction').innerText = "後退";
        } 
        //a
        else if (keys[65]) {
            vec3 = new ROSLIB.Message({
                x: 0,
                y: 0,
                z: parseFloat(speed*1.5/100)
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
            document.getElementById('direction').innerText = "左轉";
        }
        //o start
        else if (keys[79]){
            strategy_start(1);
            $('#StartInput').prop('checked',true);
            $('#StartInput').change();
            $('#StopInput').prop('checked',false);
            $('#StopInput').change();
        }
        //p stop
        else if (keys[80]){
            strategy_start(0);
            $('#StartInput').prop('checked',false);
            $('#StartInput').change();
            $('#StopInput').prop('checked',true);
            $('#StopInput').change();
        }
        //[ open arm
        else if (keys[219]){
            arm(0);
        }//] close arm
        else if (keys[221]){
            arm(1);
        }// / shoot
        else if (keys[191]){
            PublishTopicshoot();
        }
        /*
        //m disable
        else if (keys[77]){
            PublishTopicshootstrategy(0);
            $('#disable_shoot').prop('checked',true);
            $('#disable_shoot').change();
            SendMsgss('disable');
            
        }
        */
        //, short
        else if (keys[188]){
            PublishTopicshootstrategy(1);
            $('#short_shoot').prop('checked',true);
            $('#short_shoot').change();
            SendMsgss('short');
        }
        //. far 
        else if (keys[190]){
            PublishTopicshootstrategy(2);
            $('#far_shoot').prop('checked',true);
            $('#far_shoot').change();
            SendMsgss('far');
        }
        //j red
        else if (keys[74]){
            if(document.getElementById('Red_disable').checked == true){
                document.getElementById('Red_disable').checked = false;
                SendMsgss('RED ENABLE');
                $('#Red_disable').prop('checked',false);
                $('#Red_disable').change();
                Color_Disable(1);
            }else {
                document.getElementById('Red_disable').checked = true;
                SendMsgss('RED DISABLE');
                $('#Red_disable').prop('checked',true);
                $('#Red_disable').change();
                Color_Disable(1);
            }
            
        }
        //k blue
        else if (keys[75]){
            if(document.getElementById('Blue_disable').checked == true){
                document.getElementById('Blue_disable').checked = false;
                SendMsgss('BLUE ENABLE');
                $('#Blue_disable').prop('checked',false);
                $('#Blue_disable').change();
                Color_Disable(2);
            }else {
                document.getElementById('Blue_disable').checked = true;
                SendMsgss('BLUE DISABLE');
                $('#Blue_disable').prop('checked',true);
                $('#Blue_disable').change();
                Color_Disable(2);
            }
            
        }
        //l yellow
        else if (keys[76]){
            if(document.getElementById('Yellow_disable').checked == true){
                document.getElementById('Yellow_disable').checked = false;
                SendMsgss('YELLOW ENABLE');
                $('#Yellow_disable').prop('checked',false);
                $('#Yellow_disable').change();
                Color_Disable(3);
            }else {
                document.getElementById('Yellow_disable').checked = true;
                SendMsgss('YELLOW DISABLE');
                $('#Yellow_disable').prop('checked',true);
                $('#Yellow_disable').change();
                Color_Disable(3);
            }
        }
    }
}

function releasebutton(state) {
    let vec3_ = new ROSLIB.Message({
        x: 0,
        y: 0,
        z: 0
    });
    switch(state){
      case 81:
        vec3.z = 0;
        break;
      case 69:
        vec3.z = 0;
        break;
      case 87:
        vec3.y = 0;
        break;
      case 65:
        vec3.x = 0;
        break;
      case 83:
        vec3.y = 0;
        break;
      case 68:
        vec3.x = 0;
        break;
      default:
        vec3.x = 0;
        vec3.y = 0;
        vec3.Z = 0;
    }
    PublishTopicCmdVel(vec3_);
    document.getElementById('direction').innerText = "停止";
    //PublishTopicCmdVel(vec3);
}

function keyuped(e) {
    if (start) {
        if (keys[e.keyCode] == true) releasebutton(e.keyCode);
        //else if (keys[69] == true) releasebutton(69);
        //else if (keys[87] == true) releasebutton(87);
        //else if (keys[65] == true) releasebutton(65);
        //else if (keys[83] == true) releasebutton(83);
        //else if (keys[68] == true) releasebutton(68);
        keys[e.keyCode] = false;
    }
}
