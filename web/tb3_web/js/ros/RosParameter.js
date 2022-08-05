//===================================================================
// HSV 
var ParameterHSV_Red = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/HSV/Red',
});
var ParameterHSV_Blue = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/HSV/Blue',
});
var ParameterHSV_Yellow = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/HSV/Yellow',
});
var ParameterHSV_White = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/HSV/White',
});
var ParameterHSV_Black = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/HSV/Black',
});
var ParameterHSV_RedGoal = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/HSV/RedGoal',
});

var ParameterHSV_BlueGoal = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/HSV/BlueGoal',
});

var ParameterHSV_YellowGoal = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/HSV/YellowGoal',
});

function Save_HSV() {

    var mode = parseInt(document.getElementById('HSVSelect').value);
    var box = [];
    let camera1 = document.getElementById('Camera1').checked;
    let camera2 = document.getElementById('Camera2').checked;
    switch (mode) {
        case 0:
            if(camera1){
                for (var i = 0; i < 6; i++) {
                    box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
                    RedBox[i] = box[i];
                }
                if (Param_HSV_Flag)
                    console.log('Param Redbox ' + box);
                //ParameterHSV_Red.set(box);
            }
            if(camera2){
                for (var i = 0; i < 6; i++) {
                    box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
                    RedGoalBox[i] = box[i];
                }
                if (Param_HSV_Flag)
                    console.log('Param Redbox ' + box);
                //ParameterHSV_Red.set(box);
            }
            break;
        case 1:
            if(camera1){
                for (var i = 0; i < 6; i++) {
                    box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
                    BlueBox[i] = box[i];
                }
                if (Param_HSV_Flag)
                    console.log('Param Bluebox ' + box);
                //ParameterHSV_Blue.set(box);
            }
            if(camera2){
                for (var i = 0; i < 6; i++) {
                    box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
                    BlueGoalBox[i] = box[i];
                }
                if (Param_HSV_Flag)
                    console.log('Param Bluebox ' + box);
                //ParameterHSV_Blue.set(box);
            }
            break;
        case 2:
            if(camera1){
                for (var i = 0; i < 6; i++) {
                    box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
                    YellowBox[i] = box[i];
                }
                if (Param_HSV_Flag)
                    console.log('Param Yellowbox ' + box);
                //ParameterHSV_Yellow.set(box);
            }
            if(camera2){
                for (var i = 0; i < 6; i++) {
                    box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
                    YellowGoalBox[i] = box[i];
                }
                if (Param_HSV_Flag)
                    console.log('Param Yellowbox ' + box);
                //ParameterHSV_Yellow.set(box);
            }
            break;
        case 3:
            for (var i = 0; i < 6; i++) {
                box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
                WhiteBox[i] = box[i];
            }
            if (Param_HSV_Flag)
                console.log('Param Whitebox ' + box);
            //ParameterHSV_White.set(box);
            break;
        case 4:
            for (var i = 0; i < 6; i++) {
                box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
                BlackBox[i] = box[i];
            }
            if (Param_HSV_Flag)
                console.log('Param Blackbox' + box);
            //ParameterHSV_Black.set(box);
            break;
    }
    //======================
    if(Param_HSV_Flag){
        if(camera1){
            ParameterHSV_Red.set(RedBox);
            ParameterHSV_Blue.set(BlueBox);
            ParameterHSV_Yellow.set(YellowBox);
            ParameterHSV_White.set(WhiteBox);
            ParameterHSV_Black.set(BlackBox);
        }
        if(camera2){
            ParameterHSV_RedGoal.set(RedGoalBox);
            ParameterHSV_BlueGoal.set(BlueGoalBox);
            ParameterHSV_YellowGoal.set(YellowGoalBox);
        }
    }
    setTimeout(Pub_Save(),3000);
    setTimeout(savecall(),3000);
	
}

function camera1_hsv_parameter_get(){
    ParameterHSV_Red.get(function(value) {
        var obj = document.getElementsByName("HSVElement");
        if (value != null) {
            for (var i = 0; i < obj.length; i++) {
                //obj[i].value = value[i];
                RedBox[i] = value[i];
                obj[i].value = value[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = value[i];
            }
        } else {
            for (var i = 0; i < RedBox.length; i++) {
                obj[i].value = RedBox[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = RedBox[i];
            }
        }
    });

    ParameterHSV_Blue.get(function(value) {
        if (value != null) {
            for (var i = 0; i < value.length; i++) {
                BlueBox[i] = value[i];
            }
        }else {
            for (var i = 0; i < BlueBox.length; i++) {
                obj[i].value = BlueBox[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = BlueBox[i];
            }
        }
    });

    ParameterHSV_Yellow.get(function(value) {
        if (value != null) {
            for (var i = 0; i < value.length; i++) {
                YellowBox[i] = value[i];
            }
        }else {
            for (var i = 0; i < YellowBox.length; i++) {
                obj[i].value = YellowBox[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = YellowBox[i];
            }
        }
    });

    ParameterHSV_White.get(function(value) {
        if (value != null) {
            for (var i = 0; i < value.length; i++) {
                WhiteBox[i] = value[i];
            }
        }
    });

    ParameterHSV_Black.get(function(value) {
        if (value != null) {
            for (var i = 0; i < value.length; i++) {
                BlackBox[i] = value[i];
            }
        }
    });
}
function camera2_hsv_parameter_get(){
    let obj = document.getElementsByName("HSVElement");
    ParameterHSV_RedGoal.get(function(value) {
        if (value != null) {
            for (var i = 0; i < obj.length; i++) {
                //obj[i].value = value[i];
                RedGoalBox[i] = value[i];
                obj[i].value = value[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = value[i];
            }
        } else {
            for (var i = 0; i < RedGoalBox.length; i++) {
                obj[i].value = RedGoalBox[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = RedGoalBox[i];
            }
        }
    });

    ParameterHSV_BlueGoal.get(function(value) {
        if (value != null) {
            for (var i = 0; i < value.length; i++) {
                BlueGoalBox[i] = value[i];
            }
        }else {
            for (var i = 0; i < BlueGoalBox.length; i++) {
                obj[i].value = BlueGoalBox[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = BlueGoalBox[i];
            }
        }
    });

    ParameterHSV_YellowGoal.get(function(value) {
        if (value != null) {
            for (var i = 0; i < value.length; i++) {
                YellowGoalBox[i] = value[i];
            }
        }else {
            for (var i = 0; i < YellowGoalBox.length; i++) {
                obj[i].value = YellowGoalBox[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = YellowGoalBox[i];
            }
        }
    });
}

//===================================================================
// vision param 
var Vision_Param = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/center',
});

function Save_VisionParam() {
    document.getElementById('Red_disable').checked = false;
    document.getElementById('Blue_disable').checked = false;
    document.getElementById('Yellow_disable').checked = false;
    //document.getElementById('White_disable').checked = false;
    //document.getElementById('Black_disable').checked = false;

    let obj = document.getElementsByName('VisionParamElement');
    var box = []
    for (let i = 0; i < obj.length; i++) {
        box[i] = parseInt(obj[i].value);
    }


    if (Param_Vison_Flag)
        console.log('Param Vision ' + box);
    let camera1 = document.getElementById('Camera1').checked;
    let camera2 = document.getElementById('Camera2').checked;
    if(camera1==true){
        Vision_Param.set(box);
    }
    if(camera2==true){
        Vision_Param2.set(box);
    }
    setTimeout(Pub_Save(),3000);
    setTimeout(savecall(),3000);
}
function camera1_center_parameter_get(){
    Vision_Param.get(function(value) {
        var obj = document.getElementsByName('VisionParamElement');
        var obj2 = document.getElementsByName('VisionParamLabelElement');
        if (value != null) {
            for (var i = 0; i < value.length; i++) {
                obj[i].value = value[i];
                obj2[i].innerText = value[i];
            }
            //document.getElementById("catch_distance_Input").innerText = 123;
            //obj2[4].innerText = value[4];
        } else {
            for (let i = 0; i < VisionParam.length; i++) {
                obj[i].value = VisionParam[i];
                obj2[i].innerText = VisionParam[i];
            }
            //obj2[4].innerText = 0;
        }
    });
}
var Vision_Param2 = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/center2',
});
function camera2_center_parameter_get(){
    Vision_Param2.get(function(value) {
        var obj = document.getElementsByName('VisionParamElement');
        var obj2 = document.getElementsByName('VisionParamLabelElement');
        if (value != null) {
            for (var i = 0; i < value.length; i++) {
                obj[i].value = value[i];
                obj2[i].innerText = value[i];
            }
            //document.getElementById("catch_distance_Input").innerText = 123;
            //obj2[4].innerText = value[4];
        } else {
            for (let i = 0; i < VisionParam.length; i++) {
                obj[i].value = VisionParam[i];
                obj2[i].innerText = VisionParam[i];
            }
            //obj2[4].innerText = 0;
        }
    });
}
var robot_Param = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/strategy/robot',
});
robot_Param.get(function(value) {
    let obj = document.getElementsByName('ParameterElement');
    if (value != null) {
        for (var i = 0; i < value.length; i++) {
            obj[i].innerText = value[i];
        }
    } else {
        for (let i = 0; i < RobotParam.length; i++) {
            obj[i].innerText = RobotParam[i];
        }
        //obj2[4].innerText = 0;
    }
});
var splanning_Param = new ROSLIB.Param({
    ros: ros,
    name: '/tb3/strategy/s_planning',
});
splanning_Param.get(function(value) {
    let obj = document.getElementsByName('s_planningElement');
    if (value != null) {
        for (var i = 0; i < value.length; i++) {
            obj[i].innerText = value[i];
        }
    } else {
        for (let i = 0; i < S_planningParam.length; i++) {
            obj[i].innerText = S_planningParam[i];
        }
        //obj2[4].innerText = 0;
    }
});
//============
function Set_strategy_Param() {
    let obj = document.getElementsByName('ParameterElement');
    let obj2 = document.getElementsByName('s_planningElement');
    let box = [];
    let box2 =[];
    for (let i = 0; i < obj.length; i++) {
        box[i] = parseFloat(obj[i].value);
    }
    for (let i = 0; i < obj2.length; i++) {
        box2[i] = parseFloat(obj2[i].value);
    }
    console.log('Param Robot ' + box);
    console.log('Param S_planning ' + box2);
    robot_Param.set(box);
    splanning_Param.set(box2);
    setTimeout(Pub_Save(),3000);
    setTimeout(savecall(),3000);
    setTimeout(strategy_savecall(),3000);
}
