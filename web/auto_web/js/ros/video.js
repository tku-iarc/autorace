//===========================================================================
//new

function Open_Camera(value) {

    var video = document.getElementById("player");
    var view = document.getElementById("Camera-View");
    
    if (view.checked) {
        if (value == 1) 
        {
            video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/camera/image"
        }
        else if(value == 2)
        {
            
            video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/detect/image_lane/compressed"
            console.log(video.src)
        }
    } 
    else 
    {
        video.src = "img/offline.png";
    }
}

