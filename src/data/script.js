var ids = ['mess', 'freq', 'degsec', 'pidmax', 'proll', 'iroll','droll','ppitch','ipitch','dpitch','pyaw','iyaw','dyaw'];
var vals = ['' ,250, 360, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0];

function httpGetAsync(theUrl, callback)
{
    var xmlHttp = new XMLHttpRequest();
    xmlHttp.onreadystatechange = function() { 
        if (xmlHttp.readyState == 4 && xmlHttp.status == 200)
            callback(xmlHttp.responseText);
    }
    xmlHttp.open("GET", theUrl, false); // true for asynchronous 
    xmlHttp.send(null);
}

for (var i = 0; i < ids.length; i = i + 1)
{
	var cb;
	httpGetAsync('192.168.4.1/getVar?q=' + ids[i], cb);
	if(cb != undefined) 
	{
		document.getElementById(ids[i]).value = cb;
	}
	else
	{
		document.getElementById(ids[i]).value = vals[i];
	}
}

