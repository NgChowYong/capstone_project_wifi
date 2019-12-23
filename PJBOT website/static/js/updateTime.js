function updateTime() {
	let date = new Date();
	document.getElementById("times").innerHTML = date;
	document.getElementById("times").style.fontSize = "16px";
	setTimeout(updateTime, 1000);
}
