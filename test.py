#!flask/bin/python
from flask import Flask, request, render_template    #記得要import render_template

app = Flask(__name__)

counter = 0

# #網頁執行/say_hello時，會導至index.html
@app.route('/', methods=['GET'])
def initialize():
	global counter
	counter = 0
	return render_template('index.html',call=counter)

#index.html按下submit時，會取得前端傳來的username，並回傳"Hello, World! "+name
@app.route('/submit', methods=['POST'])
def submit():
	global counter
	counter += 1
	return render_template('index.html',call=counter)

@app.route('/reset', methods=['POST'])
def reset():
	global counter
	counter =0
	return render_template('index.html',call=counter)


if __name__ == '__main__':
	app.run(debug=True, port=5000)
# localhost:5000