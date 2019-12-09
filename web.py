from flask import Flask,request,render_template
import time

app=Flask(__name__)
counter = 0

@app.route('/',methods=['GET'])
def initialize():
	global counter
	counter = 0
	return render_template('index.html',call=counter)

@app.route('/submit',methods=['POST'])
def submit():
	global counter
	counter += 1
	return render_template('index.html',call=counter)

@app.route('/reset',methods=['POST'])
def reset():
	global counter
	counter = 0
	return render_template('index.html',call=counter)

if __name__ == '__main__':
	app.run(debug=True,port=5000)
	t = time.time()
	while 1:
		n = time.time()
		if n-t > 1:
			print('count is : ',counter)
			t = n

# localhost:5000
