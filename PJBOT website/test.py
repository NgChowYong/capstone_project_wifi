#!/usr/bin/python

from flask import Flask, request, render_template

app = Flask(__name__)

counter = 0

Robot_1 = ['Robot_1', 'Working', 'R_2']
Robot_2 = ['Robot_2', 'Working', 'T_5']

@app.route("/")
def index():
    return render_template('index.html',call=counter,
					Robot_1_Name=Robot_1[0],Robot_1_Status=Robot_1[1],Robot_1_Node=Robot_1[2],
					Robot_2_Name=Robot_2[0],Robot_2_Status=Robot_2[1],Robot_2_Node=Robot_2[2])

@app.route('/submit', methods=['POST'])
def submit():
	global counter
	counter += 1
	print "counter = : ", counter
	return render_template('index.html',call=counter,
					Robot_1_Name=Robot_1[0],Robot_1_Status=Robot_1[1],Robot_1_Node=Robot_1[2],
					Robot_2_Name=Robot_2[0],Robot_2_Status=Robot_2[1],Robot_2_Node=Robot_2[2])

@app.route('/reset', methods=['POST'])
def reset():
	global counter
	counter =0
	print "counter = : ", counter
	return render_template('index.html',call=counter,
					Robot_1_Name=Robot_1[0],Robot_1_Status=Robot_1[1],Robot_1_Node=Robot_1[2],
					Robot_2_Name=Robot_2[0],Robot_2_Status=Robot_2[1],Robot_2_Node=Robot_2[2])

if __name__ == "__main__":
    app.run(debug=True, port=5000)
