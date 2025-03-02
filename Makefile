init:
	python3 -m venv simulation-pybullet
	. simulation-pybullet/bin/activate

activate:
# run: "source simulation-pybullet/bin/activate" in shell

deactivate:
# run "deactivate" in shell

install:
	python3 -m pip install -r requirements.txt

start:
	python3 main.py