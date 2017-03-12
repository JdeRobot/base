cd ..

python3.5 setup.py build install --user
#python3.5 ./MAVProxy/mavproxy.py --master=10.1.1.191:14550 --console
python3.5 ./MAVProxy/mavproxy.py --master=0.0.0.0:14550
