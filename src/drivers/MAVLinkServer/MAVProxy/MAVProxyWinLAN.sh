cd ..

python2.7 setup.py build install --user
python2.7 ./MAVProxy/mavproxy.py ./MAVProxy/uav_viewer_py.yml --master=10.1.1.191:14550 --console
