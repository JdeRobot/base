# Configuring libfreenect2

```sh
$ git clone https://github.com/OpenKinect/libfreenect2
$ cd libfreenect2/depends
$ ./install_ubuntu.sh
$ cd ..
$ export LIBFREENECT_ROOT=$PWD #you may want to include this ENV VARIABLE to your .bash.rc
$ cd examples/protonect/
$ cmake .
$ make 
$ sudo make install
```

# Troubleshooting
OpenCL not found:
/home/frivas/svn/others/libfreenect2/examples/protonect/src/opencl_depth_packet_processor.cpp:45:21: fatal error: CL/cl.hpp: No existe el archivo o el directorio

 
```sh
$ cd $LIBFREENECT_ROOT
$ cd examples/protonect/
$ cmake -DENABLE_OPENCL=OFF .
$ make 
$ sudo make install
```

OpenGL acceleration is not working (black image on IR and depth data)
```sh
$ cd $LIBFREENECT_ROOT
$ cd examples/protonect/
$ cmake -DENABLE_OPENGL=OFF .
$ make 
$ sudo make install
```
