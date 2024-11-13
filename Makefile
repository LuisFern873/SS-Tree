# Makefile

# Variables
PYTHON_PATH = C:\Users\LENOVO\AppData\Local\Programs\Python\Python312
EIGEN_PATH = .\eigen-3.4.0
MPLCPP_PATH = .\matplotlib-cpp-master
NUMPY_PATH = $(PYTHON_PATH)\Lib\site-packages\numpy\core\include

# Compilation rule
all: main

main: main.cpp Point.cpp SSTree.cpp
	g++ -I$(PYTHON_PATH)\include -I$(EIGEN_PATH) -I$(MPLCPP_PATH) -I$(NUMPY_PATH) main.cpp Point.cpp SSTree.cpp -o main -L$(PYTHON_PATH)\libs -lpython312
