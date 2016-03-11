all: ahrs

obj = *.cpp

ahrs: main.cpp
	g++ -I../include $(obj) -o getahrs

clean:
	 del  $(obj)/*o getahrs