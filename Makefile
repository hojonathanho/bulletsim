CFLAGS=`pkg-config --cflags bullet eigen2` -Iexternal
LIBS=`pkg-config --libs bullet eigen2` -lglut -lGLU
EXECUTABLE=bulletscene
DEBUG=-g

all: bulletscene

bulletscene.o: bulletscene.cc
	g++ -c $(DEBUG) $(CFLAGS) bulletscene.cc -o bulletscene.o

external/GLDebugDrawer.o: external/GLDebugDrawer.cc external/GLDebugDrawer.h
	g++ -c $(DEBUG) $(CFLAGS) external/GLDebugDrawer.cc -o external/GLDebugDrawer.o

external/UDPSocket.o: external/UDPSocket.cpp external/UDPSocket.h
	g++ -c $(DEBUG) $(CFLAGS) external/UDPSocket.cpp -o external/UDPSocket.o

external/thread_socket_interface.o: external/thread_socket_interface.cpp external/thread_socket_interface.h
	g++ -c $(DEBUG) $(CFLAGS) external/thread_socket_interface.cpp -o external/thread_socket_interface.o

bulletscene: bulletscene.o external/GLDebugDrawer.o external/UDPSocket.o external/thread_socket_interface.o
	g++ bulletscene.o external/GLDebugDrawer.o external/UDPSocket.o external/thread_socket_interface.o $(LIBS) -o $(EXECUTABLE) $(DEBUG)

clean:
	-rm -f *.o external/*.o $(EXECUTABLE)
