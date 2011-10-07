CFLAGS=`pkg-config --cflags bullet` -Iexternal
LIBS=`pkg-config --libs bullet` -lglut -lGLU
EXECUTABLE=bulletscene
DEBUG=-g

all: bulletscene

bulletscene.o: bulletscene.cc
	g++ -c $(DEBUG) $(CFLAGS) bulletscene.cc -o bulletscene.o

GLDebugDrawer.o: external/GLDebugDrawer.cc external/GLDebugDrawer.h
	g++ -c $(DEBUG) $(CFLAGS) external/GLDebugDrawer.cc -o external/GLDebugDrawer.o

bulletscene: bulletscene.o GLDebugDrawer.o
	g++ bulletscene.o external/GLDebugDrawer.o $(LIBS) -o $(EXECUTABLE) $(DEBUG)

clean:
	-rm -f *.o external/*.o $(EXECUTABLE)
