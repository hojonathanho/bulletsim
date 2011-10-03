CFLAGS=`pkg-config --cflags bullet`
LIBS=`pkg-config --libs bullet` -lglut -lGLU
EXECUTABLE=bulletscene
DEBUG=-g

all: bulletscene

bulletscene.o: bulletscene.cc
	g++ -c $(DEBUG) $(CFLAGS) bulletscene.cc -o bulletscene.o

GLDebugDrawer.o: GLDebugDrawer.cc GLDebugDrawer.h
	g++ -c $(DEBUG) $(CFLAGS) GLDebugDrawer.cc -o GLDebugDrawer.o

bulletscene: bulletscene.o GLDebugDrawer.o
	g++ bulletscene.o GLDebugDrawer.o $(LIBS) -o $(EXECUTABLE) $(DEBUG)

clean:
	-rm -f *.o $(EXECUTABLE)
