CFLAGS=`pkg-config --cflags bullet`
LIBS=`pkg-config --libs bullet` -lglut -lGLU
EXECUTABLE=bulletscene

all: bulletscene

bulletscene.o:
	g++ -c $(CFLAGS) bulletscene.cc -o bulletscene.o

GLDebugDrawer.o:
	g++ -c $(CFLAGS) GLDebugDrawer.cc -o GLDebugDrawer.o

bulletscene: bulletscene.o GLDebugDrawer.o
	g++ bulletscene.o GLDebugDrawer.o $(LIBS) -o $(EXECUTABLE)

clean:
	-rm -f *.o $(EXECUTABLE)
