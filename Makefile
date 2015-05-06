CC = g++ -w
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lGL -lGLU -lm -lstdc++
else
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -Iglut-3.7.6-bin -IEigen
	LDFLAGS = -lglut -lGLU -lGL -lEigen
endif
	
RM = /bin/rm -f 
all: main 
main: iksolver.o 
	$(CC) $(CFLAGS) -o as4 iksolver.o $(LDFLAGS) 
iksolver.o: iksolver.cpp
	$(CC) $(CFLAGS) -c iksolver.cpp -o iksolver.o
clean: 
	$(RM) *.o as4
 


