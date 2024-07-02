CC=gcc
LIBS=-lm
CFLAGS=-Wall -Wextra -Wconversion -pedantic
GLFLAGS=-lglfw -lGL -lGLEW
EXE=./run
TRASH=./run *.o *.so
SRCS = primitives.c shader.c alloc.c frames.c physics.c tree.c io.c nerd.c
OBJS = $(SRCS:.c=.o)

.PHONY: clean

all: clean main
	$(EXE)

main: $(OBJS) main.c
	$(CC) -o $(EXE) main.c $(CFLAGS) $(GLFLAGS) $(OBJS) $(LIBS)

$(OBJS): %.o: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -rf $(TRASH)
