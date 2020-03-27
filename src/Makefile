CC = clang
LDFLAGS += -lm

all:
	$(CC) $(LDFLAGS) *.c -g -Wpedantic -o tilt

format:
	clang-format -i *.[ch]
