cosby: cosby.c Makefile
	gcc -O3 cosby.c -lasound -lsndfile -lfftw3 -lm -Wall -std=c99 -o cosby

static: cosby.c Makefile
	gcc -O3 cosby.c -lasound -lm /usr/local/lib/libsndfile.a /usr/local/lib/libfftw3.a -std=c99 -o cosby
	strip cosby

debug: cosby.c Makefile
	gcc -g  cosby.c -lasound -lsndfile -lfftw3 -lm -Wall -std=c99 -o cosby

clean:
	rm cosby
