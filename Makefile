all:switcher_danfysik

switcher_danfysik: switcher_danfysik.c
	$(CC) -o switcher_danfysik -Wall -g switcher_danfysik.c -I. -lX11

clean:
	$(RM) switcher_danfysik
